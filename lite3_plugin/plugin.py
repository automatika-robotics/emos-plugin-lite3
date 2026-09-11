"""``Lite3Plugin`` — a Sugarcoat robot plugin for the DeepRobotics Lite3.

This is a direct, class-based conversion of the DeepRobotics ``message_transformer``
ROS 2 package. Where ``message_transformer`` runs two C++ bridge nodes that
translate UDP <-> ROS topics, this plugin folds that translation into the
Sugarcoat plugin framework: it speaks the Lite3 Motion Host UDP protocol
directly, so no separate bridge process is needed.

* **Feedback** — binds UDP ``:43897`` for the robot's telemetry stream and
  decodes ``RobotState`` packets into standard ``Odometry`` and ``Imu`` inputs,
  a ``Float64`` battery level, two ``Range`` ultrasonic distances (front/back),
  a ``String`` status token, and ``Bool`` balance / fallen flags; it also
  decodes the ``JointState`` (12 leg-joint angles) and ``HandleState``
  (operator joystick, as a ``Twist``) streams.
* **Commands** — a standard ``Twist`` output is encoded to the Lite3's three
  ``ComplexCMD`` velocity packets and sent to UDP ``:43893``; an ``Audio``
  output is streamed as raw PCM to the Motion Host speaker.
* **Actions** — the Lite3's named ``SimpleCMD`` behaviours (sit/stand, hello,
  twist, gaits, mode switches, ...).
* **Events** — ``low_battery``, ``obstacle_ahead`` (front ultrasound),
  ``balance_disturbed`` and ``fallen`` events built from the feedback streams.
* **Sensors** — the robot's Livox Mid-360 LiDAR and Intel RealSense are exposed
  as native-ROS feedbacks (``lidar`` point cloud; ``camera`` / ``camera_info`` /
  ``rgbd``), and their vendor drivers (``livox_ros_driver2`` /
  ``realsense2_camera``) are started on demand from ``required_processes`` --
  the plugin declares the drivers rather than bridging the data itself, since
  both ship real ROS drivers whose data rides native DDS.
* **Heartbeat** — the ``0x21040001`` keep-alive is sent at 4 Hz while active.
"""

import socket
from pathlib import Path
from typing import Callable, List, Optional

import numpy as np
from geometry_msgs.msg import Twist as RosTwist
from nav_msgs.msg import Odometry as RosOdometry
from sensor_msgs.msg import Imu as RosImu
from sensor_msgs.msg import JointState as RosJointState
from sensor_msgs.msg import Range as RosRange
from std_msgs.msg import Bool as RosBool
from std_msgs.msg import Float64 as RosFloat64
from std_msgs.msg import String as RosString

from rclpy.logging import get_logger
from rclpy.qos import ReliabilityPolicy

from ros_sugar.config import (
    AngularCtrlLimits,
    LinearCtrlLimits,
    QoSConfig,
    RobotConfig,
)
from ros_sugar.core.action import Action
from ros_sugar.core.event import Event
from ros_sugar.robot import (
    ActionRegistry,
    EventRegistry,
    Feedback,
    PluginMetadata,
    ProcessSpec,
    RobotCommand,
    RobotPlugin,
    RosTopicTransport,
    UdpTransport,
    create_supported_type,
    Mount,
)
from ros_sugar.supported_types import (
    Bool,
    CameraInfo,
    Float64,
    Image,
    Imu,
    JointState,
    Odometry,
    PointCloud2,
    Range,
    String,
    Twist,
)

from . import audio as audio_codec
from . import codecs, protocol
from .protocol import CommandCode



# --------------------------------------------------------------------------
# Telemetry decoders (raw UDP packet -> ROS message, or None to ignore)
# --------------------------------------------------------------------------
def _decode_odometry(raw: bytes) -> Optional[RosOdometry]:
    """Decode a Lite3 ``RobotState`` packet into ``nav_msgs/Odometry``."""
    state = codecs.parse_robot_state(raw)
    if state is None:
        return None
    msg = RosOdometry()
    msg.header.frame_id = "odom"
    msg.child_frame_id = "body"
    # pos_world is {x, y, yaw}: the third element is the world-frame heading
    # in radians, NOT a Z position. The Lite3 walks on the ground plane, so z
    # stays 0 and the heading goes into the orientation quaternion.
    msg.pose.pose.position.x = state.pos_world[0]
    msg.pose.pose.position.y = state.pos_world[1]
    msg.pose.pose.position.z = 0.0
    qx, qy, qz, qw = codecs.quaternion_from_rpy_degrees(0.0, 0.0, state.rpy[2])
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    # twist is expressed in the child (body) frame, so use the body-frame
    # velocities: vel_body is {x_vel, y_vel, yaw_vel}.
    msg.twist.twist.linear.x = state.vel_body[0]
    msg.twist.twist.linear.y = state.vel_body[1]
    msg.twist.twist.angular.z = state.vel_body[2]
    return msg


def _decode_imu(raw: bytes) -> Optional[RosImu]:
    """Decode a Lite3 ``RobotState`` packet into ``sensor_msgs/Imu``."""
    state = codecs.parse_robot_state(raw)
    if state is None:
        return None
    msg = RosImu()
    msg.header.frame_id = "imu"
    qx, qy, qz, qw = codecs.quaternion_from_rpy_degrees(
        state.rpy[0], state.rpy[1], state.rpy[2]
    )
    msg.orientation.x = qx
    msg.orientation.y = qy
    msg.orientation.z = qz
    msg.orientation.w = qw
    msg.angular_velocity.x = state.rpy_vel[0]
    msg.angular_velocity.y = state.rpy_vel[1]
    msg.angular_velocity.z = state.rpy_vel[2]
    msg.linear_acceleration.x = state.xyz_acc[0]
    msg.linear_acceleration.y = state.xyz_acc[1]
    msg.linear_acceleration.z = state.xyz_acc[2]
    return msg


def _decode_battery(raw: bytes) -> Optional[RosFloat64]:
    """Decode the battery percentage from a Lite3 ``RobotState`` packet."""
    state = codecs.parse_robot_state(raw)
    if state is None:
        return None
    msg = RosFloat64()
    msg.data = float(state.battery_level)
    return msg


def _make_range(distance: float, frame_id: str) -> RosRange:
    """Build a ``sensor_msgs/Range`` for a Lite3 ultrasonic rangefinder."""
    msg = RosRange()
    msg.header.frame_id = frame_id
    msg.radiation_type = RosRange.ULTRASOUND
    # The interface doc does not publish a beam width; this is a nominal
    # estimate for a small ultrasonic cone and is not load-bearing.
    msg.field_of_view = 0.5
    msg.min_range = codecs.ULTRASOUND_MIN_RANGE
    msg.max_range = codecs.ULTRASOUND_MAX_RANGE
    msg.range = float(distance)
    return msg


def _decode_ultrasound_front(raw: bytes) -> Optional[RosRange]:
    """Decode the front obstacle distance from a Lite3 ``RobotState`` packet."""
    state = codecs.parse_robot_state(raw)
    if state is None:
        return None
    return _make_range(state.ultrasound[0], "ultrasound_front")


def _decode_ultrasound_back(raw: bytes) -> Optional[RosRange]:
    """Decode the rear obstacle distance from a Lite3 ``RobotState`` packet."""
    state = codecs.parse_robot_state(raw)
    if state is None:
        return None
    return _make_range(state.ultrasound[1], "ultrasound_back")


def _decode_status(raw: bytes) -> Optional[RosString]:
    """Decode a human-readable status token from a Lite3 ``RobotState`` packet.

    Combines the basic / gait / motion state fields into one of the tokens in
    :data:`codecs.BASIC_STATE_NAMES` / ``GAIT_NAMES`` / ``MOTION_STATE_NAMES``
    (e.g. ``"sitting"``, ``"standing"``, ``"walking_flat_fast"``,
    ``"long_jump"``, ``"lose_control_protection"``)."""
    state = codecs.parse_robot_state(raw)
    if state is None:
        return None
    msg = RosString()
    msg.data = codecs.describe_robot_status(state)
    return msg


def _decode_is_balanced(raw: bytes) -> Optional[RosBool]:
    """Decode the balance flag from a Lite3 ``RobotState`` packet.

    ``True`` while the robot can hold its balance; ``False`` (from
    ``is_robot_need_move``) when an external force has disturbed it and it must
    step to recover."""
    state = codecs.parse_robot_state(raw)
    if state is None:
        return None
    msg = RosBool()
    msg.data = not bool(state.is_robot_need_move)
    return msg


def _decode_is_fallen(raw: bytes) -> Optional[RosBool]:
    """Decode a fallen flag from a Lite3 ``RobotState`` packet.

    ``True`` when ``robot_basic_state`` is a lose-control-protection or
    flipping-over state (see :data:`codecs.FALLEN_BASIC_STATES`)."""
    state = codecs.parse_robot_state(raw)
    if state is None:
        return None
    msg = RosBool()
    msg.data = state.robot_basic_state in codecs.FALLEN_BASIC_STATES
    return msg


def _decode_joint_state(raw: bytes) -> Optional[RosJointState]:
    """Decode a Lite3 ``JointState`` packet into ``sensor_msgs/JointState``.

    Populates the 12 leg-joint names / positions (radians) in order.
    The Lite3 reports angles with the opposite sign to the URDF convention,
    so positions are negated to match."""
    state = codecs.parse_joint_state(raw)
    if state is None:
        return None
    msg = RosJointState()
    msg.header.frame_id = "body"
    msg.name = list(codecs.JOINT_NAMES)
    msg.position = [-getattr(state, name) for name in codecs.JOINT_NAMES]
    return msg


def _decode_handle(raw: bytes) -> Optional[RosTwist]:
    """Decode a Lite3 ``HandleState`` (operator joystick) packet into a
    ``geometry_msgs/Twist``.

    The left stick maps to linear x/y and the right stick to yaw; yaw is
    negated to match the plugin's velocity-command convention."""
    state = codecs.parse_handle_state(raw)
    if state is None:
        return None
    msg = RosTwist()
    msg.linear.x = state.left_axis_forward
    msg.linear.y = state.left_axis_side
    msg.angular.z = -state.right_axis_yaw
    return msg


def _packaged_config(filename: str) -> Optional[str]:
    """Absolute path to a config file shipped with this package, or ``None``.

    Prefers the installed ament share copy, falls back to the source tree so the
    plugin also works from a checkout. Returns ``None`` rather than a missing
    path.
    """
    try:
        from ament_index_python.packages import get_package_share_directory

        candidate = (
            Path(get_package_share_directory("lite3_plugin")) / "config" / filename
        )
        if candidate.is_file():
            return str(candidate)
    except Exception:  # not built/installed, or ament unavailable
        pass
    candidate = Path(__file__).resolve().parent.parent / "config" / filename
    return str(candidate) if candidate.is_file() else None


_RGBD_TYPE = None
_RGBD_UNAVAILABLE = False


def _rgbd_type():
    """The ``realsense2_camera_msgs/RGBD`` ``SupportedType``, built once and cached."""
    global _RGBD_TYPE, _RGBD_UNAVAILABLE
    if _RGBD_TYPE is not None or _RGBD_UNAVAILABLE:
        return _RGBD_TYPE
    try:
        from realsense2_camera_msgs.msg import RGBD as RosRGBD

        _RGBD_TYPE = create_supported_type(RosRGBD, module=__name__)
    except Exception:  # realsense2_camera_msgs not installed
        _RGBD_UNAVAILABLE = True
    return _RGBD_TYPE


class Lite3Plugin(RobotPlugin):
    """Sugarcoat robot plugin for the DeepRobotics Lite3 quadruped.

    Construction is zero-argument: ``Lite3Plugin()`` — every endpoint and tuning
    knob is part of the robot's identity, baked into the class. Override via
    subclass when you need a non-default deployment:

        class MyLite3(Lite3Plugin):
            MOTION_HOST_IP = "10.0.0.42"     # robot lives on a different subnet
            VEL_X_FACTOR = 0.85              # this unit's forward calibration
    """

    # --- Robot-specific endpoints on the robot's internal LAN ---
    #: IP of the Motion Host (QNX) board, reached from the compute board where
    #: sugarcoat runs.
    MOTION_HOST_IP = protocol.DEFAULT_ROBOT_IP
    #: Motion Host UDP port that accepts Motion Host commands.
    COMMAND_PORT = protocol.DEFAULT_COMMAND_PORT
    #: Local UDP port the Motion Host streams telemetry to.
    TELEMETRY_PORT = protocol.DEFAULT_TELEMETRY_PORT
    #: Local interface to bind the telemetry receiver on. ``0.0.0.0`` is the
    #: safe default — only worth pinning to a specific NIC IP for firewall scope.
    BIND_HOST = "0.0.0.0"
    #: Scale applied to forward velocity commands (mirrors the ``vel_x_factor``
    #: parameter of the DeepRobotics bridge).
    VEL_X_FACTOR = 1.0
    #: Send the ``0x21040001`` keep-alive at 4 Hz while active. The Lite3 expects
    #: a heartbeat to retain external control; only disable for offline tests.
    SEND_HEARTBEAT = True
    #: Host the speaker audio is streamed to. ``None`` -> the Motion Host.
    AUDIO_HOST = None
    #: UDP port the Motion Host audio receiver listens on (see README).
    #: Kept well clear of the robot's own ``43xxx`` port block (jy_exe).
    AUDIO_PORT = 5005
    #: Sample rate the Motion Host receiver expects; match it in the
    #: receiver's gstreamer caps. Default 24000 Hz -- the native rate of the
    #: local (sherpa-onnx Kokoro) TTS model; override for other models.
    AUDIO_SAMPLE_RATE = 24000
    #: Audio frames per UDP packet.
    AUDIO_BLOCK_SIZE = 1024

    # --- Robot model ---
    # The Lite3 is a quadruped that accepts a Twist, so for kompass planning
    # purposes it is modelled as DIFFERENTIAL_DRIVE with a BOX footprint.
    ROBOT_DRIVE_TYPE = "DIFFERENTIAL_DRIVE"
    ROBOT_GEOMETRY_TYPE = "BOX"
    #: ``[length, width, height]`` in metres -- the Lite3 is ~61x37x40 cm.
    ROBOT_GEOMETRY_PARAMS = (0.61, 0.37, 0.4)
    #: Forward velocity limits (m/s, m/s^2).
    ROBOT_VX_MAX = 1.0
    ROBOT_VX_ACC = 1.5
    ROBOT_VX_DECEL = 2.5
    #: Angular velocity limits (rad/s, rad/s^2).
    ROBOT_OMEGA_MAX = 2.0
    ROBOT_OMEGA_ACC = 3.0
    ROBOT_OMEGA_DECEL = 3.0
    ROBOT_STEER_MAX = np.pi

    # Where the built-in ultrasonic rangefinders sit on the body, as
    # (xyz, rpy) relative to ``base_frame``: nose and tail of the trunk on
    # its centre line, the rear one facing backwards.
    SENSOR_MOUNTS = {
        "ultrasound_front": ((0.305, 0.0, 0.0), (0.0, 0.0, 0.0)),
        "ultrasound_back": ((-0.305, 0.0, 0.0), (0.0, 0.0, np.pi)),
    }

    # --- Livox Mid-360 LiDAR (driver started by required_processes) ----------
    #: Expose the Mid-360 point cloud, and start livox_ros_driver2 for a recipe
    #: that binds it. Set False on a unit with no LiDAR.
    HAS_LIDAR = True
    LIDAR_DRIVER_PACKAGE = "livox_ros_driver2"
    LIDAR_DRIVER_EXECUTABLE = "livox_ros_driver2_node"
    #: Livox ``user_config`` JSON (host + lidar IPs and ports). The packaged default
    #: carries the Mid-360 network defaults, so point ``LIDAR_CONFIG`` at the JSON
    #: whose IPs match this robot. ``None`` means the driver is not started, and it
    #: says so.
    LIDAR_CONFIG: Optional[str] = _packaged_config("mid360_config.json")
    #: Topic the driver publishes the cloud on, and the frame it is expressed in.
    LIDAR_TOPIC = "/livox/lidar"
    LIDAR_FRAME = "livox_frame"
    #: Livox transfer format (0 = ``sensor_msgs/PointCloud2``) and publish Hz.
    LIDAR_XFER_FORMAT = 0
    LIDAR_PUBLISH_FREQ = 10.0
    #: Host UDP ports the Mid-360 streams to. Checked free before the driver is
    #: started. Keep in step with the ports in ``LIDAR_CONFIG``.
    LIDAR_HOST_PORTS = (56101, 56201, 56301)

    # --- Intel RealSense camera (driver started by required_processes) -------
    #: Expose the RealSense streams, and start realsense2_camera for a recipe
    #: that binds them.
    HAS_CAMERA = True
    CAMERA_DRIVER_PACKAGE = "realsense2_camera"
    CAMERA_DRIVER_EXECUTABLE = "realsense2_camera_node"
    #: Node name the driver runs as.
    CAMERA_NODE_NAME = "camera"
    #: Bind a specific device only when more than one RealSense is attached.
    CAMERA_SERIAL_NO: Optional[str] = None
    #: Topic overrides for the colour image, its CameraInfo, and the synchronised
    #: RGBD packet. ``None`` derives ``/<CAMERA_NODE_NAME>/<stream>``; set a string
    #: only for a non-standard namespace / remap.
    CAMERA_COLOR_TOPIC: Optional[str] = None
    CAMERA_INFO_TOPIC: Optional[str] = None
    CAMERA_RGBD_TOPIC: Optional[str] = None

    # --- Sensor feedback QoS -------------------------------------------------
    #: Feedbacks subscribe BEST_EFFORT so they receive from a driver publishing
    #: either reliability; sensor streams are high-rate and drop-tolerant.
    SENSOR_QOS_RELIABILITY = ReliabilityPolicy.BEST_EFFORT
    SENSOR_QOS_DEPTH = 5

    def __init__(self):
        self.metadata = PluginMetadata(
            name="Lite3",
            vendor="DeepRobotics",
            version="1.0",
            description=(
                "A DeepRobotics Lite3: a small, agile quadruped (four-legged) "
                "robot roughly the size of a medium dog. It moves on legs "
                "rather than wheels, so it walks, turns in place, climbs "
                "stairs and handles uneven terrain, and can perform dynamic "
                "maneuvers such as jumps. It carries an onboard IMU and "
                "reports leg odometry and battery state. It is used for "
                "inspection, research and education."
            ),
        )
        self._vel_x_factor = self.VEL_X_FACTOR

        # The frame rigidly attached to the robot's body
        self.base_frame = "body"
        # Static transforms body -> sensor frames, published by the launcher
        self.mounts = [
            Mount(parent=self, child=frame, xyz=xyz, rpy=rpy)
            for frame, (xyz, rpy) in self.SENSOR_MOUNTS.items()
        ]

        # Define robot config
        self.robot_config = RobotConfig(
            model_type=self.ROBOT_DRIVE_TYPE,
            geometry_type=self.ROBOT_GEOMETRY_TYPE,
            geometry_params=np.array(self.ROBOT_GEOMETRY_PARAMS),
            ctrl_vx_limits=LinearCtrlLimits(
                max_vel=self.ROBOT_VX_MAX,
                max_acc=self.ROBOT_VX_ACC,
                max_decel=self.ROBOT_VX_DECEL,
            ),
            ctrl_omega_limits=AngularCtrlLimits(
                max_omega=self.ROBOT_OMEGA_MAX,
                max_acc=self.ROBOT_OMEGA_ACC,
                max_decel=self.ROBOT_OMEGA_DECEL,
                max_ang=self.ROBOT_STEER_MAX,
            ),
        )

        # Send-only command endpoint, carrying the 4 Hz heartbeat.
        command = UdpTransport(
            "command",
            send_to=(self.MOTION_HOST_IP, self.COMMAND_PORT),
            keep_alive_fn=self._heartbeat if self.SEND_HEARTBEAT else None,
            keep_alive_rate_hz=4.0 if self.SEND_HEARTBEAT else None,
        )
        # Receive-only telemetry endpoint — the Lite3 streams unprompted.
        telemetry = UdpTransport(
            "telemetry", bind=(self.BIND_HOST, self.TELEMETRY_PORT)
        )
        # Send-only audio endpoint to the Motion Host speaker.
        audio = UdpTransport(
            "audio",
            send_to=(self.AUDIO_HOST or self.MOTION_HOST_IP, self.AUDIO_PORT),
        )
        self.transports = {
            "command": command,
            "telemetry": telemetry,
            "audio": audio,
        }

        self.feedbacks = {
            "Odometry": Feedback(
                key="Odometry",
                msg_type=Odometry,
                transport=telemetry,
                decoder=_decode_odometry,
                rate_hz=100.0,
                description="Leg odometry decoded from the Lite3 RobotState stream",
            ),
            "Imu": Feedback(
                key="Imu",
                msg_type=Imu,
                transport=telemetry,
                decoder=_decode_imu,
                rate_hz=100.0,
                description="Body IMU decoded from the Lite3 RobotState stream",
            ),
            # Keyed "battery" (a role name) rather than the bare type name.
            "battery": Feedback(
                key="battery",
                msg_type=Float64,
                transport=telemetry,
                decoder=_decode_battery,
                rate_hz=100.0,
                description="Battery percentage from the Lite3 RobotState stream",
            ),
            # Front / back ultrasonic rangefinders (metres, valid 0.28-4.50 m).
            "ultrasound_front": Feedback(
                key="ultrasound_front",
                msg_type=Range,
                transport=telemetry,
                decoder=_decode_ultrasound_front,
                rate_hz=50.0,
                description="Front obstacle distance from the Lite3 ultrasonic sensor",
            ),
            "ultrasound_back": Feedback(
                key="ultrasound_back",
                msg_type=Range,
                transport=telemetry,
                decoder=_decode_ultrasound_back,
                rate_hz=50.0,
                description="Rear obstacle distance from the Lite3 ultrasonic sensor",
            ),
            # Human-readable status token (sitting / standing / walking_* /
            # long_jump / lose_control_protection / ...), for LLM monitors.
            "robot_status": Feedback(
                key="robot_status",
                msg_type=String,
                transport=telemetry,
                decoder=_decode_status,
                rate_hz=50.0,
                description=(
                    "Lite3 status token combining basic, gait and motion state "
                    "(e.g. sitting, standing, walking_flat_fast, long_jump)"
                ),
            ),
            # Balance flag: False when an external force has disturbed the robot
            # and it must step to recover (from is_robot_need_move).
            "is_balanced": Feedback(
                key="is_balanced",
                msg_type=Bool,
                transport=telemetry,
                decoder=_decode_is_balanced,
                rate_hz=50.0,
                description="True while the Lite3 can hold its balance",
            ),
            # Fallen flag: True in a lose-control-protection or flipping-over
            # basic state.
            "is_fallen": Feedback(
                key="is_fallen",
                msg_type=Bool,
                transport=telemetry,
                decoder=_decode_is_fallen,
                rate_hz=50.0,
                description="True when the Lite3 has lost its footing / flipped over",
            ),
            # The 12 leg-joint angles, from the Lite3 JointState stream (2306).
            "JointState": Feedback(
                key="JointState",
                msg_type=JointState,
                transport=telemetry,
                decoder=_decode_joint_state,
                rate_hz=100.0,
                description="The 12 leg-joint angles decoded from the Lite3 JointState stream",
            ),
            # Operator joystick / handle command as a Twist, from the Lite3
            # HandleState stream (2309).
            "handle": Feedback(
                key="handle",
                msg_type=Twist,
                transport=telemetry,
                decoder=_decode_handle,
                rate_hz=50.0,
                description="Operator joystick command (as a Twist) from the Lite3 HandleState stream",
            ),
        }

        # Driver-backed sensors (Livox Mid-360, Intel RealSense) as native-ROS
        # feedbacks.
        self._add_ros_sensors()

        self.commands = {
            # A standard Twist output becomes the Lite3's three velocity packets.
            "Twist": RobotCommand(
                key="Twist",
                transport=command,
                encoder=self._encode_twist,
                description="Base velocity, sent as three Lite3 ComplexCMD packets",
            ),
            # An Audio output (e.g. from a TextToSpeech component) is streamed
            # as raw PCM to the Motion Host speaker.
            "Audio": RobotCommand(
                key="Audio",
                transport=audio,
                encoder=self._encode_audio,
                description="Speech audio, streamed as raw PCM to the Motion Host speaker",
            ),
        }

        # Named SimpleCMD behaviours, exposed as Action factories. Each carries
        # a tool description with state-machine guidance so an LLM-driven
        # monitor (e.g. Cortex) knows when it is safe / valid to invoke the
        # action. The Lite3 has two orthogonal state axes:
        #
        #   - Pose / Move mode  -- POSE_MODE enables trick routines; MOVE_MODE
        #     enables Twist velocity + gait selectors. The robot ignores
        #     commands that don't belong to its current mode.
        #   - Sitting / Standing -- the robot starts sitting (legs folded).
        #     Trick routines and velocity commands require the robot to be
        #     standing first (call ``sit_stand`` to bring it up).
        self.actions = ActionRegistry(
            {
                # one-shot trick actions -- require POSE_MODE and (for routines
                # other than sit_stand) require the robot to be standing first
                "sit_stand": self._simple_cmd_action(
                    CommandCode.SIT_STAND,
                    description=(
                        "Toggle between sitting and standing. The Lite3 starts "
                        "in a sitting pose with its legs folded; calling this "
                        "action once brings it up to standing. Calling it "
                        "again while standing makes it sit back down. Most "
                        "other trick actions and velocity commands only work "
                        "while the robot is standing -- invoke this first if "
                        "the robot is still sitting. Requires POSE_MODE."
                    ),
                ),
                "say_hello": self._simple_cmd_action(
                    CommandCode.SAY_HELLO,
                    description=(
                        "Play the Lite3's greeting routine (waves a front leg "
                        "in a hello gesture). Requires the robot to be "
                        "standing and in POSE_MODE."
                    ),
                ),
                "twist": self._simple_cmd_action(
                    CommandCode.TWIST,
                    description=(
                        "Play a body-twist demo routine in place (the robot "
                        "rotates its torso side-to-side without translating). "
                        "Requires the robot to be standing and in POSE_MODE."
                    ),
                ),
                "twist_jump": self._simple_cmd_action(
                    CommandCode.TWIST_JUMP,
                    description=(
                        "Play the twisting-jump demo routine in place (the "
                        "robot hops while rotating). Requires the robot to be "
                        "standing and in POSE_MODE; leave clearance around it."
                    ),
                ),
                "moonwalk": self._simple_cmd_action(
                    CommandCode.MOONWALK,
                    description=(
                        "Play the moonwalk demo routine (the robot performs a "
                        "scripted backward gliding gait). Requires the robot "
                        "to be standing and in POSE_MODE; needs clear floor "
                        "space behind it."
                    ),
                ),
                "long_jump": self._simple_cmd_action(
                    CommandCode.LONG_JUMP,
                    description=(
                        "Perform a forward long-jump. Requires the robot to "
                        "be standing and in POSE_MODE; needs clear floor "
                        "space (a couple of metres) ahead."
                    ),
                ),
                "stand_zero": self._simple_cmd_action(
                    CommandCode.STAND_ZERO,
                    description=(
                        "Return all joints to the zero (calibration) pose "
                        "while standing. Requires the robot to be standing "
                        "and in POSE_MODE. Useful as a reset between trick "
                        "actions."
                    ),
                ),
                # mode selection -- idempotent; safe to call any time
                "set_pose_mode": self._simple_cmd_action(
                    CommandCode.MODE_POSE,
                    description=(
                        "Switch the robot into POSE_MODE. Required before "
                        "calling any one-shot trick action (sit_stand, "
                        "say_hello, twist, twist_jump, moonwalk, long_jump, "
                        "stand_zero). Idempotent."
                    ),
                ),
                "set_move_mode": self._simple_cmd_action(
                    CommandCode.MODE_MOVE,
                    description=(
                        "Switch the robot into MOVE_MODE. Required before "
                        "sending Twist velocity commands or selecting a gait. "
                        "Idempotent."
                    ),
                ),
                "set_manual_mode": self._simple_cmd_action(
                    CommandCode.CONTROL_MANUAL,
                    description=(
                        "Switch the control source to MANUAL (the robot "
                        "responds to operator/controller velocity commands "
                        "from this plugin). Idempotent."
                    ),
                ),
                "set_navigation_mode": self._simple_cmd_action(
                    CommandCode.CONTROL_NAVIGATION,
                    description=(
                        "Switch the control source to NAVIGATION mode or AUTONOMOUS mode. The robot can move autonomously"
                        "by following commands from the onboard "
                        "navigation system. Idempotent."
                    ),
                ),
                # gait selectors -- only take effect in MOVE_MODE
                "gait_slow": self._simple_cmd_action(
                    CommandCode.GAIT_FLAT_SLOW,
                    description=(
                        "Select the slow flat-terrain gait. Only takes "
                        "effect while in MOVE_MODE and standing."
                    ),
                ),
                "gait_medium": self._simple_cmd_action(
                    CommandCode.GAIT_FLAT_MEDIUM,
                    description=(
                        "Select the medium flat-terrain gait. Only takes "
                        "effect while in MOVE_MODE and standing."
                    ),
                ),
                "gait_fast": self._simple_cmd_action(
                    CommandCode.GAIT_FLAT_FAST,
                    description=(
                        "Select the fast flat-terrain gait. Only takes "
                        "effect while in MOVE_MODE and standing."
                    ),
                ),
                # misc -- mode-independent
                "save_data": self._simple_cmd_action(
                    CommandCode.SAVE_DATA,
                    description=(
                        "Save the previous ~100 s of the Lite3's onboard "
                        "data log to its storage. Works in any mode."
                    ),
                ),
                # KEEP_STEPPING with cmd_value 2 disables stepping == stop
                "stop": self._simple_cmd_action(
                    CommandCode.KEEP_STEPPING,
                    2,
                    description=(
                        "Halt the robot by disabling the stepping "
                        "controller. The robot stops moving but remains "
                        "standing. Safe to call any time; primarily useful "
                        "while in MOVE_MODE to abort an ongoing motion."
                    ),
                ),
            }
        )

        # Pre-built event factories.
        self.events = EventRegistry(
            {
                "low_battery": self._make_low_battery_event,
                "obstacle_ahead": self._make_obstacle_ahead_event,
                "balance_disturbed": self._make_balance_disturbed_event,
                "fallen": self._make_fallen_event,
            }
        )

    # -- heartbeat -----------------------------------------------------------
    def _heartbeat(self) -> None:
        """Send the Lite3 keep-alive packet."""
        self.transports["command"].send(codecs.encode_heartbeat())

    # -- command encoders ----------------------------------------------------
    def _encode_twist(self, output):
        """Encode a component Twist output ``[vx, vy, wz]`` into the Lite3's
        three velocity packets, applying ``vel_x_factor`` to forward speed."""
        vx, vy, wz = float(output[0]), float(output[1]), float(output[2])
        return codecs.encode_velocity(vx * self._vel_x_factor, vy, wz)

    def _encode_audio(self, output):
        """Decode an Audio output into raw PCM blocks for the Motion Host."""
        from rclpy.logging import get_logger

        return audio_codec.encode_audio(
            output,
            block_size=self.AUDIO_BLOCK_SIZE,
            expected_rate=self.AUDIO_SAMPLE_RATE,
            logger=get_logger("lite3_plugin"),
        )

    # -- action / event factories -------------------------------------------
    def _simple_cmd_action(
        self,
        cmd_code: int,
        cmd_value: int = 0,
        type_: int = 0,
        *,
        description: str = "",
    ) -> Callable[[], Action]:
        """Return a factory that builds an Action sending one ``SimpleCMD``.

        ``description`` is stamped onto the factory as ``_tool_description``
        so `~ros_sugar.robot.ActionRegistry` surfaces it for LLM-driven
        consumers (e.g. EmbodiedAgents' Cortex).
        """
        command = self.transports["command"]
        payload = codecs.encode_simple_cmd(cmd_code, cmd_value, type_)
        factory: Callable[[], Action] = lambda: Action(
            method=lambda: command.send(payload)
        )
        if description:
            factory._tool_description = description  # type: ignore[attr-defined]
        return factory

    def _make_low_battery_event(self, threshold: float = 20.0) -> Event:
        """Build an Event that fires when the battery drops below ``threshold``
        percent."""
        battery = self.feedbacks["battery"].as_topic()
        return Event(event_condition=battery.msg.data < threshold, on_change=True)

    def _make_obstacle_ahead_event(self, threshold: float = 0.5) -> Event:
        """Build an Event that fires when the front ultrasonic distance drops
        below ``threshold`` metres."""
        front = self.feedbacks["ultrasound_front"].as_topic()
        return Event(event_condition=front.msg.range < threshold, on_change=True)

    def _make_balance_disturbed_event(self) -> Event:
        """Build an Event that fires when the robot can no longer hold its
        balance and must step to recover."""
        balanced = self.feedbacks["is_balanced"].as_topic()
        return Event(event_condition=~balanced.msg.data, on_change=True)

    def _make_fallen_event(self) -> Event:
        """Build an Event that fires when the robot has lost its footing
        (lose-control-protection or flipping-over state)."""
        fallen = self.feedbacks["is_fallen"].as_topic()
        return Event(event_condition=fallen.msg.data == True, on_change=True)

    # -- driver-backed sensors (Livox Mid-360, Intel RealSense) --------------
    def _add_ros_sensor(self, key: str, topic: str, msg_type, description: str) -> None:
        """Register a driver-backed sensor stream as a native-ROS feedback."""
        transport = RosTopicTransport(
            key,
            topic_name=topic,
            msg_type=msg_type,
            qos=QoSConfig(
                reliability=self.SENSOR_QOS_RELIABILITY,
                queue_size=self.SENSOR_QOS_DEPTH,
            ),
        )
        self.transports[key] = transport
        self.feedbacks[key] = Feedback(
            key=key, msg_type=msg_type, transport=transport, description=description
        )

    def _add_ros_sensors(self) -> None:
        """Register the Livox and RealSense feedbacks the plugin exposes."""
        if self.HAS_LIDAR:
            self._add_ros_sensor(
                "lidar",
                self.LIDAR_TOPIC,
                PointCloud2,
                "Livox Mid-360 point cloud (from livox_ros_driver2)",
            )
        if self.HAS_CAMERA:
            # A directly-launched realsense2_camera node publishes its streams
            # under the node name, so derive /<node>/<stream> unless overridden.
            ns = self.CAMERA_NODE_NAME
            color_topic = self.CAMERA_COLOR_TOPIC or f"/{ns}/color/image_raw"
            info_topic = self.CAMERA_INFO_TOPIC or f"/{ns}/color/camera_info"
            rgbd_topic = self.CAMERA_RGBD_TOPIC or f"/{ns}/rgbd"
            self._add_ros_sensor(
                "camera",
                color_topic,
                Image,
                "Intel RealSense colour image",
            )
            self._add_ros_sensor(
                "camera_info",
                info_topic,
                CameraInfo,
                "Intel RealSense colour camera intrinsics",
            )
            rgbd_type = _rgbd_type()
            if rgbd_type is not None:
                self._add_ros_sensor(
                    "rgbd",
                    rgbd_topic,
                    rgbd_type,
                    "Intel RealSense synchronised colour + depth (RGBD)",
                )
            else:
                get_logger(self.metadata.name).warning(
                    "realsense2_camera_msgs is not available, so the 'rgbd' "
                    "feedback is not exposed. Install realsense2_camera to use "
                    "the synchronised RGBD stream; 'camera' and 'camera_info' "
                    "are unaffected."
                )

    def required_processes(self):
        """Start the Mid-360 and RealSense drivers. Each only if the recipe
        binds that sensor.

        These sensors ship real vendor ROS drivers, so their data rides native
        DDS. The launcher owns each process (respawn, captured output, teardown
        ordered with the recipe), and a recipe that ignores a sensor never pays
        to run its driver.
        """
        processes = []
        requested = self.requested_feedbacks

        if self.HAS_LIDAR and "lidar" in requested:
            if self.LIDAR_CONFIG:
                processes.append(
                    ProcessSpec(
                        package=self.LIDAR_DRIVER_PACKAGE,
                        executable=self.LIDAR_DRIVER_EXECUTABLE,
                        name="lite3_livox",
                        parameters=[
                            {
                                "xfer_format": self.LIDAR_XFER_FORMAT,
                                "multi_topic": 0,
                                "publish_freq": self.LIDAR_PUBLISH_FREQ,
                                "frame_id": self.LIDAR_FRAME,
                                "user_config_path": self.LIDAR_CONFIG,
                            }
                        ],
                        precondition=self._lidar_ports_are_free,
                    )
                )
            else:
                get_logger(self.metadata.name).warning(
                    "Recipe binds 'lidar' but no Livox config was found. The "
                    "packaged config/mid360_config.json is missing -- build the "
                    "package, or set LIDAR_CONFIG to a Mid-360 user_config JSON "
                    "whose IPs match this robot. Not starting the LiDAR driver."
                )

        if self.HAS_CAMERA and {"camera", "camera_info", "rgbd"} & requested:
            want_rgbd = "rgbd" in requested
            params = {
                "enable_color": True,
                # RGBD needs depth + time-synced alignment; a colour-only recipe
                # skips them.
                "enable_depth": want_rgbd,
                "enable_sync": want_rgbd,
                "align_depth.enable": want_rgbd,
                "enable_rgbd": want_rgbd,
                "pointcloud.enable": False,
            }
            if self.CAMERA_SERIAL_NO:
                params["serial_no"] = str(self.CAMERA_SERIAL_NO)
            processes.append(
                ProcessSpec(
                    package=self.CAMERA_DRIVER_PACKAGE,
                    executable=self.CAMERA_DRIVER_EXECUTABLE,
                    name=self.CAMERA_NODE_NAME,
                    parameters=[params],
                )
            )
        return processes

    def _lidar_ports_are_free(self) -> bool:
        """True when no process already holds the Mid-360's host UDP ports.

        Checked before starting so that reads as a port conflict rather than a crash.
        """
        for port in self.LIDAR_HOST_PORTS:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.bind(("", port))
            except OSError:
                get_logger(self.metadata.name).warning(
                    f"UDP {port} is already bound, so the Livox driver will NOT "
                    "be started -- a second binder receives nothing. A Mid-360 "
                    "driver is likely already running."
                )
                return False
            finally:
                sock.close()
        return True


__all__ = ["Lite3Plugin"]
