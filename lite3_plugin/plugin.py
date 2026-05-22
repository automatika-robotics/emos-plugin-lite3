"""``Lite3Plugin`` — a Sugarcoat robot plugin for the DeepRobotics Lite3.

This is a direct, class-based conversion of the DeepRobotics ``message_transformer``
ROS 2 package. Where ``message_transformer`` runs two C++ bridge nodes that
translate UDP <-> ROS topics, this plugin folds that translation into the
Sugarcoat plugin framework: it speaks the Lite3 Motion Host UDP protocol
directly, so no separate bridge process is needed.

* **Feedback** — binds UDP ``:43897`` for the robot's telemetry stream and
  decodes ``RobotState`` packets into standard ``Odometry`` and ``Imu`` inputs
  plus a ``Float64`` battery level.
* **Commands** — a standard ``Twist`` output is encoded to the Lite3's three
  ``ComplexCMD`` velocity packets and sent to UDP ``:43893``; an ``Audio``
  output is streamed as raw PCM to the Motion Host speaker.
* **Actions** — the Lite3's named ``SimpleCMD`` behaviours (sit/stand, hello,
  twist, gaits, mode switches, ...).
* **Events** — a ``low_battery`` event built from the battery feedback.
* **Heartbeat** — the ``0x21040001`` keep-alive is sent at 4 Hz while active.
"""

from typing import Callable, Optional

from nav_msgs.msg import Odometry as RosOdometry
from sensor_msgs.msg import Imu as RosImu
from std_msgs.msg import Float64 as RosFloat64

from ros_sugar.core.action import Action
from ros_sugar.core.event import Event
from ros_sugar.robot import (
    ActionRegistry,
    EventRegistry,
    Feedback,
    PluginMetadata,
    RobotCommand,
    RobotPlugin,
    UdpTransport,
)
from ros_sugar.supported_types import Float64, Odometry

from . import audio as audio_codec
from . import codecs, protocol
from .protocol import CommandCode
from .types import Lite3Imu


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
    msg.pose.pose.position.x = state.pos_world[0]
    msg.pose.pose.position.y = state.pos_world[1]
    msg.pose.pose.position.z = state.pos_world[2]
    qx, qy, qz, qw = codecs.quaternion_from_rpy_degrees(0.0, 0.0, state.rpy[2])
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    msg.twist.twist.linear.x = state.vel_body[0]
    msg.twist.twist.linear.y = state.vel_body[1]
    msg.twist.twist.angular.z = state.rpy_vel[2]
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


class Lite3Plugin(RobotPlugin):
    """Sugarcoat robot plugin for the DeepRobotics Lite3 quadruped.

    Construction is zero-argument: ``Lite3Plugin()`` — every endpoint and tuning
    knob is part of the robot's identity, baked into the class. Override via
    subclass when you need a non-default deployment (testing on localhost,
    custom subnet, calibration tweak)::

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
    AUDIO_PORT = 43899
    #: Sample rate the Motion Host receiver expects; match it in the
    #: receiver's gstreamer caps. Default 24000 Hz -- the native rate of the
    #: local (sherpa-onnx Kokoro) TTS model; override for other models.
    AUDIO_SAMPLE_RATE = 24000
    #: Audio frames per UDP packet.
    AUDIO_BLOCK_SIZE = 1024

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
                "manoeuvres such as jumps. It carries an onboard IMU and "
                "reports leg odometry and battery state. It is used for "
                "inspection, research and education."
            ),
        )
        self._vel_x_factor = self.VEL_X_FACTOR

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
                msg_type=Lite3Imu,
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
        }

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
                        "Switch the control source to NAVIGATION (the robot "
                        "follows commands from its onboard "
                        "navigation/perception host). Idempotent."
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
        self.events = EventRegistry({"low_battery": self._make_low_battery_event})

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
        so :class:`~ros_sugar.robot.ActionRegistry` surfaces it for LLM-driven
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


__all__ = ["Lite3Plugin"]
