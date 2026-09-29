"""End-to-end tests for the DeepRobotics Lite3 Sugarcoat plugin.

Exercises the binary protocol, the codecs, and the full plugin HOST flow
against the mock UDP Lite3 from ``server_node.py`` — no hardware required.

Run from the repo root with a Sugarcoat that has the ``ros_sugar.robot``
framework importable::

    python3 -m pytest test/lite3_plugin_test.py -v
"""

import base64
import ctypes
import inspect
import io
import os
import socket
import sys
import threading
import time

import attrs
import numpy as np
import pytest

# Make `lite3_plugin` and `server_node` importable when run from the repo root.
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from nav_msgs.msg import Odometry as RosOdometry  # noqa: E402
from sensor_msgs.msg import Imu as RosImu  # noqa: E402
from std_msgs.msg import Float64 as RosFloat64  # noqa: E402

from ros_sugar.core.action import Action  # noqa: E402
from ros_sugar.robot import (  # noqa: E402
    InProcessFeedbackBus,
    RobotPlugin,
    RobotPluginHost,
)

from lite3_plugin import codecs, protocol  # noqa: E402
from lite3_plugin.protocol import CommandCode  # noqa: E402
from lite3_plugin import Lite3Plugin  # noqa: E402
from lite3_plugin import plugin as _plugin_module  # noqa: E402
from lite3_plugin.plugin import _ImuDecoder, _rgbd_type  # noqa: E402
from server_node import MockLite3  # noqa: E402


class _Lite3PluginForTest(Lite3Plugin):
    """Lite3Plugin variant for localhost testing.

    Overrides the robot-specific endpoints as instance attributes (set *before*
    ``super().__init__()``) so the production ``Lite3Plugin`` constructor reads
    them in place of the class defaults — the recommended override pattern.
    """

    #: Pinned rather than detected: what LiDAR is on the machine running the
    #: tests is not the subject of most of them. The detection itself has its
    #: own tests below.
    LIDAR_KIND = "livox"

    def __init__(self, *, command_port: int, telemetry_port: int):
        self.MOTION_HOST_IP = "127.0.0.1"
        self.COMMAND_PORT = command_port
        self.TELEMETRY_PORT = telemetry_port
        super().__init__()


def _free_port() -> int:
    import socket

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture
def mock_lite3():
    """A running mock UDP Lite3 on free command/telemetry ports."""
    import threading

    command_port = _free_port()
    telemetry_port = _free_port()
    robot = MockLite3(
        command_port=command_port,
        telemetry_port=telemetry_port,
        telemetry_rate_hz=100.0,
    )
    thread = threading.Thread(target=robot.run, daemon=True)
    thread.start()
    time.sleep(0.2)
    yield robot, command_port, telemetry_port
    robot._stop.set()
    thread.join(timeout=2.0)


# ---------------------------------------------------------------------------
# Binary protocol
# ---------------------------------------------------------------------------
def test_protocol_struct_sizes_are_distinct():
    """Telemetry packet sizes must differ so length-based dispatch works."""
    sizes = {
        protocol.ROBOT_STATE_SIZE,
        protocol.ROBOT_STATE_WITH_POLICY_SIZE,
        protocol.IMU_SIZE,
        protocol.JOINT_STATE_SIZE,
        protocol.HANDLE_STATE_SIZE,
    }
    assert len(sizes) == 5
    # The IMU frame as the robot sends it: the 12-byte header, then a
    # timestamp and nine floats.
    assert protocol.IMU_SIZE == 52
    # The newer RobotState layout is exactly one extra int (robot_policy_state).
    assert protocol.ROBOT_STATE_WITH_POLICY_SIZE == protocol.ROBOT_STATE_SIZE + 4
    # Command structs: 3x int32, and that + an 8-byte double.
    assert ctypes.sizeof(protocol.SimpleCMD) == 12
    assert ctypes.sizeof(protocol.ComplexCMD) == 20


def test_protocol_struct_roundtrip():
    """A RobotState frame survives a bytes round-trip with its fields intact."""
    frame = protocol.RobotStateReceived()
    frame.code = protocol.ROBOT_STATE_CODE
    frame.data.pos_world[0] = 1.5
    frame.data.rpy[2] = 42.0
    frame.data.battery_level = 87.5
    raw = bytes(frame)
    assert len(raw) == protocol.ROBOT_STATE_SIZE
    parsed = protocol.RobotStateReceived.from_buffer_copy(raw)
    assert parsed.code == protocol.ROBOT_STATE_CODE
    assert parsed.data.pos_world[0] == 1.5
    assert parsed.data.rpy[2] == 42.0
    assert parsed.data.battery_level == 87.5


# ---------------------------------------------------------------------------
# Codecs
# ---------------------------------------------------------------------------
def test_encode_commands():
    """SimpleCMD / ComplexCMD encoders produce the documented wire layout."""
    simple = codecs.encode_simple_cmd(CommandCode.SIT_STAND)
    assert len(simple) == 12
    assert protocol.SimpleCMD.from_buffer_copy(simple).cmd_code == CommandCode.SIT_STAND

    complex_ = codecs.encode_complex_cmd(CommandCode.VEL_FORWARD, 8, 1, 0.5)
    parsed = protocol.ComplexCMD.from_buffer_copy(complex_)
    assert (parsed.cmd_code, parsed.cmd_value, parsed.type) == (
        CommandCode.VEL_FORWARD,
        8,
        1,
    )
    assert parsed.data == 0.5


def test_encode_velocity_is_three_packets():
    """A base velocity becomes three ComplexCMD packets; yaw is negated."""
    packets = codecs.encode_velocity(0.4, 0.1, 0.2)
    assert len(packets) == 3
    codes = [protocol.ComplexCMD.from_buffer_copy(p).cmd_code for p in packets]
    assert codes == [
        CommandCode.VEL_FORWARD,
        CommandCode.VEL_LATERAL,
        CommandCode.VEL_YAW,
    ]
    yaw_packet = protocol.ComplexCMD.from_buffer_copy(packets[2])
    assert yaw_packet.data == pytest.approx(-0.2)


def test_parse_robot_state_rejects_other_packets():
    """parse_robot_state only accepts well-formed RobotState frames."""
    frame = protocol.RobotStateReceived()
    frame.code = protocol.ROBOT_STATE_CODE
    frame.data.battery_level = 55.0
    assert codecs.parse_robot_state(bytes(frame)).battery_level == 55.0
    # A SimpleCMD-sized packet is not robot state
    assert codecs.parse_robot_state(codecs.encode_simple_cmd(1)) is None
    # Right size, wrong code
    frame.code = 9999
    assert codecs.parse_robot_state(bytes(frame)) is None


def test_parse_robot_state_accepts_newer_layout():
    """parse_robot_state auto-detects the 4-byte-larger RobotStateWithPolicy frame
    (newer Lite3_ROS firmware) without breaking the default layout."""
    frame = protocol.RobotStateReceivedWithPolicy()
    frame.code = protocol.ROBOT_STATE_CODE
    frame.data.battery_level = 73.0
    frame.data.robot_policy_state = 5
    raw = bytes(frame)
    assert len(raw) == protocol.ROBOT_STATE_WITH_POLICY_SIZE
    state = codecs.parse_robot_state(raw)
    assert state is not None
    assert state.battery_level == 73.0
    # The extra field is exposed on the with-policy payload.
    assert state.robot_policy_state == 5


def test_parse_imu_roundtrip():
    """parse_imu decodes the dedicated IMU frame and rejects everything else."""
    frame = protocol.ImuReceived()
    frame.code = protocol.IMU_CODE
    frame.data.timestamp = 3247931
    frame.data.rpy[1] = 0.74
    frame.data.rpy_vel[0] = -0.0029
    frame.data.xyz_acc[2] = 9.81
    raw = bytes(frame)
    assert len(raw) == protocol.IMU_SIZE
    imu = codecs.parse_imu(raw)
    assert imu is not None
    assert imu.timestamp == 3247931
    # The fields are floats here, doubles in a RobotState frame.
    assert imu.rpy[1] == pytest.approx(0.74)
    assert imu.rpy_vel[0] == pytest.approx(-0.0029)
    assert imu.xyz_acc[2] == pytest.approx(9.81)
    # A robot-state frame is not an IMU frame, nor is the right size with the
    # wrong code.
    state = protocol.RobotStateReceived()
    state.code = protocol.ROBOT_STATE_CODE
    assert codecs.parse_imu(bytes(state)) is None
    frame.code = protocol.ROBOT_STATE_CODE
    assert codecs.parse_imu(bytes(frame)) is None


def test_parse_joint_state_roundtrip():
    """parse_joint_state decodes the 12 leg-joint angles in JOINT_NAMES order."""
    frame = protocol.JointStateReceived()
    frame.code = protocol.JOINT_STATE_CODE
    values = [0.1 * i for i in range(12)]
    for name, value in zip(codecs.JOINT_NAMES, values):
        setattr(frame.data, name, value)
    state = codecs.parse_joint_state(bytes(frame))
    assert state is not None
    assert [getattr(state, name) for name in codecs.JOINT_NAMES] == values
    # JOINT_NAMES is exactly the struct's field order.
    assert list(codecs.JOINT_NAMES) == [f[0] for f in protocol.JointState._fields_]
    # Wrong-sized / wrong-code packets are rejected.
    assert codecs.parse_joint_state(codecs.encode_simple_cmd(1)) is None


def test_parse_handle_state_roundtrip():
    """parse_handle_state decodes the operator joystick frame."""
    frame = protocol.HandleStateReceived()
    frame.code = protocol.HANDLE_STATE_CODE
    frame.data.left_axis_forward = 0.5
    frame.data.left_axis_side = -0.25
    frame.data.right_axis_yaw = 0.75
    state = codecs.parse_handle_state(bytes(frame))
    assert state is not None
    assert state.left_axis_forward == 0.5
    assert state.left_axis_side == -0.25
    assert state.right_axis_yaw == 0.75


def test_quaternion_from_rpy():
    """Zero RPY is the identity quaternion; 180 deg yaw flips z."""
    assert codecs.quaternion_from_rpy_degrees(0, 0, 0) == pytest.approx(
        (0.0, 0.0, 0.0, 1.0)
    )
    x, y, z, w = codecs.quaternion_from_rpy_degrees(0, 0, 180)
    assert (x, y) == pytest.approx((0.0, 0.0))
    assert abs(z) == pytest.approx(1.0)
    assert w == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# Plugin construction & introspection
# ---------------------------------------------------------------------------
# The full feedback / event surface the plugin exposes.
_BASE_FEEDBACKS = {
    "Odometry",
    "Imu",
    "battery",
    "ultrasound_front",
    "ultrasound_back",
    "robot_status",
    "is_balanced",
    "is_fallen",
    "JointState",
    "handle",
}
# Driver-backed sensors (Livox Mid-360 + Intel RealSense), started via
# required_processes and consumed on native ROS topics. Each has a transport of
# the same key. RGBD is only exposed when realsense2_camera_msgs is installed.
_SENSOR_FEEDBACKS = {"lidar", "lidar_imu", "camera", "camera_info"}
if _rgbd_type() is not None:
    _SENSOR_FEEDBACKS = _SENSOR_FEEDBACKS | {"rgbd"}
# Every stream on a native ROS topic, each with a transport of the same key: the
# sensors, and the EKF's estimate.
_ROS_FEEDBACKS = _SENSOR_FEEDBACKS | {"odometry_filtered"}
_EXPECTED_FEEDBACKS = _BASE_FEEDBACKS | _ROS_FEEDBACKS
_EXPECTED_EVENTS = {"low_battery", "obstacle_ahead", "balance_disturbed", "fallen"}


def test_plugin_construction():
    """The plugin builds declaratively and exposes the expected surface."""
    plugin = Lite3Plugin()
    assert plugin.metadata.vendor == "DeepRobotics"
    assert set(plugin.transports) == (
        {"command", "telemetry", "audio"} | _ROS_FEEDBACKS
    )
    assert set(plugin.feedbacks) == _EXPECTED_FEEDBACKS
    assert set(plugin.commands) == {"Twist", "Audio"}
    # robot_config is a kompass RobotConfig (kompass is a hard dep of this
    # plugin -- import would have sys.exit'd otherwise).
    assert plugin.robot_config.model_type == "DIFFERENTIAL_DRIVE"
    assert plugin.robot_config.geometry_type.value == "BOX"
    for action in ("sit_stand", "say_hello", "set_move_mode", "stop", "gait_fast"):
        assert action in plugin.actions
    for event in _EXPECTED_EVENTS:
        assert event in plugin.events
    # The command transport carries the 4 Hz heartbeat
    assert plugin.transports["command"].keep_alive_rate_hz == 4.0


def test_plugin_spec_roundtrip():
    """Production ``Lite3Plugin()`` round-trips through its spec, which carries
    the LiDAR it resolved; an override subclass captures its own kwargs."""
    plugin = Lite3Plugin()
    spec = plugin.to_spec()
    assert spec["class"].endswith(":Lite3Plugin")
    assert spec["kwargs"] == {"lidar_kind": plugin.LIDAR_KIND}
    rebuilt = RobotPlugin.from_spec(spec)
    assert set(rebuilt.feedbacks) == _EXPECTED_FEEDBACKS
    assert rebuilt.MOTION_HOST_IP == protocol.DEFAULT_ROBOT_IP

    # An override subclass captures its constructor kwargs and applies them
    # as instance attributes that shadow the class defaults.
    test_plugin = _Lite3PluginForTest(command_port=40001, telemetry_port=40002)
    test_spec = test_plugin.to_spec()
    assert test_spec["class"].endswith(":_Lite3PluginForTest")
    assert test_spec["kwargs"] == {"command_port": 40001, "telemetry_port": 40002}
    assert test_plugin.MOTION_HOST_IP == "127.0.0.1"


def test_plugin_introspection():
    """``describe`` reflects the Lite3 plugin's surface."""
    desc = Lite3Plugin().describe()
    assert desc["metadata"]["name"] == "Lite3"
    assert {f["key"] for f in desc["feedbacks"]} == _EXPECTED_FEEDBACKS
    assert {c["key"] for c in desc["commands"]} == {"Twist", "Audio"}
    assert "sit_stand" in {a["name"] for a in desc["actions"]}
    assert {e["name"] for e in desc["events"]} == _EXPECTED_EVENTS


def test_required_processes_gated_on_requested():
    """Sensor drivers are declared only for the sensors a recipe binds."""
    plugin = Lite3Plugin()

    # Nothing requested -> no drivers started.
    plugin._set_requested(frozenset(), frozenset())
    assert plugin.required_processes() == []

    # Binding the LiDAR declares the Livox driver with its packaged config.
    plugin._set_requested(frozenset({"lidar"}), frozenset())
    specs = plugin.required_processes()
    assert [s.package for s in specs] == ["livox_ros_driver2"]
    assert specs[0].parameters[0]["user_config_path"] == plugin.lidar.config

    # A colour-only camera bind starts RealSense without depth / RGBD.
    plugin._set_requested(frozenset({"camera"}), frozenset())
    (rs,) = plugin.required_processes()
    assert rs.package == "realsense2_camera"
    assert rs.parameters[0]["enable_rgbd"] is False
    assert rs.parameters[0]["enable_depth"] is False

    # Binding RGBD turns on depth + sync + alignment.
    plugin._set_requested(frozenset({"rgbd"}), frozenset())
    (rs,) = plugin.required_processes()
    params = rs.parameters[0]
    assert params["enable_rgbd"] and params["enable_depth"]
    assert params["align_depth.enable"] and params["enable_sync"]

    # Both sensors at once -> both drivers.
    plugin._set_requested(frozenset({"lidar", "camera_info"}), frozenset())
    assert {s.package for s in plugin.required_processes()} == {
        "livox_ros_driver2",
        "realsense2_camera",
    }


# ---------------------------------------------------------------------------
# End-to-end against the mock Lite3
# ---------------------------------------------------------------------------
def test_host_telemetry_decoding(mock_lite3):
    """The HOST plugin decodes the Lite3 telemetry stream into Odometry, Imu
    and battery feedback on the bus, and feeds the monitor."""
    robot, command_port, telemetry_port = mock_lite3
    plugin = _Lite3PluginForTest(
        command_port=command_port, telemetry_port=telemetry_port
    )
    bus = InProcessFeedbackBus()
    monitor_feed = []
    host = RobotPluginHost(
        plugin,
        node=None,
        bus=bus,
        monitor_feed=lambda name, msg: monitor_feed.append(name),
    )
    host.open()
    try:
        # The in-process bus hands over the decoded message itself.
        odom, imu, battery = [], [], []
        bus.subscribe("robot/feedback/Odometry", odom.append)
        bus.subscribe("robot/feedback/Imu", imu.append)
        bus.subscribe("robot/feedback/battery", battery.append)
        deadline = time.time() + 2.0
        while (not odom or not imu or not battery) and time.time() < deadline:
            time.sleep(0.02)
        assert odom and imu and battery, "telemetry was not decoded onto the bus"
        assert isinstance(odom[0], RosOdometry)
        assert isinstance(imu[0], RosImu)
        assert isinstance(battery[-1], RosFloat64)
        assert battery[-1].data == pytest.approx(100.0, abs=1.0)
        # All three feedback channels were also pushed to the monitor
        assert {"robot/feedback/Odometry", "robot/feedback/Imu"} <= set(
            monitor_feed
        )
    finally:
        host.close()


def test_host_twist_command(mock_lite3):
    """A Twist command is encoded to three ComplexCMD packets the robot acts on,
    and the resulting motion shows up in the decoded odometry."""
    robot, command_port, telemetry_port = mock_lite3
    plugin = _Lite3PluginForTest(
        command_port=command_port, telemetry_port=telemetry_port
    )
    bus = InProcessFeedbackBus()
    host = RobotPluginHost(plugin, node=None, bus=bus)
    host.open()
    try:
        twist = plugin.commands["Twist"]
        # Component output [vx, vy, wz]
        plugin.send_command(twist, twist.encoder([0.6, 0.0, 0.0]))
        time.sleep(0.4)
        assert robot._vx == pytest.approx(0.6), "robot did not receive the velocity"

        decoded = []
        bus.subscribe("robot/feedback/Odometry", decoded.append)
        deadline = time.time() + 2.0
        while not decoded and time.time() < deadline:
            time.sleep(0.02)
        assert decoded[-1].pose.pose.position.x > 0.0
        assert decoded[-1].twist.twist.linear.x == pytest.approx(0.6, abs=1e-6)
    finally:
        host.close()


def test_named_action_sends_simple_cmd(mock_lite3):
    """A named action factory builds an Action that sends one SimpleCMD."""
    robot, command_port, telemetry_port = mock_lite3
    plugin = _Lite3PluginForTest(
        command_port=command_port, telemetry_port=telemetry_port
    )
    host = RobotPluginHost(plugin, node=None, bus=InProcessFeedbackBus())
    host.open()

    received = []
    original = robot._handle_inbound

    def _capture(data: bytes):
        if len(data) == ctypes.sizeof(protocol.SimpleCMD):
            received.append(protocol.SimpleCMD.from_buffer_copy(data).cmd_code)
        original(data)

    robot._handle_inbound = _capture
    try:
        action = plugin.actions.sit_stand()
        succeeded, message = action()
        assert succeeded, message
        # The result says which action ran, not just a command code
        assert "sit_stand" in message
        deadline = time.time() + 1.0
        while CommandCode.SIT_STAND not in received and time.time() < deadline:
            time.sleep(0.02)
        assert CommandCode.SIT_STAND in received
    finally:
        host.close()


def test_actions_are_named_after_their_keys():
    """Every action is named after its registry key, which is what a Routine's
    cursor and the Monitor's logs report. Each factory names its own Action,
    so building one never renames another."""
    plugin = Lite3Plugin()
    for name in plugin.actions.names():
        assert getattr(plugin.actions, name)().action_name == name


def test_action_factory_forwards_action_kwargs():
    """Keyword arguments given to an action factory reach the Action it builds,
    so a recipe can configure a plugin action like any other."""
    plugin = Lite3Plugin()
    action = plugin.actions.stop(description="Halt before the doorway")
    assert action.description == "Halt before the doorway"
    assert action.action_name == "stop"


@pytest.mark.skipif(
    "success" not in inspect.signature(Action.__init__).parameters,
    reason="this Sugarcoat's Action does not support monitoring",
)
def test_action_factory_accepts_a_monitoring_policy():
    """The Lite3 never acknowledges a command, so a success condition on its
    own telemetry is the only way an action can tell it worked. It has to
    reach the Action through the factory."""
    plugin = Lite3Plugin()
    status = plugin.feedbacks["robot_status"].as_topic()
    action = plugin.actions.sit_stand(
        success=status.msg.data == "standing", timeout=10.0
    )
    assert action.is_monitored
    assert action.success_event is not None
    assert action.action_name == "sit_stand"
    # A name given by the recipe wins over the registry key
    assert plugin.actions.stop(name="halt", timeout=1.0).action_name == "halt"


def test_encode_audio_blocks():
    """``encode_audio`` decodes an audio blob into raw mono F32LE PCM blocks."""
    sf = pytest.importorskip("soundfile")
    np = pytest.importorskip("numpy")
    from lite3_plugin.audio import encode_audio

    rate = 16000
    samples = np.zeros(rate, dtype="float32")  # 1 s of silence, mono
    buf = io.BytesIO()
    sf.write(buf, samples, rate, format="WAV", subtype="FLOAT")
    wav_bytes = buf.getvalue()

    blocks = encode_audio(wav_bytes, block_size=1024, expected_rate=rate)
    assert blocks, "no PCM blocks produced"
    assert len(blocks[0]) == 1024 * 4  # block_size mono float32 frames
    assert sum(len(b) // 4 for b in blocks) == rate  # all frames accounted for
    # base64 string input is accepted too
    assert encode_audio(base64.b64encode(wav_bytes).decode(), expected_rate=rate) == blocks


def test_audio_command_streams_over_udp(mock_lite3):
    """The ``Audio`` command encodes an audio blob and streams raw PCM to the
    plugin's audio UDP endpoint."""
    sf = pytest.importorskip("soundfile")
    np = pytest.importorskip("numpy")

    _robot, command_port, telemetry_port = mock_lite3
    audio_port = _free_port()

    # A receiver standing in for the Motion Host speaker.
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("127.0.0.1", audio_port))
    rx.settimeout(2.0)

    class _AudioLite3(_Lite3PluginForTest):
        AUDIO_HOST = "127.0.0.1"
        AUDIO_PORT = audio_port
        AUDIO_SAMPLE_RATE = 16000  # matches the test WAV below

    plugin = _AudioLite3(command_port=command_port, telemetry_port=telemetry_port)
    host = RobotPluginHost(plugin, node=None, bus=InProcessFeedbackBus())
    host.open()
    try:
        buf = io.BytesIO()
        sf.write(buf, np.zeros(2048, dtype="float32"), 16000,
                 format="WAV", subtype="FLOAT")
        audio_cmd = plugin.commands["Audio"]
        plugin.send_command(audio_cmd, audio_cmd.encoder(buf.getvalue()))
        data, _ = rx.recvfrom(8192)
        assert len(data) == 1024 * 4  # first raw F32LE mono block
    finally:
        host.close()
        rx.close()


def test_heartbeat_is_sent(mock_lite3):
    """The plugin sends the 4 Hz Lite3 heartbeat while active."""
    robot, command_port, telemetry_port = mock_lite3
    plugin = _Lite3PluginForTest(
        command_port=command_port, telemetry_port=telemetry_port
    )
    host = RobotPluginHost(plugin, node=None, bus=InProcessFeedbackBus())

    heartbeats = {"count": 0}
    original = robot._handle_inbound

    def _capture(data: bytes):
        if len(data) == ctypes.sizeof(protocol.SimpleCMD):
            if protocol.SimpleCMD.from_buffer_copy(data).cmd_code == (
                CommandCode.HEARTBEAT
            ):
                heartbeats["count"] += 1
        original(data)

    robot._handle_inbound = _capture
    host.open()
    try:
        deadline = time.time() + 1.6
        while heartbeats["count"] < 3 and time.time() < deadline:
            time.sleep(0.05)
        assert heartbeats["count"] >= 3
    finally:
        host.close()


def test_ultrasound_mounts_place_the_beams_on_the_body():
    """The plugin declares where its ultrasounds sit, so the launcher can
    publish body -> ultrasound_* and consumers can tell which way each beam
    faces: the front one along +x, the rear one turned by pi."""
    import math

    plugin = _Lite3PluginForTest(command_port=_free_port(), telemetry_port=_free_port())
    mounts = {m.child_frame: m for m in plugin.mounts}
    assert {"ultrasound_front", "ultrasound_back"} <= set(mounts)
    assert all(m.parent_frame == "body" for m in mounts.values())
    assert mounts["ultrasound_front"].xyz[0] > 0 and mounts["ultrasound_front"].rpy[2] == 0.0
    assert mounts["ultrasound_back"].xyz[0] < 0
    assert math.isclose(abs(mounts["ultrasound_back"].rpy[2]), math.pi)
    # The frames are the ones the Range messages name
    assert "ultrasound_front" in plugin.feedbacks and "ultrasound_back" in plugin.feedbacks


def test_lidar_mount_places_the_cloud_on_the_body():
    """The Mid-360 cloud is published in LIDAR_FRAME with zero driver
    extrinsics, so its pose on the body must come from a mount.

    The numbers are DeepRobotics' own, from the ``extrinsic_parameter`` in the
    ``MID360_config.json`` of their LiDAR-edition image; the position also
    matches the Motion Development Manual's sensor table (section 1.1.3).
    """
    import math

    plugin = _Lite3PluginForTest(command_port=_free_port(), telemetry_port=_free_port())
    mounts = {m.child_frame: m for m in plugin.mounts}
    lidar = mounts[plugin.lidar.frame]
    assert lidar.parent_frame == "body"
    assert tuple(lidar.xyz) == plugin.lidar.mount[0]
    assert tuple(lidar.rpy) == plugin.lidar.mount[1]
    # 187 mm forward, 129 mm up, pitched 15 degrees nose-down
    assert tuple(lidar.xyz) == pytest.approx((0.187, 0.0, 0.129))
    assert math.degrees(lidar.rpy[1]) == pytest.approx(15.0)
    assert (lidar.rpy[0], lidar.rpy[2]) == (0.0, 0.0)


def test_no_lidar_mount_without_a_lidar():
    class NoLidar(_Lite3PluginForTest):
        HAS_LIDAR = False

    plugin = NoLidar(command_port=_free_port(), telemetry_port=_free_port())
    assert plugin.lidar.frame not in {m.child_frame for m in plugin.mounts}


def test_packaged_lidar_config_carries_the_lite3_addresses():
    """The Mid-360 streams to the Jetson; the driver adds no extrinsics, so the
    cloud and the Mid-360 IMU share one frame for mapping."""
    import json

    with open(Lite3Plugin.LIDARS["livox"].config) as f:
        config = json.load(f)
    host = config["MID360"]["host_net_info"]
    assert {host[k] for k in ("cmd_data_ip", "push_msg_ip", "point_data_ip", "imu_data_ip")} == {
        "192.168.1.103"
    }
    (lidar,) = config["lidar_configs"]
    assert lidar["ip"] == "192.168.1.201"
    assert not any(lidar["extrinsic_parameter"].values())


def _with_robosense_ports(*ports: int) -> dict:
    """The plugin's LiDARs, with the RoboSense listening on scratch ports.

    Detection binds the real MSOP port, which a test may neither assume free
    nor squat on, so only that field is evolved and the rest of the spec is
    the plugin's own.
    """
    return {
        **Lite3Plugin.LIDARS,
        "robosense": attrs.evolve(Lite3Plugin.LIDARS["robosense"], host_ports=ports),
    }


@pytest.fixture(autouse=True)
def _pin_lidar_kind(monkeypatch):
    """Keep detection out of tests that are not about it.

    Detection asks the network of whatever machine runs the suite, which has no
    Lite3 on it. The tests below that *are* about detection opt back in by
    declaring their own kind.
    """
    monkeypatch.setattr(Lite3Plugin, "LIDAR_KIND", "livox")


class _AutoDetectLite3(_Lite3PluginForTest):
    """A unit whose LiDAR is worked out on startup, as a shipped plugin does."""

    LIDAR_KIND = "auto"
    LIDAR_DETECT_SECONDS = 0.5
    # No Livox address to fall back on, so detection rests on the listen alone
    LIDARS = {**Lite3Plugin.LIDARS,
              "livox": attrs.evolve(Lite3Plugin.LIDARS["livox"], config=None)}


def _stream_robosense_packets(port: int, stop: threading.Event) -> None:
    """A RoboSense's own behaviour: point packets, unasked, until stopped."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    packet = b"\x55\xaa\x05\x0a" + bytes(1244)
    while not stop.is_set():
        sock.sendto(packet, ("127.0.0.1", port))
        time.sleep(0.005)
    sock.close()


def test_detection_finds_the_robosense_that_is_streaming():
    """The one question the hardware answers by itself: a RoboSense streams
    point packets as soon as it has power, and nothing else does."""
    port = _free_port()
    stop = threading.Event()
    sender = threading.Thread(
        target=_stream_robosense_packets, args=(port, stop), daemon=True
    )
    sender.start()
    try:

        class Detected(_AutoDetectLite3):
            LIDARS = _with_robosense_ports(port, _free_port())

        plugin = Detected(command_port=_free_port(), telemetry_port=_free_port())
    finally:
        stop.set()
    assert plugin.LIDAR_KIND == "robosense"
    assert plugin.lidar.topic == "/rslidar_points"
    assert "lidar_imu" not in plugin.feedbacks


def test_a_held_robosense_port_means_its_driver_is_already_running():
    """Not being able to listen is itself the answer: on this robot only a
    RoboSense driver binds that port."""
    port = _free_port()
    holder = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    holder.bind(("", port))
    try:

        class Detected(_AutoDetectLite3):
            LIDARS = _with_robosense_ports(port, _free_port())

        plugin = Detected(command_port=_free_port(), telemetry_port=_free_port())
    finally:
        holder.close()
    assert plugin.LIDAR_KIND == "robosense"


def test_detection_settles_on_the_mid360_when_nothing_streams():
    """A Mid-360 says nothing until a host configures it, so silence plus no
    other maker's hardware is what a Livox unit looks like."""

    class Detected(_AutoDetectLite3):
        LIDARS = _with_robosense_ports(_free_port(), _free_port())

    plugin = Detected(command_port=_free_port(), telemetry_port=_free_port())
    assert plugin.LIDAR_KIND == "livox"
    assert plugin.lidar.topic == "/livox/lidar"
    assert "lidar_imu" in plugin.feedbacks


def test_detection_is_not_run_on_a_unit_with_no_lidar():
    """Nothing to detect, and no reason to make every startup wait for it."""

    class NoLidar(_AutoDetectLite3):
        HAS_LIDAR = False
        LIDARS = _with_robosense_ports(_free_port(), _free_port())

    started = time.monotonic()
    plugin = NoLidar(command_port=_free_port(), telemetry_port=_free_port())
    assert plugin.LIDAR_KIND == "livox"
    assert time.monotonic() - started < NoLidar.LIDAR_DETECT_SECONDS


class _AutoDetectUnit(Lite3Plugin):
    """A shipped plugin as a recipe gets one: which LiDAR is fitted is worked
    out on startup. Declared here rather than inside a test, so a spec can name
    it the way the launcher does."""

    LIDAR_KIND = "auto"


def _never_probed(*_):
    raise AssertionError("the robot was asked which LiDAR it carries")


def test_the_detected_lidar_rides_in_the_spec(monkeypatch):
    """Detection asks the robot's network, and every component subprocess
    rebuilds this plugin from its spec. Without the answer in the spec each of
    them would probe again, pay for it at startup, and could settle on a
    different LiDAR than the launcher did."""
    probes = []

    def _streaming(port, _timeout):
        probes.append(port)
        return True

    monkeypatch.setattr(_plugin_module, "_robosense_is_streaming", _streaming)

    plugin = _AutoDetectUnit()
    assert plugin.LIDAR_KIND == "robosense"
    assert len(probes) == 1

    spec = plugin.to_spec()
    assert spec["kwargs"] == {"lidar_kind": "robosense"}

    # What a component subprocess does with that spec
    monkeypatch.setattr(_plugin_module, "_robosense_is_streaming", _never_probed)
    rebuilt = RobotPlugin.from_spec(spec)
    assert rebuilt.LIDAR_KIND == "robosense"
    assert rebuilt.lidar.topic == plugin.lidar.topic
    assert "lidar_imu" not in rebuilt.feedbacks


def test_a_recipe_can_name_the_lidar_instead_of_detecting_it(monkeypatch):
    """Pinning it without subclassing, for a unit that is known."""
    monkeypatch.setattr(_plugin_module, "_robosense_is_streaming", _never_probed)

    plugin = _AutoDetectUnit(lidar_kind="robosense")

    assert plugin.LIDAR_KIND == "robosense"
    assert plugin.lidar.topic == "/rslidar_points"
    assert plugin.to_spec()["kwargs"] == {"lidar_kind": "robosense"}


def test_a_subclass_with_its_own_constructor_keeps_its_spec(monkeypatch):
    """The kind is recorded only where the constructor takes it: rebuilding
    this one with a 'lidar_kind' it does not accept would raise."""
    plugin = _Lite3PluginForTest(
        command_port=_free_port(), telemetry_port=_free_port()
    )

    spec = plugin.to_spec()
    assert "lidar_kind" not in spec["kwargs"]
    assert RobotPlugin.from_spec(spec).LIDAR_KIND == "livox"


class _RoboSenseLite3(_Lite3PluginForTest):
    """A unit fitted with the RoboSense instead of the Mid-360."""

    LIDAR_KIND = "robosense"


def test_robosense_unit_uses_its_own_driver_topic_and_geometry():
    """DeepRobotics fits either LiDAR at the same address, so the plugin has to
    be told which, and everything downstream follows from that one attribute.

    The numbers are the vendor's own, from the extrinsic_T their faster_lio
    configuration carries on these units.
    """
    plugin = _RoboSenseLite3(command_port=_free_port(), telemetry_port=_free_port())
    assert plugin.lidar.topic == "/rslidar_points"
    assert plugin.lidar.frame == "rslidar"
    assert plugin.lidar.driver_package == "rslidar_sdk"
    assert plugin.lidar.host_ports == (6699, 7788)
    mounts = {m.child_frame: m for m in plugin.mounts}
    assert tuple(mounts["rslidar"].xyz) == pytest.approx((0.12815, 0.0, 0.10596))
    assert tuple(mounts["rslidar"].rpy) == (0.0, 0.0, 0.0)
    # The Mid-360's frame belongs to a Mid-360 unit, and never both at once
    assert "livox_frame" not in mounts


def test_robosense_unit_serves_no_lidar_imu():
    """A RoboSense has no IMU, so that feedback is absent rather than silent.
    Mapping then fuses the body IMU instead, which is what the vendor's own
    faster-LIO does on these units."""
    plugin = _RoboSenseLite3(command_port=_free_port(), telemetry_port=_free_port())
    assert "lidar" in plugin.feedbacks
    assert "lidar_imu" not in plugin.feedbacks
    assert plugin.lidar.feedbacks == frozenset({"lidar"})
    assert plugin.MAPPING.cloud == "lidar"
    assert plugin.MAPPING.imu == "Imu"
    # A unit that does have one fuses that. The class attribute carries
    # neither: which IMU to fuse is a property of the LiDAR that is fitted.
    mid360 = _Lite3PluginForTest(command_port=_free_port(), telemetry_port=_free_port())
    assert mid360.MAPPING.imu == "lidar_imu"
    assert mid360.MAPPING.imu_xyz == pytest.approx((0.011, 0.02329, -0.04412))
    assert Lite3Plugin.MAPPING.imu is None


def test_robosense_driver_is_left_to_the_robot_when_not_installed():
    """rslidar_sdk is not a released ROS package, so an EMOS environment
    usually lacks it while the robot always has it. That must not abort the
    launch: the plugin reads the cloud the robot's own driver publishes."""
    plugin = _RoboSenseLite3(command_port=_free_port(), telemetry_port=_free_port())
    plugin._set_requested(frozenset({"lidar"}), frozenset())
    packages = [p.package for p in plugin.required_processes()]
    if _plugin_module._package_available("rslidar_sdk"):
        assert packages == ["rslidar_sdk"]
    else:
        assert packages == []


def test_an_unknown_lidar_kind_is_refused():
    class Mystery(_Lite3PluginForTest):
        LIDAR_KIND = "hokuyo"

    with pytest.raises(ValueError, match="LIDAR_KIND"):
        Mystery(command_port=_free_port(), telemetry_port=_free_port())


def test_a_lidar_address_is_identified_by_its_hardware_vendor():
    """The wrong driver on the right address is silent, so the plugin says so
    first. 40:2c:76:8* is Suteng Innovation (RoboSense) in the IEEE registry,
    and a Lite3 carries either that or a Mid-360, which is in no registry."""
    assert _plugin_module._is_robosense_mac("40:2c:76:82:1a:23")
    assert _plugin_module._is_robosense_mac("40-2C-76-82-1A-23")
    # A neighbouring MA-M assignment is a different company, not a RoboSense
    assert not _plugin_module._is_robosense_mac("40:2c:76:12:34:56")
    # The Jetson's own NIC, which shares the subnet with the LiDAR
    assert not _plugin_module._is_robosense_mac("3c:6d:66:4d:b1:2f")


def test_native_mapping_names_sensors_this_plugin_serves():
    """EMOS builds the map itself, so its session resolves the declaration
    against this plugin: the two feedback keys, the LiDAR frame's mount (which
    tells the grid builder where the ground is), and the Mid-360's IMU offset,
    which nothing else in the system carries.
    """
    plugin = _Lite3PluginForTest(command_port=_free_port(), telemetry_port=_free_port())
    mapping = plugin.MAPPING
    assert mapping.kind == "native"
    # Named by feedback key, and both are served by the one Livox driver
    assert {mapping.cloud, mapping.imu} <= set(plugin.feedbacks)
    assert {mapping.cloud, mapping.imu} == set(plugin.lidar.feedbacks)
    # The mapping session builds its input topics from these type names
    assert plugin.feedbacks[mapping.cloud].msg_type.__name__ == "PointCloud2"
    assert plugin.feedbacks[mapping.imu].msg_type.__name__ == "Imu"
    # The Mid-360's IMU, 11.0 / 23.29 / -44.12 mm from the LiDAR's origin
    assert mapping.imu_xyz == pytest.approx((0.011, 0.02329, -0.04412))
    assert mapping.imu_rpy == (0.0, 0.0, 0.0)
    # Where the ground is: the LiDAR's mount above the body, plus the body
    # above the ground. Both have to be there for the grid to be flattened.
    mounts = {m.child_frame: m for m in plugin.mounts}
    assert mounts[plugin.lidar.frame].xyz[2] > 0
    assert plugin.base_height and plugin.base_height > 0
    # Obstacles are taken from a band no taller than the robot
    assert 0 < mapping.z_min < mapping.z_max <= plugin.robot_config.height


def test_a_lidar_without_an_imu_maps_with_the_body_imu():
    """A RoboSense carries no IMU, and mapping without one falls back to
    LiDAR-only odometry, which a gait pitching the sensor makes hard. The robot
    has an IMU of its own, and the vendor's faster-LIO maps these units with
    exactly it."""
    plugin = _RoboSenseLite3(command_port=_free_port(), telemetry_port=_free_port())
    mapping = plugin.MAPPING

    assert mapping.imu == "Imu"
    # The session resolves it against the plugin, so it has to be served
    assert plugin.feedbacks[mapping.imu].msg_type.__name__ == "Imu"
    # The vendor's own extrinsic_T places the LiDAR in the IMU frame; mapping
    # wants the other direction, so these are its negatives
    assert mapping.imu_xyz == pytest.approx((-0.12815, 0.0, -0.10596))
    assert mapping.imu_rpy == pytest.approx((0.0, 0.0, 0.0))
    assert mapping.imu_xyz == pytest.approx(
        tuple(-v for v in plugin.lidar.mount[0])
    ), "the body IMU sits at the body origin, so this is the LiDAR mount inverted"


def test_the_mid360_keeps_its_own_imu():
    """A LiDAR with an IMU inside it has the one worth fusing: it shares the
    sensor's clock and rate, and the body IMU is neither."""
    plugin = _Lite3PluginForTest(command_port=_free_port(), telemetry_port=_free_port())

    assert plugin.MAPPING.imu == "lidar_imu"
    assert plugin.MAPPING.imu_xyz == pytest.approx((0.011, 0.02329, -0.04412))


def test_mapping_with_the_body_imu_can_be_turned_off():
    """For a unit where it turns out not to help: the backend then maps from
    the cloud alone."""

    class LidarOnly(_RoboSenseLite3):
        MAP_WITH_BODY_IMU = False

    mapping = LidarOnly(
        command_port=_free_port(), telemetry_port=_free_port()
    ).MAPPING

    assert mapping.imu is None
    assert mapping.imu_xyz is None


def test_the_body_imu_extrinsic_follows_a_rotated_mount():
    """The transform is composed rather than negated, so a LiDAR mounted at an
    angle -- as the Mid-360 is -- would still be placed correctly."""
    mount = ((0.1, 0.0, 0.2), (0.0, np.pi / 2, 0.0))

    class Tilted(_RoboSenseLite3):
        LIDARS = {
            **Lite3Plugin.LIDARS,
            "robosense": attrs.evolve(Lite3Plugin.LIDARS["robosense"], mount=mount),
        }

    plugin = Tilted(command_port=_free_port(), telemetry_port=_free_port())

    # The body origin is 0.2 m ahead of a LiDAR pitched a quarter turn, and
    # 0.1 m along what is now its own downward axis
    assert plugin.MAPPING.imu_xyz == pytest.approx((0.2, 0.0, -0.1))
    assert plugin.MAPPING.imu_rpy == pytest.approx((0.0, -np.pi / 2, 0.0))


def _capture_simple_cmds(robot) -> list:
    """Record the code of every SimpleCMD the mock robot receives."""
    seen = []
    original = robot._handle_inbound

    def _capture(data):
        if len(data) == ctypes.sizeof(protocol.SimpleCMD):
            seen.append(protocol.SimpleCMD.from_buffer_copy(data).cmd_code)
        original(data)

    robot._handle_inbound = _capture
    return seen


class _HostNode:
    """The host's ROS node as the plugin sees it: without a context until it is
    initialized, then with one that is up until the recipe shuts ROS down."""

    class _Context:
        def __init__(self):
            self.up = True

        def ok(self):
            return self.up

    def initialize(self):
        self.context = self._Context()


def test_closing_the_host_returns_the_robot_to_manual(mock_lite3):
    """A recipe that ends cleanly must hand the handset back.

    The launcher tears every plugin host down after the launch service exits,
    and ``on_detached`` runs before the transports close, so this is what the
    robot hears last.
    """
    robot, command_port, telemetry_port = mock_lite3
    plugin = _Lite3PluginForTest(
        command_port=command_port, telemetry_port=telemetry_port
    )
    seen = _capture_simple_cmds(robot)
    host = RobotPluginHost(plugin, node=None, bus=InProcessFeedbackBus())
    host.open()
    seen.clear()
    host.close()
    time.sleep(0.3)
    assert CommandCode.CONTROL_MANUAL in seen, (
        f"expected CONTROL_MANUAL on teardown, saw {[hex(c) for c in seen]}"
    )


def test_the_robot_returns_to_manual_as_soon_as_the_recipe_shuts_down(mock_lite3):
    """Ctrl+C, or EMOS stopping a recipe, shuts ROS down at once, but the launch
    can still be tearing down when EMOS kills it, and then ``on_detached`` never
    runs. The heartbeat notices ROS going down and hands the handset back then.
    """
    robot, command_port, telemetry_port = mock_lite3
    plugin = _Lite3PluginForTest(
        command_port=command_port, telemetry_port=telemetry_port
    )
    seen = _capture_simple_cmds(robot)
    node = _HostNode()
    host = RobotPluginHost(plugin, node=node, bus=InProcessFeedbackBus())
    host.open()
    try:
        # Bringup: the node is not initialized yet, then it is up
        time.sleep(0.6)
        node.initialize()
        time.sleep(0.6)
        assert CommandCode.CONTROL_MANUAL not in seen, "not while the recipe runs"

        node.context.up = False
        deadline = time.time() + 2.0
        while CommandCode.CONTROL_MANUAL not in seen and time.time() < deadline:
            time.sleep(0.05)
        assert CommandCode.CONTROL_MANUAL in seen, (
            f"expected CONTROL_MANUAL once ROS shut down, saw {[hex(c) for c in seen]}"
        )
    finally:
        host.close()
    time.sleep(0.3)
    assert seen.count(CommandCode.CONTROL_MANUAL) == 1, "sent once, not again on teardown"


def test_a_host_node_not_yet_up_is_not_taken_for_a_shutdown(mock_lite3):
    """Before the host's node is initialized it has no context; that is bringup,
    and must not hand the robot back."""
    robot, command_port, telemetry_port = mock_lite3
    plugin = _Lite3PluginForTest(
        command_port=command_port, telemetry_port=telemetry_port
    )
    seen = _capture_simple_cmds(robot)
    host = RobotPluginHost(plugin, node=_HostNode(), bus=InProcessFeedbackBus())
    host.open()
    time.sleep(0.8)
    assert CommandCode.CONTROL_MANUAL not in seen
    host.close()


def _packages(plugin, keys):
    plugin._set_requested(frozenset(keys), frozenset())
    return {spec.package for spec in plugin.required_processes()}


def test_camera_driver_starts_for_any_camera_feedback():
    """Binding any RealSense stream is reason to start the driver -- a recipe
    that wants only camera_info still needs the node running."""
    plugin = Lite3Plugin()
    for key in ("camera", "camera_info", "rgbd"):
        assert plugin.CAMERA_DRIVER_PACKAGE in _packages(plugin, {key}), (
            f"binding '{key}' should start the camera driver"
        )


def test_lidar_driver_starts_for_the_imu_alone():
    """The Mid-360's cloud and IMU come from one driver process, so binding
    either has to start it. Gating on the cloud alone leaves a recipe that
    wants only the IMU with a silent topic and no clue why."""
    plugin = Lite3Plugin()
    assert plugin.lidar.driver_package in _packages(plugin, {"lidar_imu"})
    assert plugin.lidar.driver_package in _packages(plugin, {"lidar"})


def test_no_drivers_when_nothing_is_bound():
    plugin = Lite3Plugin()
    assert _packages(plugin, set()) == set()


@pytest.mark.skipif(_rgbd_type() is None, reason="realsense2_camera_msgs not built")
def test_rgbd_uses_the_consumer_packages_type():
    """A plugin wraps only messages custom to its robot. RGBD belongs to
    embodied-agents, and a second wrapper of it is dropped by the type registry,
    so a Topic built from the plugin's feedback failed validation in any
    component that consumed it."""
    from agents.ros import RGBD
    from ros_sugar.io.topic import Topic

    plugin = Lite3Plugin()
    assert plugin.feedbacks["rgbd"].msg_type is RGBD
    Topic(name="rgbd", msg_type=plugin.feedbacks["rgbd"].msg_type)


@pytest.mark.skipif(_rgbd_type() is None, reason="realsense2_camera_msgs not built")
def test_rgbd_without_embodied_agents_says_what_is_missing(monkeypatch):
    """A bare 'No module named agents' does not tell anyone that a camera
    plugin needs embodied-agents, or why."""
    monkeypatch.setitem(sys.modules, "agents", None)
    monkeypatch.setitem(sys.modules, "agents.ros", None)
    with pytest.raises(ImportError, match="embodied-agents"):
        _rgbd_type()


# ---------------------------------------------------------------------------
# Filtered odometry (robot_localization EKF)
# ---------------------------------------------------------------------------
def test_ekf_is_started_only_for_odometry_filtered():
    """Binding the estimate is what starts the EKF; nothing else does."""
    plugin = Lite3Plugin()

    plugin._set_requested(frozenset({"Odometry", "Imu"}), frozenset())
    assert plugin.required_processes() == []

    plugin._set_requested(frozenset({"odometry_filtered"}), frozenset())
    (ekf,) = plugin.required_processes()
    assert (ekf.package, ekf.executable) == ("robot_localization", "ekf_node")
    assert ekf.parameters[0] == plugin.EKF_CONFIG


def test_ekf_reads_the_topics_the_host_publishes_the_feedbacks_on():
    """The EKF's inputs and the parameters naming its input topics come from the
    same attributes, so the host publishes exactly where the EKF subscribes."""
    plugin = Lite3Plugin()
    plugin._set_requested(frozenset({"odometry_filtered"}), frozenset())
    (ekf,) = plugin.required_processes()

    overrides = ekf.parameters[1]
    assert ekf.inputs == {"Odometry": overrides["odom0"], "Imu": overrides["imu0"]}
    # The frames the decoders stamp and the plugin's base frame
    assert overrides["odom_frame"] == _decode_odometry_frames()[0]
    assert overrides["base_link_frame"] == plugin.base_frame == "body"
    assert overrides["map_frame"] == overrides["world_frame"] == plugin.EKF_WORLD_FRAME
    # Its output lands where the feedback subscribes, whatever the namespace
    assert ("odometry/filtered", plugin.EKF_OUTPUT_TOPIC) in ekf.remappings
    assert plugin.feedbacks["odometry_filtered"].transport.topic_name == (
        plugin.EKF_OUTPUT_TOPIC
    )


def _decode_odometry_frames():
    """(frame_id, child_frame_id) the odometry decoder stamps."""
    from lite3_plugin.plugin import _decode_odometry

    frame = protocol.RobotStateReceived()
    frame.code = protocol.ROBOT_STATE_CODE
    msg = _decode_odometry(bytes(frame))
    return msg.header.frame_id, msg.child_frame_id


def test_ekf_inputs_resolve_to_host_publications(monkeypatch):
    """Through the real launcher: both inputs are decoded from UDP, so the host
    publishes them rather than the EKF being remapped."""
    from ros_sugar import Launcher

    plugin = Lite3Plugin()
    launcher = Launcher(robot_plugin=plugin)
    plugin._set_requested(frozenset({"odometry_filtered"}), frozenset())
    added = []
    monkeypatch.setattr(launcher, "add_ros_node", lambda **kw: added.append(kw))

    published = launcher._launch_plugin_processes(plugin)

    assert sorted(published) == [("Imu", "/imu/data"), ("Odometry", "/odom")]
    (ekf,) = added
    assert "inputs" not in ekf


def test_odometry_owns_odom_to_body_on_tf():
    """The EKF publishes map -> odom and leaves odom -> body to the odometry."""
    plugin = Lite3Plugin()
    assert plugin.feedbacks["Odometry"].publish_tf
    assert _decode_odometry_frames() == ("odom", plugin.base_frame)
    assert not plugin.feedbacks["Imu"].publish_tf


def test_both_imu_packets_decode_to_the_same_message():
    """The Lite3 carries the IMU in two packets; both hold the same fields in
    the same units, so the faster one is a drop-in for the slower."""
    imu_frame = protocol.ImuReceived()
    imu_frame.code = protocol.IMU_CODE
    state_frame = protocol.RobotStateReceived()
    state_frame.code = protocol.ROBOT_STATE_CODE
    for frame in (imu_frame, state_frame):
        frame.data.rpy[2] = 90.0
        frame.data.rpy_vel[0] = 0.25
        frame.data.xyz_acc[2] = 9.81

    for msg in (_ImuDecoder()(bytes(imu_frame)), _ImuDecoder()(bytes(state_frame))):
        assert msg is not None
        assert msg.angular_velocity.x == pytest.approx(0.25)
        assert msg.linear_acceleration.z == pytest.approx(9.81)
        # Angles are in degrees in both packets.
        assert (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ) == pytest.approx(codecs.quaternion_from_rpy_degrees(0.0, 0.0, 90.0))


def test_only_one_imu_source_is_published_at_a_time():
    """Publishing both packets would report every sample twice. The dedicated
    stream wins while it is alive, and the RobotState copy covers a robot whose
    firmware does not send it -- it is undocumented."""
    now = [1000.0]
    decoder = _ImuDecoder(clock=lambda: now[0])
    imu_frame = protocol.ImuReceived()
    imu_frame.code = protocol.IMU_CODE
    state_frame = protocol.RobotStateReceived()
    state_frame.code = protocol.ROBOT_STATE_CODE

    # Nothing seen yet, so the copy carries the feedback.
    assert decoder(bytes(state_frame)) is not None
    # Once the dedicated stream arrives it takes over and the copy is dropped.
    assert decoder(bytes(imu_frame)) is not None
    assert decoder(bytes(state_frame)) is None
    # If that stream goes quiet, the copy carries it again.
    now[0] += _ImuDecoder.STALE_AFTER
    assert decoder(bytes(state_frame)) is not None
    # Anything else on the telemetry port is ignored by both paths.
    assert decoder(codecs.encode_simple_cmd(1)) is None


def test_the_imu_feedback_reads_the_dedicated_stream():
    """The Imu feedback is wired to the fast decoder, and declares its rate."""
    plugin = Lite3Plugin()
    feedback = plugin.feedbacks["Imu"]
    assert feedback.rate_hz == 200.0
    frame = protocol.ImuReceived()
    frame.code = protocol.IMU_CODE
    frame.data.xyz_acc[2] = 9.81
    msg = feedback.decoder(bytes(frame))
    assert msg is not None
    assert msg.linear_acceleration.z == pytest.approx(9.81)


def test_body_imu_is_mounted_so_the_ekf_can_use_it():
    """robot_localization drops every IMU sample it cannot transform into the
    body frame, and the IMU's messages name their own frame."""
    plugin = Lite3Plugin()
    frame = protocol.RobotStateReceived()
    frame.code = protocol.ROBOT_STATE_CODE
    imu_frame = plugin.feedbacks["Imu"].decoder(bytes(frame)).header.frame_id

    mounts = {m.child_frame: m for m in plugin.mounts}
    assert imu_frame in mounts
    assert mounts[imu_frame].parent_frame == plugin.base_frame


def test_no_ekf_without_a_config():
    class NoEkfConfig(Lite3Plugin):
        EKF_CONFIG = None

    plugin = NoEkfConfig()
    plugin._set_requested(frozenset({"odometry_filtered"}), frozenset())
    assert plugin.required_processes() == []


def test_packaged_ekf_config_applies_to_any_node_name():
    """Keyed by the wildcard, so the parameters are not silently ignored when
    the node is renamed."""
    import yaml

    plugin = Lite3Plugin()
    with open(plugin.EKF_CONFIG) as f:
        config = yaml.safe_load(f)
    params = config["/**"]["ros__parameters"]
    assert params["publish_tf"] is True, "the EKF owns map -> odom"
    assert params["two_d_mode"] is True


