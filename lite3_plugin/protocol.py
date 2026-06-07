"""Binary protocol for the DeepRobotics Lite3 Motion Host UDP interface.

This mirrors, field for field, the C structs in the DeepRobotics
``message_transformer`` package (``include/protocol.h``, ``#pragma pack(4)``)
and the command-code set documented in the *Jueying Lite3 Motion Host
Communication Interface*. Using :mod:`ctypes` with ``_pack_ = 4`` reproduces the
exact on-the-wire C layout, so :class:`~ctypes.Structure` sizes can be used for
packet-type dispatch just like the C++ bridge does (``switch(recv_num_)``).

Two UDP endpoints:

* **Commands** → robot ``:43893`` — ``SimpleCMD`` (12 bytes) / ``ComplexCMD``
  (20 bytes).
* **Telemetry** ← bind ``:43897`` — ``RobotStateReceived`` (code 2305),
  ``JointStateReceived`` (2306), ``HandleStateReceived`` (2309).
"""

import ctypes

# Default network endpoints (DeepRobotics factory defaults).
DEFAULT_ROBOT_IP = "192.168.1.120"
DEFAULT_COMMAND_PORT = 43893
DEFAULT_TELEMETRY_PORT = 43897


# --------------------------------------------------------------------------
# Command structs (host -> robot)
# --------------------------------------------------------------------------
class SimpleCMD(ctypes.Structure):
    """12-byte command: ``cmd_code``, ``cmd_value``, ``type`` (3x int32)."""

    _pack_ = 4
    _fields_ = [
        ("cmd_code", ctypes.c_int32),
        ("cmd_value", ctypes.c_int32),
        ("type", ctypes.c_int32),
    ]


class ComplexCMD(ctypes.Structure):
    """20-byte command: a :class:`SimpleCMD` plus an 8-byte ``data`` double."""

    _pack_ = 4
    _fields_ = [
        ("cmd_code", ctypes.c_int32),
        ("cmd_value", ctypes.c_int32),
        ("type", ctypes.c_int32),
        ("data", ctypes.c_double),
    ]


# --------------------------------------------------------------------------
# Telemetry structs (robot -> host)
# --------------------------------------------------------------------------
class RobotState(ctypes.Structure):
    """Robot base state — pose, IMU, velocities, battery, ultrasound.

    Field order/layout matches the doc's ``RobotStateUpload`` struct exactly.
    Four fields are documented as **invalid placeholders** (``touch_down_and_
    stair_trot``, ``is_charging``, ``error_state``, ``task_state``) — they are
    present only to keep the wire layout right and must not be relied upon.
    """

    _pack_ = 4
    _fields_ = [
        ("robot_basic_state", ctypes.c_int),   # see codecs.BASIC_STATE_NAMES
        ("robot_gait_state", ctypes.c_int),    # see codecs.GAIT_NAMES
        ("rpy", ctypes.c_double * 3),          # IMU angle (degrees)
        ("rpy_vel", ctypes.c_double * 3),      # IMU angular velocity (rad/s)
        ("xyz_acc", ctypes.c_double * 3),      # IMU acceleration (m/s^2)
        ("pos_world", ctypes.c_double * 3),    # world frame {x, y, yaw(rad)}
        ("vel_world", ctypes.c_double * 3),    # world frame {x_vel, y_vel, yaw_vel}
        ("vel_body", ctypes.c_double * 3),     # body frame {x_vel, y_vel, yaw_vel}
        ("touch_down_and_stair_trot", ctypes.c_uint),  # INVALID placeholder
        ("is_charging", ctypes.c_bool),        # INVALID placeholder (not real charge state)
        ("error_state", ctypes.c_uint),        # INVALID placeholder (not a fault code)
        ("robot_motion_state", ctypes.c_int),  # see codecs.MOTION_STATE_NAMES
        ("battery_level", ctypes.c_double),    # battery percentage
        ("task_state", ctypes.c_int),          # INVALID placeholder
        ("is_robot_need_move", ctypes.c_bool), # 1: lost balance, must step to recover
        ("zero_position_flag", ctypes.c_bool), # 1: reset-to-zero completed
        ("ultrasound", ctypes.c_double * 2),   # {front, back} obstacle dist (m), [0.28, 4.50]
    ]


class RobotStateWithPolicy(ctypes.Structure):
    """Lite3 ``RobotState`` layout on newer ``Lite3 transfer package``
    Adds ``robot_policy_state`` int after ``robot_gait_state``, making the
    frame 4 bytes larger.

    Plugin auto-detects which one a packet is by its length. The extra field
    reports the robot's active control policy and is not otherwise consumed.
    """

    _pack_ = 4
    _fields_ = (
        list(RobotState._fields_[:2])
        + [("robot_policy_state", ctypes.c_int)]
        + list(RobotState._fields_[2:])
    )


class RobotStateReceived(ctypes.Structure):
    """Framed :class:`RobotState` packet — ``code`` is 2305."""

    _pack_ = 4
    _fields_ = [
        ("code", ctypes.c_int),
        ("size", ctypes.c_int),
        ("cons_code", ctypes.c_int),
        ("data", RobotState),
    ]


class RobotStateReceivedWithPolicy(ctypes.Structure):
    """Framed :class:`RobotStateWithPolicy` packet — ``code`` is 2305 (newer layout)."""

    _pack_ = 4
    _fields_ = [
        ("code", ctypes.c_int),
        ("size", ctypes.c_int),
        ("cons_code", ctypes.c_int),
        ("data", RobotStateWithPolicy),
    ]


class JointState(ctypes.Structure):
    """The 12 leg-joint angles (LF/RF/LB/RB x 3)."""

    _pack_ = 4
    _fields_ = [
        ("LF_Joint", ctypes.c_double),
        ("LF_Joint_1", ctypes.c_double),
        ("LF_Joint_2", ctypes.c_double),
        ("RF_Joint", ctypes.c_double),
        ("RF_Joint_1", ctypes.c_double),
        ("RF_Joint_2", ctypes.c_double),
        ("LB_Joint", ctypes.c_double),
        ("LB_Joint_1", ctypes.c_double),
        ("LB_Joint_2", ctypes.c_double),
        ("RB_Joint", ctypes.c_double),
        ("RB_Joint_1", ctypes.c_double),
        ("RB_Joint_2", ctypes.c_double),
    ]


class JointStateReceived(ctypes.Structure):
    """Framed :class:`JointState` packet — ``code`` is 2306."""

    _pack_ = 4
    _fields_ = [
        ("code", ctypes.c_int),
        ("size", ctypes.c_int),
        ("cons_code", ctypes.c_int),
        ("data", JointState),
    ]


class HandleState(ctypes.Structure):
    """Operator joystick / handle state."""

    _pack_ = 4
    _fields_ = [
        ("left_axis_forward", ctypes.c_double),
        ("left_axis_side", ctypes.c_double),
        ("right_axis_yaw", ctypes.c_double),
        ("goal_vel_forward", ctypes.c_double),
        ("goal_vel_side", ctypes.c_double),
        ("goal_vel_yaw", ctypes.c_double),
    ]


class HandleStateReceived(ctypes.Structure):
    """Framed :class:`HandleState` packet — ``code`` is 2309."""

    _pack_ = 4
    _fields_ = [
        ("code", ctypes.c_int),
        ("size", ctypes.c_int),
        ("cons_code", ctypes.c_int),
        ("data", HandleState),
    ]


# Telemetry frame ``code`` values (see the C++ bridge's parse routines).
ROBOT_STATE_CODE = 2305
JOINT_STATE_CODE = 2306
HANDLE_STATE_CODE = 2309

# Packet sizes, used for type dispatch exactly as the C++ bridge does.
ROBOT_STATE_SIZE = ctypes.sizeof(RobotStateReceived)
# Newer RobotState layout — 4 bytes larger; supported alongside the
# default so both firmware revisions work (see codecs.parse_robot_state).
ROBOT_STATE_WITH_POLICY_SIZE = ctypes.sizeof(RobotStateReceivedWithPolicy)
JOINT_STATE_SIZE = ctypes.sizeof(JointStateReceived)
HANDLE_STATE_SIZE = ctypes.sizeof(HandleStateReceived)


class CommandCode:
    """Lite3 Motion Host command codes.

    Velocity is sent as three :class:`ComplexCMD` packets (the small numeric
    codes); everything else is a :class:`SimpleCMD`. Codes are from the
    DeepRobotics ``message_transformer`` README and the *Jueying Lite3 Motion
    Host Communication Interface* document.
    """

    # --- velocity (ComplexCMD; cmd_value 8, type 1, data = velocity) ---
    VEL_FORWARD = 320      # linear x
    VEL_YAW = 321          # angular z (sent negated, per the DeepRobotics bridge)
    VEL_LATERAL = 325      # linear y

    # --- mode selection ---
    MODE_POSE = 0x21010D05         # pose mode (enables action commands)
    MODE_MOVE = 0x21010D06         # move mode (enables movement commands)
    CONTROL_MANUAL = 0x21010C02    # respond to operator/controller
    CONTROL_NAVIGATION = 0x21010C03  # respond to the navigation/perception host

    # --- named actions (pose mode) ---
    SIT_STAND = 0x21010202         # toggle sit / stand
    SAY_HELLO = 0x21010507
    TWIST = 0x21010204
    TWIST_JUMP = 0x2101020D
    MOONWALK = 0x2101030C
    LONG_JUMP = 0x2101050B
    STAND_ZERO = 0x21010C05        # return joints to the zero stand pose

    # --- gaits ---
    GAIT_FLAT_SLOW = 0x21010300
    GAIT_FLAT_MEDIUM = 0x21010307
    GAIT_FLAT_FAST = 0x21010303
    GAIT_FLAT_CRAWL = 0x21010406
    GAIT_RUG_GRIP = 0x21010402
    GAIT_RUG_GENERAL = 0x21010401
    GAIT_RUG_HSTEP = 0x21010407

    # --- misc ---
    SAVE_DATA = 0x21010C01         # save the previous 100 s of data
    KEEP_STEPPING = 0x21010C06     # cmd_value: -1 enable, 2 disable
    HEARTBEAT = 0x21040001         # keep-alive, ~4 Hz


__all__ = [
    "DEFAULT_ROBOT_IP",
    "DEFAULT_COMMAND_PORT",
    "DEFAULT_TELEMETRY_PORT",
    "SimpleCMD",
    "ComplexCMD",
    "RobotState",
    "RobotStateReceived",
    "RobotStateWithPolicy",
    "RobotStateReceivedWithPolicy",
    "JointState",
    "JointStateReceived",
    "HandleState",
    "HandleStateReceived",
    "ROBOT_STATE_CODE",
    "JOINT_STATE_CODE",
    "HANDLE_STATE_CODE",
    "ROBOT_STATE_SIZE",
    "ROBOT_STATE_WITH_POLICY_SIZE",
    "JOINT_STATE_SIZE",
    "HANDLE_STATE_SIZE",
    "CommandCode",
]
