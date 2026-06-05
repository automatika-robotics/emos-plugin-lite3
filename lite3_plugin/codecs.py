"""Encode/decode helpers for the Lite3 UDP protocol.

Commands are built with the :mod:`ctypes` structs from :mod:`.protocol`;
telemetry packets are parsed by matching their length against the known struct
sizes (mirroring the C++ bridge's ``switch(recv_num_)``).
"""

import math
from typing import List, Optional

from . import protocol
from .protocol import CommandCode


# --------------------------------------------------------------------------
# Command encoding (host -> robot)
# --------------------------------------------------------------------------
def encode_simple_cmd(cmd_code: int, cmd_value: int = 0, type_: int = 0) -> bytes:
    """Encode a :class:`~.protocol.SimpleCMD` packet."""
    return bytes(protocol.SimpleCMD(cmd_code, cmd_value, type_))


def encode_complex_cmd(
    cmd_code: int, cmd_value: int, type_: int, data: float
) -> bytes:
    """Encode a :class:`~.protocol.ComplexCMD` packet."""
    return bytes(protocol.ComplexCMD(cmd_code, cmd_value, type_, float(data)))


def encode_velocity(vx: float, vy: float, wz: float) -> List[bytes]:
    """Encode a base velocity as the three ``ComplexCMD`` packets the Lite3
    expects (forward, lateral, yaw).

    The yaw rate is negated to match the DeepRobotics bridge convention.
    """
    return [
        encode_complex_cmd(CommandCode.VEL_FORWARD, 8, 1, vx),
        encode_complex_cmd(CommandCode.VEL_LATERAL, 8, 1, vy),
        encode_complex_cmd(CommandCode.VEL_YAW, 8, 1, -wz),
    ]


def encode_heartbeat() -> bytes:
    """Encode the Lite3 keep-alive packet."""
    return encode_simple_cmd(CommandCode.HEARTBEAT)


# --------------------------------------------------------------------------
# Telemetry decoding (robot -> host)
# --------------------------------------------------------------------------
def parse_robot_state(raw: bytes) -> Optional[protocol.RobotState]:
    """Return the :class:`~.protocol.RobotState` from a telemetry packet, or
    ``None`` if ``raw`` is not a well-formed robot-state frame."""
    if len(raw) != protocol.ROBOT_STATE_SIZE:
        return None
    frame = protocol.RobotStateReceived.from_buffer_copy(raw)
    if frame.code != protocol.ROBOT_STATE_CODE:
        return None
    return frame.data


def parse_joint_state(raw: bytes) -> Optional[protocol.JointState]:
    """Return the :class:`~.protocol.JointState` from a telemetry packet, or
    ``None`` if ``raw`` is not a well-formed joint-state frame."""
    if len(raw) != protocol.JOINT_STATE_SIZE:
        return None
    frame = protocol.JointStateReceived.from_buffer_copy(raw)
    if frame.code != protocol.JOINT_STATE_CODE:
        return None
    return frame.data


def parse_handle_state(raw: bytes) -> Optional[protocol.HandleState]:
    """Return the :class:`~.protocol.HandleState` from a telemetry packet, or
    ``None`` if ``raw`` is not a well-formed handle-state frame."""
    if len(raw) != protocol.HANDLE_STATE_SIZE:
        return None
    frame = protocol.HandleStateReceived.from_buffer_copy(raw)
    if frame.code != protocol.HANDLE_STATE_CODE:
        return None
    return frame.data


# --------------------------------------------------------------------------
# Robot-status interpretation (RobotState lookup tables)
# --------------------------------------------------------------------------
# Mapping of ``robot_basic_state`` -> short token. From the "Lookup Table for
# Robot Status" in the Jueying Lite3 Motion Host Communication Interface.
BASIC_STATE_NAMES = {
    1: "sitting",
    4: "preparing",
    5: "standing_up",
    6: "standing",  # torque-control (standing) state
    7: "sitting_down",
    8: "lose_control_protection",
    9: "posture_adjustment",
    11: "flipping_over",
    17: "resetting_to_zero",
    18: "backflip",
    20: "hello",
}

# Mapping of ``robot_gait_state`` -> short token (only meaningful while moving).
GAIT_NAMES = {
    0: "flat_slow",
    2: "rug_general",
    4: "flat_medium",
    5: "flat_fast",
    6: "rug_grip",
    12: "moonwalk",
    13: "rug_h_step",
}

# Mapping of ``robot_motion_state`` -> the trick the robot is performing. Value
# 0 means "in the basic state", 1 means "stepping with the current gait"; both
# defer to the basic-/gait-state tokens above.
MOTION_STATE_NAMES = {
    2: "twist",
    4: "twist_jump",
    11: "long_jump",
}

# ``robot_basic_state`` values in which the robot has lost its footing — used to
# raise a "fallen" event.
FALLEN_BASIC_STATES = frozenset({8, 11})  # lose-control-protection, flipping-over

# Valid range of the ultrasonic rangefinders, in metres (per the interface doc:
# values are clamped to this band).
ULTRASOUND_MIN_RANGE = 0.28
ULTRASOUND_MAX_RANGE = 4.50


def describe_robot_status(state: protocol.RobotState) -> str:
    """Return a short, stable token describing what the Lite3 is doing now.

    Combines ``robot_basic_state``, ``robot_gait_state`` and
    ``robot_motion_state`` the way the interface doc's status lookup table does:
    a one-shot trick (``robot_motion_state`` 2/4/11) takes precedence, then an
    active gait while stepping, otherwise the bare basic state.
    """
    motion = MOTION_STATE_NAMES.get(state.robot_motion_state)
    if motion is not None:
        return motion
    if state.robot_motion_state == 1:  # stepping with the current gait
        gait = GAIT_NAMES.get(state.robot_gait_state)
        return f"walking_{gait}" if gait else "walking"
    return BASIC_STATE_NAMES.get(
        state.robot_basic_state, f"unknown_{state.robot_basic_state}"
    )


# --------------------------------------------------------------------------
# Math helper
# --------------------------------------------------------------------------
def quaternion_from_rpy_degrees(roll: float, pitch: float, yaw: float):
    """Convert roll/pitch/yaw **in degrees** (the Lite3's IMU units) to a
    ``(x, y, z, w)`` quaternion."""
    r, p, y = (math.radians(v) for v in (roll, pitch, yaw))
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


__all__ = [
    "encode_simple_cmd",
    "encode_complex_cmd",
    "encode_velocity",
    "encode_heartbeat",
    "parse_robot_state",
    "parse_joint_state",
    "parse_handle_state",
    "describe_robot_status",
    "BASIC_STATE_NAMES",
    "GAIT_NAMES",
    "MOTION_STATE_NAMES",
    "FALLEN_BASIC_STATES",
    "ULTRASOUND_MIN_RANGE",
    "ULTRASOUND_MAX_RANGE",
    "quaternion_from_rpy_degrees",
]
