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
    "quaternion_from_rpy_degrees",
]
