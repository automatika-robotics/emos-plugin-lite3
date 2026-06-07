"""SupportedType wrappers for Lite3 feedback.

The Lite3's odometry and battery map onto Sugarcoat's built-in ``Odometry`` and
``Float64`` types, its robot-status / balance flags onto the built-in
``String`` / ``Bool`` types, and its operator-joystick handle state onto the
built-in ``Twist`` type. The IMU, the ultrasound rangefinders and the leg
``JointState`` need custom wrappers.
"""

import numpy as np
from sensor_msgs.msg import Imu as RosImu
from sensor_msgs.msg import JointState as RosJointState
from sensor_msgs.msg import Range as RosRange

from ros_sugar.io.supported_types import _additional_types
from ros_sugar.robot import create_supported_type


def _relocate(supported_type: type) -> type:
    """Re-stamp a ``create_supported_type`` result so it survives multiprocess
    launch.

    ``create_supported_type`` builds the class with a bare ``type()`` call from
    inside ``ros_sugar.robot.types``, so the result inherits that module and is
    registered under an unimportable key (e.g. ``ros_sugar.robot.types.Imu``).
    When a component runs in its own process, ``ros_sugar`` serializes that key
    and the child rebuilds the type with ``getattr(import_module(module), name)``
    -- which then fails. Re-stamping ``__module__`` to this module (where the
    type is actually bound) and fixing the registry key makes that round-trip
    resolve.
    """
    stale_key = f"{supported_type.__module__}.{supported_type.__qualname__}"
    supported_type.__module__ = __name__
    _additional_types.pop(stale_key, None)
    _additional_types[f"{__name__}.{supported_type.__qualname__}"] = supported_type
    return supported_type


def _imu_callback(msg: RosImu) -> np.ndarray:
    """Lite3 IMU message -> ``[qx, qy, qz, qw, wx, wy, wz, ax, ay, az]``."""
    o, w, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
    return np.array(
        [o.x, o.y, o.z, o.w, w.x, w.y, w.z, a.x, a.y, a.z], dtype=np.float64
    )


def _range_callback(msg: RosRange) -> float:
    """Lite3 ultrasound message -> the measured distance in metres."""
    return float(msg.range)


def _joint_state_callback(msg: RosJointState) -> np.ndarray:
    """Lite3 JointState message -> array of the 12 leg-joint positions (rad),
    ordered as `lite3_plugin.codecs.JOINT_NAMES`."""
    return np.asarray(msg.position, dtype=np.float64)


# Registered SupportedType wrapping sensor_msgs/Imu. The plugin's IMU feedback
# decoder produces RosImu instances; this type's callback turns them into an
# array for recipe code.
Imu = _relocate(create_supported_type(RosImu, callback=_imu_callback))

# Registered SupportedType wrapping sensor_msgs/Range, used for both the front
# and back Lite3 ultrasonic rangefinders. The callback exposes the bare
# distance (metres) to recipe code.
Range = _relocate(create_supported_type(RosRange, callback=_range_callback))

# Registered SupportedType wrapping sensor_msgs/JointState for the 12 leg
# joints. The plugin's JointState feedback decoder produces RosJointState
# instances; this type's callback exposes the position array to recipe code.
JointState = _relocate(
    create_supported_type(RosJointState, callback=_joint_state_callback)
)


__all__ = ["Imu", "Range", "JointState"]
