"""SupportedType wrappers for Lite3 feedback.

The Lite3's odometry and battery map onto Sugarcoat's built-in ``Odometry`` and
``Float64`` types, and its robot-status / balance flags onto the built-in
``String`` / ``Bool`` types, so only the IMU and the ultrasound rangefinders
need custom wrappers — built here with
:func:`ros_sugar.robot.create_supported_type`.
"""

import numpy as np
from sensor_msgs.msg import Imu as RosImu
from sensor_msgs.msg import Range as RosRange

from ros_sugar.robot import create_supported_type


def _imu_callback(msg: RosImu) -> np.ndarray:
    """Lite3 IMU message -> ``[qx, qy, qz, qw, wx, wy, wz, ax, ay, az]``."""
    o, w, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
    return np.array(
        [o.x, o.y, o.z, o.w, w.x, w.y, w.z, a.x, a.y, a.z], dtype=np.float64
    )


def _range_callback(msg: RosRange) -> float:
    """Lite3 ultrasound message -> the measured distance in metres."""
    return float(msg.range)


# Registered SupportedType wrapping sensor_msgs/Imu. The plugin's IMU feedback
# decoder produces RosImu instances; this type's callback turns them into an
# array for recipe code.
Imu = create_supported_type(RosImu, callback=_imu_callback, module="lite3_plugin.types")

# Registered SupportedType wrapping sensor_msgs/Range, used for both the front
# and back Lite3 ultrasonic rangefinders. The callback exposes the bare
# distance (metres) to recipe code.
Range = create_supported_type(
    RosRange, callback=_range_callback, module="lite3_plugin.types"
)


__all__ = ["Imu", "Range"]
