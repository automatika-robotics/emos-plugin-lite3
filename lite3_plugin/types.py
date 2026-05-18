"""SupportedType wrappers for Lite3 feedback.

The Lite3's odometry and battery map onto Sugarcoat's built-in ``Odometry`` and
``Float64`` types, so only the IMU needs a custom wrapper — built here with
:func:`ros_sugar.robot.create_supported_type`.
"""

import numpy as np
from sensor_msgs.msg import Imu as RosImu

from ros_sugar.robot import create_supported_type


def _imu_callback(msg: RosImu) -> np.ndarray:
    """Lite3 IMU message -> ``[qx, qy, qz, qw, wx, wy, wz, ax, ay, az]``."""
    o, w, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
    return np.array(
        [o.x, o.y, o.z, o.w, w.x, w.y, w.z, a.x, a.y, a.z], dtype=np.float64
    )


# Registered SupportedType wrapping sensor_msgs/Imu. The plugin's IMU feedback
# decoder produces RosImu instances; this type's callback turns them into an
# array for recipe code.
Lite3Imu = create_supported_type(RosImu, callback=_imu_callback)

__all__ = ["Lite3Imu"]
