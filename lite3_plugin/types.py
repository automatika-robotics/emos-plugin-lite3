"""SupportedType wrapper for the Lite3's ultrasonic rangefinders.

Almost all of the Lite3's feedback maps onto Sugarcoat's built-in
``SupportedType``s: ``Odometry`` and ``Float64`` (odometry, battery),
``String`` / ``Bool`` (status / balance flags), ``Twist`` (the operator handle),
``Imu`` and ``JointState``. Only the ultrasonic ``Range`` still needs a
custom wrapper, created here.
"""

from sensor_msgs.msg import Range as RosRange

from ros_sugar.robot import create_supported_type


def _range_callback(msg: RosRange) -> float:
    """Lite3 ultrasound message -> the measured distance in metres."""
    return float(msg.range)


#: Registered SupportedType wrapping ``sensor_msgs/Range``, used for both the
#: front and back Lite3 ultrasonic rangefinders. The callback exposes the bare
#: distance (metres) to recipe code.
Range = create_supported_type(RosRange, callback=_range_callback)


__all__ = ["Range"]
