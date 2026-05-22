"""Sugarcoat robot plugin for the DeepRobotics Lite3 quadruped.

A class-based conversion of the DeepRobotics ``message_transformer`` ROS 2
package: it speaks the Lite3 Motion Host UDP protocol directly, so the original
C++ bridge process is not needed.

Usage::

    from ros_sugar.launch import Launcher
    from lite3_plugin import Lite3Plugin

    launcher = Launcher(robot_plugin=Lite3Plugin())
    launcher.add_pkg(components=[...])
    launcher.bringup()
"""

from .plugin import Lite3Plugin
from .republishers import Lite3FeedbackPublisher

__all__ = [
    "Lite3Plugin",
    "Lite3FeedbackPublisher",
]
