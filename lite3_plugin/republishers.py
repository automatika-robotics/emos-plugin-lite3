"""Bridges that re-publish robot plugin feedbacks on real ROS2 topics."""

from attrs import define, field
from functools import partial
from typing import Any, List, Optional, Sequence

from geometry_msgs.msg import TransformStamped

from ros_sugar.core import BaseComponent
from ros_sugar.core.component import BaseComponentConfig
from ros_sugar.io.topic import Topic

from .types import Lite3Imu


@define(frozen=True)
class FeedbackBridge:
    """One ``feedback-key -> ROS2-topic`` mapping for :class:`FeedbackRepublisher`.

    :param feedback_key: Key on ``plugin.feedbacks`` to bridge (e.g. ``"Odometry"``).
    :param out_topic_name: ROS2 topic name to publish on (e.g. ``"/odom"``).
    :param msg_type: Input message type. Must match the feedback's declared
        ``msg_type``. Accepts a ``SupportedType`` subclass or its name as a string.
    :param qos_profile: QoS depth (or full QoS profile) for the output publisher.
    :param tf_parent_frame: When set together with ``tf_child_frame``, the
        bridge also broadcasts a ``/tf`` transform from the feedback's pose.
        Only valid for ``Odometry``-typed feedbacks (the message must carry a
        ``pose.pose``).
    :param tf_child_frame: Child frame for the broadcast transform.
    """

    feedback_key: str = field()
    out_topic_name: str = field()
    msg_type: Any = field()
    qos_profile: Any = field(default=10)
    tf_parent_frame: Optional[str] = field(default=None)
    tf_child_frame: Optional[str] = field(default=None)

    @property
    def publishes_tf(self) -> bool:
        """Whether this bridge also broadcasts a ``/tf`` transform."""
        return bool(self.tf_parent_frame and self.tf_child_frame)


class FeedbackRepublisher(BaseComponent):
    """Subscribe to one or more robot-plugin feedbacks and republish them on
    ROS2 topics.
    """

    def __init__(
        self,
        *,
        component_name: str,
        bridges: Sequence[FeedbackBridge],
        config: Optional[BaseComponentConfig] = None,
    ):
        self._bridges: List[FeedbackBridge] = list(bridges)
        # One input topic per bridge, keyed by the feedback key for easy lookup in the callbacks.
        inputs = [
            Topic(
                name=bridge.feedback_key,
                msg_type=bridge.msg_type,
                use_plugin=True,
            )
            for bridge in self._bridges
        ]
        super().__init__(
            component_name=component_name,
            inputs=inputs,
            outputs=None,
            config=config,
        )
        self._ros_publishers: dict = {}
        self._tf_broadcaster = None

    def custom_on_activate(self) -> None:
        plugin = self._robot_plugin
        if plugin is None:
            self.get_logger().error(
                f"{self.node_name}: no robot plugin attached; cannot "
                "republish feedbacks"
            )
            return
        if any(bridge.publishes_tf for bridge in self._bridges):
            from tf2_ros import TransformBroadcaster

            self._tf_broadcaster = TransformBroadcaster(self)
        for bridge in self._bridges:
            feedback = plugin.feedbacks.get(bridge.feedback_key)
            if feedback is None:
                self.get_logger().error(
                    f"{self.node_name}: feedback '{bridge.feedback_key}' not "
                    f"found on plugin '{plugin.metadata.name}'"
                )
                continue
            callback = self.callbacks.get(bridge.feedback_key)
            if callback is None:
                self.get_logger().error(
                    f"{self.node_name}: input callback slot "
                    f"'{bridge.feedback_key}' missing -- feedback bus wiring "
                    "did not run"
                )
                continue
            ros_type = feedback.msg_type.get_ros_type()
            publisher = self.create_publisher(
                ros_type, bridge.out_topic_name, bridge.qos_profile
            )
            self._ros_publishers[bridge.feedback_key] = publisher
            callback.on_callback_execute(
                partial(self._forward, bridge), get_processed=False
            )
            tf_note = (
                f" + TF '{bridge.tf_parent_frame}' -> '{bridge.tf_child_frame}'"
                if bridge.publishes_tf
                else ""
            )
            self.get_logger().info(
                f"{self.node_name}: republishing feedback "
                f"'{bridge.feedback_key}' on '{bridge.out_topic_name}'{tf_note}"
            )

    def custom_on_deactivate(self) -> None:
        # Destroy the publishers and drop them from the registry
        for publisher in self._ros_publishers.values():
            self.destroy_publisher(publisher)
        self._ros_publishers.clear()
        self._tf_broadcaster = None

    def _forward(self, bridge: FeedbackBridge, *, msg: Any, topic: Topic) -> None:
        if not self.context.ok():
            return
        publisher = self._ros_publishers.get(bridge.feedback_key)
        if publisher is None:
            return

        now = self.get_clock().now().to_msg()
        if hasattr(msg, "header"):
            msg.header.stamp = now
        publisher.publish(msg)
        if bridge.publishes_tf and self._tf_broadcaster is not None:
            transform = self._pose_to_transform(msg, bridge)
            transform.header.stamp = now
            self._tf_broadcaster.sendTransform(transform)

    @staticmethod
    def _pose_to_transform(odom_msg: Any, bridge: FeedbackBridge) -> TransformStamped:
        """Build a ``TransformStamped`` from an ``Odometry`` message's pose.

        The caller stamps ``header.stamp``.
        """
        transform = TransformStamped()
        transform.header.frame_id = bridge.tf_parent_frame
        transform.child_frame_id = bridge.tf_child_frame
        position = odom_msg.pose.pose.position
        transform.transform.translation.x = position.x
        transform.transform.translation.y = position.y
        transform.transform.translation.z = position.z
        transform.transform.rotation = odom_msg.pose.pose.orientation
        return transform

    def _execution_step(self):
        pass  # No regular execution; all work happens in the callbacks


class Lite3FeedbackPublisher(FeedbackRepublisher):
    """Bridge the Lite3 plugin's ``Odometry`` and ``Imu`` feedbacks to real
    ROS2 topics.

    Defaults: ``Odometry`` -> ``/odom`` (plus an ``odom`` -> ``body`` TF),
    ``Imu`` -> ``/imu/data``::

        from lite3_plugin import Lite3Plugin, Lite3FeedbackPublisher
        from kompass.launcher import Launcher

        launcher = Launcher(config_file="lite3.yaml", robot_plugin=Lite3Plugin())
        launcher.add_pkg(components=[..., Lite3FeedbackPublisher()])
        launcher.bringup()
    """

    def __init__(
        self,
        *,
        component_name: str = "lite3_feedback_publisher",
        odom_topic_name: str = "/odom",
        imu_topic_name: str = "/imu/data",
        odom_frame: str = "odom",
        body_frame: str = "body",
        publish_tf: bool = True,
        qos_profile: Any = 10,
        config: Optional[BaseComponentConfig] = None,
    ):
        super().__init__(
            component_name=component_name,
            bridges=[
                FeedbackBridge(
                    feedback_key="Odometry",
                    out_topic_name=odom_topic_name,
                    msg_type="Odometry",
                    qos_profile=qos_profile,
                    tf_parent_frame=odom_frame if publish_tf else None,
                    tf_child_frame=body_frame if publish_tf else None,
                ),
                FeedbackBridge(
                    feedback_key="Imu",
                    out_topic_name=imu_topic_name,
                    msg_type=Lite3Imu,
                    qos_profile=qos_profile,
                ),
            ],
            config=config,
        )


__all__ = ["FeedbackBridge", "FeedbackRepublisher", "Lite3FeedbackPublisher"]
