#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class UrdfTopicPublisher(Node):
    def __init__(self):
        super().__init__('urdf_topic_publisher')

        self.declare_parameter('robot_description', '')
        self.declare_parameter('topic_name', 'robot_description')

        description = self.get_parameter('robot_description').value
        topic_name = self.get_parameter('topic_name').value
        self.topic_name = topic_name

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.publisher = self.create_publisher(String, topic_name, qos)

        msg = String()
        msg.data = description

        # Publish repeatedly for a short window so late-joining tools reliably receive it.
        self._msg = msg
        self._publish_count = 0
        self._timer = self.create_timer(0.5, self._publish_once)

    def _publish_once(self):
        self.publisher.publish(self._msg)
        self._publish_count += 1

        if self._publish_count == 1:
            self.get_logger().info(
                f"Publishing URDF on '{self.topic_name}'"
            )

        if self._publish_count >= 10:
            self._timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = UrdfTopicPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
