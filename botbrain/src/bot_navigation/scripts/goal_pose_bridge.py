#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class GoalPoseBridge(Node):
    def __init__(self):
        super().__init__("goal_pose_bridge")

        self.declare_parameter("default_goal_frame", "map")
        self.default_goal_frame = self.get_parameter("default_goal_frame").value

        self._nav_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._goal_pose_sub = self.create_subscription(
            PoseStamped,
            "goal_pose",
            self.goal_pose_callback,
            10,
        )

        self.get_logger().info(
            "Goal pose bridge ready: topic 'goal_pose' -> action 'navigate_to_pose'"
        )

    def goal_pose_callback(self, msg: PoseStamped) -> None:
        if not self._nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("NavigateToPose action server is not available yet.")
            return

        goal_pose = PoseStamped()
        goal_pose.header = msg.header
        goal_pose.pose = msg.pose

        if not goal_pose.header.frame_id:
            goal_pose.header.frame_id = self.default_goal_frame
            self.get_logger().warn(
                f"Received goal_pose without frame_id, defaulting to '{self.default_goal_frame}'."
            )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info(
            "Forwarding goal_pose to NavigateToPose: "
            f"x={goal_pose.pose.position.x:.3f}, "
            f"y={goal_pose.pose.position.y:.3f}, "
            f"frame={goal_pose.header.frame_id}"
        )
        self._nav_to_pose_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
