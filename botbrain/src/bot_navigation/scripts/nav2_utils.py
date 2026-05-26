#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from action_msgs.srv import CancelGoal


class Nav2Utils(Node):
    """
    Nav2 utility node that provides a service to cancel navigation goals.
    Uses relative topic/service names so that the namespace set in the
    launch file (e.g. 'g1_robot') is automatically prepended.
    """

    def __init__(self):
        super().__init__('nav2_utils')

        self._callback_group = ReentrantCallbackGroup()

        # --- Action-cancel clients (relative paths → inherit namespace) ---
        # Actual resolved paths will be e.g.
        #   /g1_robot/navigate_to_pose/_action/cancel_goal
        #   /g1_robot/follow_waypoints/_action/cancel_goal
        self._cancel_nav_client = self.create_client(
            CancelGoal,
            'navigate_to_pose/_action/cancel_goal',
            callback_group=self._callback_group
        )

        self._cancel_waypoints_client = self.create_client(
            CancelGoal,
            'follow_waypoints/_action/cancel_goal',
            callback_group=self._callback_group
        )

        # --- Cancel service (relative path → /g1_robot/cancel_nav2_goal) ---
        self._cancel_service = self.create_service(
            Trigger,
            'cancel_nav2_goal',
            self.cancel_goal_callback,
            callback_group=self._callback_group
        )

        self.get_logger().info('Nav2 Utils node started')
        self.get_logger().info('Service cancel_nav2_goal is active and ready')

    # ------------------------------------------------------------------
    def cancel_goal_callback(self, request, response):
        """
        Service callback to cancel the current Nav2 goal
        (both NavigateToPose and FollowWaypoints).
        """
        goal_cancelled = False

        # Cancel NavigateToPose goals
        if self._cancel_nav_client.service_is_ready():
            try:
                cancel_request = CancelGoal.Request()
                result = self._cancel_nav_client.call(cancel_request)

                if result and len(result.goals_canceling) > 0:
                    goal_cancelled = True
                    self.get_logger().info('NavigateToPose goal cancelled')

            except Exception as e:
                self.get_logger().error(f'Error canceling NavigateToPose goal: {str(e)}')

        # Cancel FollowWaypoints goals
        if self._cancel_waypoints_client.service_is_ready():
            try:
                cancel_request = CancelGoal.Request()
                result = self._cancel_waypoints_client.call(cancel_request)

                if result and len(result.goals_canceling) > 0:
                    goal_cancelled = True
                    self.get_logger().info('FollowWaypoints goal cancelled')

            except Exception as e:
                self.get_logger().error(f'Error canceling FollowWaypoints goal: {str(e)}')

        # Build response
        if goal_cancelled:
            response.success = True
            response.message = 'Navigation goal cancelled'
            self.get_logger().info('Navigation goal cancelled successfully')
        else:
            response.success = True
            response.message = 'No active navigation goals to cancel'
            self.get_logger().info('No active navigation goals found')

        return response


def main(args=None):
    rclpy.init(args=args)

    nav2_utils_node = Nav2Utils()
    executor = MultiThreadedExecutor()
    executor.add_node(nav2_utils_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        nav2_utils_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
