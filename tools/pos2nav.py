import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class NavigateNode(Node):
    def __init__(self):
        super().__init__('navigate_node')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.subscription = self.create_subscription(
            PoseStamped,
            'pallet_pose',
            self.pose_callback,
            10
        )
        self.sentGoal = False

    def pose_callback(self, msg):
        if(not self.sentGoal):
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = msg
            self.send_goal(goal_msg)
            self.sentGoal = True


    def send_goal(self, goal_msg):
        self.get_logger().info('Sending goal to the navigation server...')
        self.action_client.wait_for_server()

        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by the navigation server')
            return

        self.get_logger().info('Goal accepted by the navigation server')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.goal_completion_callback)

    def goal_completion_callback(self, future):
        goal_handle = future.result()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    navigate_node = NavigateNode()
    rclpy.spin(navigate_node)


if __name__ == '__main__':
    main()