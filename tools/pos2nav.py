import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import math
import numpy as np

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q


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

        self.publisher_ = self.create_publisher(PoseStamped, 'test', 10)

    def pose_callback(self, msg):
        if(not self.sentGoal):
            goal_msg = NavigateToPose.Goal()
            r,p,y = euler_from_quaternion(msg.pose.orientation)
            newX = msg.pose.position.x + math.cos(y)*0.85
            newY = msg.pose.position.y + math.sin(y)*0.85
            quat = quaternion_from_euler(r, p, y)
            goal_msg.pose = msg
            goal_msg.pose.pose.position.x = newX
            goal_msg.pose.pose.position.y = newY
            goal_msg.pose.pose.orientation.x = quat[0]
            goal_msg.pose.pose.orientation.y = quat[1]
            goal_msg.pose.pose.orientation.z = quat[3]
            goal_msg.pose.pose.orientation.w = quat[0]
            self.send_goal(goal_msg)
            self.sentGoal = True
            self.publisher_.publish(goal_msg.pose)


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