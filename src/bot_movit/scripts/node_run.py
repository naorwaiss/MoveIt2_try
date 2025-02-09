#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from bot_movit.srv import SetJointP

class MoveRobotClient(Node):
    def __init__(self):
        super().__init__('move_robot_client')

        # Declare and get ROS parameters
        self.declare_parameter('joint1', 0.0)
        self.declare_parameter('joint2', 0.0)

        joint1 = self.get_parameter('joint1').value
        joint2 = self.get_parameter('joint2').value

        self.client = self.create_client(SetJointP, '/move_robot')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /move_robot to be available...')
        self.send_request([joint1, joint2])
    
    def send_request(self, joint_positions):
        request = SetJointP.Request()
        request.joint_positions = joint_positions

        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)
    
    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Response: {response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
