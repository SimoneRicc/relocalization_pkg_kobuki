import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool, Float64
from interfaces.msg import Position
import numpy as np
import os

TEST_PATH = '/workspace/src/relocalization_pkg/test_real/'

class EvalTestNode(Node):
    def __init__(self):
        super().__init__('eval_test_node')
        
        # Parameters
        self.declare_parameter('position',0)
        self.number_position = int(self.get_parameter('position').value)
        
        # Subscriptions
        self.initial_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'initialpose',
            self.initial_pose_callback,
            10)
        self.amcl_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.amcl_pose_callback,
            10)
        self.status_initialpose_subscription = self.create_subscription(
            Bool,
            'status_initialpose',
            self.status_initialpose_callback,
            10)
        self.convergence_status_subscription = self.create_subscription(
            Bool,
            'convergence_status',
            self.convergence_status_callback,
            10)
        self.start_time_subscription = self.create_subscription(
            Float64,
            'start_time',
            self.start_time_callback,
            10)
        self.finish_time_subscription = self.create_subscription(
            Float64,
            'finish_time',
            self.finish_time_callback,
            10)
        self.vicon_pose_subscription = self.create_subscription(
            Position,
            '/vicon/kobuki/kobuki',
            self.vicon_pose_callback,
            10)
        
        # Load the numpy with test for performances
        self.test_array = np.array([])
        self.test_vector = np.zeros(16)
        
        
        # General variables
        self.initial_pose = None
        self.amcl_pose = None
        self.test_counter = 1
        self.finish_time = None
        self.vicon_pose = None
        
    def vicon_pose_callback(self, msg):
        self.vicon_pose = msg
        
    def start_time_callback(self, msg):
        self.get_logger().info(f'Start time: {msg.data}')
        self.test_vector[14] = msg.data
        
    def finish_time_callback(self, msg):
        self.get_logger().info(f'Finish time: {msg.data}')
        self.test_vector[15] = msg.data
        self.test_vector[1] = self.vicon_pose.x_trans
        self.test_vector[2] = self.vicon_pose.y_trans
        self.test_vector[3] = self.vicon_pose.z_rot
        self.test_vector[4] = self.vicon_pose.w
        self.get_logger().info(f'Vicon pose: {self.vicon_pose.x_trans}, {self.vicon_pose.y_trans}, {self.vicon_pose.z_rot}, {self.vicon_pose.w}')
        self.get_logger().info('-'*50)
        
    def initial_pose_callback(self, msg):
        self.initial_pose = msg

    def amcl_pose_callback(self, msg):
        self.amcl_pose = msg

    def status_initialpose_callback(self, msg):
        if msg.data:
            self.get_logger().info(f'Position: {self.number_position}')
            self.get_logger().info(f'Test number: {self.test_counter}')
            self.get_logger().info(f'Initial pose: {self.initial_pose.pose.pose.position.x}, {self.initial_pose.pose.pose.position.y}, {self.initial_pose.pose.pose.orientation.z}, {self.initial_pose.pose.pose.orientation.w}')
            self.test_vector[0] = self.number_position
            # self.test_array 1-->4 coordinates position with Vicon
            self.test_vector[5] = self.test_counter
            self.test_counter += 1
            self.test_vector[6] = self.initial_pose.pose.pose.position.x # X
            self.test_vector[7] = self.initial_pose.pose.pose.position.y # Y
            self.test_vector[8] = self.initial_pose.pose.pose.orientation.z   # quaternion Z   # quaternion Z
            self.test_vector[9] = self.initial_pose.pose.pose.orientation.w   # quaternion Z   # quaternion W
            self.get_logger().info('')            
              

    def convergence_status_callback(self, msg):
        if msg.data:
            self.get_logger().info(f'Final pose: {self.amcl_pose.pose.pose.position.x}, {self.amcl_pose.pose.pose.position.y}, {self.amcl_pose.pose.pose.orientation.z}, {self.amcl_pose.pose.pose.orientation.w}')
            self.test_vector[10] = self.amcl_pose.pose.pose.position.x
            self.test_vector[11] = self.amcl_pose.pose.pose.position.y
            self.test_vector[12] = self.amcl_pose.pose.pose.orientation.z
            self.test_vector[13] = self.amcl_pose.pose.pose.orientation.w

            
            self.test_array = np.concatenate((self.test_array, self.test_vector), axis=0)
            
    def destroy_node(self):
        np.save(os.path.join(TEST_PATH, f'test_dynamic_{self.number_position}'), self.test_array)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    eval_test_node = EvalTestNode()

    try:
        rclpy.spin(eval_test_node)
    except KeyboardInterrupt:
        pass

    eval_test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()