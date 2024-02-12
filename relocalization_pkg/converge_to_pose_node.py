import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist
import time
from rclpy.executors import MultiThreadedExecutor

MAX_WEIGHT_THRESHOLD = 0.00085
MAX_TIME_THRESHOLD = 10.0 # seconds
TWIST_ANGULAR_Z = 1.0 # rad/s

class ConvergeToPose(Node):
    def __init__(self):
        super().__init__('converge_to_pose_node')
         
        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.convergence_status_publisher = self.create_publisher(Bool, '/convergence_status', 10)
        self.finish_time_pub = self.create_publisher(Float64, '/finish_time', 10)
        
        # Subscriptions
        self.subscription_max_particle_weight = self.create_subscription(
            Float64,
            '/max_particle_weight',
            self.max_particle_weight_callback,
            10)
        self.subscription_status_initialpose = self.create_subscription(
            Bool,
            '/status_initialpose',
            self.status_initialpose_callback,
            10)
        
        # General variables
        self.status_initialpose = False
        self.max_particle_weight = MAX_WEIGHT_THRESHOLD
        self.count = 1
        self.counter_timer_callback = 0
        self.flag = 0
        
        # Timer
        self.start_time = None
        self.timer = self.create_timer(0.4, self.timer_callback)
        self.rate = self.create_rate(10)

    
    def max_particle_weight_callback(self, msg):
        self.max_particle_weight = msg.data
        #self.get_logger().info(f'[CALLBACK] Max particle weight: {self.max_particle_weight}')
            
    def status_initialpose_callback(self, msg):
        self.status_initialpose = msg.data
        self.get_logger().info(f'[CALLBACK] Status initialpose: {self.status_initialpose}')
        if self.status_initialpose:
            self.max_particle_weight = 0.0
            self.counter_timer_callback = 0
            self.start_time = time.time()
                
    def timer_callback(self):
        if self.counter_timer_callback == 0 and self.flag==0:
            self.max_particle_weight = 0.0
            self.flag += 1
        if self.status_initialpose:
            self.get_logger().info(f'[TIMER CALLBACK] Max particle weight: {self.max_particle_weight}')
            self.rate.sleep()
            if self.max_particle_weight < MAX_WEIGHT_THRESHOLD and (time.time() - self.start_time) < MAX_TIME_THRESHOLD:
                self.get_logger().info(f'[TIMER CALLBACK] Moving robot...')
                twist = Twist()
                twist.angular.z = TWIST_ANGULAR_Z
                self.pub_cmd_vel.publish(twist)
            else:
                if (time.time() - self.start_time) < MAX_TIME_THRESHOLD:
                    #print('-'*30)
                    # self.get_logger().info(f'[TEST {self.count}]: SUCCESS!')
                    # self.get_logger().info(f'[TEST {self.count}]: Time elapsed: {time.time() - self.start_time} seconds')
                    # self.get_logger().info(f'[TEST {self.count}]: Max particle weight: {self.max_particle_weight}')
                    self.count += 1
                    self.status_initialpose = False
                    self.max_particle_weight = 0.0
                    self.counter_timer_callback += 1
                    self.flag = 0
                    
                    self.convergence_status = True
                    msg = Bool()
                    msg.data = self.convergence_status
                    self.convergence_status_publisher.publish(msg)
                    
                    msg_finish_time = Float64()
                    msg_finish_time.data = self.get_clock().now().nanoseconds/1e9
                    self.finish_time_pub.publish(msg_finish_time)
                    
                else:
                    # print('-'*30)
                    # self.get_logger().info(f'[TEST {self.count}]: FAILURE!')
                    # self.get_logger().info(f'[TEST {self.count}]: Time elapsed: {time.time() - self.start_time} seconds')
                    # self.get_logger().info(f'[TEST {self.count}]: Max particle weight: {self.max_particle_weight}')
                    self.count += 1
                    self.status_initialpose = False
                    self.max_particle_weight = 0.0
                    self.counter_timer_callback += 1
                    self.flag = 0
                    
                    self.convergence_status = True
                    msg = Bool()
                    msg.data = self.convergence_status
                    self.convergence_status_publisher.publish(msg)
                    
                    msg_finish_time = Float64()
                    msg_finish_time.data = self.get_clock().now().nanoseconds/1e9
                    self.finish_time_pub.publish(msg_finish_time)
                   
    
def main(args=None):
    rclpy.init(args=args)
    node = ConvergeToPose()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    
    executor.remove_node(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()