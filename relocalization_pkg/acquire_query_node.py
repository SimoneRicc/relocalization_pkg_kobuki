import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import SetBool
import threading
from math import pi
from std_msgs.msg import Bool, Float64
import cv2
import os
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy


FREQUENCY_QUERY = 2.5  # Hz
TWIST_ANGULAR_Z = 1.0 # rad/s
SAVE_PATH = "/workspace/src/relocalization_pkg/reloc_test/test_17"
 
class AcquireQueryNode(Node):
    def __init__(self):
        super().__init__('acquire_query_node')
        
        # General variables
        self.trigger_relocalization = False # Set to True to enable relocalization task
        self.image_count = 0
        self.image = None # Image message
        self.bridge = CvBridge()
        self.rate = self.create_rate(100)
        self.moving_average = 1
        
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
        )        
        # Subscribers
        self.pi_camera_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, qos_profile)
        self.moving_average_weights_sub = self.create_subscription(Float64, '/moving_average_weight', self.moving_average_weights_callback, qos_profile)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.query_image_pub = self.create_publisher(Image, '/query_image', 10)
        self.status_query_pub = self.create_publisher(Bool, '/status_acquisition_query',10)
        self.start_time_pub = self.create_publisher(Float64, '/start_time', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Service to trigger relocalization task
        self.trigger_relocalization_service = self.create_service(SetBool, 'trigger_relocalization', self.handle_trigger_relocalization)
        
        # Create callback for relocalization task
        self.msg = Bool()
        self.msg.data = False
        self.status_query_pub.publish(self.msg)
        self.check_trigger = self.create_timer(0.1, self.check_trigger)
        self.image_buffer = []

        self.get_logger().info('Waiting to trigger relocalization with the command ros2 service call /trigger_relocalization std_srvs/srv/SetBool "{data: true}" ...')
    
    
    def moving_average_weights_callback(self, msg):
        self.moving_average = msg.data  
        
    def check_trigger(self):
        if self.trigger_relocalization:# and (self.moving_average < 0.55):
            self.get_logger().info('Relocalization pipeline triggered...')
            self.trigger_relocalization = False

            # Start the calculation function in a new thread
            threading.Thread(target=self.publisher_query).start()
            
    def publisher_query(self):
        '''Robot starts to rotate about 360 degrees and publish query images at a certain frequency'''

        # Start rotation
        twist = Twist()
        twist.angular.z = TWIST_ANGULAR_Z
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Robot is rotating...')
        
        # Wait for the robot to rotate about 360 degrees
        msg = Float64()
        start_time = self.get_clock().now()
        msg.data = start_time.nanoseconds/1e9
        self.start_time_pub.publish(msg)
        last_query_time = self.get_clock().now()
        
        while ((self.get_clock().now() - start_time).nanoseconds/1e9) < 2.6 * pi/ TWIST_ANGULAR_Z:
            self.msg.data = True
            self.status_query_pub.publish(self.msg)
            self.cmd_vel_pub.publish(twist)
            # Publish a query image every FREQUENCY_QUERY seconds
            if ((self.get_clock().now() - last_query_time).nanoseconds/1e9) >= 1.0 / FREQUENCY_QUERY:
                
                self.publish_query()
                last_query_time = self.get_clock().now()
            self.rate.sleep()
        
        
        self.image_count = 0
        # Stop rotation
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        self.msg.data = False
        self.status_query_pub.publish(self.msg)
        
        # DEBUG - save images in directory
        
        self.image_buffer = []       
        
    
    def publish_query(self):
        '''Acquire an image from the camera publish it to the query_image topic'''
        if self.image is not None:
            #try:
            #    cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding='bgr8')
            # except CvBridgeError as e:
            #   print(e)
            # image_filename = os.path.join(SAVE_PATH, f"image_{self.image_count}.jpg")
            # cv2.imwrite(image_filename, cv_image)
            self.query_image_pub.publish(self.image)
            self.image_buffer.append(self.image)
            self.get_logger().info(f'Image {self.image_count} acquired...')
            self.image_count += 1
    
    def handle_trigger_relocalization(self, request, response):
        '''Service to handle the trigger_relocalization flag'''
        self.trigger_relocalization = request.data
        response.success = True
        response.message = 'Relocalization task triggered...'
        return response
    
    def image_callback(self, msg):
        self.image = msg

def main(args=None):
    
    rclpy.init(args=args)
    node = AcquireQueryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()