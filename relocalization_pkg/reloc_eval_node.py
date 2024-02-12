import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from nav2_msgs.msg import ParticleCloud 
from std_msgs.msg import Float64
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
class RelocEvalNode(Node):
    def __init__(self):
        super().__init__('reloc_eval_node')
        self.mean_weight_buffer = deque(maxlen=5)
        self.max_current_weight_buffer = []
        #self.buffer = deque(maxlen=100)
        #self.fig, self.ax = plt.subplots()
        
        
        self.pub_max_weight_particle = self.create_publisher(Float64, '/max_particle_weight', 10)
        #self.pub_entropy = self.create_publisher(Float64, '/entropy', 10)
        self.pub_moving_average_weights = self.create_publisher(Float64, '/moving_average_weight', 10)
        #self.pub = self.create_publisher(Float64, '/entropy_change', 10)
        
        qos_profile = QoSProfile(depth=10, history=QoSHistoryPolicy.KEEP_LAST, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            ParticleCloud,  
            '/particle_cloud',
            self.max_weight_particle_callback,
            qos_profile) 
           
        # self.subscription_v2 = self.create_subscription(
        #     ParticleCloud,  
        #     '/particle_cloud',
        #     self.entropy_callback,
        #     qos_profile)
        
        self.subscription_v3 = self.create_subscription(
            ParticleCloud,  
            '/particle_cloud',
            self.mean_weight_callback,
            qos_profile)
        
    # def entropy_callback(self, msg):
    #     weights = np.array([particle.weight for particle in msg.particles])
    #     weights = weights / np.sum(weights)
    #     entropy = -np.sum(weights * np.log2(weights))
    #     msg = Float64()
    #     msg.data = entropy
    #     self.pub_entropy.publish(msg)
    
    def max_weight_particle_callback(self, msg):
        # Max Current Weight
        max_weight = max([particle.weight for particle in msg.particles])
        msg = Float64()
        msg.data = max_weight
        #self.get_logger().info(f'Max weight: {max_weight}')
        self.pub_max_weight_particle.publish(msg)
        
    def mean_weight_callback(self, msg):
        # Mean Current Weights
        weights = max([particle.weight for particle in msg.particles])
        self.mean_weight_buffer.append(weights)
        moving_average = sum(self.mean_weight_buffer) / len(self.mean_weight_buffer)
        #self.buffer.append(moving_average)
        # self.ax.clear()
        # self.ax.plot(self.buffer, label='Weight')
        # plt.pause(0.01)
        mean_msg = Float64()
        mean_msg.data = moving_average*1000
        self.get_logger().info(f'Mean weight: {mean_msg.data}')
        self.pub_moving_average_weights.publish(mean_msg)
        
    

def main(args=None):
    rclpy.init(args=args)

    max_particle_weight_publisher = RelocEvalNode()
    rclpy.spin(max_particle_weight_publisher)

    max_particle_weight_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()