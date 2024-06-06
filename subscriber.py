import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class DepthSubscriber(Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/kinect2/depth/raw', 
            self.depth_callback,
            10)
        self.get_logger().info('Subscription to depth topic initialized.')

    def depth_callback(self, msg):
        self.get_logger().info('Depth message received.')
        
        # Converte a mensagem Image para um array numpy
        depth_array = np.frombuffer(msg.data, dtype=np.float32)
        depth_array = depth_array.reshape((msg.height, msg.width))
        
        # Verificar se há algo muito próximo ao robô
        min_distance = np.min(depth_array)
        self.get_logger().info(f'Minimum distance: {min_distance}')
        if min_distance < 0.5:  # Defina a distância mínima de segurança desejada
            self.get_logger().info('Algo muito próximo! O robô não pode se mover.')
        else:
            self.get_logger().info('Nada muito próximo. O robô pode se mover.')

def main(args=None):
    rclpy.init(args=args)
    depth_subscriber = DepthSubscriber()
    rclpy.spin(depth_subscriber)
    depth_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



"""
header:
    stamp:
        sec: 17177703666
        nanosec: 966138819
    frame_id: kinect2_link
height: 424
width: 512
encoding: 32FC1
is_bigendian: 0
step: 2048
data:
- 0
- 0
- 0
- 0
...
"""