import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import ctypes

class DepthSubscriber(Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/depth_topic',  # Substitua 'depth_topic' pelo nome do tópico do sensor de profundidade
            self.depth_callback,
            10)
        self.subscription  # To prevent unused variable warning

    def depth_callback(self, msg):
        # Convertendo a mensagem PointCloud2 para um array numpy
        depth_array = np.frombuffer(msg.data, dtype=np.float32)
        depth_array = depth_array.reshape((msg.height, msg.width))

        # Verificar se há algo muito próximo ao robô
        min_distance = np.min(depth_array)
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