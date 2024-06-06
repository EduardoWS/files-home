import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class DepthSubscriber(Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/depth_topic',  # Substitua 'depth_topic' pelo nome do tópico do sensor de profundidade
            self.depth_callback,
            10)
        self.get_logger().info('Subscription to depth topic initialized.')

    def depth_callback(self, msg):
        self.get_logger().info('Depth message received.')
        
        # Converte a mensagem Image para um array numpy
        try:
            depth_array = np.frombuffer(msg.data, dtype=np.float32)
            self.get_logger().info(f'Depth array shape before reshape: {depth_array.shape}')
            depth_array = depth_array.reshape((msg.height, msg.width))
            self.get_logger().info(f'Depth array shape after reshape: {depth_array.shape}')
        except Exception as e:
            self.get_logger().error(f'Error processing depth message: {e}')
            return
        
        # Filtrar valores inválidos (zeros) e calcular a distância mínima válida
        valid_depths = depth_array[depth_array > 0]
        if valid_depths.size > 0:
            min_distance = np.min(valid_depths)
            self.get_logger().info(f'Minimum valid distance: {min_distance}')
            if min_distance < 0.5:  # Defina a distância mínima de segurança desejada
                self.get_logger().info('Algo muito próximo! O robô não pode se mover.')
            else:
                self.get_logger().info('Nada muito próximo. O robô pode se mover.')
        else:
            self.get_logger().info('Nenhuma profundidade válida detectada.')

def main(args=None):
    rclpy.init(args=args)
    depth_subscriber = DepthSubscriber()
    rclpy.spin(depth_subscriber)
    depth_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
