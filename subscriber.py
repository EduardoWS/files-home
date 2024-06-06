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

        # Inicializar variáveis para armazenar a profundidade mínima
        self.latest_depth_array = None
        self.timer = self.create_timer(2.0, self.timer_callback)

    def depth_callback(self, msg):
        self.get_logger().info('Depth message received.')

        # Converte a mensagem Image para um array numpy
        try:
            depth_array = np.frombuffer(msg.data, dtype=np.float32)
            self.get_logger().info(f'Depth array shape before reshape: {depth_array.shape}')
            depth_array = depth_array.reshape((msg.height, msg.width))
            self.get_logger().info(f'Depth array shape after reshape: {depth_array.shape}')
            self.latest_depth_array = depth_array
        except Exception as e:
            self.get_logger().error(f'Error processing depth message: {e}')

    def timer_callback(self):
        if self.latest_depth_array is not None:
            # Filtrar valores inválidos (zeros) e calcular a distância mínima válida
            valid_depths = self.latest_depth_array[self.latest_depth_array > 0]
            if valid_depths.size > 0:
                min_distance = np.min(valid_depths)
                self.get_logger().info(f'Minimum valid distance: {min_distance}')
                if min_distance < 0.8:  # Defina a distância mínima de segurança desejada
                    self.get_logger().info('Algo muito próximo! O robô não pode se mover.')
                else:
                    self.get_logger().info('Nada muito próximo. O robô pode se mover.')
            else:
                self.get_logger().info('Nenhuma profundidade válida detectada.')
        else:
            self.get_logger().info('Nenhum dado de profundidade recebido ainda.')

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