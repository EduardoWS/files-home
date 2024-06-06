import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class DepthSubscriber(Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/kinect2/depth/raw',  # Substitua 'depth_topic' pelo nome do tópico do sensor de profundidade
            self.depth_callback,
            10)
        self.get_logger().info('Subscription to depth topic initialized.')

        # Inicializar variáveis para armazenar a profundidade mínima
        self.min_distance = float('inf')
        self.timer = self.create_timer(2.0, self.timer_callback)

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
            self.min_distance = np.min(valid_depths)
        else:
            self.min_distance = float('inf')
    
    def timer_callback(self):
        # Imprimir a cada 2 segundos com base na distância mínima armazenada
        if self.min_distance < float('inf'):
            self.get_logger().info(f'Minimum valid distance: {self.min_distance}')
            if self.min_distance < 0.5:  # Defina a distância mínima de segurança desejada
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