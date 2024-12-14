import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('color_detection_node')
        self.bridge = CvBridge()

        # Suscriptor al topic de la cámara
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Topic de entrada
            self.image_callback,
            10
        )

        # Publicador del resultado procesado
        self.publisher_ = self.create_publisher(
            Image,
            '/camera/color_detection',  # Topic de salida
            10
        )

        self.get_logger().info("Nodo de reconocimiento de colores inicializado.")

    def image_callback(self, msg):
        try:
            # Verifica el formato de la imagen
            if msg.encoding == '32FC1':  # Imagen de profundidad
                self.get_logger().warn("Imagen en formato de profundidad. Conversión no realizada.")
                return
            elif msg.encoding != 'bgr8':  # Si no es RGB, conviértelo
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:  # Si ya es RGB
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Procesamiento del color
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Define los rangos de color para rojo a amarillo
            lower_red = np.array([0, 120, 70])    # Rojo bajo
            upper_red = np.array([10, 255, 255]) # Rojo alto
            lower_yellow = np.array([10, 120, 70])  # Transición rojo a amarillo
            upper_yellow = np.array([30, 255, 255]) # Amarillo

            # Máscaras para los colores
            mask_red = cv2.inRange(hsv_image, lower_red, upper_red)
            mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

            # Combina las máscaras
            mask = cv2.bitwise_or(mask_red, mask_yellow)

            # Resultado final
            result = cv2.bitwise_and(cv_image, cv_image, mask=mask)

            # Publica la imagen procesada
            result_msg = self.bridge.cv2_to_imgmsg(result, encoding="bgr8")
            self.publisher_.publish(result_msg)
            self.get_logger().info("Imagen procesada y publicada correctamente.")

        except CvBridgeError as e:
            self.get_logger().error(f"Error procesando la imagen: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
