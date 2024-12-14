"""
@file object_detection_node.py
@Autor: Sofia Milagros Castaño Vanegas
@date: 2024-12-10
@version: 1.0
@brief Implementación de un nodo ROS 2 para detección de objetos utilizando YOLOv8.
Este archivo contiene la implementación de un nodo ROS 2 llamado `ObjectDetectionNode` que procesa imágenes 
recibidas de un tópico, aplica un modelo de detección de objetos YOLOv8 y publica tanto las imágenes con 
los objetos detectados como información sobre las clases y coordenadas de los bounding boxes detectados.

@includes
- rclpy: Biblioteca de ROS 2 para manejo de nodos y comunicación.
- sensor_msgs.msg.Image: Mensaje estándar para transmitir imágenes.
- std_msgs.msg.String: Mensaje estándar para publicar datos de texto.
- cv_bridge: Herramienta para convertir imágenes entre ROS y OpenCV.
- cv2: Biblioteca OpenCV para procesamiento de imágenes.
- ultralytics.YOLO: Clase para usar el modelo YOLOv8.
- math: Biblioteca matemática para redondeos y cálculos.

@clase ObjectDetectionNode
@brief Nodo ROS 2 que realiza detección de objetos en imágenes y publica resultados.

@metodos
- __init__: Constructor que inicializa el nodo, sus suscripciones y publicadores, y carga el modelo YOLOv8.
- image_callback: Callback que procesa las imágenes recibidas desde un tópico.
- process_data: Aplica detección de objetos a las imágenes utilizando YOLOv8 y dibuja los bounding boxes.
- timer_callback: Publica periódicamente las clases detectadas, coordenadas de bounding boxes, y las imágenes procesadas.

@miembros
- bridge: Objeto CvBridge para convertir entre imágenes ROS y OpenCV.
- image_sub: Suscripción al tópico que transmite imágenes de cámara.
- image_pub: Publicador del tópico que transmite imágenes procesadas con objetos detectados.
- text_publisher: Publicador para la clase del objeto detectado.
- bbox_publisher: Publicador para las coordenadas del bounding box.
- model: Modelo YOLOv8 cargado para detección de objetos.
- classNames: Lista de clases que puede detectar el modelo YOLOv8.
- latest_image: Imagen más reciente recibida del tópico de cámara.
- filtered_frame: Imagen procesada con objetos detectados y bounding boxes dibujados.
- class_name: Nombre de la clase del objeto detectado más reciente.
- bbox_name: Coordenadas del bounding box del objeto detectado más reciente.
- timer: Temporizador que ejecuta publicaciones periódicas.

"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import math

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.bridge = CvBridge()

        # Suscripción a la imagen de la cámara
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publicador de la imagen con objetos detectados
        self.image_pub = self.create_publisher(
            Image,
            '/detected_image',
            10
        )

        # Publicador para la clase detectada
        self.text_publisher = self.create_publisher(String, 'class_topic', 10)
        self.text_msg = String()

        # Publicador para las coordenadas del bounding box
        self.bbox_publisher = self.create_publisher(String, 'bbox_topic', 10)
        self.bbox_msg = String()

        # Cargo el modelo YOLOv8 preentrenado
        self.model = YOLO("/home/sofia/robotics_ws/src/prueba_robotics4_0/prueba_robotics4_0/yolov8n.pt")

        # Clases de objetos detectables
        self.classNames = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
            'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 
            'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 
            'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 
            'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 
            'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 
            'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 
            'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 
            'hair drier', 'toothbrush'
        ]

        self.latest_image = None
        self.filtered_frame = None
        self.class_name = ""
        self.bbox_name = []

        self.get_logger().info("Object detection node initialized.")

        # Crear un temporizador para publicaciones periódicas
        timer_period = 0.1  # segundos
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def image_callback(self, msg):
        """ Callback para procesar la imagen recibida. """
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.process_data()

    def process_data(self):
        """ Procesa la imagen con detección de objetos usando YOLO. """
        if self.latest_image is None:
            return

        results = self.model(self.latest_image)

        img_cv = self.latest_image.copy()

        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(img_cv, (x1, y1), (x2, y2), (255, 0, 0), 2)
                confidence = math.ceil((box.conf[0] * 100)) / 100
                cls = int(box.cls[0])
                self.class_name = self.classNames[cls]
                label = f"Class: {self.class_name}, Conf: {confidence:.2f}"
                cv2.putText(img_cv, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        self.filtered_frame = img_cv

    def timer_callback(self):
        """ Publica clase, bounding boxes e imagen periódicamente. """
        self.text_msg.data = self.class_name
        self.text_publisher.publish(self.text_msg)
        bbox_str = ' '.join(map(str, self.bbox_name))
        self.bbox_msg.data = bbox_str
        self.bbox_publisher.publish(self.bbox_msg)
        if self.filtered_frame is not None:
            ros_image = self.bridge.cv2_to_imgmsg(self.filtered_frame, encoding="bgr8")
            self.image_pub.publish(ros_image)
        self.get_logger().info("Publishing video frame")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
