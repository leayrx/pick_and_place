import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from queue import Queue

class ArUcoDetector(Node):
    def __init__(self, image_queue, stop_event, counter):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        self.id_publisher = self.create_publisher(Int32, 'aruco_marker_id', 10)
        self.subscription = self.create_subscription(
            Image,
            '/head_front_camera/image',
            self.image_callback,
            10
        )
        self.get_logger().info("Abonné au topic /head_front_camera/image")
        self.image_queue = image_queue  
        self.stop_event = stop_event  # Event pour signaler l'arrêt
        self.marker_detected = False
        self.counter = counter  # Compteur des requêtes envoyées

    def image_callback(self, msg):
        try:
            if self.marker_detected or self.stop_event.is_set() or self.counter[0] >= 10:
                return  # Arrêter si 10 publications ont été envoyées ou l'événement d'arrêt est signalé
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.detect_aruco_markers(cv_image)
            self.image_queue.put(cv_image)  
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la conversion de l'image : {e}")

    def detect_aruco_markers(self, image):
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        aruco_params = cv2.aruco.DetectorParameters_create()
        corners, ids, rejected = cv2.aruco.detectMarkers(image, aruco_dict, parameters=aruco_params)

        if len(corners) > 0:
            self.get_logger().info(f"Marqueurs détectés: {ids}")
            central_marker_id = None
            min_distance = float('inf')
            center_of_image = (image.shape[1] // 2, image.shape[0] // 2)

            for i, corner in enumerate(corners):
                marker_center = np.mean(corner[0], axis=0)
                distance = np.linalg.norm(marker_center - center_of_image)

                if distance < min_distance:
                    min_distance = distance
                    central_marker_id = int(ids[i][0])

            if central_marker_id is not None:
                self.x = central_marker_id
                self.get_logger().info(f"Marqueur central détecté : ID = {self.x}")
                self.publish_marker_id(self.x)

            if self.x is not None:
                image_with_markers = cv2.aruco.drawDetectedMarkers(image, corners, ids)
                self.get_logger().info(f"Marqueur au centre : {self.x}")
                self.marker_detected = True
        else:
            self.get_logger().info("Aucun marqueur ArUco détecté")

    def publish_marker_id(self, marker_id):
        msg = Int32()
        msg.data = marker_id
        self.id_publisher.publish(msg)
        self.get_logger().info(f"ID du marqueur publié : {msg.data}")
        self.counter[0] += 1  # Incrémenter le compteur après chaque publication

def ros_thread(image_queue, stop_event, counter):
    rclpy.init()
    aruco_detector = ArUcoDetector(image_queue, stop_event, counter)
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

def main():
    # Créer une Queue pour envoyer les images au thread principal
    image_queue = Queue()
    stop_event = threading.Event()
    counter = [0]  # Utilisation d'une liste pour partager le compteur entre les threads

    # Créer un thread pour ROS
    ros_thread_instance = threading.Thread(target=ros_thread, args=(image_queue, stop_event, counter), daemon=True)
    ros_thread_instance.start()

    # Attendre une temporisation avant de commencer l'affichage
    time.sleep(1)

    # Boucle d'affichage OpenCV dans le thread principal
    while True:
        if not image_queue.empty():
            
            cv_image = image_queue.get()
            cv2.imshow("Camera Feed", cv_image)  # Affichage de l'image avec les marqueurs

        # Vérifier si l'utilisateur a pressé la touche 'q' pour fermer la fenêtre
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Fermeture de la fenêtre...")
            stop_event.set()  
            cv2.destroyAllWindows()
            break
        
        # Si 10 requêtes ont été envoyées, arrêter le programme
        if counter[0] >= 1:
            time.sleep(3)
            print("3 marqueurs ont été publiés. Arrêt du programme.")
            stop_event.set()  
            break

if __name__ == '__main__':
    main()
