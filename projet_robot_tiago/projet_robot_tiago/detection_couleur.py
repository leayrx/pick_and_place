import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import json


class CanetteDetector(Node):
    def __init__(self):
        super().__init__('canette_detector')
        self.bridge = CvBridge()

        # Abonnement au topic de la caméra
        self.subscription = self.create_subscription(
            Image,
            '/head_front_camera/image', 
            self.image_callback,
            10
        )

        self.get_logger().info("Abonné au topic /head_front_camera/image")

        # fenêtre OpenCV pour l'affichage
        cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)

        self.candidate_detected = False
        self.done = False

    def image_callback(self, msg):
        try:
            # Convertir le message ROS en image OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Détecter la canette la plus proche du centre de l'image
            couleur, can_id = self.detect_can_color(cv_image)
            if couleur and not self.candidate_detected:
                self.get_logger().info(f"Couleur de la canette la plus proche du centre: {couleur} (ID: {can_id})")
                self.candidate_detected = True  # Indiquer qu'une couleur a été détectée

                # Enregistrer la couleur et l'ID dans un fichier JSON
                self.save_color_and_id_to_file(couleur, can_id)

                # Arrêter le nœud après la publication
                self.done = True
                self.get_logger().info("Couleur et ID enregistrés et programme arrêté.")

                # Fermer proprement la fenêtre OpenCV
                cv2.destroyAllWindows()

        except Exception as e:
            self.get_logger().error(f"Erreur lors de la conversion de l'image : {e}")

    def save_color_and_id_to_file(self, couleur, can_id):
        try:
            with open('can_color.json', 'w') as file:
                json.dump({"can_color": couleur, "can_id": can_id}, file)
            self.get_logger().info(f"Couleur et ID enregistrés dans le fichier can_color.json: {couleur}, {can_id}")
        except Exception as e:
            self.get_logger().error(f"Erreur lors de l'enregistrement dans le fichier: {e}")

    def calculer_barycentre(self, corners):
        """
        Calcule les barycentres pour chaque marqueur détecté.
        """
        barycentres = []
        for corner in corners:
            points = corner.reshape(-1, 2)  #(x, y)
            Gx = np.mean(points[:, 0])  # Moyenne des coordonnées x
            Gy = np.mean(points[:, 1])  # Moyenne des coordonnées y
            barycentres.append((Gx, Gy))
        return barycentres

    def trouver_marker_plus_proche(self, centre, barycentres, ids):
        """
        Trouve le marqueur dont le barycentre est le plus proche du centre de l'image.
        """
        min_distance = float('inf')
        closest_marker = None
        for i, (Gx, Gy) in enumerate(barycentres):
            distance = np.sqrt((Gx - centre[0])**2 + (Gy - centre[1])**2)
            if distance < min_distance:
                min_distance = distance
                closest_marker = i  
        return closest_marker

    def detect_can_color(self, image):
        """
        Détecte la couleur de la canette la plus proche du centre de l'image.
        Retourne la couleur et l'ID associé (30 pour rouge, 40 pour vert).
        """
        
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Plages de couleur pour le rouge et le vert en HSV
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask_red = cv2.inRange(hsv_image, lower_red, upper_red)

        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask_red2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

        # Amélioration de la plage pour le vert
        lower_green = np.array([35, 50, 50])  
        upper_green = np.array([90, 255, 255])
        mask_green = cv2.inRange(hsv_image, lower_green, upper_green)

        # masques rouges
        mask_red_combined = cv2.bitwise_or(mask_red, mask_red2)

        # contours de la canette
        contours_red, _ = cv2.findContours(mask_red_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # aucun contour trouvé
        if not contours_red and not contours_green:
            return "autre", -1

        # barycentre 
        barycentres_red = self.calculer_barycentre([c.reshape(-1, 2) for c in contours_red])
        barycentres_green = self.calculer_barycentre([c.reshape(-1, 2) for c in contours_green])

        # centre de l'image
        center_of_image = (image.shape[1] // 2, image.shape[0] // 2)

        # marqueur rouge le plus proche du centre
        closest_red_marker = self.trouver_marker_plus_proche(center_of_image, barycentres_red, contours_red)
        closest_green_marker = self.trouver_marker_plus_proche(center_of_image, barycentres_green, contours_green)

        closest_can_color = None
        can_id = None
        if closest_red_marker is not None and closest_green_marker is None:
            closest_can_color = "rouge"
            can_id = 30  # ID associé au rouge
        elif closest_red_marker is None and closest_green_marker is not None:
            closest_can_color = "vert"
            can_id = 40  # ID associé au vert
        elif closest_red_marker is not None and closest_green_marker is not None:
            # Si les deux marqueurs existent, on compare les distances
            dist_red = np.sqrt((barycentres_red[closest_red_marker][0] - center_of_image[0])**2 + 
                               (barycentres_red[closest_red_marker][1] - center_of_image[1])**2)
            dist_green = np.sqrt((barycentres_green[closest_green_marker][0] - center_of_image[0])**2 + 
                                 (barycentres_green[closest_green_marker][1] - center_of_image[1])**2)

            if dist_red < dist_green:
                closest_can_color = "rouge"
                can_id = 30
            else:
                closest_can_color = "vert"
                can_id = 40

        return closest_can_color, can_id

def main(args=None):
    # Initialiser ROS2
    rclpy.init(args=args)
    canette_detector = CanetteDetector()

    # Maintenir le nœud en vie pour recevoir les messages
    while not canette_detector.done:
        rclpy.spin_once(canette_detector)

    # Fermer proprement
    canette_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
