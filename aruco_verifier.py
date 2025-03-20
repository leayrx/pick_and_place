import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
import json

class ArUcoIdReceiver(Node):
    def __init__(self):
        super().__init__('aruco_id_receiver')

        # Créer un abonné pour écouter les messages sur le topic 'aruco_marker_id'
        self.subscription = self.create_subscription(
            Int32,  # Le type du message que l'on écoute
            'aruco_marker_id',  # Le topic sur lequel on écoute
            self.listener_callback,  # Le callback qui sera appelé quand un message est reçu
            10  # La taille de la file d'attente
        )

        # Créer un publisher pour publier le résultat (True ou False) sur le topic 'marker_valid'
        self.publisher = self.create_publisher(Bool, 'marker_valid', 10)

        self.get_logger().info('Abonné au topic "aruco_marker_id"')

        # Charger la couleur et l'ID attendu depuis le fichier
        self.expected_id, self.expected_color = self.load_color_and_id_from_file()

        # Variable pour suivre le nombre de messages traités
        self.message_count = 0
        self.max_messages = 1  # Arrêter après le premier message

    def listener_callback(self, msg):
        """
        Cette méthode est appelée chaque fois qu'un message est reçu.
        Elle récupère l'ID du marqueur et vérifie s'il correspond à l'ID attendu.
        """
        self.message_count += 1

        marker_id = msg.data  # Récupérer l'ID du marqueur depuis le message
        self.get_logger().info(f"ID du marqueur détecté : {marker_id}")

        # Vérifier si l'ID correspond à la valeur attendue
        is_valid = marker_id == self.expected_id
        color_is_valid = self.check_color_validity()

        # Publier True si l'ID est valide et la couleur correspond, sinon publier False
        result_msg = Bool()
        result_msg.data = is_valid and color_is_valid
        self.publisher.publish(result_msg)

        # Afficher dans le log si l'ID est valide ou non
        if is_valid and color_is_valid:
            self.get_logger().info(f"L'ID et la couleur sont valides (ID={self.expected_id}, Couleur={self.expected_color}). Publier True.")
        else:
            self.get_logger().info(f"L'ID ou la couleur ne sont pas valides. Publier False.")
        
        # Sauvegarder les données analysées dans un fichier JSON
        self.save_data_to_json(marker_id, is_valid, color_is_valid)

        # Arrêter après avoir reçu un certain nombre de messages
        if self.message_count >= self.max_messages:
            self.get_logger().info("Arrêt du programme après le premier message.")
            rclpy.shutdown()  # Arrêter le programme immédiatement

    def load_color_and_id_from_file(self):
        """
        Charge l'ID attendu et la couleur depuis le fichier JSON.
        """
        try:
            with open('can_color.json', 'r') as file:
                data = json.load(file)
                expected_color = data.get("can_color", "aucune couleur détectée")
                expected_id = data.get("aruco_id", 50)  # L'ID attendu est dans le fichier JSON
                return expected_id, expected_color
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la lecture du fichier: {e}")
            return None, None

    def check_color_validity(self):
        """
        Vérifie si la couleur chargée correspond à celle du fichier.
        """
        couleur = self.expected_color  # Charger la couleur à partir du fichier
        if couleur:
            self.get_logger().info(f"La couleur de la canette détectée est : {couleur}")
            return True
        else:
            self.get_logger().info("Aucune couleur n'a été détectée ou le fichier est manquant.")
            return False

    def save_data_to_json(self, marker_id, is_valid, color_is_valid):
        """
        Sauvegarder les données analysées (ID du marqueur et validité de la couleur)
        dans un fichier JSON.
        """
        data_to_save = {
            "aruco_id": marker_id,
            "is_valid": is_valid,
            "color_is_valid": color_is_valid,
            "expected_id": self.expected_id,
            "expected_color": self.expected_color
        }

        try:
            # Ouvrir le fichier JSON et y écrire les données
            with open('aruco_analysis_results.json', 'w') as json_file:
                json.dump(data_to_save, json_file, indent=4)
            self.get_logger().info(f"Données sauvegardées dans le fichier JSON : {data_to_save}")
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la sauvegarde dans le fichier JSON: {e}")


def main(args=None):
    # Ne pas appeler rclpy.init() ici car il a déjà été appelé dans le programme principal.
    # Créer une instance de notre nœud
    aruco_id_receiver = ArUcoIdReceiver()

    # Créer un exécuteur multi-threadé
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()

    # Ajouter le nœud à l'exécuteur
    executor.add_node(aruco_id_receiver)

    # Exécuter l'exécuteur
    executor.spin()

    # Fermer proprement
    aruco_id_receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
