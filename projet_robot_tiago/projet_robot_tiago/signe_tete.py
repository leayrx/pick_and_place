import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import json

class RobotHeadControlNode(Node):
    def __init__(self):
        super().__init__('robot_head_control')

        # Initialiser 'valid' à False et indiquer si le robot est en mouvement
        self.valid = False
        self.en_mouvement = False

        # Abonnement au topic 'marker_valid' pour recevoir les informations de détection
        self.subscription = self.create_subscription(
            Bool,
            'marker_valid',  # Nom du topic auquel vous vous abonnez
            self.listener_callback,
            10
        )

        # Créer un éditeur pour publier sur le topic /head_controller/joint_trajectory
        self.publisher_ = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)

        # Créer un éditeur pour publier l'état de mouvement du robot
        self.movement_publisher = self.create_publisher(Bool, '/robot_moving', 10)

        self.counter = 0
        self.direction = 1
        self.movement_count = 0
        self.stop_moving = False
        self.total_movements = 0  # Nouveau compteur pour suivre les mouvements effectués

        # Charger les informations du fichier JSON
        self.load_json_data()

        # Exécution de la boucle pour publier régulièrement
        self.timer = self.create_timer(0.5, self.timer_callback)

    def load_json_data(self):
        # Charger les données depuis le fichier result.json
        try:
            with open('aruco_analysis_results.json', 'r') as f:
                data = json.load(f)
                # Exemple de récupération de l'état de détection du marqueur
                self.valid = data.get('marker_valid', False)
                self.get_logger().info(f"Données JSON chargées: Marqueur valide - {self.valid}")
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la lecture du fichier JSON: {str(e)}")

    def listener_callback(self, msg):
        # Cette fonction est appelée lorsqu'un message est reçu sur le topic 'marker_valid'
        self.get_logger().info(f"Statut du marqueur détecté: {msg.data}")
        
        # Mettre à jour le paramètre 'valid' en fonction de l'état de détection
        self.valid = msg.data

    def timer_callback(self):
        # Publier l'état du mouvement du robot
        movement_msg = Bool()
        movement_msg.data = self.en_mouvement
        self.movement_publisher.publish(movement_msg)
        self.get_logger().info(f"État du mouvement publié: {'En mouvement' if self.en_mouvement else 'À l\'arrêt'}")

        # Si on doit arrêter le mouvement, on ne fait rien
        if self.stop_moving:
            self.get_logger().info("Arrêt des mouvements.")
            self.get_logger().info("Arrêt du programme...")
            self.destroy_node()  # Arrêter le nœud proprement
            return

        # Créer le message JointTrajectory
        msg = JointTrajectory()
        msg.joint_names = ['head_1_joint', 'head_2_joint']

        if self.en_mouvement:  # Si le robot est en mouvement, ne pas traiter les marqueurs ArUco
            self.get_logger().info("Le robot est en mouvement, ignorer la détection ArUco")
            return

        if self.valid:  # Si le marqueur est détecté (True)
            # Mouvement pour dire "oui" de la tête (par exemple, incliner la tête en avant)
            if self.counter < 3:
                point = JointTrajectoryPoint()
                if self.direction == 1:
                    point.positions = [0.0, 0.5]  # Incliner la tête en avant
                else:
                    point.positions = [0.0, -0.5]  # Incliner la tête en arrière

                self.get_logger().info(f'Signe: Oui, mouvement de head_1_joint, direction: {"avant" if self.direction == 1 else "arrière"}')

                point.time_from_start.sec = 1
                msg.points = [point]
                self.movement_count += 1

                if self.movement_count >= 5:
                    self.counter += 1
                    self.movement_count = 0
                    self.direction = -self.direction  # Alterner la direction

            else:
                self.get_logger().info('Retour au centre')
                point = JointTrajectoryPoint()
                point.positions = [0.0, 0.0]  # Revenir au centre
                point.time_from_start.sec = 1
                msg.points = [point]

                self.counter = 0
                self.movement_count = 0
                self.direction = 1  # Réinitialiser la direction pour le prochain cycle

                self.total_movements += 1  # Incrémenter le compteur de mouvements
                if self.total_movements >= 3:  # Si 3 mouvements ont été effectués, arrêter
                    self.stop_moving = True
                    self.en_mouvement = True  # Indiquer que le robot est en mouvement
        else:  # Si le marqueur n'est pas détecté (False), faire le mouvement "non"
            # Mouvement pour dire "non" de la tête (par exemple, tourner la tête à gauche/droite)
            if self.counter < 3:
                point = JointTrajectoryPoint()
                if self.direction == 1:
                    point.positions = [0.5, 0.0]  # Tourner la tête à gauche
                else:
                    point.positions = [-0.5, 0.0]  # Tourner la tête à droite

                self.get_logger().info(f'Signe: Non, mouvement de head_2_joint, direction: {"gauche" if self.direction == 1 else "droite"}')

                point.time_from_start.sec = 1
                msg.points = [point]
                self.movement_count += 1

                if self.movement_count >= 5:
                    self.counter += 1
                    self.movement_count = 0
                    self.direction = -self.direction  # Alterner la direction

            else:
                self.get_logger().info('Retour au centre')
                point = JointTrajectoryPoint()
                point.positions = [0.0, 0.0]  # Revenir au centre
                point.time_from_start.sec = 1
                msg.points = [point]

                self.counter = 0
                self.movement_count = 0
                self.direction = 1  # Réinitialiser la direction pour le prochain cycle

                self.total_movements += 1  # Incrémenter le compteur de mouvements
                if self.total_movements >= 3:  # Si 3 mouvements ont été effectués, arrêter
                    self.stop_moving = True
                    self.en_mouvement = True  # Indiquer que le robot est en mouvement

        # Publier le message sur le topic
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    # Créer une instance du node qui contrôle la tête
    robot_head_control_node = RobotHeadControlNode()

    try:
        rclpy.spin(robot_head_control_node)  # Bloque l'exécution jusqu'à ce que le nœud soit détruit
    except KeyboardInterrupt:
        pass  # Interrompre proprement si nécessaire

    # Détruire le nœud et arrêter ROS proprement
    robot_head_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
