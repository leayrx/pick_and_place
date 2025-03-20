import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import json

class RobotHeadControlNode(Node):
    def __init__(self):
        super().__init__('robot_head_control')

        self.valid = False
        self.en_mouvement = False

        self.subscription = self.create_subscription(
            Bool,
            'marker_valid',  
            self.listener_callback,
            10
        )

        # Créer un éditeur pour publier sur le topic /head_controller/joint_trajectory
        self.publisher_ = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)
        self.movement_publisher = self.create_publisher(Bool, '/robot_moving', 10)

        self.counter = 0
        self.direction = 1
        self.movement_count = 0
        self.stop_moving = False
        self.total_movements = 0  # mouvements effectués

        # Charger les informations du fichier JSON
        self.load_json_data()
        self.timer = self.create_timer(0.5, self.timer_callback)

    def load_json_data(self):
        # Charger les données depuis le fichier result.json
        try:
            with open('aruco_analysis_results.json', 'r') as f:
                data = json.load(f)
                self.valid = data.get('marker_valid', False)
                self.get_logger().info(f"Données JSON chargées: Marqueur valide - {self.valid}")
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la lecture du fichier JSON: {str(e)}")

    def listener_callback(self, msg):
        self.get_logger().info(f"Statut du marqueur détecté: {msg.data}")
        self.valid = msg.data

    def timer_callback(self):
        movement_msg = Bool()
        movement_msg.data = self.en_mouvement
        self.movement_publisher.publish(movement_msg)
        self.get_logger().info(f"État du mouvement publié: {'En mouvement' if self.en_mouvement else 'À l\'arrêt'}")

        if self.stop_moving:
            self.get_logger().info("Arrêt des mouvements.")
            self.get_logger().info("Arrêt du programme...")
            self.destroy_node() 
            return

        msg = JointTrajectory()
        msg.joint_names = ['head_1_joint', 'head_2_joint']

        if self.en_mouvement:  # Si le robot est en mouvement, ne pas traiter les marqueurs ArUco
            self.get_logger().info("Le robot est en mouvement, ignorer la détection ArUco")
            return

        if self.valid:  
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
                    self.direction = -self.direction  

            else:
                self.get_logger().info('Retour au centre')
                point = JointTrajectoryPoint()
                point.positions = [0.0, 0.0]  # Revenir au centre
                point.time_from_start.sec = 1
                msg.points = [point]

                self.counter = 0
                self.movement_count = 0
                self.direction = 1  # Réinitialiser 

                self.total_movements += 1  
                if self.total_movements >= 3:  
                    self.stop_moving = True
                    self.en_mouvement = True  
        else:  

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
                    self.direction = -self.direction  

            else:
                self.get_logger().info('Retour au centre')
                point = JointTrajectoryPoint()
                point.positions = [0.0, 0.0]  # Revenir au centre
                point.time_from_start.sec = 1
                msg.points = [point]

                self.counter = 0
                self.movement_count = 0
                self.direction = 1  

                self.total_movements += 1  
                if self.total_movements >= 3:  
                    self.stop_moving = True
                    self.en_mouvement = True  

        # Publier le message sur le topic
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    # Créer une instance du node qui contrôle la tête
    robot_head_control_node = RobotHeadControlNode()

    try:
        rclpy.spin(robot_head_control_node)  
    except KeyboardInterrupt:
        pass  

    # Détruire le nœud et arrêter ROS proprement
    robot_head_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
