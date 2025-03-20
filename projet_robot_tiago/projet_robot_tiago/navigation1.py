import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient


class SendGoalNode(Node):
    def __init__(self):
        super().__init__('send_goal_node')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for the navigate_to_pose action server...')
        self.goal_reached = False  # Variable pour savoir si le but est atteint

    def send_goal(self):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        #current_orientation = goal_pose.pose.orientation

        # Point fixe (par exemple, un seul point ici)
        goal_pose.pose.position.x = -3.5  # Position X du but
        goal_pose.pose.position.y = -1.50 # Position Y du but
        goal_pose.pose.position.z = 0.0
        # Garder la même orientation que le robot avait en arrivant
        #goal_pose.pose.orientation = current_orientation
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 90.0
        goal_pose.pose.orientation.w = 1.0

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: ' + str(result))

        # Lorsque le résultat est reçu, définissez goal_reached sur True
        self.goal_reached = True
        self.get_logger().info('Robot reached the goal and will stop.')

def main():
    rclpy.init()
    node = SendGoalNode()
    node.send_goal()  # Envoie un objectif unique

    while rclpy.ok():
        rclpy.spin_once(node)  # Faire une seule boucle de gestion des événements
        if node.goal_reached:  # Si l'objectif est atteint, arrêtez le nœud
            node.get_logger().info('Stopping the node after goal is reached.')
            break  # Sortie de la boucle

    node.destroy_node()
    rclpy.shutdown()  # Arrêt du programme
