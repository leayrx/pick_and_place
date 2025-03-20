import sys
import os
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Ajouter le répertoire contenant les scripts au PYTHONPATH si nécessaire
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'projet_robot_tiago')))

# Importer les autres programmes
from projet_robot_tiago import navigation
from projet_robot_tiago import navigation1
from projet_robot_tiago import detection_couleur
from projet_robot_tiago import aruco_detector
from projet_robot_tiago import aruco_verifier
from projet_robot_tiago import signe_tete


def run_aruco_detector(executor):
    print("Lancement de aruco_detector...")
    aruco_detector.main()


def run_aruco_verifier(executor):
    print("Lancement de aruco_verifier...")
    aruco_verifier.main() 


def run_signe_tete():
    print("Lancement du programme 'signe de la tête'...")
    signe_tete.main()  


def main():
    
    rclpy.init()  

    print("Lancement du programme global...")

    # exécuteur multi-thread pour gérer plusieurs nœuds en parallèle
    executor = MultiThreadedExecutor()

    
    print("Lancement de navigation...")
    navigation.main()  

    print("Lancement de detection_couleur...")
    detection_couleur.main()  

    print("Lancement de la navigation vers les aruco..")
    navigation1.main()  

    # Créer les nœuds pour l'ArUco detector et verifier dans des threads séparés
    thread_aruco_detector = threading.Thread(target=run_aruco_detector, args=(executor,))
    thread_aruco_verifier = threading.Thread(target=run_aruco_verifier, args=(executor,))

    # Démarrer les threads
    thread_aruco_detector.start()
    thread_aruco_verifier.start()

    
    try:
        print("Démarrage de l'exécution ROS2...")
        executor.spin()  # Exécuter les nœuds ROS2

    except KeyboardInterrupt:
        print("Interruption du programme")

    finally:
        # Attendre la fin des threads avant de passer à la suite
        thread_aruco_detector.join()
        thread_aruco_verifier.join()

        signe_tete.main()
        
        rclpy.shutdown()


if __name__ == "__main__":
    main()
