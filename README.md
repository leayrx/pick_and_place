Project name: pick and place 
 
👨‍💻 Eleves: Yrieix L et Leger V

📄 Le projet en quelques lignes :

Le projet est développé dans un environnement ROS2 et utilise  Nav2 et OpenCV. Le robot commence par se déplacer vers des canettes afin de les récupérer. Une fois à proximité, il détecte la couleur de la canette détectée au centre de sa vision afin de l’identifier. Par la suite, Tiago navigue vers des boîtes de rangement spécifique en fonction de la couleur de la canette perçue. On vient alors scanner un marqueur Aruco sité au dessus de chaques boîte  pour vérifier que le robot se trouve devant la bonne boîte de rangement. Si le code Aruco est correct, le robot acquiesce en faisant un signe "oui" de la tête. Sinon, il tourne la tête de gauche à droite pour indiquer "non".

technologie utilisé : 
- ROS2 jazzy : framework de robotique
- Gazebo : Simulation du robot
- Nav2 : Navigation autonome
- OpenCV : détection et traitement d'image pour l'aruco et couleur 




🚀 Quickstart

Instruction d'installation : 

Attention ce projet est sous la distribution Jazzy et utilise le robot Tiago simulé dans Tiago Harmonic, installer depuis https://github.com/Tiago-Harmonic/tiago_harmonic 

Les projets et les dépendances sont a installer avec : 

Launch instructions :

Start DevContainer
Il vous faudra avoir VsCode d'installé sur votre ordinateur

git clone https://gitlab.com/f2m2robserv/jazzy-ros-ynov
code jazzy-ros-ynov/

When VSCode opens, trust the sources, and accept the installation of the Dev Container extension.
To build the workspace use:

cd ~/ros2_ws
colcon build --symlink-install
source ~/.bashrc

Tiago 2D navigation using Nav2:

Documentation : https://docs.pal-robotics.com/sdk-dev/navigation

1️⃣ Start Gazebo simulation of Tiago robot:
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=pick_and_place

2️⃣ Start 2D navigation by loading your our_map map, using:
ros2 launch tiago_2dnav tiago_nav_bringup.launch.py is_public_sim:=True world_name:=our_map

Lancement programme : 

3️⃣ start program projet_global
ros2 run projet_robot_tiago projet_global



📚 References and bibliography :

- Documentation Jazzy : https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html
- Tourner la tête du robot : https://gitlab.com/-/snippets/4826611
- Ouvrir la webcam de l'ordinateur : https://docs.opencv.org/4.x/dd/d43/tutorial_py_video_display.html
- Lecture de QrCode sur une image : https://pypi.org/project/pyzbar/
- Lecture des codes ARUCO : https://www.eirlab.net/2022/03/22/traitement-dimage-pour-la-detection-de-tag-aruco-avec-opencv-en-python-3-4/
- Faire une map + dev container ros jazzy : https://gitlab.com/f2m2robserv/jazzy-ros-ynov#ros2-jazzy--robots-devcontainer
- Modifier un modèle : https://gazebosim.org/docs/latest/sdf_worlds/#download-the-model



...
