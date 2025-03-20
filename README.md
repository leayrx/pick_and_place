Project name: pick and place 
 
👨‍💻 Students: Yrieix L et Leger V

📄 This project in short
Le projet est développé dans l’environnement ROS2 et utilise  Nav2 et OpenCV. Le robot commence par se déplacer vers la canette à récupérer. Une fois à proximité, il détecte sa couleur afin de l’identifier. Ensuite, il navigue vers la boîte de rangement correspondante et scanne un marqueur ArUco pour vérifier qu’il est au bon emplacement. Si le code ArUco est correct, le robot acquiesce en faisant un signe de tête "oui", sinon, il secoue la tête pour indiquer "non".


🚀 Quickstart


Launch instructions :

Start DevContainer
You need Visual Studio Code preinstalled

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

3️⃣ start program projet_global
ros2 run projet_robot_tiago projet_global



📚 References and bibliography


...
