from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'projet_robot_tiago'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosdev',
    maintainer_email='rosdev@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'projet_global = projet_robot_tiago.projet_global:main',
            'navigation = projet_robot_tiago.navigation:main',
            'navigation1 = projet_robot_tiago.navigation1:main',
            'aruco_detector = projet_robot_tiago.aruco_detector:main',
            'aruco_verifier = projet_robot_tiago.aruco_verifier:main',
            'detection_couleur = projet_robot_tiago.detection_couleur:main',
            'signe_tete = projet_robot_tiago.signe_tete:main'
        ],
    },
)
