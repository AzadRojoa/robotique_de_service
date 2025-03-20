📄 This project in short
Ce dépôt contient le package ROS2 permettant de respecter le comportement de notre robot dans le simulateur Gazebo.

Notre projet a pour but d'automatiser une action d'inventaire que l'on peut retrouver dans le monde de l'industrie. Grâce à des codes Aruco, le robot reconnaît l'étagère et les produits qu'il voit, pour ensuite les compter et comparer ces informations avec celles présentes dans une base de données.
Notre robot de base est le robot Tiago. Nous utilisons donc les dépendences liés à ce robot.
Notre package est uniquement compatible avec la distribution Jarry de ROS2. 

🚀 Quickstart

Prérequis:
Pour utiliser notre package, la distribution Jazzy de ROS2 doit être installée sur votre ordinateur. Vous pouvez suivre [ce tutoriel]([url](https://docs.ros.org/en/jazzy/Installation.html)) pour installer ROS2 Jazzy.

Instructions d'installation:
Pour installer notre package, créez tout d'abord un dossier réprésentant votre workspace. Celui-ci contiendra tout votre espace de travail. Lancez la commande

mkdir <nom_de_votre_workspace>

Ensuite, dans votre workspace, créez un dossier "src" (source), qui contiendra tout vos packages ROS. Lancez les commande:

cd ~/<nom_de_votre_workspace>
mkdir src

Rentrez dans ce dossier et clonez le package ROS 2 en utilisant la commande:

git clone https://github.com/AzadRojoa/robotique_de_service.git

Vous êtes dorénavant prêt à utiliser notre package. 

Instructions de lancement:
Pour démarrer notre package, dirigez-vous dans votre workspace et compilez le avec la commande:

colcon build --symilnk-install

Activez ensuite ROS2 dans votre workspace en utilisant la commande:

source install/setup.bash

Démarrer ensuite 3 terminal. Vous pouvez utilisez la commande Ctrl + Alt + T pour en ouvrir un.

1. Dans votre premier terminal, ouvrez votre monde Gazebo avec:

ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=warehouse

2. Dans le deuxième terminal, lancez un script pour téléopérer votre robot:

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/key_vel

3. Dans le troisième terminal, lancez le script correspondant au comportement du robot:

ros2 run projet box


À l'aide du terminal de téléopération, déplacez votre robot face à un code Aruco. Il devraitle détecter et s'approcher de ce code automatiquement.

