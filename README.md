# 📄 Projet inventaire d'atelier

Ce dépôt contient le package ROS2 permettant de respecter le comportement de notre robot dans le simulateur Gazebo.

Notre projet a pour but d'automatiser une action d'inventaire que l'on peut retrouver dans le monde de l'industrie. Grâce à des codes ArUco, le robot reconnaît l'étagère et les produits qu'il voit, pour ensuite les compter et comparer ces informations avec celles présentes dans une base de données. 

Notre robot de base est le robot **Tiago**. Nous utilisons donc les dépendances liées à ce robot. Notre package est uniquement compatible avec la distribution **Jazzy** de ROS2.

---

## 🚀 Quickstart

### 📌 Prérequis
Pour utiliser notre package, la distribution **Jazzy** de ROS2 doit être installée sur votre ordinateur. Vous pouvez suivre ce tutoriel pour installer ROS2 Jazzy.

### 🛠️ Installation
1. **Créez un workspace** :
   ```sh
   mkdir <nom_de_votre_workspace>
   ```
2. **Créez un dossier `src` dans votre workspace** :
   ```sh
   cd ~/<nom_de_votre_workspace>
   mkdir src
   ```
3. **placez le package "projet" dans votre workspace**

4. **placez le fichier warehouse.world dans le worksapce de tiago**:
   ```sh
   cd ~/tiago/src/br2_gazebo_worlds\worlds
   ```
4. **placez le contenu du dossier models dans le chemain suivant**:
   ```sh
   cd ~/tiago/src/br2_gazebo_worlds\models
   ```



Vous êtes dorénavant prêt à utiliser notre package.

---

### ▶️ Lancement du package

1. **Compilez le workspace br2_gazebo_worlds de tiago** :
   ```sh
   cd ~/tiago
   colcon build --symlink-install --packages-select br2_gazebo_worlds
   ```

2. **Compilez votre workspace** :
   ```sh
   cd ~/<nom_de_votre_workspace>
   colcon build --symlink-install
   ```
3. **Activez ROS2 dans votre workspace** :
   ```sh
   source install/setup.bash
   ```
4. **Ouvrez trois terminaux** (Utilisez `Ctrl + Alt + T` pour en ouvrir un).

#### Terminal 1 : Lancer le monde Gazebo
```sh
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=warehouse
```

#### Terminal 2 : Lancer le script de téléopération du robot
```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/key_vel
```

#### Terminal 3 : Lancer le comportement du robot
```sh
ros2 run projet box
```

### 🔍 Utilisation
À l'aide du terminal de téléopération, déplacez votre robot face à un code ArUco. Il devrait le détecter et s'en approcher automatiquement.

---

## 📂 Génération de la structure de fichiers pour les tags ArUco

Un script Python est fourni pour générer automatiquement la structure de fichiers nécessaire pour les modèles ArUco dans Gazebo.

### 📥 Installation
Assurez-vous d'avoir Python installé sur votre système.

### ▶️ Exécution
1. **Lancez le script en indiquant le numéro du tag ArUco souhaité** :
   ```sh
   python script.py <numéro_du_tag>
   ```
   Exemple :
   ```sh
   python script.py 110
   ```
   **ou**
   ```sh
   python3 script.py <numéro_du_tag>
   ```
   Exemple :
   ```sh
   python3 script.py 110
   ```

2. **Le script créera une structure de dossiers et de fichiers**, comprenant :
   - Un dossier `aruco_tag_<numéro>` contenant :
     - Un fichier `model.config` définissant le modèle.
     - Un fichier `model.sdf` décrivant la structure du modèle dans Gazebo.
     - Un dossier `materials/textures/` pour stocker les textures des tags.
     - Un dossier `meshes/` contenant les fichiers `.obj` et `.mtl` décrivant la géométrie du tag.

3. **Une fois le modèle généré, ajoutez-le à votre environnement Gazebo pour utilisation.**

