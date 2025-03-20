import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        self.bridge = CvBridge()

        # Souscription à la caméra du robot
        self.subscription = self.create_subscription(
            Image,
            '/head_front_camera/image',
            self.listener_callback,
            10
        )

        # Publisher pour le contrôle des vitesses
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_muxed', 10)

        # Timer pour exécuter le contrôle du mouvement périodiquement
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        # Paramètres de la caméra (matrice intrinsèque et distorsion)
        self.camera_matrix = np.array([[600, 0, 320],  # fx, 0, cx
                                       [0, 600, 240],  # 0, fy, cy
                                       [0, 0, 1]], dtype=np.float32)  # 0, 0, 1
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)  # Supposons aucune distorsion

        # Taille du marqueur (à adapter selon tes codes ArUco en mètres)
        self.marker_size = 0.05  # 5 cm

        # Dictionnaire ArUco utilisé
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()

        # Stockage de la position du dernier marqueur détecté
        self.target_position = None  # (x, y, z)
        
        #Définir l'état initial
        self.state = "APPROCHE_150CM"
        
        # Vitesse de déplacement
        self.linear_speed = 0.2  # m/s max
        self.angular_speed = 0.3  # rad/s max
        
        self.marker_id = None
        

    def listener_callback(self, msg):
        """ Fonction appelée à chaque nouvelle image """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Erreur conversion image: {e}")
            return

        # Convertir en niveaux de gris pour la détection ArUco
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Détection des marqueurs
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            for i in range(len(ids)):
                self.marker_id = ids[i][0]  # ID du marqueur

                # Estimer la position du marqueur
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corners[i], self.marker_size, self.camera_matrix, self.dist_coeffs
                )

                # Extraire la position en x, y, z
                x, y, z = tvec[0][0]

                # Mettre à jour la position cible
                self.target_position = (x, y, z)

                # Afficher la position sur l'image
                cv2.putText(cv_image, f"ID: {self.marker_id}",
                            (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Dessiner l'axe du marqueur
                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)

            # Afficher les positions détectées dans le terminal
            x, y, z = self.target_position
            self.get_logger().info(f"Marqueur détecté - Position : (x={x:.2f}, y={y:.2f}, z={z:.2f})")

        # Afficher l'image avec les marqueurs détectés
        cv2.imshow("Aruco Detection", cv_image)
        cv2.waitKey(1)

    def timer_callback(self):
        """ Fonction appelée à chaque tick du timer pour ajuster la vitesse du robot """
        twist = Twist()
        
        if self.state == "APPROCHE_150CM":
            
            self.get_logger().info("Phase: APPROCHE_150CM")

            # Vérifier si un marqueur a été détecté
            if self.target_position:
                x, y, z = self.target_position

                # Distance cible (40 cm devant le robot)
                target_distance = 1.5  # 40 cm
                distance_error = z - target_distance  # Différence entre la distance actuelle et la distance cible

                # Vitesse de déplacement
                linear_speed = 0.2  # m/s max
                angular_speed = 0.3  # rad/s max

                # Ajustement de la vitesse linéaire
                if abs(distance_error) > 0.1:  # Tolérance de 10 cm
                    twist.linear.x = min(linear_speed, max(-linear_speed, distance_error * 0.5))  # PID simple
                else:
                    twist.linear.x = 0.0  # Arrêt si assez proche
                    self.state = "SCAN_CODES"
                    self.get_logger().info("Arrivé au point de scan.")
                        
                # Ajustement de la rotation pour s'aligner avec le marqueur
                if abs(x) > 0.05:  # Si l'erreur latérale est significative
                    twist.angular.z = min(angular_speed, max(-angular_speed, -x * 2.0))  # Correction latérale
                else:
                    twist.angular.z = 0.0  # Arrêt de la rotation si bien aligné

                #self.get_logger().info(f"Commandes: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}")

            # Publier la commande de vitesse
            self.cmd_vel_pub.publish(twist)
            
        elif self.state == "SCAN_CODES":
            self.get_logger().info("Phase: SCAN_CODES")
            if len(self.target_position) > 1:
                self.aruco_queue = sorted([self.marker_id])  # Trier les ID
                self.get_logger().info(f"Liste des ArUco à visiter: {self.aruco_queue}")
                self.current_target = self.aruco_queue.pop(0)  # Premier code
                self.state = "APPROCHE_50CM"
            else:
                self.get_logger().info("Pas assez de codes détectés pour continuer.")
                
        elif self.state == "APPROCHE_50CM":
            self.get_logger().info("Phase: APPROCHE_50CM")
            if self.current_target in self.target_position:
                target_x, target_y, target_z = self.target_position[self.current_target]
                distance = math.sqrt(target_x**2 + target_y**2)

                if distance > 0.5:  # S'il est à plus de 50 cm
                    twist.linear.x = self.linear_speed  # Avancer
                else:
                    self.get_logger().info(f"Arrivé à 50cm de l'ArUco {self.current_target}")
                    if self.aruco_queue:
                        self.current_target = self.aruco_queue.pop(0)  # Prochain code
                    else:
                        self.state = "RETOUR_BASE"

def main():
    rclpy.init()
    node = ArucoDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
