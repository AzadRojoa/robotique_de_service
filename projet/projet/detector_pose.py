import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
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

        # Stockage des positions détectées
        self.detected_positions = {}

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
                marker_id = ids[i][0]  # ID du marqueur

                # Estimer la position du marqueur
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corners[i], self.marker_size, self.camera_matrix, self.dist_coeffs
                )

                # Extraire la position en x, y, z
                x, y, z = tvec[0][0]

                # Sauvegarder la position du marqueur par rapport au robot
                self.detected_positions[marker_id] = (x, y, z)

                # Afficher la position sur l'image
                cv2.putText(cv_image, f"ID: {marker_id}",
                            (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Dessiner l'axe du marqueur
                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)

            # Afficher les positions détectées dans le terminal
            self.get_logger().info(f"Positions détectées:")
            for marker_id in sorted(self.detected_positions.keys()):
                x, y, z = self.detected_positions[marker_id]
                self.get_logger().info(f"- ID {marker_id}: (x={x:.6f} ; y={y:.6f} ; z={z:.6f})")


        # Afficher l'image avec les marqueurs détectés
        cv2.imshow("Aruco Detection", cv_image)
        cv2.waitKey(1)

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
