import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco  # Importation du module ArUco d'OpenCV
import numpy as np

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        self.bridge = CvBridge()
        
        # Souscription au topic de la caméra
        self.subscription = self.create_subscription(
            Image,
            '/head_front_camera/image',  # Assurez-vous que le topic est correct
            self.listener_callback,
            10
        )
        self.subscription

        # Liste des dictionnaires ArUco supportés
        self.ARUCO_DICT = {
            "DICT_4X4_50": aruco.DICT_4X4_50,
            "DICT_4X4_100": aruco.DICT_4X4_100,
            "DICT_4X4_250": aruco.DICT_4X4_250,
            "DICT_4X4_1000": aruco.DICT_4X4_1000,
            "DICT_5X5_50": aruco.DICT_5X5_50,
            "DICT_5X5_100": aruco.DICT_5X5_100,
            "DICT_5X5_250": aruco.DICT_5X5_250,
            "DICT_5X5_1000": aruco.DICT_5X5_1000,
            "DICT_6X6_50": aruco.DICT_6X6_50,
            "DICT_6X6_100": aruco.DICT_6X6_100,
            "DICT_6X6_250": aruco.DICT_6X6_250,
            "DICT_6X6_1000": aruco.DICT_6X6_1000,
            "DICT_7X7_50": aruco.DICT_7X7_50,
            "DICT_7X7_100": aruco.DICT_7X7_100,
            "DICT_7X7_250": aruco.DICT_7X7_250,
            "DICT_7X7_1000": aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": aruco.DICT_ARUCO_ORIGINAL,
        }

    def listener_callback(self, msg):
        # Conversion de l'image ROS en OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la conversion de l'image: {e}")
            return
        
        # Convertir en niveaux de gris
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        detected_markers = []  # Stocker les marqueurs détectés
        
        #Binariser l'image
        ret, binary_image = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        
        # Tester chaque dictionnaire ArUco
        for dict_name, dict_type in self.ARUCO_DICT.items():
            aruco_dict = aruco.getPredefinedDictionary(dict_type)
            parameters = aruco.DetectorParameters_create()
            
            # Détection des marqueurs
            corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            
            # Si des marqueurs sont détectés
            if ids is not None:
                aruco.drawDetectedMarkers(cv_image, corners, ids)
                for i, marker_id in enumerate(ids):
                    detected_markers.append((marker_id[0], dict_name))
                    
                    # Tracer un cercle au centre du marqueur
                    corner_points = corners[i][0]
                    center_x = int(np.mean(corner_points[:, 0]))
                    center_y = int(np.mean(corner_points[:, 1]))
                    cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)

                    # # Afficher l'ID du marqueur détecté
                    # cv2.putText(cv_image, f"{marker_id[0]}",
                    #             (center_x - 50, center_y - 10),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Affichage des marqueurs détectés
        if detected_markers:
            self.get_logger().info(f"Marqueurs détectés: {detected_markers}")

        # Afficher l'image avec les marqueurs
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
