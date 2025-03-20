import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoAndBoxDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_box_detector_node')
        self.bridge = CvBridge()

        # Souscription à la caméra
        self.subscription = self.create_subscription(
            Image,
            '/head_front_camera/image',
            self.listener_callback,
            10
        )

        self.get_logger().info("Abonné au topic /head_front_camera/image")

        # Publisher pour le contrôle des vitesses
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_muxed', 10)

        # Timer pour le contrôle du mouvement
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Paramètres de la caméra
        self.camera_matrix = np.array([[600, 0, 320], 
                                       [0, 600, 240], 
                                       [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)

        # Dictionnaire ArUco
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()

        # Stockage des positions détectées
        self.target_position = None
        self.box_count = 0  # Nombre de boîtes détectées

    def listener_callback(self, msg):
        """ Fonction appelée à chaque nouvelle image """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Erreur conversion image: {e}")
            return

        # Conversion en niveaux de gris pour ArUco
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            for i in range(len(ids)):
                marker_id = ids[i][0]
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corners[i], 0.05, self.camera_matrix, self.dist_coeffs
                )
                x, y, z = tvec[0][0]
                self.target_position = (x, y, z)

                cv2.putText(cv_image, f"ID: {marker_id}",
                            (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)

            self.get_logger().info(f"Marqueur détecté - Position : {self.target_position}")

        # 🎯 Détection des boîtes oranges sombres
        self.detect_boxes(cv_image)

        # Affichage de l'image
        cv2.imshow("Aruco + Box Detection", cv_image)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):  
            self.get_logger().info("Fermeture de la fenêtre")
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def detect_boxes(self, image):
        """ Détecte et dessine les contours des boîtes oranges sombres """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 🎨 Plage HSV pour l'orange sombre
        lower_color = np.array([5, 100, 50])   # H min, S min, V min
        upper_color = np.array([25, 255, 255]) # H max, S max, V max

        # Masque de la couleur
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Filtrage pour améliorer la détection
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        # Détection des contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.box_count = len(contours)  # Nombre de boîtes détectées

        for contour in contours:
            if cv2.contourArea(contour) > 500:  # Filtrer les petits objets
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Dessiner un rectangle

        self.get_logger().info(f"Nombre de boîtes détectées: {self.box_count}")

    def timer_callback(self):
        """ Fonction pour le mouvement du robot """
        twist = Twist()
        if self.target_position:
            x, y, z = self.target_position
            target_distance = 0.5
            distance_error = z - target_distance
            linear_speed = 0.2
            angular_speed = 0.3

            if abs(distance_error) > 0.1:
                twist.linear.x = min(linear_speed, max(-linear_speed, distance_error * 0.5))
            else:
                twist.linear.x = 0.0

            if abs(x) > 0.05:
                twist.angular.z = min(angular_speed, max(-angular_speed, -x * 2.0))
            else:
                twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

def main():
    rclpy.init()
    node = ArucoAndBoxDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
