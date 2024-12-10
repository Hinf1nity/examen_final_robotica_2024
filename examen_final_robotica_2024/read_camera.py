import cv2
import rclpy
from custom_msgs.msg import Box, Corner
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.listener_callback,
            10)

        self.publisher_ = self.create_publisher(Image, 'processed_image', 10)
        self.publisher_box = self.create_publisher(Box, 'data', 10)
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info('Receiving image frame')
        current_frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # pipeline

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        arucoParameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, arucoParameters)
        corners, ids, rejectedImgPoints = detector.detectMarkers(current_frame)
        current_frame = cv2.aruco.drawDetectedMarkers(
            current_frame, corners, ids)

        # centro de aruco tag y publicar
        if ids is not None:
            msg = Box()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            for i in range(len(ids)):
                msg.header.frame_id = str(ids[i][0])
                x = float((corners[i][0][0][0] + corners[i][0][2][0]) / 2)
                y = float((corners[i][0][0][1] + corners[i][0][2][1]) / 2)
                msg.centre = Corner(x=x, y=y)
                msg.corners = [
                    Corner(x=float(corners[i][0][0][0]),
                           y=float(corners[i][0][0][1])),
                    Corner(x=float(corners[i][0][1][0]),
                           y=float(corners[i][0][1][1])),
                    Corner(x=float(corners[i][0][2][0]),
                           y=float(corners[i][0][2][1])),
                    Corner(x=float(corners[i][0][3][0]),
                           y=float(corners[i][0][3][1]))
                ]
                self.publisher_box.publish(msg)
        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)

        # publicando imagenes
        processed_image_msg = self.br.cv2_to_imgmsg(
            current_frame, encoding='bgr8')
        self.publisher_.publish(processed_image_msg)


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
