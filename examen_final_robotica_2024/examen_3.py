import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from .ArucoDistanceEstimator import ArUcoDistanceEstimator


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Int32, 'action', 10)
        self.br = CvBridge()
        self.estimator = ArUcoDistanceEstimator()

    def listener_callback(self, msg):
        self.get_logger().info('Receiving image frame')
        current_frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # pipeline
        action = self.estimator.process_frame_distance_ids(current_frame, 5)
        current_frame = self.estimator.process_frame(current_frame)

        msg = Int32()
        msg.data = int(action)
        self.publisher_.publish(msg)
        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
