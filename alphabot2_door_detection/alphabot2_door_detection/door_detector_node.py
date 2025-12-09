import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from alphabot2_door_detection.msg import DoorDetection


class DoorDetector(Node):
    """
    Detects a 'door' marked with a colored sheet (default: red) in the camera image.
    Publishes DoorDetection with bearing and a rough distance estimate.
    """

    def __init__(self):
        super().__init__('door_detector')
        self.bridge = CvBridge()
        self.image_width = None

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('area_min', 1000)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.area_min = self.get_parameter('area_min').get_parameter_value().integer_value

        self.sub = self.create_subscription(Image, image_topic, self.image_cb, 10)
        self.pub = self.create_publisher(DoorDetection, '/door_detection', 10)

        self.get_logger().info(f'Subscribing to {image_topic}')

    def image_cb(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.image_width is None:
            self.image_width = cv_img.shape[1]

        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        # red color thresholds in HSV
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        det = DoorDetection()
        det.header = msg.header
        det.detected = False
        det.bearing = 0.0
        det.distance = -1.0

        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > self.area_min:
                x, y, w, h = cv2.boundingRect(c)
                cx = x + w / 2.0

                if self.image_width:
                    error_px = cx - self.image_width / 2.0
                    fov_rad = math.radians(60.0)  # approximate camera FOV
                    det.bearing = (error_px / self.image_width) * fov_rad

                # simple distance proxy: larger height => closer
                det.distance = 1.0 / max(float(h), 1.0)
                det.detected = True

        self.pub.publish(det)


def main(args=None):
    rclpy.init(args=args)
    node = DoorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
