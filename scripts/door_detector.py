#!/usr/bin/env python3
import math

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from alphabot2_door_detection.msg import DoorDetection


class DoorDetector(object):
    """
    Detects a 'door' marked with a colored (red) sheet in the camera image.
    Publishes DoorDetection with bearing and a rough distance estimate.
    """

    def __init__(self):
        self.bridge = CvBridge()
        self.image_width = None

        image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        self.area_min = rospy.get_param('~area_min', 1000)

        self.pub = rospy.Publisher('/door_detection', DoorDetection, queue_size=10)
        self.sub = rospy.Subscriber(image_topic, Image, self.image_cb, queue_size=1)

        rospy.loginfo("DoorDetector subscribing to %s", image_topic)

    def image_cb(self, msg):
        # Convert to OpenCV image
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

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

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
                    fov_rad = math.radians(60.0)  # approximate FOV
                    det.bearing = (error_px / float(self.image_width)) * fov_rad

                det.distance = 1.0 / max(float(h), 1.0)
                det.detected = True

        self.pub.publish(det)


def main():
    rospy.init_node('door_detector')
    node = DoorDetector()
    rospy.spin()


if __name__ == '__main__':
    main()
