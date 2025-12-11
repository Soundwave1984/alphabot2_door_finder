#!/usr/bin/env python3
import math

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from alphabot2_door_detection.msg import DoorDetection


class DoorDetector(object):
    """
    Detects a 'door' marked with a red sheet in the camera image.

    Subscribes to a compressed image topic (sensor_msgs/CompressedImage)
    and publishes DoorDetection with bearing and a rough distance estimate.
    """

    def __init__(self):
        # image topic parameter; default to compressed image
        image_topic = rospy.get_param('~image_topic',
                                      '/camera/image/compressed')

        # minimum blob area to count as a door
        self.area_min = rospy.get_param('~area_min', 400)
        self.image_width = None

        self.pub = rospy.Publisher('/door_detection',
                                   DoorDetection,
                                   queue_size=10)

        # NOTE: we subscribe to CompressedImage here
        self.sub = rospy.Subscriber(image_topic,
                                    CompressedImage,
                                    self.image_cb,
                                    queue_size=1)
        rospy.loginfo("DoorDetector subscribing to: %s", image_topic)

    def image_cb(self, msg):
        # Decode JPEG/PNG byte data into an OpenCV BGR image
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if cv_img is None:
            rospy.logwarn("DoorDetector: failed to decode compressed image")
            return

        if self.image_width is None:
            self.image_width = cv_img.shape[1]

        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        # Red color thresholds in HSV (two ranges for wrap-around)
        lower_red1 = np.array([0, 80, 40])
        upper_red1 = np.array([15, 255, 255])
        lower_red2 = np.array([165, 80, 40])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Clean up noise
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask,
                                       cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        det = DoorDetection()
        det.header.stamp = rospy.Time.now()
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
                    fov_rad = math.radians(60.0)  # approx FOV
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
