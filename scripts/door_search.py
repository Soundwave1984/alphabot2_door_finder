#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from alphabot2_door_detection.msg import DoorDetection


class DoorSearch(object):
    """
    Simple controller:
      - If no door detection recently: rotate to search.
      - If door detected: turn to center it and move forward until "close".
    """

    def __init__(self):
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/door_detection', DoorDetection,
                                    self.det_cb, queue_size=10)

        self.last_detection = None
        self.last_det_time = rospy.Time(0)

        # tunable params
        self.search_angular_speed = rospy.get_param('~search_angular_speed', 0.3)
        self.approach_linear_speed = rospy.get_param('~approach_linear_speed', 0.1)
        self.bearing_gain = rospy.get_param('~bearing_gain', 1.5)
        self.close_distance_threshold = rospy.get_param('~close_distance_threshold', 0.01)
        self.timeout_no_detection = rospy.get_param('~timeout_no_detection', 1.0)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)  # 10 Hz

    def det_cb(self, msg):
        if msg.detected:
            self.last_detection = msg
            self.last_det_time = rospy.Time.now()

    def control_loop(self, event):
        now = rospy.Time.now()
        dt = (now - self.last_det_time).to_sec()

        twist = Twist()

        if self.last_detection is None or dt > self.timeout_no_detection:
            # SEARCH mode: rotate slowly
            twist.angular.z = self.search_angular_speed
            twist.linear.x = 0.0
        else:
            # APPROACH mode
            bearing = self.last_detection.bearing
            twist.angular.z = -self.bearing_gain * bearing

            if self.last_detection.distance > self.close_distance_threshold:
                twist.linear.x = self.approach_linear_speed
            else:
                twist.linear.x = 0.0  # close enough

        self.cmd_pub.publish(twist)


def main():
    rospy.init_node('door_search')
    node = DoorSearch()
    rospy.spin()


if __name__ == '__main__':
    main()
