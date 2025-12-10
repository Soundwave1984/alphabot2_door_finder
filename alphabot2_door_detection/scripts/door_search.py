#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from alphabot2_door_detection.msg import DoorDetection


class DoorSearch(object):
    """
    Very simple behavior:
      - If we have a recent detection with detected=True:
           turn toward the door and drive forward.
      - Otherwise:
           spin in place to search.
    """

    def __init__(self):
        # Spin speed when searching
        self.search_angular_speed = rospy.get_param("~search_angular_speed", 0.2)
        # Forward speed when approaching
        self.approach_linear_speed = rospy.get_param("~approach_linear_speed", 0.06)
        # How strongly to turn based on bearing
        self.turn_gain = rospy.get_param("~turn_gain", 1.0)
        # How long a detection is considered "fresh" (seconds)
        self.detection_timeout = rospy.get_param("~detection_timeout", 0.6)

        self.last_detection = None
        self.last_detection_time = None

        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/door_detection", DoorDetection, self.detection_cb)

        # Run control loop at 10 Hz
        self.timer = rospy.Timer(rospy.Duration(0.1), self.update)

        rospy.loginfo("DoorSearch: controller started")

    def detection_cb(self, msg):
        self.last_detection = msg
        self.last_detection_time = rospy.Time.now()

    def update(self, event):
        cmd = Twist()
        now = rospy.Time.now()

        use_detection = False
        if (
            self.last_detection is not None
            and self.last_detection_time is not None
            and (now - self.last_detection_time).to_sec() < self.detection_timeout
            and self.last_detection.detected
        ):
            use_detection = True

        if use_detection:
            bearing = self.last_detection.bearing
            distance = self.last_detection.distance

            # Turn to center the door
            cmd.angular.z = -self.turn_gain * bearing

            # Always move forward when we "see" the door,
            # even if distance estimate is bad.
            cmd.linear.x = self.approach_linear_speed

            rospy.loginfo_throttle(
                1.0,
                "APPROACH: bearing=%.3f, distance=%.3f"
                % (bearing, distance),
            )
        else:
            # No recent detection -> search by spinning in place
            cmd.angular.z = self.search_angular_speed
            cmd.linear.x = 0.0
            rospy.loginfo_throttle(1.0, "SEARCH: spinning")

        self.pub_cmd.publish(cmd)


def main():
    rospy.init_node("door_search")
    node = DoorSearch()
    rospy.spin()


if __name__ == "__main__":
    main()
