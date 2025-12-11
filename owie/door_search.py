#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from alphabot2_door_detection.msg import DoorDetection
from waveshare_alphabot2.msg import Obstacle_Stamped
from waveshare_alphabot2.msg import RGB_LED

from enum import Enum
from random import random

class State(Enum):
    ROTATING = 0 # When we are rotating to look for the target
    TARGETING = 1 # when we detected target so should not be interrupted
    MOVING = 2 # When we are just moving forward

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
        # time counter required to check surrounding when going in a line
        self.move_timeout = rospy.get_param("~move_timeout", 2)
        # Forward speed when approaching
        self.approach_linear_speed = rospy.get_param("~approach_linear_speed", 0.02)
        # How strongly to turn based on bearing
        self.turn_gain = rospy.get_param("~turn_gain", 1.0)
        # How long a detection is considered "fresh" (seconds)
        self.detection_timeout = rospy.get_param("~detection_timeout", 0.6)

        self.last_detection = None
        self.current_timeout = None

        self.negativeRotate = False
        self.startRotating(1)

        self.obstacle_left = False
        self.obstacle_right = False

        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/door_detection", DoorDetection, self.detection_cb)
        rospy.Subscriber("/obstacle_left", Obstacle_Stamped, self.set_obstacle_left)
        rospy.Subscriber("/obstacle_right", Obstacle_Stamped, self.set_obstacle_right)

        self.pub_led = rospy.Publisher("/rgb_leds", RGB_LED, queue_size=10)

        # Run control loop at 10 Hz
        self.timer = rospy.Timer(rospy.Duration(0.1), self.update)

        rospy.loginfo("DoorSearch: controller started")

    def startRotating(self, spin):
        if self.state != State.ROTATING:
            self.state = State.ROTATING
            led = RGB_LED()
            led.function = 'setallred'

            # red = (255 << 16)
            # black = 0

            # left_col = black if self.negativeRotate else red
            # right_col = red if self.negativeRotate else black

            # led.led1_color = left_col
            # led.led2_color = left_col
            # led.led3_color = right_col
            # led.led4_color = right_col
            # led.function = 'setLED'
            self.pub_led.publish(led)
        
        self.negativeRotate = spin < 0
        if (self.negativeRotate):
            spin = -spin
        self.current_timeout = rospy.Time.now() + spin / self.search_angular_speed

    def detection_cb(self, msg):
        self.last_detection = msg
        if self.state != State.TARGETING:
            rospy.loginfo("Object spotted, confirmed to set to targetting")
            self.state = State.TARGETING

            led = RGB_LED()
            led.function = 'setallgreen'
            self.pub_led.publish(led)
        
        self.current_timeout = rospy.Time.now() + self.detection_timeout

    def set_obstacle_left(self, msg):
        self.obstacle_left = msg.obstacle
        if (self.obstacle_left or self.obstacle_right) and self.state != State.ROTATING:
            rotation = 0.75 if random() < 0.5 else 0.75
            rospy.loginfo("Obstacle detected on left, rotating %s", rotation)
            self.startRotating(rotation)
    
    def set_obstacle_right(self, msg):
        self.obstacle_left = msg.obstacle
        if (self.obstacle_left or self.obstacle_right) and self.state != State.ROTATING:
            rotation = 0.75 if random() < 0.5 else 0.75
            rospy.loginfo("Obstacle detected on left, rotating %s", rotation)
            self.startRotating(rotation)

    def update(self, event):
        cmd = Twist()
        cmd.angular.z = 0
        cmd.linear.x = 0
        now = rospy.Time.now()

        timingOut = now > self.current_timeout
        if self.state == State.ROTATING:
            if timingOut:
                if self.obstacle_left or self.obstacle_right:
                    self.startRotating(-0.25 if self.negativeRotate else 0.25)
                    rospy.loginfo("Rotated but still blocked, will keep rotating a quarter way")
                else:
                    self.state = State.MOVING
                    self.current_timeout = now + self.move_timeout
                    rospy.loginfo("rotating done, moving now")

                    led = RGB_LED()
                    led.function = 'setallblue'
                    self.pub_led.publish(led)
            else:
                cmd.angluar.z = (-1 if self.negativeRotate else 1) * self.search_angular_speed
        elif self.state == State.TARGETING:
            if timingOut:
                self.startRotating(1)
                rospy.loginfo("targetting timed out, back to searching")
            else:
                bearing = self.last_detection.bearing
                distance = self.last_detection.distance

                # Turn to center the door
                cmd.angular.z = -self.turn_gain * bearing

                # Always move forward when we "see" the door,
                # even if distance estimate is bad.
                cmd.linear.x = self.approach_linear_speed
        elif self.state == State.MOVING:
            if timingOut:
                self.startRotating(1)
                rospy.loginfo("moving welfare check")
            else:
                cmd.angular.z = -self.turn_gain * bearing
                cmd.linear.x = self.approach_linear_speed

        self.pub_cmd.publish(cmd)


def main():
    rospy.init_node("door_search")
    node = DoorSearch()
    rospy.spin()


if __name__ == "__main__":
    main()
