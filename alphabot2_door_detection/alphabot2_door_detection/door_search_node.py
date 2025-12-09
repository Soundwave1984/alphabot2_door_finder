import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from alphabot2_door_detection.msg import DoorDetection


class DoorSearch(Node):
    """
    Simple controller:
      - If no door detection recently: rotate to search.
      - If door detected: turn to center it and move forward until "close".
    """

    def __init__(self):
        super().__init__('door_search')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(
            DoorDetection, '/door_detection', self.det_cb, 10)

        self.last_detection = None
        self.last_det_time = self.get_clock().now()

        self.search_angular_speed = 0.3
        self.approach_linear_speed = 0.1
        self.bearing_gain = 1.5
        self.close_distance_threshold = 0.01  # tune depending on detector

        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

    def det_cb(self, msg: DoorDetection):
        if msg.detected:
            self.last_detection = msg
            self.last_det_time = self.get_clock().now()

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_det_time).nanoseconds * 1e-9

        twist = Twist()

        if self.last_detection is None or dt > 1.0:
            # SEARCH: rotate slowly
            twist.angular.z = self.search_angular_speed
            twist.linear.x = 0.0
        else:
            # APPROACH: use bearing and distance
            bearing = self.last_detection.bearing
            twist.angular.z = -self.bearing_gain * bearing

            if self.last_detection.distance > self.close_distance_threshold:
                twist.linear.x = self.approach_linear_speed
            else:
                twist.linear.x = 0.0  # close enough

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = DoorSearch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
