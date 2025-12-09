# alphabot2_door_detection (ROS2)

ROS2 package for **automatic door detection and search** with a Waveshare Alphabot2.

The idea:
- Use the Alphabot2 camera (`/camera/image_raw`) as input.
- Detect a **door marked with a red sheet** (or any chosen color).
- When the door is not seen ⇒ robot rotates to search.
- When the door is seen ⇒ robot turns to center it and approaches, then stops.

This package provides:
- `DoorDetection.msg` – simple message for detection result.
- `door_detector` node – OpenCV-based color detection of the “door”.
- `door_search` node – state machine that searches and drives toward the door.

## Dependencies

- ROS2 (tested with Humble / Foxy style).
- An Alphabot2 ROS2 driver that provides:
  - camera topic: `/camera/image_raw` (`sensor_msgs/Image`)
  - cmd_vel: `geometry_msgs/Twist` subscriber
  (e.g. https://github.com/Mik3Rizzo/alphabot2-ros2)

- Python libs:
  - `opencv-python`
  - `cv_bridge` (usually `sudo apt install ros-<distro>-cv-bridge`)

## Build

Inside a ROS2 workspace:

```bash
mkdir -p ~/alphabot2_ws/src
cd ~/alphabot2_ws/src
git clone https://github.com/<your-username>/alphabot2_door_finder.git
# also clone your Alphabot2 base driver here

cd ..
colcon build
source install/setup.bash
