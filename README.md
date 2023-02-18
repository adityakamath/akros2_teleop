# akros2_drive
Package to drive the AKROS2 robot

* Uses the [joy](https://github.com/adityakamath/joystick_drivers/tree/ros2/joy) node with the [teleop_twist_joy](https://github.com/ros2/teleop_twist_joy) package to publish the status of the controller's buttons/joysticks, and Twist messages based on user-defined configuration in the config directory. The configuration supports Stadia, PS4 and PS3/Sixaxis controllers.
* Implements the ```joy_feedback``` node, which subscribes to the joystick status, and publishes [Mode](https://github.com/adityakamath/akros2_msgs/blob/master/msg/Mode.msg) messages to indicate ```stop```, ```auto```, or ```teleop```. It also publishes the ```stop``` signal as a separate boolean message. This node supports Stadia, PS4 and PS3/Sixaxis controllers.
* Implements the ```twist_mixer``` node that subscribes to the Mode messages from ```ds4_feedback```, the Twist messages from ```ds4_twist``` and Twist messages from an external autonomous node. Based on the mode, it then publishes either the Twist message from the PS4 controller, or the autonomous node, or zero values if the emergency stop is pressed.
* Uses the [twist_mux](https://github.com/ros-teleop/twist_mux) package to implement a multiplexer, that switches between the Twist messages from the ```twist_mixer``` and external teleop nodes, based on a user-defined priority. Also uses the boolean message published from the ```joy_feedback``` node, to add a redundant e-stop functionality.
* Launches the [micro-ROS Agent](https://github.com/micro-ROS/micro-ROS-Agent) with the transport and port number, to connect to the microcontroller (a Teensy 4.1 board). The Teensy is running the [akros2_firmware](https://github.com/adityakamath/akros2_firmware), which is based on [linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware). The Teensy subscribes to Twist messages from the ```twist_mixer``` node, Mode messages from the ```ds4_feedback``` node, and publishes IMU, Odometry and [Feedback](https://github.com/adityakamath/akros2_msgs/blob/master/msg/Feedback.msg) messages for PID tuning.
