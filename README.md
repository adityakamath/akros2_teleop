# akros2_teleop
Package to teleoperate the AKROS2 robot.

* Uses the [joy](https://github.com/adityakamath/joystick_drivers/tree/ros2/joy) node with the [teleop_twist_joy](https://github.com/ros2/teleop_twist_joy) package to publish the status of the controller's buttons/joysticks, and Twist messages based on user-defined configuration in the config directory. The configuration currently supports [PS4](https://www.playstation.com/nl-nl/accessories/dualshock-4-wireless-controller/), [Stadia](https://stadia.google.com/controller/), [8BitDo SN30 Pro](https://www.8bitdo.com/sn30-pro-g-classic-or-sn30-pro-sn/) and [Steam Deck](https://store.steampowered.com/steamdeck) controllers. Similarly, mappings and mode/twist config files for other controllers can also be made. Then, the ```joy_config``` launch argument needs to be updated accordingly.
* Implements the ```joy_mode_handler``` node, which subscribes to the joystick status, and publishes [Mode](https://github.com/adityakamath/akros2_msgs/blob/master/msg/Mode.msg) messages to indicate ```stop```, ```auto```, or ```teleop```. Uses parameters `estop_button` and `auto_button`, which represent the mapping for the E-Stop and Auto/Teleop buttons for the joystick that is used. Joystick mappings and reference parameter values can be found in the ```config``` directory (```_mode_config.yaml```)
* Implements the ```twist_mixer``` node that subscribes to the Mode messages from ```ds4_feedback```, the Twist messages from ```ds4_twist``` and Twist messages from an external autonomous node. Based on the mode, it then publishes either the Twist message from the PS4 controller, or the autonomous node, or zero values if the emergency stop is pressed. The parameter `timer_period` can be updated in the launch file (Default: 0.01 seconds).
* Both the ```joy_mode_handler``` and ```twist_mixer``` nodes are composed into a single ```drive_node``` (multi-threaded) executable that is then launched from the launch file. Both nodes can also be run individually. 

## drive_launch.py
This is the main launch file that launches either the ```twist_mixer``` and ```joy_mode_handler``` separately or the ```drive_node``` executable that runs both nodes using a multi-threaded executor. This launch file uses the following launch arguments:

* ```joy```: Enable/Disable Joy related packages in the Joy launch file: [joy_launch.py](https://github.com/adityakamath/akros2_teleop/blob/humble/launch/joy_launch.py) (Default: ```True```)
* ```joy_config```: Configures joystick mapping: ```ps4``` ([PS4](https://www.playstation.com/nl-nl/accessories/dualshock-4-wireless-controller/)), ```stadia``` ([Stadia](https://stadia.google.com/controller/)), ```sn30pro``` ([8BitDo SN30 Pro](https://www.8bitdo.com/sn30-pro-g-classic-or-sn30-pro-sn/), ```steamdeck``` ([Steam Deck](https://store.steampowered.com/steamdeck): mapping is identical to PS3/Sixaxis controllers) (Default: ```steamdeck```)
* ```executor```: If True, runs ```drive_node```, else runs ```twist_mixer``` and ```joy_mode_handler``` separately. 

## joy_launch.py
This launch files launches ```joy``` and ```teleop_twist_joy``` nodes and uses the ```joy_config``` launch argument to configure the controller. This node can be launched individually or via ```drive_launch.py``` by setting the ```joy``` launch argument.
