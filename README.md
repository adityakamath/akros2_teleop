# akros2_drive
Package to drive the AKROS2 robot

* Uses the [joy](https://github.com/adityakamath/joystick_drivers/tree/ros2/joy) node with the [teleop_twist_joy](https://github.com/ros2/teleop_twist_joy) package to publish the status of the controller's buttons/joysticks, and Twist messages based on user-defined configuration in the config directory. The configuration currently supports [PS3/Sixaxis](https://en.wikipedia.org/wiki/Sixaxis), [PS4](https://www.playstation.com/nl-nl/accessories/dualshock-4-wireless-controller/), [Stadia](https://stadia.google.com/controller/) and [8BitDo SN30 Pro](https://www.8bitdo.com/sn30-pro-g-classic-or-sn30-pro-sn/) controllers. Additionally, the config directory also provides the mappings for the [Thrustmaster USB Joystick](https://www.thrustmaster.com/nl-be/products/usb-joystick/) controller, which can be used to create mode/twist config files. Then, the ```joy_config``` launch argument needs to be updated.
* Implements the ```joy_mode_handler``` node, which subscribes to the joystick status, and publishes [Mode](https://github.com/adityakamath/akros2_msgs/blob/master/msg/Mode.msg) messages to indicate ```stop```, ```auto```, or ```teleop```.
* Implements the ```twist_mixer``` node that subscribes to the Mode messages from ```ds4_feedback```, the Twist messages from ```ds4_twist``` and Twist messages from an external autonomous node. Based on the mode, it then publishes either the Twist message from the PS4 controller, or the autonomous node, or zero values if the emergency stop is pressed.
* Both the ```joy_mode_handler``` and ```twist_mixer``` nodes are composed into a single ```drive_node``` (single-threaded) executable that is then launched from the launch file. Both nodes can also be run individually.

## drive_launch.py
This is the main launch file and has the following launch arguments:

* ```joy```: Enable/Disable Joy related packages in the Joy launch file: [joy_launch.py](https://github.com/adityakamath/akros2_drive/blob/humble/launch/joy_launch.py) (Default: ```True```)
* ```joy_config```: Configures joystick mapping: ```ps3``` ([PS3/Sixaxis](https://en.wikipedia.org/wiki/Sixaxis)), ```ps4``` ([PS4](https://www.playstation.com/nl-nl/accessories/dualshock-4-wireless-controller/), ```stadia``` ([Stadia](https://stadia.google.com/controller/)), ```sn30pro``` ([8BitDo SN30 Pro](https://www.8bitdo.com/sn30-pro-g-classic-or-sn30-pro-sn/)) (Default: ```sn30pro```)

## joy_launch.py
This launch files launches the ```joy```, ```teleop_twist_joy``` and ```joy_mode_handler``` nodes and uses the ```joy_config``` launch argument to configure the controller. This node can be launched individually or from within ```drive_launch.py```.

## drive_node_launch.py
This launch file runs the ```drive_node``` executable as well as related packages - ```joy``` and ```teleop_twist_joy```. The only launch argument needed is the ```joy_config```, which is used to select a controller configuration.