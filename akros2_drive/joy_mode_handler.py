# Copyright (c) 2023 Aditya Kamath
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from akros2_msgs.msg import Mode
from std_msgs.msg import Bool
from rclpy.executors import ExternalShutdownException

class JoystickModeHandler(Node):
    def __init__(self, node_name='joy_mode_handler'):
        super().__init__(node_name)
        
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('mode_topic', 'mode')
        self.declare_parameter('estop_lock_topic', 'e_stop')
        self.declare_parameter('joystick', 'ps4')
        self.declare_parameter('estop_button', 9)  # defaults to PS4:L1 and Stadia:LJoy button
        self.declare_parameter('auto_button', 0) # defaults to PS4:X and Stadia:A
        
        self._prev = None
        self._mode = Mode()
        self._mode.estop  = False
        self._mode.auto_t = False
        
        self._joystick = self.get_parameter('joystick').value
        self._estop_button = self.get_parameter('estop_button').value
        self._auto_button = self.get_parameter('auto_button').value
        
        self.create_subscription(Joy, 
                                 self.get_parameter('joy_topic').value, 
                                 self.cb_joy, 1)
        self._pub_mode = self.create_publisher(Mode, self.get_parameter('mode_topic').value, 1)
        self._pub_estop_lock = self.create_publisher(Bool, self.get_parameter('estop_lock_topic').value, 1)
        
        self.get_logger().info('Initialized')

    def cb_joy(self, msg):
        """
        :type msg: Joy
        """
        if self._prev is None:
            self._prev = msg
            return
    
        if msg.buttons[self._estop_button] and not self._prev.buttons[self._estop_button]:
            self._mode.estop = not self._mode.estop
          
        if msg.buttons[self._auto_button] and not self._prev.buttons[self._auto_button]:
            self._mode.auto_t = not self._mode.auto_t
        
        if self._mode.estop: # STOP! - red
            self._mode.auto_t = False
        
        estop_lock = Bool()
        estop_lock.data = self._mode.estop
        
        self._pub_estop_lock.publish(estop_lock)
        self._pub_mode.publish(self._mode)
        
        self._prev = msg

def main(args=None):
    rclpy.init(args=args)
    node = JoystickModeHandler()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()