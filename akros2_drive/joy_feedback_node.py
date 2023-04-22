# Copyright (c) 2022 Aditya Kamath
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
from sensor_msgs.msg import Joy
from akros2_msgs.msg import Mode
from std_msgs.msg import Bool

class JoystickHandler(object):
    def __init__(self, node, joy_topic='joy', mode_topic='mode', estop_lock_topic='e_stop'):
        self._node = node
        
        self._prev = None

        self._mode = Mode()
        self._mode.estop  = False
        self._mode.auto_t = False
        
        self._node.declare_parameter('joystick', 'ps4')
        self._node.declare_parameter('estop_button', 9)  # defaults to PS4:L1 and Stadia:LJoy button
        self._node.declare_parameter('auto_button', 0) # defaults to PS4:X and Stadia:A
        
        self._joystick = self._node.get_parameter('joystick').value
        self._estop_button = self._node.get_parameter('estop_button').value
        self._auto_button = self._node.get_parameter('auto_button').value
        
        self._node.create_subscription(Joy, joy_topic, self.cb_joy, 1)
        self._pub_mode = self._node.create_publisher(Mode, mode_topic, 1)
        self._pub_estop_lock = self._node.create_publisher(Bool, estop_lock_topic, 1)

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

def main():
    rclpy.init()
    node = rclpy.create_node("joystick_mode")
    JoystickHandler(node)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()