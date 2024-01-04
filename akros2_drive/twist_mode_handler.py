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
from geometry_msgs.msg import Twist
from akros2_msgs.msg import Mode
from std_msgs.msg import Bool
from rclpy.executors import ExternalShutdownException

class TwistModeHandler(Node):
    def __init__(self, node_name='twist_mode_handler'):
        super().__init__(node_name)
        
        self.declare_parameter('timer_period', 0.01)
        self.declare_parameter('estop_button', 4) # defaults to Sixaxis/SteamDeck:L1
        self.declare_parameter('auto_button', 0) # defaults to Sixaxis/SteamDeck:A
        
        self._teleop = Twist()
        self._auto   = Twist()
        self._mixed  = Twist()
        self._mode   = Mode()
        
        self._zero   = Twist()
        self._zero.linear.x  = 0.0
        self._zero.linear.y  = 0.0
        self._zero.angular.z = 0.0
        
        self._prev = None
        self._mode.estop = False
        self._mode.auto_t = False
        
        self._estop_button = self.get_parameter('estop_button').value
        self._auto_button = self.get_parameter('auto_button').value
        
        self.create_subscription(Joy, 'joy', self.cb_joy, 1)
        self.create_subscription(Twist, 'teleop_vel', self.cb_teleop, 1)
        self.create_subscription(Twist, 'auto_vel', self.cb_auto, 1)
        self._pub_twist = self.create_publisher(Twist, 'mix_vel', 1)
        self._pub_mode = self.create_publisher(Mode, 'mode', 1)
        self.timer = self.create_timer(self.get_parameter('timer_period').value, self.cb_timer)
        
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
        
        if self._mode.estop:
            self._mode.auto_t = False
        
        self._prev = msg
        
    def cb_teleop(self, msg):
        """
        :type msg: Twist
        """
        self._teleop = msg
    
    def cb_auto(self, msg):
        """
        :type msg: Twist
        """
        self._auto = msg

    def cb_timer(self):
        if self._mode.estop:
            self._mixed = self._zero
        else:
            if self._mode.auto_t:
                self._mixed = self._auto if self._auto else self._zero
            else:
                self._mixed = self._teleop if self._teleop else self._zero

        self._pub_twist.publish(self._mixed)
        self._pub_mode.publish(self._mode)
        
def main(args=None):
    rclpy.init(args=args)
    node = TwistModeHandler()
    
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