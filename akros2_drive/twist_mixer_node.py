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
from rclpy.node import Node
from geometry_msgs.msg import Twist
from akros2_msgs.msg import Mode

class TwistMixer(Node):
    def __init__(self):
        super().__init__('twist_mixer')
        
        self.declare_parameter('auto_vel_topic', 'auto_vel')
        self.declare_parameter('teleop_vel_topic', 'teleop_vel')
        self.declare_parameter('mix_vel_topic', 'mix_vel')
        self.declare_parameter('mode_topic', 'mode')
        self.declare_parameter('timer_period', 0.01)
        self.declare_parameter('mix_vel_buffer', 1)
        
        self._teleop = Twist()
        self._auto   = Twist()
        self._mixed  = Twist()
        self._mode   = Mode()
        
        self._zero   = Twist()
        self._zero.linear.x  = 0.0
        self._zero.linear.y  = 0.0
        self._zero.angular.z = 0.0
        
        self.create_subscription(Mode, 
                                 self.get_parameter('mode_topic').value, 
                                 self.cb_mode, 1)
        self.create_subscription(Twist, 
                                 self.get_parameter('teleop_vel_topic').value, 
                                 self.cb_teleop, 1)
        self.create_subscription(Twist, 
                                 self.get_parameter('auto_vel_topic').value, 
                                 self.cb_auto, 1)
        self._pub_twist = self.create_publisher(Twist, 
                                                self.get_parameter('mix_vel_topic').value, 
                                                self.get_parameter('mix_vel_buffer').value)
        self.timer = self.create_timer(self.get_parameter('timer_period').value, self.cb_timer)
        
    def cb_mode(self, msg):
        """
        :type msg: Mode
        """
        self._mode = msg
        
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
        
def main(args=None):
    rclpy.init()
    node = TwistMixer()
    
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