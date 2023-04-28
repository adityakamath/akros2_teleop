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
#from time import time
from geometry_msgs.msg import Twist
from akros2_msgs.msg import Mode

class TwistMixer(object):
    def __init__(self, node, auto_vel_topic='auto_vel', teleop_vel_topic='teleop_vel', mix_vel_topic = 'mix_vel', mode_topic='mode', timer_period=0.01, teleop_timeout=1.0, auto_timeout=1.0):
        self._node = node
        
        self._teleop = Twist()
        self._auto   = Twist()
        self._mixed  = Twist()
        self._mode   = Mode()
        
        self._zero   = Twist()
        self._zero.linear.x  = 0.0
        self._zero.linear.y  = 0.0
        self._zero.angular.z = 0.0
        
        #self._teleop_time = self._auto_time = None
        #self._teleop_timeout = teleop_timeout
        #self._auto_timeout = auto_timeout
        
        self._node.create_subscription(Mode, mode_topic, self.cb_mode, 1)
        self._node.create_subscription(Twist, teleop_vel_topic, self.cb_teleop, 1)
        self._node.create_subscription(Twist, auto_vel_topic, self.cb_auto, 1)
        self._pub_twist = self._node.create_publisher(Twist, mix_vel_topic, 1)
        self.timer = self._node.create_timer(timer_period, self.cb_timer)
        
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
        #self._teleop_time = time()
    
    def cb_auto(self, msg):
        """
        :type msg: Twist
        """
        self._auto = msg
        #self._auto_time = time()

    def cb_timer(self):
        if self._mode.estop:
            self._mixed = self._zero
        else:
            if self._mode.auto_t:
                #if self._auto and (self._auto_time is None or (time() - self._auto_time) <= self._auto_timeout):
                #    self._mixed = self._auto
                #else:
                #    self._mixed = self._zero
                self._mixed = self._auto if self._auto else self._zero
            else:
                #if self._teleop and (self._teleop_time is None or (time() - self._teleop_time) <= self._teleop_timeout):
                #    self._mixed = self._teleop
                #else:
                #    self._mixed = self._zero
                self._mixed = self._teleop if self._teleop else self._zero

        self._pub_twist.publish(self._mixed)
        
def main(args=None):
    rclpy.init()
    node = rclpy.create_node("twist_mixer")
    TwistMixer(node)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()