#!/usr/bin/env python3

import rclpy
from ds4_driver.msg import Feedback, Status
from akros2_msgs.msg import Mode
from std_msgs.msg import Bool

class FeedbackHandler(object):
    def __init__(self, node, status_topic='status', feedback_topic='set_feedback', mode_topic='mode', estop_lock_topic='e_stop'):
        self._node = node
        
        self._min_interval  = 0.05
        self._last_pub_time = self._node.get_clock().now()
        
        self._prev = Status()
        self._led = {
            'r': 0.0,
            'g': 1.0,
            'b': 1.0,
            'flashing': False,
        }
        
        self._mode = Mode()
        self._mode.estop  = False
        self._mode.auto_t = False
        
        self._rumble     = False
        self._playback   = 0.0
        self._rumble_val = 0.0
        
        self._node.create_subscription(Status, status_topic, self.cb_status, 1)
        self._pub_feedback = self._node.create_publisher(Feedback, feedback_topic, 1)
        self._pub_mode = self._node.create_publisher(Mode, mode_topic, 1)
        self._pub_estop_lock = self._node.create_publisher(Bool, estop_lock_topic, 1)

    def cb_status(self, msg):
        """
        :type msg: Status
        """
        now = self._node.get_clock().now()
        if (now - self._last_pub_time) < rclpy.duration.Duration(seconds=self._min_interval): return

        feedback = Feedback()
        
        if msg.button_circle and not self._prev.button_circle:
            self._mode.estop = not self._mode.estop
            
        if msg.button_ps and not self._prev.button_ps:
            self._rumble = not self._rumble
            
        if msg.button_cross and not self._prev.button_cross:
            self._mode.auto_t = not self._mode.auto_t

        feedback.set_led = True
        
        if self._mode.estop: # STOP! - red
            self._rumble = False
            self._mode.auto_t = False
            self._led['r'] = 1.0
            self._led['g'] = 0.0
            self._led['b'] = 0.0
        else:
            if self._mode.auto_t: # AUTO - blue
                self._led['r'] = 0.0
                self._led['g'] = 55/255
                self._led['b'] = 1.0                     
                    
            else: # TELEOP - green
                self._led['r'] = 0.0
                self._led['g'] = 1.0
                self._led['b'] = 55/255
                    
        if self._rumble:
            feedback.set_rumble   = True
            feedback.rumble_small = (abs(msg.axis_left_y) + abs(msg.axis_left_x) + abs(msg.axis_right_x) + abs(msg.button_dpad_up - msg.button_dpad_down) + abs(msg.button_dpad_left - msg.button_dpad_right)) / 4
        else:
            feedback.set_rumble   = False
            feedback.rumble_small = 0.0
                
        feedback.led_r = self._led['r']
        feedback.led_g = self._led['g']
        feedback.led_b = self._led['b']
        
        estop_lock = Bool()
        estop_lock.data = self._mode.estop
        
        self._pub_estop_lock.publish(estop_lock)
        self._pub_mode.publish(self._mode)
        self._pub_feedback.publish(feedback)
        
        self._prev = msg
        self._last_pub_time = now

def main():
    rclpy.init()
    node = rclpy.create_node("ds4_feedback")
    FeedbackHandler(node)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()