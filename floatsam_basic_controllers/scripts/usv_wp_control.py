#! /usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Int32, Bool, Float64
from smarc_msgs.msg import ThrusterRPM

class SimpleUSVController(object):

    def __init__(self):

        heading_sp_top = rospy.get_param('~heading_setpoint_topic', '/sam/ctrl/dynamic_heading/setpoint')
        heading_state_top = rospy.get_param('~heading_state_topic', '')
        heading_enable_top = rospy.get_param('~heading_enable_topic', '')
        rpm_enable_top = rospy.get_param("~rpm_enable_top", "rpm/enable")
        rpm_sp_top = rospy.get_param("~rpm_sp_top", "rpm/setpoint")
        rpm1_cmd_top = rospy.get_param("~rpm1_cmd_topic", "rpm/setpoint")
        rpm2_cmd_top = rospy.get_param("~rpm2_cmd_topic", "rpm/setpoint")
        self.d_thrusters = rospy.get_param("~d_between_thrusters", 0.5)
        self.heading_kp = rospy.get_param("~heading_kp", 1000)

        self.rpm_1_pub = rospy.Publisher(rpm1_cmd_top, ThrusterRPM, queue_size=1)
        self.rpm_2_pub = rospy.Publisher(rpm2_cmd_top, ThrusterRPM, queue_size=1)

        self.rpm_enable_sub = rospy.Subscriber(rpm_enable_top, Bool, self.rpm_enable_cb)
        self.rpm_sp_sub = rospy.Subscriber(rpm_sp_top, Int32, self.rpm_sp_cb)
        self.head_sp_sub = rospy.Subscriber(heading_sp_top, Float64, self.heading_sp_cb)
        self.head_enable_sub = rospy.Subscriber(heading_enable_top, Bool, self.heading_enable_cb)

        self.heading_enable = False
        self.rpm_enable = False
        self.stopped = False
        self.rpm_desired = 0.

        rospy.spin()

    def heading_enable_cb(self, enable):

        self.heading_enable = enable.data
    
    def rpm_enable_cb(self, enable):

        self.rpm_enable = enable.data

        if not self.rpm_enable and not self.stopped:

            rpm_prt = ThrusterRPM()
            rpm_stb = ThrusterRPM()
            rpm_prt.rpm = 0.
            rpm_stb.rpm = 0.

            self.rpm_1_pub.publish(rpm_prt)
            self.rpm_2_pub.publish(rpm_stb)
            self.stopped = True

    def rpm_sp_cb(self, rpm):

        self.rpm_desired = rpm.data

    def heading_sp_cb(self, heading_sp):

        rpm_prt = ThrusterRPM()
        rpm_stb = ThrusterRPM()
        rpm_prt.rpm = self.rpm_desired
        rpm_stb.rpm = self.rpm_desired

        if self.heading_enable and self.rpm_enable:
            
            v_dot = self.heading_kp * (heading_sp.data * self.d_thrusters)

            # Safety check for heading_kp vals
            v_dot = v_dot if v_dot < self.rpm_desired*0.8 else self.rpm_desired*0.8

            # If turning left, reduce port thrustering
            if np.sign(v_dot) == 1:
                rpm_prt.rpm -= int(np.abs(v_dot))
            # Otherwise, reduce starboard
            else:
                rpm_stb.rpm -= int(np.abs(v_dot))

            # Safety check
            rpm_prt.rpm = max(min(rpm_prt.rpm, self.rpm_desired), -self.rpm_desired)
            rpm_stb.rpm = max(min(rpm_stb.rpm, self.rpm_desired), -self.rpm_desired)

            self.rpm_1_pub.publish(rpm_prt)
            self.rpm_2_pub.publish(rpm_stb)
            self.stopped = False
          


if __name__ == '__main__':

    rospy.init_node('goto_wp_action_planner')
    wp_control = SimpleUSVController()
