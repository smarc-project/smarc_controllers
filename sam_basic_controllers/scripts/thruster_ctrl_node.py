#!/usr/bin/python

import rospy
import numpy as np
import message_filters
# from sbg_driver.msg import SbgEkfEuler
from sam_msgs.msg import ThrusterAngles
from std_msgs.msg import Float64


class ThrustCtrlNode(object):

    def __init__(self):
        self.heading_effort = rospy.get_param('~rudder_pid_effort', '/yomama')
        self.elevator_effort = rospy.get_param('~elevator_pid_effort', '/yomama')
        self.thrust_cmd_top = rospy.get_param('~thrust_cmd_topic', '/yomama')
       
        # Connect
        self.head_effort_sub = message_filters.Subscriber(self.heading_effort, Float64)
        self.elev_effort_sub = message_filters.Subscriber(self.elevator_effort, Float64)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.head_effort_sub, self.elev_effort_sub],
                                                                20, slop=20.0, allow_headerless=True)
        self.ts.registerCallback(self.thrust_cb)

        self.thrust_pub = rospy.Publisher(self.thrust_cmd_top, ThrusterAngles, queue_size=100)

        rospy.spin()

    def thrust_cb(self, heading_msg, elev_msg):
        cmd = ThrusterAngles()
        cmd.header.stamp = rospy.Time.now()
        cmd.thruster_horizontal_radians = heading_msg.data
        cmd.thruster_vertical_radians = elev_msg.data

        self.thrust_pub.publish(cmd)


if __name__ == "__main__":
    rospy.init_node('thrust_ctrl_node')
    try:
        ThrustCtrlNode()
    except rospy.ROSInterruptException:
        pass
