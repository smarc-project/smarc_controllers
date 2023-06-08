#!/usr/bin/python3

import rospy
import numpy as np
import message_filters
from std_msgs.msg import Float64

class LookAheadDepthNode(object):

    def __init__(self):

        depth_top = rospy.get_param('~depth_top', '/yomama')
        pitch_top = rospy.get_param('~pitch_top', '/yomama')
        ahead_depth = rospy.get_param('~lookahead_depth', '/yomama')
        self.dist = rospy.get_param('~lookahead_dist', '/yomama')
       
        self.depth_ahead_pub = rospy.Publisher(ahead_depth, Float64, queue_size=10)

        # Connect
        self.depth_sub = message_filters.Subscriber(depth_top, Float64)
        self.pitch_sub = message_filters.Subscriber(pitch_top, Float64)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.depth_sub, self.pitch_sub],
                                                            10, slop=1.0, allow_headerless=True)
        self.ts.registerCallback(self.lookahead)

        rospy.loginfo("%s running ", rospy.get_name())

        rospy.spin()

    def lookahead(self, depth_msg, pitch_msg):

        depth_ahead = depth_msg.data - np.abs(self.dist) * np.sin(pitch_msg.data)

        self.depth_ahead_pub.publish(depth_ahead)


if __name__ == "__main__":
    rospy.init_node('lookahead_depth_node')
    try:
        LookAheadDepthNode()
    except rospy.ROSInterruptException:
        pass
