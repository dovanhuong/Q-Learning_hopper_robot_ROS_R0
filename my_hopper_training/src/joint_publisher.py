#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from std_msgs.msg import Float64


class JointPub(object):
    def __init__(self):

        self.publishers_array = []
        self._haa_joint_pub = rospy.Publisher('/monoped/haa_joint_position_controller/command', Float64, queue_size=1)
        self._hfe_joint_pub = rospy.Publisher('/monoped/hfe_joint_position_controller/command', Float64, queue_size=1)
        self._kfe_joint_pub = rospy.Publisher('/monoped/kfe_joint_position_controller/command', Float64, queue_size=1)

        self.publishers_array.append(self._haa_joint_pub)
        self.publishers_array.append(self._hfe_joint_pub)
        self.publishers_array.append(self._kfe_joint_pub)

        self.init_pos = [0.0, 0.0, 0.0]

    def set_init_pose(self):
        """
        Sets joints to initial position [0,0,0]
        :return:
        """
        self.check_publishers_connection()
        self.move_joints(self.init_pos)

    def check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while (self._haa_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _haa_joint_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_haa_joint_pub Publisher Connected")

        while (self._hfe_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _hfe_joint_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_hfe_joint_pub Publisher Connected")

        while (self._kfe_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _kfe_joint_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_kfe_joint_pub Publisher Connected")

        rospy.logdebug("All Publishers READY")

    def joint_mono_des_callback(self, msg):
        rospy.logdebug(str(msg.joint_state.position))

        self.move_joints(msg.joint_state.position)

    def move_joints(self, joints_array):

        i = 0
        for publisher_object in self.publishers_array:
            joint_value = Float64()
            joint_value.data = joints_array[i]
            rospy.logdebug("JointsPos>>" + str(joint_value))
            publisher_object.publish(joint_value)
            i += 1

    def start_loop(self, rate_value=2.0):
        rospy.logdebug("Start Loop")
        pos1 = [0.0, 0.0, 1.6]
        pos2 = [0.0, 0.0, -1.6]
        position = "pos1"
        rate = rospy.Rate(rate_value)
        while not rospy.is_shutdown():
            if position == "pos1":
                self.move_joints(pos1)
                position = "pos2"
            else:
                self.move_joints(pos2)
                position = "pos1"
            rate.sleep()

    def start_sinus_loop(self, rate_value=2.0):
        rospy.logdebug("Start Loop")
        w = 0.0
        x = 2.0 * math.sin(w)
        # pos_x = [0.0,0.0,x]
        # pos_x = [x, 0.0, 0.0]
        pos_x = [0.0, x, 0.0]
        rate = rospy.Rate(rate_value)
        while not rospy.is_shutdown():
            self.move_joints(pos_x)
            w += 0.05
            x = 2.0 * math.sin(w)
            # pos_x = [0.0, 0.0, x]
            # pos_x = [x, 0.0, 0.0]
            pos_x = [0.0, x, 0.0]
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('joint_publisher_node')
    joint_publisher = JointPub()
    rate_value = 50.0
    # joint_publisher.start_loop(rate_value)
    joint_publisher.start_sinus_loop(rate_value)
