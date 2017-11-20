#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

# Simple talker demo that published std_msgs/Strings messages
# to the 'chatter' topic
import sys
import rospy
# from std_msgs.msg import String, Int64
from sensor_msgs.msg import JointState
import numpy as np
import time as t
sys.path.append("/home/mikekaram/PycharmProjects/SMS_python")
sys.path.append("/home/mikekaram/PycharmProjects/SMS_python/ikpy_code")
import sms_library_oo_c_code as sms
import geometry_utils as gu


def talker():
    # sms.init(1)
    serial = sms.zo_sms_serial()
    # print(serial)
    serial.serialPortOpen('/dev/ttyUSB1', 57600)
    if (serial.getCommunicationSuccess()):
        print("Communication Success")
    pub = rospy.Publisher('chatter', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    # limits_ticks = np.array([[9606, 9754], [192, 5921], [909, 4498], [], [17054, 7374], []])
    # absolute_positions_homing = np.array([2734, 11700, 7000, 6622, 29925, 22493])
    motorIds = [4, 5, 6, 7, 8, 9]
    resolution_bits = [14, 14, 14, 14, 15, 15]
    motors = list()
    for i in range(len(motorIds)):
        # motor = sms.sms_motor(motorIds[i], resolution_bits[i], limits_ticks[i], absolute_positions_homing[i])
        motor = sms.sms_motor(serial, motorIds[i], resolution_bits[i])
        motors.append(motor)
        # motor.resetErrors()
        # if i == 3:
        #     motor.setProfiledVelocitySetpoint(256)
        #     sms.broadcastDoMove()
        #     t.sleep(20)
    t.sleep(2)
    for i, motor in enumerate(motors):
        print(motor.resetErrors())
        t.sleep(0.05)
    # print("hello world %s" % rospy.get_time())
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        motor_position = []
        motor_angles = []
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        for i, motor in enumerate(motors):
            pos = motor.getPosition()
            tries = 1
            while np.isnan(pos) and tries < 3:
                pos = motor.getPosition()
                t.sleep(0.05)
                tries += 1
            if tries == 3:
                print("Problem detected with motor having Id %d: ABORTING" % motor.motorId)
                exit(1)
            print(pos)
            motor_position.append(pos)
            motor_angles.append(gu.ticks_to_angle(pos, resolution_bits[i]))
            t.sleep(0.05)
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.position = motor_angles
        joint_state_msg.name = ["base_joint", "joint_1", "joint_2", "joint_3", "joint_4", "gripper_joint"]
        pub.publish(joint_state_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
