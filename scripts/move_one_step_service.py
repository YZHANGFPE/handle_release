#!/usr/bin/env python

import sys
import rospy
from handle_release.srv import *
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

def move_one_step_client():
    rospy.wait_for_service('move_one_step_server')
    try:
        move_one_step = rospy.ServiceProxy('move_one_step_server', MoveOneStep)
        initial_pose = PoseStamped()
        predict_pose = PoseStamped()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        arm = 'left'
        target_dx = 0.1
        target_dy = 0.1
        target_dz = 0.1
        resp = move_one_step(initial_pose, predict_pose, hdr, arm, target_dx, target_dy, target_dz)
        print resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    rospy.init_node('move_one_step_client')
    move_one_step_client()