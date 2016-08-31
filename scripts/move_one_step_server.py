#!/usr/bin/env python

from handle_release.srv import MoveOneStep, MoveOneStepResponse
from geometry_msgs.msg import PoseStamped
import rospy

def handle_move_one_step(req):
    print "Returning initial_pose: %s, predict_pose %s, header %s, arm %s, target_dx %s, target_dy %s, target_dz %s" \
    % (req.initial_pose, req.predict_pose, req.header, req.arm, req.target_dx, req.target_dy, req.target_dz)
    res_pose = PoseStamped()
    return MoveOneStepResponse(1, res_pose)

def move_one_step_server():
    rospy.init_node('move_one_step_server')
    s = rospy.Service('move_one_step_server', MoveOneStep, handle_move_one_step)
    rospy.loginfo("Ready use move one step server") 
    rospy.loginfo("Press Ctrl-C to shut down") 
    rospy.spin()

if __name__ == "__main__":
    move_one_step_server()