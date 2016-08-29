#!/usr/bin/env python
import sys
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander, conversions
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from baxter_core_msgs.msg import EndEffectorState
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from baxter_interface import CameraController, Gripper, Limb
from sensor_msgs.msg import Image, Range
import numpy as np
import math
import copy
import time

from handle_release.srv import *

class Tracker():
    
    def __init__(self, hdr, arm):
        self.hdr = hdr
        self.busy = False
        self.history = np.arange(0,20)*-1
        self.control_arm = arm
        self.control_joint_names = self.control_arm.joint_names()
        self.dx = 0
        self.dy = 0

    def detection_request(self, controlID):        
        try:
            rospy.wait_for_service('microwave_handle_detection')
            req = rospy.ServiceProxy('microwave_handle_detection', HandleDetection)
            return req(controlID)
        
        except (rospy.ServiceException,rospy.ROSInterruptException), e:
            print "Service call failed: %s" % e
            self.clean_shutdown_service()
 
    def PID(self):
        Kp = 0.5E-5
        vx = Kp * self.dx
        vy = Kp * self.dy
        return vx,vy

    def track(self):
        # Position Control #
        initial_pose = get_current_pose(self.hdr, self.control_arm)
        predict_pose = initial_pose
        finish = False
        # calculate the gripper angle with controlID 0
        resp = self.detection_request(0)
        while not finish:
            resp = self.detection_request(1)
            print resp
            self.dy = resp.dy
            self.dx = resp.dx
            if (abs(self.dx) + abs(self.dy)) < 20:
                finish = True
            vx, vy = self.PID()   
            print "vx", vx
            print "vy", vy       
            solution_found, predict_pose = ik_move_one_step(initial_pose, predict_pose, self.hdr, self.control_arm, target_dx = vx, target_dy = vy)

    def clean_shutdown(self):
        print "Demo finished"
        rospy.signal_shutdown("Done")
        
    def clean_shutdown_service(self):
        print "Service shut down"
        rospy.signal_shutdown("Done")
        sys.exit()
                
def set_joints( target_angles_right = None, target_angles_left = None, timeout= 40000):

    right = Limb("right")
    left = Limb("left")
    
    if target_angles_right == None:
        reach_right = True
    else:
        reach_right = False
    

    if target_angles_left == None:
        reach_left = True
    else:
        reach_left = False
    
    time = 0

    while not reach_right or not reach_left:

            if target_angles_right: right.set_joint_positions(target_angles_right)
            if target_angles_left: left.set_joint_positions(target_angles_left)
            current_angles_right = right.joint_angles()
            current_angles_left = left.joint_angles()

            
            if reach_right == False:
                for k, v in current_angles_right.iteritems():
                    if abs(target_angles_right[k] - v) > 0.01:
                        reach_right = False
                        break
                    reach_right = True

            if reach_left == False:
                for k, v in current_angles_left.iteritems():
                    if abs(target_angles_left[k] - v) > 0.01:
                        reach_left = False
                        break
                    reach_left = True

            time+=1
            if time > timeout:
                print "Time out"
                break

def get_current_pose(hdr,arm,initial_pose = None):
    ep_position = arm.endpoint_pose()['position']
    ep_orientation = arm.endpoint_pose()['orientation']

    if initial_pose == None:  
        current_pose = copy.deepcopy(initial_pose)
        current_pose = PoseStamped(
                    header=hdr,
                    pose=Pose(
                        position=Point(
                            x=ep_position.x,
                            y=ep_position.y,
                            z=ep_position.z,
                        ),
                        orientation=Quaternion(
                            x=ep_orientation.x,
                            y=ep_orientation.y,
                            z=ep_orientation.z,
                            w=ep_orientation.w,
                        ),
                    )
        )
    else:
        current_pose = copy.deepcopy(initial_pose)
        current_pose.pose.position.x = ep_position.x
        current_pose.pose.position.y = ep_position.y
        current_pose.pose.position.z = ep_position.z

    return current_pose

def update_current_pose(current_pose,dx,dy,dz):
    new_pose = copy.deepcopy(current_pose)
    dx = dx/1.0
    dy = dy/1.0
    dz = dz/1.0
    new_pose.pose.position.x += dx
    new_pose.pose.position.y += dy
    new_pose.pose.position.z += dz
    return new_pose

def ik(pose):
    ns = "ExternalTools/left/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    ikreq.pose_stamp.append(pose)

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 0

    if (resp.isValid[0]):
        #print("SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        arm = Limb("left")
        arm.set_joint_positions(limb_joints)
        return 1
        #rospy.sleep(0.05)

    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        return 0

def ik_move_to_pose(arm,kin,pose,timeout= 60000):

    ns = "ExternalTools/left/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    ikreq.pose_stamp.append(pose)

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 0

    if (resp.isValid[0]):
        #print("SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        set_joints(target_angles_right = limb_joints,timeout = timeout)
        return 1
        #rospy.sleep(0.05)

    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        return 0
  
def ik_move(hdr, arm, target_dx = None, target_dy = None, target_dz = None, x = None, y = None, z = None, timeout= 50,speed = 1.0):

    initial_pose = get_current_pose(hdr, arm)
    target_x = initial_pose.pose.position.x
    target_y = initial_pose.pose.position.y
    target_z = initial_pose.pose.position.z
    
    if target_dx != None: target_x += target_dx
    if target_dy != None: target_y += target_dy
    if target_dz != None: target_z += target_dz

    if x != None: target_x = x
    if y != None: target_y = y
    if z != None: target_z = z

    dx = 100
    dy = 100
    dz = 100

    solution_found = 1

    time = 0

    predict_pose = initial_pose


    while (abs(dx) > 0.01 or abs(dy) > 0.01 or abs(dz) > 0.01) and solution_found == 1:        
        current_pose = get_current_pose(hdr, arm, initial_pose)
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        current_z = current_pose.pose.position.z
        dx = target_x - current_x
        dy = target_y - current_y
        dz = target_z - current_z
        vx = dx*speed
        vy = dy*speed
        vz = dz*speed
        #print dx, dy, dz
        solution_found, predict_pose = ik_move_one_step(initial_pose, predict_pose, hdr, arm, target_dx = vx, target_dy = vy, target_dz = vz)
        time += 1
        if time > timeout:
            print "Time out"
            break
    return solution_found
        
def ik_move_one_step(initial_pose, predict_pose, hdr, arm, target_dx = None, target_dy = None, target_dz = None):
    current_pose = get_current_pose(hdr, arm, initial_pose)

    if target_dx == None: 
        target_dx = initial_pose.pose.position.x - current_pose.pose.position.x
    else: 
        target_dx = target_dx + predict_pose.pose.position.x - current_pose.pose.position.x
    if target_dy == None: 
        target_dy = initial_pose.pose.position.y - current_pose.pose.position.y
    else: 
        target_dy = target_dy + predict_pose.pose.position.y - current_pose.pose.position.y
    if target_dz == None: 
        target_dz = initial_pose.pose.position.z - current_pose.pose.position.z
    else:
        target_dz = target_dz + predict_pose.pose.position.z - current_pose.pose.position.z

    new_pose = update_current_pose(current_pose,target_dx,target_dy,target_dz)
    solution_found = ik(new_pose)
    return solution_found, new_pose

def main():
    roscpp_initialize(sys.argv)
    rospy.init_node('handle_release', anonymous=True)
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    arm = Limb("left")
    tracker = Tracker(hdr, arm)
    rospy.on_shutdown(tracker.clean_shutdown)
    tracker.track()
     
if __name__=='__main__':
    sys.exit(main())
    