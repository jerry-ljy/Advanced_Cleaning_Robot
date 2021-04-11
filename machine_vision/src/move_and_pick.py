#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Twist
from yolov3_pytorch_ros.msg import BoundingBoxes
from yolov3_pytorch_ros.msg import BoundingBox
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
import threading

#######packages for OM########
import os,sys
import actionlib
import copy
import rospy
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
########################

###### methods for detect objects#######
move='s'
flag=True
laser=0

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82
LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.05
MIN_DISTANCE = 0.5


target_linear_vel   = 0.0
target_angular_vel  = 0.0
control_linear_vel  = 0.0
control_angular_vel = 0.0

e = 'Error...' 
"""
Communications Failed
"""

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    return vel


def thread_job():
    rospy.spin()

def detect_callback(data):
    if len(data.bounding_boxes)!=0:
        object = data.bounding_boxes[0]
        position = (object.xmin+object.xmax)/2
        print("object center:")
        print(position)
        global move
        if position<450:
            move='l'
        elif position>460:
            move='r'
        else:
            move='f'
    else:
        move='s'
    
def laser_callback(data):
    global flag
    global laser
    laser = data.ranges[0]
    if(data.ranges[0]<=0.2):
        flag=False
#############################################

##########methods for picking objects#########

def all_close(goal, actual, tolerance):
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonInteface(object):
  """MoveGroupPythonInteface"""
  def __init__(self):
    super(MoveGroupPythonInteface, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node('pick_move_place_Node', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the OM
    ## arm so we set ``group_name = arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the OM:
    group_name = "arm"
    group_name2 = "gripper"
    group = moveit_commander.MoveGroupCommander(group_name)
    grip = moveit_commander.MoveGroupCommander(group_name2)

    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.grip = grip
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

############################ Defining Joint Parameters ############################ 

  def Front(self):
    group = self.group

    ## Planning to a Joint Goal
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    
    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def Left(self):

    group = self.group

    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 1.571
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    
    group.go(joint_goal, wait=True)

    group.stop()

    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def Down(self):

    group = self.group

    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0.566
    joint_goal[2] = 0
    joint_goal[3] = 0

    group.go(joint_goal, wait=True)

    group.stop()

    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def gripper_open(self):

    grip = self.grip

    joint_grip = grip.get_current_joint_values()
    joint_grip[0] = 0.01
    
    grip.go(joint_grip, wait=True)

    grip.stop()

    current_grip = self.grip.get_current_joint_values()
    return all_close(joint_grip, current_grip, 0.01)

  def gripper_close(self):

    grip = self.grip

    joint_grip = grip.get_current_joint_values()
    joint_grip[0] = -0.01
    
    grip.go(joint_grip, wait=True)

    grip.stop()

    current_grip = self.grip.get_current_joint_values()
    return all_close(joint_grip, current_grip, 0.01)

def pick():
  try:
    # Setting up the moveit_commander ...
    move_OM = MoveGroupPythonInteface()

    # Execute movement using 4th joint state goal; face front for initialization...
    move_OM.Front()
    time.sleep(7)
    # Execute gripper open ...
    move_OM.gripper_open()
    time.sleep(3)

    # Execute movement using 2nd joint state goal; move down ...
    move_OM.Down()
    time.sleep(7)

    #Execute gripper close ...
    move_OM.gripper_close()
    time.sleep(3)

    # Execute movement using 4th joint state goal; face front ...
    move_OM.Front()
    time.sleep(7)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


def place():
  try:
    # Setting up the moveit_commander ...
    move_OM = MoveGroupPythonInteface()

    # Execute movement using 2nd joint state goal; move down ...
    move_OM.Down()
    time.sleep(7)

    # Execute gripper open ...
    move_OM.gripper_open()
    time.sleep(3)

    # Execute movement using 4th joint state goal; face front ...
    move_OM.Front()
    time.sleep(7)

    #Execute gripper close ...
    move_OM.gripper_close()
    time.sleep(3)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


##############################################
if __name__ == '__main__':    
    rospy.init_node('move_and_pick')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    add_thread=threading.Thread(target=thread_job)
    add_thread.start()

    rospy.Subscriber("/detected_objects_in_image", BoundingBoxes ,detect_callback, queue_size= 1)
    rospy.Subscriber("/scan", LaserScan, laser_callback)
    while not rospy.is_shutdown():
        print(laser)
        if flag:
            if move=='l':
                print('turn left')
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
            elif move=='r':
                print('turn right')
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)

            elif move=='f':
                print('go forward!')
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                # break       
            else:
                print('stop')
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0 
            try:
                twist = Twist()
                control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

                control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
                
                rospy.sleep(2)
                pub.publish(twist)

                ################################################stop###############################
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0

                twist = Twist()
                control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

                control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
                
                rospy.sleep(1)
                pub.publish(twist)
            except:
                print(e)
        else:
            print("start picking")
            pick()
    


