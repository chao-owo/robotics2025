#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import atan2, acos, asin, sqrt, sin, cos, pi
from moveit_commander.conversions import pose_to_list
import numpy as np
from numpy import sin, cos
from std_msgs.msg import Float64MultiArray

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


class MoveGroupPythonIntefaceTutorial(object):
  
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "ldsc_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    #display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
    #                                                moveit_msgs.msg.DisplayTrajectory,
    #                                                queue_size=20)
    display_trajectory_publisher = rospy.Publisher('/real_robot_arm_joint',
                                                   Float64MultiArray,
                                                   queue_size=10)


    planning_frame = move_group.get_planning_frame()
    # print "============ Planning frame: %s" % planning_frame

    # move_group.set_workspace([-0.2984,-0.2984,0.0,0.2984,0.2984,0.4404])

    group_names = robot.get_group_names()


    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    
    self.group_names = group_names

    joint_angles = move_group.get_current_joint_values()
    self.joint_angles = joint_angles
    joint_msg = Float64MultiArray()
    joint_msg.data = joint_angles
    display_trajectory_publisher.publish(joint_msg)
    
  def go_to_joint_state(self):
    
    move_group = self.move_group
    joint_angles = self.joint_angles

    joint_goal = move_group.get_current_joint_values()

    joint_goal[0] = joint_angles[0].item()
    joint_goal[1] = joint_angles[1].item()
    joint_goal[2] = joint_angles[2].item()
    joint_goal[3] = joint_angles[3].item()

    move_group.go(joint_goal, wait=True)

    move_group.stop()

    current_joints = move_group.get_current_joint_values()
    current_pose = self.move_group.get_current_pose('link5').pose
    print ("current pose:")
    # print (current_pose.position) 
    print ("x: %.5f" %current_pose.position.x)
    print ("y: %.5f" %current_pose.position.y)
    print ("z: %.5f" %current_pose.position.z)

    current_rpy = self.move_group.get_current_rpy('link5')
    print ("rol: %.5f" %current_rpy[0])
    print ("pit: %.5f" %current_rpy[1])
    print ("yaw: %.5f" %current_rpy[2])
    print ("")
    return all_close(joint_goal, current_joints, 0.01)


def Your_IK(x, y, z, p): 
    Xd = np.array([x, y, z], ndmin=2).T
    K, error_margin = 0.05, 0.001

    joint_angle = np.array([0.5, 1.14, 0.93, 0.4], ndmin=2).T
    while True:
        j1, j2, j3, j4 = joint_angle.T[0]
        # kinematics
        x_ = (
             (33 * cos(j1) * sin(j2)) / 250
             + (6 * cos(p - j2 - j3) * (cos(j1) * cos(j2) * sin(j3) + cos(j1) * cos(j3) * sin(j2))) / 125
             + (cos(p - j2 - j3) * (cos(j1) * cos(j2) * cos(j3) - cos(j1) * sin(j2) * sin(j3))) / 250
             - (sin(p - j2 - j3) * (cos(j1) * cos(j2) * sin(j3) + cos(j1) * cos(j3) * sin(j2))) / 250
             - (6 * sin(j2 + j3 - p) * (cos(j1) * cos(j2) * cos(j3) - cos(j1) * sin(j2) * sin(j3))) / 125
             + (104 * cos(j1) * cos(j2) * sin(j3)) / 625
             + (104 * cos(j1) * cos(j3) * sin(j2)) / 625
             )
         
        y_ = (
              (33 * sin(j1) * sin(j2)) / 250
              + (6 * cos(p - j2 - j3) * (cos(j2) * sin(j1) * sin(j3) + cos(j3) * sin(j1) * sin(j2))) / 125
              + (cos(p - j2 - j3) * (cos(j2) * cos(j3) * sin(j1) - sin(j1) * sin(j2) * sin(j3))) / 250
              - (sin(p - j2 - j3) * (cos(j2) * sin(j1) * sin(j3) + cos(j3) * sin(j1) * sin(j2))) / 250
              - (6 * sin(j2 + j3 - p) * (cos(j2) * cos(j3) * sin(j1) - sin(j1) * sin(j2) * sin(j3))) / 125
              + (104 * cos(j2) * sin(j1) * sin(j3)) / 625
              + (104 * cos(j3) * sin(j1) * sin(j2)) / 625
              )
        z_ = (
              (33 * cos(j2)) / 250
              - (cos(p - j2 - j3) * (cos(j2) * sin(j3) + cos(j3) * sin(j2))) / 250
              - (6 * cos(p - j2 - j3) * (sin(j2) * sin(j3) - cos(j2) * cos(j3))) / 125
              - (104 * sin(j2) * sin(j3)) / 625
              - (6 * sin(p - j2 - j3) * (cos(j2) * sin(j3) + cos(j3) * sin(j2))) / 125
              + (sin(p - j2 - j3) * (sin(j2) * sin(j3) - cos(j2) * cos(j3))) / 250
              + (104 * cos(j2) * cos(j3)) / 625
              + 0.142
              )

        Xe = np.array([x_, y_, z_], ndmin=2).T

        print("Xd:", Xd)
        print("q:", joint_angle.T[0], "\nXe:", Xe)

        dist = np.linalg.norm(Xd-Xe)
        print("d:", dist)
        if dist < error_margin:
            break
        Ja = [
             [
             (6 * sin(j2 + j3 - p) * (cos(j2) * cos(j3) * sin(j1) - sin(j1) * sin(j2) * sin(j3))) / 125
             - (6 * cos(j2 + j3 - p) * (cos(j2) * sin(j1) * sin(j3) + cos(j3) * sin(j1) * sin(j2))) / 125
             - (cos(j2 + j3 - p) * (cos(j2) * cos(j3) * sin(j1) - sin(j1) * sin(j2) * sin(j3))) / 250
             - (sin(j2 + j3 - p) * (cos(j2) * sin(j1) * sin(j3) + cos(j3) * sin(j1) * sin(j2))) / 250
             - (33 * sin(j1) * sin(j2)) / 250
             - (104 * cos(j2) * sin(j1) * sin(j3)) / 625
             - (104 * cos(j3) * sin(j1) * sin(j2)) / 625,
        
               (33 * cos(j1) * cos(j2)) / 250
             + (104 * cos(j1) * cos(j2) * cos(j3)) / 625
             - (104 * cos(j1) * sin(j2) * sin(j3)) / 625,
        
               (104 * cos(j1) * cos(j2) * cos(j3)) / 625
             - (104 * cos(j1) * sin(j2) * sin(j3)) / 625,
        
               0
               ],
               [
               (33 * cos(j1) * sin(j2)) / 250
               + (6 * cos(j2 + j3 - p) * (cos(j1) * cos(j2) * sin(j3) + cos(j1) * cos(j3) * sin(j2))) / 125
               + (cos(j2 + j3 - p) * (cos(j1) * cos(j2) * cos(j3) - cos(j1) * sin(j2) * sin(j3))) / 250
               + (sin(j2 + j3 - p) * (cos(j1) * cos(j2) * sin(j3) + cos(j1) * cos(j3) * sin(j2))) / 250
               - (6 * sin(j2 + j3 - p) * (cos(j1) * cos(j2) * cos(j3) - cos(j1) * sin(j2) * sin(j3))) / 125
               + (104 * cos(j1) * cos(j2) * sin(j3)) / 625
               + (104 * cos(j1) * cos(j3) * sin(j2)) / 625,
        
                 (33 * cos(j2) * sin(j1)) / 250
               + (104 * cos(j2) * cos(j3) * sin(j1)) / 625
               - (104 * sin(j1) * sin(j2) * sin(j3)) / 625,
        
                 (104 * cos(j2) * cos(j3) * sin(j1)) / 625
               - (104 * sin(j1) * sin(j2) * sin(j3)) / 625,
        
                  0 
                ],
                [
                0,
                -(33 * sin(j2)) / 250
                - (104 * cos(j2) * sin(j3)) / 625
                - (104 * cos(j3) * sin(j2)) / 625,
        
                -(104 * cos(j2) * sin(j3)) / 625
                - (104 * cos(j3) * sin(j2)) / 625,
        
                  0
                ]
                ]

    
        Ja = np.array(Ja)
        J_hash = np.matmul(Ja.T, np.linalg.inv(np.matmul(Ja, (Ja.T))))
        joint_angle = joint_angle + K * np.matmul(J_hash, (Xd-Xe))
        joint_angle[3] = p - (joint_angle[1]+joint_angle[2])
        

        
        
    return joint_angle

def main():
  try:
    path_object = MoveGroupPythonIntefaceTutorial()
    print("ctrl + z to close")
    while not rospy.is_shutdown():
  
        try:
          x_input=float(input("x:  "))
          y_input=float(input("y:  "))
          z_input=float(input("z:  "))
          q_input=float(input("q:  "))

          path_object.joint_angles = Your_IK(x_input,y_input,z_input,q_input)
          '''
          You just need to solve IK of a point, path planning will automatically be taken.  
          '''
          path_object.go_to_joint_state()

        except:
          '''go back to home if weird input'''
          path_object.joint_angles = [0,-pi/2,pi/2,0]
          path_object.go_to_joint_state()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()