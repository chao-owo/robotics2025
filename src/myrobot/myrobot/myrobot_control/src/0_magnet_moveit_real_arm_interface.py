#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float64MultiArray
from moveit_msgs.msg import DisplayTrajectory
from std_msgs.msg import Bool

joint_angle = Float64MultiArray()
joint_angle.data = [0,-math.pi/2,math.pi/2,0]
EEF_state = 0.0## False(magnet off)
t_last = 0


initial_offset = [0,-math.pi/2,math.pi/2,0] #difference btw ST-initialize-zero and Moveit-zero-point

def real_arm_issue(cmd_list):
    for i in range(len(cmd_list)):
        cmd_list[i] = cmd_list[i] - initial_offset[i]
    return cmd_list
    '''
    The same initial pose of real arm  btw ST and Moveit
            |
            |    
        ____|
        |
    =========

    but  their data differ  :
    
        ST          Moveit_plan
        j1:0        j1:0
        j2:0        j2:-1.5708       ---> j2(ST) = j2(Moveit) -1.5708
        j3:0        j3:1.5708      ---> j3(ST) = j3(Moveit) +1.5708
        j4:0        j4:0
    '''
    


def angle_limit(theta):

    if theta > math.pi:
        theta = math.pi
    elif theta < -math.pi:
        theta = -math.pi
    return theta

def pub_cmd_to_joint():
    global pub,rate,joint_angle, pub_EEF_state
    rospy.init_node('joint_position_pub', anonymous=True)
    pub = rospy.Publisher('/real_robot_arm_joint', Float64MultiArray, queue_size=10)
    pub_EEF_state = rospy.Publisher('/real_robot_arm_EEF', Bool, queue_size=10)
    rate = rospy.Rate(100) # 100hz
    

def sub_to_moveit():
	rospy.Subscriber("/move_group/display_planned_path",DisplayTrajectory,callback_1)

def callback_1(msg_get):
    #type of msg_get is DisplayTrajectory
    global joint_angle, EEF_state, t_last
    
    for ele in  msg_get.trajectory:

        a_list = ele.joint_trajectory.points
        
        for e in  a_list:

            joint_angle.data = real_arm_issue(list(e.positions)) +[EEF_state]
            pub.publish(joint_angle)

            t_now = e.time_from_start.secs + e.time_from_start.nsecs * 1e-9
            del_t = t_now - t_last
            if del_t < 0:
                del_t = 0
            t_last =  t_now
            print(del_t)
            rospy.sleep(del_t)
            # rospy.sleep(0.03)##check if moveit_plan has time stamp to set this

def sub_to_SetEndEffector():
    rospy.Subscriber("/SetEndEffector",Bool,callback_2)

def callback_2(msg_get):
    # type bool
    global EEF_state
    if msg_get.data is True:
        EEF_state = 1.0
    else :
        EEF_state = 0.0


if __name__ == '__main__':


    print("moveit_real_arm_interface ok!")   
    pub_cmd_to_joint()
    sub_to_SetEndEffector()
    sub_to_moveit()
    rospy.spin()

   
   
    
        
    
