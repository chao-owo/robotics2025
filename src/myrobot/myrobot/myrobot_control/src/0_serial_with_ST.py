#!/usr/bin/env python3
#-*- coding:UTF-8 -*-

##real arm (Axis2 and Axis3 relationship is cared)
import sys
import rospy
from rospy import timer 
from std_msgs.msg import Float64MultiArray
import serial
import threading
import struct

'''
Belt transmition of real arm:

    joint3 is influenced by joint2 by 0.7 
    delta(j3)= delta(j3) +0.7*delta(j2)
    line80 deals with this as if j2 j3 is independent

'''

global cmd 
cmd = []

global Stop_flag
Stop_flag = 1

COM_NAME = '/dev/ttyACM0'
BAUTRATE = 230400
TIMEOUT = 0.5##sec

def CmdtoByte(NUM):
    NUM_I = int(NUM*1000)
    # NUM_I = NUM*1000
    return struct.pack("<i", NUM_I)
    '''
    '<' means little-endian(Byte order)
    'i' means int
    refer to : https://docs.python.org/3/library/struct.html
    '''

def Cmd_pub(Serial):
    global cmd
    Command_String = 's'.encode() + CmdtoByte(cmd[0]) + CmdtoByte(cmd[1]) \
                    + CmdtoByte(cmd[2]) + CmdtoByte(cmd[3])+'e'.encode()  
    Serial.write(Command_String)
    return


def Connect_STM(com, baudrate):
    try:
        ser = serial.Serial(com, baudrate)
    except:
        print('ST Connect Error!')
        return 'Error'
    print('Connect OK! 0')
    return ser
    
def Read_data(Serial):
    global Stop_flag
    data = Serial.readline()
    print("ST" , data)


def sub_cmd():
    rospy.Subscriber('/real_robot_arm_joint', Float64MultiArray, callback_sub_cmd)

def callback_sub_cmd(msg):
    global cmd,STM

    cmd = list(msg.data)##.data is tuple
    cmd[2]=cmd[2] + cmd[1] * 0.7##belt transmition of real arm

    t = threading.Thread(target=Read_data, args=(STM,))
    t.setDaemon(True)
    t.start()
    
    # Read_data(STM)# directly do this without using thread won't work
    Cmd_pub(STM)

    
    
if __name__ == '__main__' :
 
    STM = Connect_STM(COM_NAME, BAUTRATE)
    rospy.init_node('Com_STM32_INTERFACE',anonymous= True)

    if  (STM != 'Error' ):

        sub_cmd()
        rospy.spin()
    else:
        Stop_flag = 0
    
    STM.close()
    

 