#!/usr/bin/env python3

import os
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import Bool
from std_msgs.msg import String
from math import atan2, acos, asin, sqrt, sin, cos, pi
import rospkg 
import time




from geometry_msgs.msg import PoseStamped
import cv2


from datetime import datetime

import re

import numpy as np
from numpy import sin, cos
from std_msgs.msg import Float64MultiArray


'''Variable for end-effector'''
EefState = 0

'''Hanoi tower geometry'''
#You can measure these in Lab402
Tower_base = 0.0014     #Height of tower base
Tower_height = 0.025    #Height of each tower
Tower_overlap = 0.015   #Height of tower overlap

'''Hanoi tower position'''
#You may want to slightly change this
p_Tower_x = 0.25
p_Tower_y = 0.15 #(0.15, 0, -0.15) as lab4 shown

'''Hanoi tower mesh file path'''
rospack = rospkg.RosPack()
FILE_PATH = rospack.get_path('myplan')+ "/mesh"
MESH_FILE_PATH = [FILE_PATH +"/tower1.stl",FILE_PATH +"/tower2.stl",FILE_PATH +"/tower3.stl"]

'''Robot arm geometry'''
l0 = 0.06;l1 = 0.082;l2 = 0.132;l3 = 0.1664;l4 = 0.048;d4 = 0.004
color_to_size = {'red': 1, 'blue': 2, 'green': 3}

color_ranges = {
    'red': [
        ([0, 50, 20], [10, 255, 255]),    
        ([160, 50, 20], [180, 255, 255])   
    ],
    'green': [
        ([35, 40, 20], [85, 255, 255])
    ],
    'blue': [
        ([90, 50, 20], [150, 255, 255])
    ]
}
'''
Hint:
    The output of your "Hanoi-Tower-Function" can be a series of [x, y, z, eef-state], where
    1.xyz in world frame
    2.eef-state: 1 for magnet on, 0 for off
'''
# Dictionary to keep track of disk positions
tower_disks = {
    'A': [],  # Tower A (left)
    'B': [],  # Tower B (center)
    'C': []   # Tower C (right)
}
'''
# Tower coordinates
tower_coords = {
    'A': (p_Tower_x, p_Tower_y),    # Tower A (left)
    'B': (p_Tower_x, 0),            # Tower B (center)
    'C': (p_Tower_x, -p_Tower_y)    # Tower C (right)
}
'''
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

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        planning_frame = move_group.get_planning_frame()
        group_names = robot.get_group_names()
        
        # Add box name tracking for object manipulation 
        self.box_name = '' ##
        
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.group_names = group_names

        joint_angles = move_group.get_current_joint_values()
        self.joint_angles = joint_angles

    def go_to_joint_state(self):
        move_group = self.move_group
        joint_angles = self.joint_angles

        joint_goal = move_group.get_current_joint_values()

        joint_goal[0] = joint_angles[0]
        joint_goal[1] = joint_angles[1]
        joint_goal[2] = joint_angles[2]
        joint_goal[3] = joint_angles[3]

        move_group.go(joint_goal, wait=True)
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        current_pose = self.move_group.get_current_pose('link5').pose
        print("current pose:")
        print(current_pose.position) 
        return all_close(joint_goal, current_joints, 0.01)

    # from spawn_objects.py
    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        box_name = self.box_name
        scene = self.scene
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0
            is_known = box_name in scene.get_known_object_names()
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True
            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False
    def add_box(self, box_name , box_pose, size_tuple):  
        '''
        Description: 
            1. Add a box to rviz, Moveit_planner will think of which as an obstacle.
            2. An example is shown in the main function below.
            3. Google scene.add_box for more details
        '''
        scene = self.scene
        scene.add_box(box_name, box_pose, size=size_tuple)
        timeout=4
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def add_mesh(self, mesh_name, mesh_pose, file_path, size_tuple): 
        '''
        Description: 
          1. Add a mesh to rviz, Moveit_planner will think of which as an obstacle.
          2. An example is shown in the main function below.
        '''
        scene = self.scene
        mesh_pose.pose.orientation.w = 0.7071081
        mesh_pose.pose.orientation.x = 0.7071081
        mesh_pose.pose.orientation.y = 0
        mesh_pose.pose.orientation.z = 0
        #deal with orientation-definition difference btw .stl and robot_urdf
        
        # Save the box name for tracking
        self.box_name = mesh_name
        
        scene.add_mesh(mesh_name, mesh_pose, file_path, size=size_tuple)
        timeout=4
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_mesh(self, mesh_name, link_name):
        '''
        Description: 
          1. Make sure the mesh has been added to rviz
          2. Attach a box to link_frame(usually 'link5'), and the box will move with the link_frame.
        '''
        scene = self.scene
        
        # Save the box name for tracking
        self.box_name = mesh_name
        
        scene.attach_mesh(link_name, mesh_name, touch_links=[link_name])
        timeout=4
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_mesh(self, mesh_name, link_name):
        '''Detach a box from link_frame(usually 'link5'), and the box will not move with the link_frame.'''
        scene = self.scene
        
        # Save the box name for tracking
        self.box_name = mesh_name
        
        scene.remove_attached_object(link_name, name=mesh_name)
        timeout=4
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_mesh(self, mesh_name):
        '''Remove a mesh from rviz.'''
        scene = self.scene
        
        # Save the box name for tracking
        self.box_name = mesh_name
        
        scene.remove_world_object(mesh_name)
        ## **Note:** The object must be detached before we can remove it from the world
        # We wait for the planning scene to update.
        timeout=4
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

def capture_image_after_delay(save_dir="~/Robotics/photos", delay_sec=1):
    # 展開和建立儲存資料夾
    save_dir = os.path.expanduser(save_dir)
    os.makedirs(save_dir, exist_ok=True)

    # 開啟攝影機
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ 無法開啟攝影機")
        return None

    print(f"⏳ 等待 {delay_sec} 秒後拍照...")
    time.sleep(delay_sec)

    ret, frame = cap.read()
    cap.release()

    if not ret:
        print("❌ 拍照失敗")
        return None

    # 儲存影像
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(save_dir, f"{timestamp}.jpg")
    cv2.imwrite(filename, frame)
    print(f"✅ 影像已儲存：{filename}")
    return filename

def detect_color_positions(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    positions = []

    for color, ranges in color_ranges.items():
        mask = None
        for lower, upper in ranges:
            lower_np = np.array(lower)
            upper_np = np.array(upper)
            new_mask = cv2.inRange(hsv, lower_np, upper_np)
            mask = new_mask if mask is None else cv2.bitwise_or(mask, new_mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                positions.append((cx, color_to_size[color]))

    positions.sort()  # 按照 x 座標從左到右
    return [size for _, size in positions]

def assign_towers_by_vision(image_path='photos/123.jpg'):
    image = cv2.imread(image_path)
    order = detect_color_positions(image)  # e.g. [3, 1, 2]

    if len(order) != 3:
        raise ValueError("❌ 無法正確辨識三個塔的位置")

    size_to_peg = {}
    for peg, size in zip(['A', 'B', 'C'], order):
        size_to_peg[size] = peg

    print(f"📸 影像辨識塔順序: {order}")
    print(f"🗺️ Tower 大小對應 Peg: {size_to_peg}")
    return size_to_peg

def Your_IK(x, y, z, p = pi/2): 
    Xd = np.array([x, y, z], ndmin=2).T
    
    # Increase step size for faster convergence
    K = 0.15  # Increased from 0.05
    
    # Slightly relax error margin for faster convergence
    error_margin = 0.0015  # Increased slightly from 0.001
    
    # Better initial guess for faster convergence
    # Use previous solution as initial guess if available
    if hasattr(Your_IK, "last_solution") and Your_IK.last_solution is not None:
        joint_angle = Your_IK.last_solution.copy()
    else:
        joint_angle = np.array([0.5, 1.14, 0.93, 0.4], ndmin=2).T
    
    # Add maximum iterations to prevent infinite loops
    max_iterations = 50
    iteration = 0
    
    while iteration < max_iterations:
        iteration += 1
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

        # Reduce verbosity - only print every 10th iteration
        if iteration % 10 == 0:
            print("Target:", Xd.T[0])
            print("Current Joint Angles:", joint_angle.T[0])
            print("Current End Effector Position:", Xe.T[0])
            print("Iteration:", iteration)

        dist = np.linalg.norm(Xd-Xe)
        
        # Only print distance occasionally
        if iteration % 10 == 0:
            print("Distance to target:", dist)
            
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
        
        # Use a more numerically stable pseudoinverse method
        # Add small regularization term for stability
        lambda_factor = 1e-6
        J_hash = np.matmul(Ja.T, np.linalg.inv(np.matmul(Ja, Ja.T) + lambda_factor * np.eye(3)))
        
        # Compute the update step
        update = K * np.matmul(J_hash, (Xd-Xe))
        
        # Add momentum term to speed up convergence
        if iteration > 1 and hasattr(Your_IK, "last_update"):
            momentum = 0.3  # Momentum coefficient
            update = update + momentum * Your_IK.last_update
        
        Your_IK.last_update = update.copy()
        
        # Update joint angles
        joint_angle = joint_angle + update
        joint_angle[3] = p - (joint_angle[1]+joint_angle[2])
    
    # Store solution for next call
    Your_IK.last_solution = joint_angle.copy()
    
    # Convert numpy array to plain Python list before returning
    result = joint_angle.T[0].tolist()
    print("Final joint angles:", result)
    return result

# Initialize the static variables
Your_IK.last_solution = None
Your_IK.last_update = None


def pub_EefState_to_arm():
    '''
        Description:
        Because moveit only plans the path, 
        you have to publish end-effector state for playing hanoi.
        Increased rate and queue size for faster response.
    '''
    global pub_EefState, rate
    pub_EefState = rospy.Publisher('/SetEndEffector', Bool, queue_size=10)  # Increased queue size
    rate = rospy.Rate(100)  # Increased from 100hz to 1000hz

## new stuff
def spawn_hanoi_towers_a(path_object):
    '''Spawn the three Hanoi towers in the scene'''
    tower_positions = [
        (p_Tower_x, p_Tower_y, 0.0),      # Tower A (left)
        (p_Tower_x, 0.0, 0.0),            # Tower B (center) 
        (p_Tower_x, -p_Tower_y, 0.0)     # Tower C (right)
    ]
    mesh_names = ['A', 'B', 'C']
   # Initialize tower_disks dictionary
    global tower_disks
    tower_disks = {
        'A': [],  # Tower A (left)
        'B': [],  # Tower B (center)
        'C': []   # Tower C (right)
    }
    tower_coords = {
    'A': (p_Tower_x, p_Tower_y),    # Tower A (left)
    'B': (p_Tower_x, 0),            # Tower B (center)
    'C': (p_Tower_x, -p_Tower_y)    # Tower C (right)
    }
    for i, (x, y, z) in enumerate(tower_positions):
        mesh_pose = geometry_msgs.msg.PoseStamped()
        mesh_pose.header.frame_id = 'world'
        mesh_pose.pose.position.x = x
        mesh_pose.pose.position.y = y
        mesh_pose.pose.position.z = z
        
        tower_name = mesh_names[i]
        path_object.add_mesh(tower_name, mesh_pose, MESH_FILE_PATH[i], (.00095, .00095, .00095))
        print(f"Added {tower_name} at position ({x}, {y}, {z})")
    
    # Add two tall tower-like boxes as obstacles
    tall_box_size = (0.010, 0.010, 0.4)  # width, depth, height

    box_positions = [
        (p_Tower_x, p_Tower_y / 2, 0.2),     # center at mid-height (0.2)
        (p_Tower_x, -p_Tower_y / 2, 0.2)
    ]
    box_names = ['box1', 'box2']

    for i, (x, y, z) in enumerate(box_positions):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = 'world'
        box_pose.pose.position.x = x
        box_pose.pose.position.y = y
        box_pose.pose.position.z = z  # height / 2 to make it sit on the ground

        path_object.add_box(box_names[i], box_pose, tall_box_size)
        print(f"Added box {box_names[i]} at position ({x}, {y}, {z})")

def spawn_hanoi_towers_b(path_object):
    '''A C B'''
    tower_positions = [
        (p_Tower_x, p_Tower_y, 0.0),     
        (p_Tower_x, -p_Tower_y, 0.0),    
        (p_Tower_x, 0.0, 0.0)            
    ]
    mesh_names = ['A', 'B', 'C']

    # Initialize tower_disks dictionary
    global tower_disks
    tower_disks = {
        'A': [],  # Tower A
        'C': [],  # Tower C
        'B': []   # Tower B
    }

    tower_coords = {
        'A': (p_Tower_x, p_Tower_y),
        'C': (p_Tower_x, -p_Tower_y),
        'B': (p_Tower_x, 0.0)
    }

    for i, (x, y, z) in enumerate(tower_positions):
        mesh_pose = geometry_msgs.msg.PoseStamped()
        mesh_pose.header.frame_id = 'world'
        mesh_pose.pose.position.x = x
        mesh_pose.pose.position.y = y
        mesh_pose.pose.position.z = z

        tower_name = mesh_names[i]
        path_object.add_mesh(tower_name, mesh_pose, MESH_FILE_PATH[i], (.00095, .00095, .00095))
        print(f"Added mesh {tower_name} at position ({x}, {y}, {z})")

    # Add two tall tower-like boxes as obstacles
    tall_box_size = (0.02, 0.02, 0.4)  # width, depth, height

    box_positions = [
        (p_Tower_x, p_Tower_y / 2, 0.2),     # center at mid-height (0.2)
        (p_Tower_x, -p_Tower_y / 2, 0.2)
    ]
    box_names = ['box1', 'box2']

    for i, (x, y, z) in enumerate(box_positions):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = 'world'
        box_pose.pose.position.x = x
        box_pose.pose.position.y = y
        box_pose.pose.position.z = z  # height / 2 to make it sit on the ground

        path_object.add_box(box_names[i], box_pose, tall_box_size)
        print(f"Added box {box_names[i]} at position ({x}, {y}, {z})")

def spawn_hanoi_towers_c(path_object):
    '''Spawn towers in order: C A B'''
    tower_positions = [
        (p_Tower_x, 0.0, 0.0),
        (p_Tower_x, -p_Tower_y, 0.0),  # Tower C
        (p_Tower_x, p_Tower_y, 0.0),   # Tower A
    ]
    mesh_names = ['A', 'B', 'C']

    global tower_disks
    tower_disks = {'C': [], 'A': [], 'B': []}
    tower_coords = {'C': (p_Tower_x, p_Tower_y), 'A': (p_Tower_x, 0.0), 'B': (p_Tower_x, -p_Tower_y)}

    for i, (x, y, z) in enumerate(tower_positions):
        mesh_pose = geometry_msgs.msg.PoseStamped()
        mesh_pose.header.frame_id = 'world'
        mesh_pose.pose.position.x = x
        mesh_pose.pose.position.y = y
        mesh_pose.pose.position.z = z

        tower_name = mesh_names[i]
        path_object.add_mesh(tower_name, mesh_pose, MESH_FILE_PATH[i], (.00095, .00095, .00095))
        print(f"Added mesh {tower_name} at position ({x}, {y}, {z})")

    # Add two tall tower-like boxes as obstacles
    tall_box_size = (0.02, 0.02, 0.4)  # width, depth, height

    box_positions = [
        (p_Tower_x, p_Tower_y / 2, 0.2),     # center at mid-height (0.2)
        (p_Tower_x, -p_Tower_y / 2, 0.2)
    ]
    box_names = ['box1', 'box2']

    for i, (x, y, z) in enumerate(box_positions):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = 'world'
        box_pose.pose.position.x = x
        box_pose.pose.position.y = y
        box_pose.pose.position.z = z  # height / 2 to make it sit on the ground

        path_object.add_box(box_names[i], box_pose, tall_box_size)
        print(f"Added box {box_names[i]} at position ({x}, {y}, {z})")

def spawn_hanoi_towers_d(path_object):
    '''Spawn towers in order: B A C'''
    tower_positions = [
        (p_Tower_x, 0.0, 0.0),
        (p_Tower_x, p_Tower_y, 0.0), 
        (p_Tower_x, -p_Tower_y, 0.0)  
    ]
    mesh_names = ['A', 'B', 'C']

    global tower_disks
    tower_disks = {'B': [], 'A': [], 'C': []}
    tower_coords = {'B': (p_Tower_x, p_Tower_y), 'A': (p_Tower_x, 0.0), 'C': (p_Tower_x, -p_Tower_y)}

    for i, (x, y, z) in enumerate(tower_positions):
        mesh_pose = geometry_msgs.msg.PoseStamped()
        mesh_pose.header.frame_id = 'world'
        mesh_pose.pose.position.x = x
        mesh_pose.pose.position.y = y
        mesh_pose.pose.position.z = z

        tower_name = mesh_names[i]
        path_object.add_mesh(tower_name, mesh_pose, MESH_FILE_PATH[i], (.00095, .00095, .00095))
        print(f"Added mesh {tower_name} at position ({x}, {y}, {z})")

    # Add two tall tower-like boxes as obstacles
    tall_box_size = (0.02, 0.02, 0.4)  # width, depth, height

    box_positions = [
        (p_Tower_x, p_Tower_y / 2, 0.2),     # center at mid-height (0.2)
        (p_Tower_x, -p_Tower_y / 2, 0.2)
    ]
    box_names = ['box1', 'box2']

    for i, (x, y, z) in enumerate(box_positions):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = 'world'
        box_pose.pose.position.x = x
        box_pose.pose.position.y = y
        box_pose.pose.position.z = z  # height / 2 to make it sit on the ground

        path_object.add_box(box_names[i], box_pose, tall_box_size)
        print(f"Added box {box_names[i]} at position ({x}, {y}, {z})")

def spawn_hanoi_towers_e(path_object):
    '''Spawn towers in order: C B A'''
    tower_positions = [
        (p_Tower_x, -p_Tower_y, 0.0), 
        (p_Tower_x, 0.0, 0.0), 
        (p_Tower_x, p_Tower_y, 0.0)  
    ]
    mesh_names = ['A', 'B', 'C']

    global tower_disks
    tower_disks = {'C': [], 'B': [], 'A': []}
    tower_coords = {'C': (p_Tower_x, p_Tower_y), 'B': (p_Tower_x, 0.0), 'A': (p_Tower_x, -p_Tower_y)}

    for i, (x, y, z) in enumerate(tower_positions):
        mesh_pose = geometry_msgs.msg.PoseStamped()
        mesh_pose.header.frame_id = 'world'
        mesh_pose.pose.position.x = x
        mesh_pose.pose.position.y = y
        mesh_pose.pose.position.z = z

        tower_name = mesh_names[i]
        path_object.add_mesh(tower_name, mesh_pose, MESH_FILE_PATH[i], (.00095, .00095, .00095))
        print(f"Added mesh {tower_name} at position ({x}, {y}, {z})")

    # Add two tall tower-like boxes as obstacles
    tall_box_size = (0.02, 0.02, 0.4)  # width, depth, height

    box_positions = [
        (p_Tower_x, p_Tower_y / 2, 0.2),     # center at mid-height (0.2)
        (p_Tower_x, -p_Tower_y / 2, 0.2)
    ]
    box_names = ['box1', 'box2']

    for i, (x, y, z) in enumerate(box_positions):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = 'world'
        box_pose.pose.position.x = x
        box_pose.pose.position.y = y
        box_pose.pose.position.z = z  # height / 2 to make it sit on the ground

        path_object.add_box(box_names[i], box_pose, tall_box_size)
        print(f"Added box {box_names[i]} at position ({x}, {y}, {z})")

def spawn_hanoi_towers_f(path_object):
    '''Spawn towers in order: B C A'''
    tower_positions = [
        (p_Tower_x, -p_Tower_y, 0.0), 
        (p_Tower_x, p_Tower_y, 0.0),
        (p_Tower_x, 0.0, 0.0)  
    ]
    mesh_names = ['A', 'B', 'C']

    global tower_disks
    tower_disks = {'B': [], 'C': [], 'A': []}
    tower_coords = {'B': (p_Tower_x, p_Tower_y), 'C': (p_Tower_x, 0.0), 'A': (p_Tower_x, -p_Tower_y)}

    for i, (x, y, z) in enumerate(tower_positions):
        mesh_pose = geometry_msgs.msg.PoseStamped()
        mesh_pose.header.frame_id = 'world'
        mesh_pose.pose.position.x = x
        mesh_pose.pose.position.y = y
        mesh_pose.pose.position.z = z

        tower_name = mesh_names[i]
        path_object.add_mesh(tower_name, mesh_pose, MESH_FILE_PATH[i], (.00095, .00095, .00095))
        print(f"Added mesh {tower_name} at position ({x}, {y}, {z})")

    # Add two tall tower-like boxes as obstacles
    tall_box_size = (0.02, 0.02, 0.4)  # width, depth, height

    box_positions = [
        (p_Tower_x, p_Tower_y / 2, 0.2),     # center at mid-height (0.2)
        (p_Tower_x, -p_Tower_y / 2, 0.2)
    ]
    box_names = ['box1', 'box2']

    for i, (x, y, z) in enumerate(box_positions):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = 'world'
        box_pose.pose.position.x = x
        box_pose.pose.position.y = y
        box_pose.pose.position.z = z  # height / 2 to make it sit on the ground

        path_object.add_box(box_names[i], box_pose, tall_box_size)
        print(f"Added box {box_names[i]} at position ({x}, {y}, {z})")

def case_A(path_object):
    '''
    A B C
    '''
    from std_msgs.msg import Bool
    import rospy
    from math import pi
    global EefState, pub_EefState

    def safe_ik_move(x, y, z, desc=""):
        try:
            joint_solution = Your_IK(x, y, z)
            if not joint_solution or len(joint_solution) < 4:
                raise ValueError(f"IK failed at {desc} with ({x}, {y}, {z})")
            path_object.joint_angles = joint_solution
            path_object.go_to_joint_state()
            return True
        except Exception as e:
            print(f"[ERROR] Move failed at {desc}: {e}")
            return False

    try:
        print("Step 1: Moving above tower B (safe height)")
        if not safe_ik_move(0.25, 0, 0.15, "above tower B"):
            raise Exception("Can't move above tower B")

        print("Step 2: Lowering to pick from tower B")
        if not safe_ik_move(0.25, 0, 0.025, "pick from tower B"):
            raise Exception("Can't descend to tower B")

        print("Attaching mesh B")
        EefState = 1
        pub_EefState.publish(Bool(EefState))
        rospy.sleep(0.2)
        try:
            path_object.attach_mesh("B", "link5")
        except Exception as e:
            print(f"Visualization failed (attach B): {e}")

        print("Step 3: Lifting mesh B safely")
        if not safe_ik_move(0.25, 0, 0.15, "lift from tower B"):
            raise Exception("Can't lift from tower B")

        print("Step 4: Moving above tower A")
        if not safe_ik_move(0.25, 0.15, 0.15, "above tower A"):
            raise Exception("Can't move to tower A")

        print("Step 5: Lowering to place on tower A")
        if not safe_ik_move(0.25, 0.15, 0.04, "place on tower A"):
            raise Exception("Can't descend to tower A")

        print("Placing mesh B")
        EefState = 0
        pub_EefState.publish(Bool(EefState))
        rospy.sleep(0.2)
        try:
            path_object.detach_mesh("B", "link5")
        except Exception as e:
            print(f"Visualization failed (detach B): {e}")

        print("Step 6: Lifting arm to safe height")
        if not safe_ik_move(0.25, 0.15, 0.15, "lift from tower A"):
            raise Exception("Can't lift from tower A")

        print("Step 7: Moving above tower C")
        if not safe_ik_move(0.25, -0.15, 0.15, "above tower C"):
            raise Exception("Can't move to tower C")

        print("Step 8: Lowering to pick from tower C")
        if not safe_ik_move(0.25, -0.15, 0.025, "pick from tower C"):
            raise Exception("Can't descend to tower C")

        print("Attaching mesh C")
        EefState = 1
        pub_EefState.publish(Bool(EefState))
        rospy.sleep(0.2)
        try:
            path_object.attach_mesh("C", "link5")
        except Exception as e:
            print(f"Visualization failed (attach C): {e}")

        print("Step 9: Lifting from tower C")
        if not safe_ik_move(0.25, -0.15, 0.15, "lift from tower C"):
            raise Exception("Can't lift from tower C")

        print("Step 10: Moving above tower A")
        if not safe_ik_move(0.25, 0.15, 0.15, "above tower A again"):
            raise Exception("Can't move above tower A with C")

        print("Step 11: Lowering to place on top of B")
        if not safe_ik_move(0.25, 0.15, 0.06, "place on top of B"):
            raise Exception("Can't descend to place C")

        print("Placing mesh C")
        EefState = 0
        pub_EefState.publish(Bool(EefState))
        rospy.sleep(0.2)
        try:
            path_object.detach_mesh("C", "link5")
        except Exception as e:
            print(f"Visualization failed (detach C): {e}")
        
        print("Returning to original pose")
        path_object.joint_angles = [0.06161233, 0.56530055, 1.67981925, -0.67432347]
        path_object.go_to_joint_state()


        print("Case A execution completed successfully!")
        return True

    except Exception as e:
        print(f"[ABORTED] Case A failed: {e}")
        path_object.joint_angles = [0, -pi/2, pi/2, 0]
        path_object.go_to_joint_state()
        return False


def case_B(path_object):
    '''
    A C B
    '''
    from std_msgs.msg import Bool
    import rospy
    from math import pi
    global EefState, pub_EefState
    
    def safe_ik_move(x, y, z, desc=""):
        try:
            joint_solution = Your_IK(x, y, z)
            if not joint_solution or len(joint_solution) < 4:
                raise ValueError(f"IK failed at {desc} with ({x}, {y}, {z})")
            path_object.joint_angles = joint_solution
            path_object.go_to_joint_state()
            return True
        except Exception as e:
            print(f"[ERROR] Move failed at {desc}: {e}")
            return False

    try:
        
        print("Step 1: Moving to tower B")
        if not safe_ik_move(0.25, -0.15, 0.025, "above tower B"):
            raise Exception("Can't move above tower B")
        
        print("Step 2: Lowering to pick from tower B")
        if not safe_ik_move(0.25, -0.15, 0.025, "pick from tower B"):
            raise Exception("Can't descend to tower B")

        print("Attaching mesh B")
        EefState = 1
        pub_EefState.publish(Bool(EefState))
        rospy.sleep(0.2)
        try:
            path_object.attach_mesh("B", "link5")
        except Exception as e:
            print(f"Visualization failed (attach B): {e}")

        print("Step 3: Lifting mesh B safely")
        if not safe_ik_move(0.25, -0.15, 0.15, "lift from tower B"):
            raise Exception("Can't lift from tower B")

        print("Step 4: Moving above tower A")
        if not safe_ik_move(0.25, 0.15, 0.15, "above tower A"):
            raise Exception("Can't move to tower A")
        
        print("Step 5: Lowering to place on tower A")
        if not safe_ik_move(0.25, 0.15, 0.04, "place on tower A"):
            raise Exception("Can't descend to tower A")

        print("Placing mesh B")
        EefState = 0
        pub_EefState.publish(Bool(EefState))
        rospy.sleep(0.2)
        try:
            path_object.detach_mesh("B", "link5")
        except Exception as e:
            print(f"Visualization failed (detach B): {e}")

        print("Step 6: Lifting arm to safe height")
        if not safe_ik_move(0.25, 0.15, 0.15, "lift from tower A"):
            raise Exception("Can't lift from tower A")

        print("Step 7: Moving above tower C")
        if not safe_ik_move(0.25, 0, 0.15, "above tower C"):
            raise Exception("Can't move to tower C")

        print("Step 8: Lowering to pick from tower C")
        if not safe_ik_move(0.25, 0, 0.025, "pick from tower C"):
            raise Exception("Can't descend to tower C")

        print("Attaching mesh C")
        EefState = 1
        pub_EefState.publish(Bool(EefState))
        rospy.sleep(0.2)
        try:
            path_object.attach_mesh("C", "link5")
        except Exception as e:
            print(f"Visualization failed (attach C): {e}")

        print("Step 9: Lifting from tower C")
        if not safe_ik_move(0.25, 0, 0.15, "lift from tower C"):
            raise Exception("Can't lift from tower C")

        print("Step 10: Moving above tower A")
        if not safe_ik_move(0.25, 0.15, 0.15, "above tower A again"):
            raise Exception("Can't move above tower A with C")

        print("Step 11: Lowering to place on top of B")
        if not safe_ik_move(0.25, 0.15, 0.06, "place on top of B"):
            raise Exception("Can't descend to place C")

        print("Placing mesh C")
        EefState = 0
        pub_EefState.publish(Bool(EefState))
        rospy.sleep(0.2)
        try:
            path_object.detach_mesh("C", "link5")
        except Exception as e:
            print(f"Visualization failed (detach C): {e}")
        
        print("Returning to original pose")
        path_object.joint_angles = [0.06161233, 0.56530055, 1.67981925, -0.67432347]
        path_object.go_to_joint_state()

        print("Case B execution completed successfully!")
        return True
        
    except Exception as e:
        print(f"Error executing Case 1: {e}")
        # Go to safe position in case of error
        path_object.joint_angles = [0, -pi/2, pi/2, 0]
        path_object.go_to_joint_state()
        return False


def case_C(path_object):
    '''Solve C A B configuration'''
    from std_msgs.msg import Bool
    import rospy
    from math import pi
    global EefState, pub_EefState

    def safe_ik_move(x, y, z, desc=""):
        try:
            joint_solution = Your_IK(x, y, z)
            if not joint_solution or len(joint_solution) < 4:
                raise ValueError(f"IK failed at {desc} with ({x}, {y}, {z})")
            path_object.joint_angles = joint_solution
            path_object.go_to_joint_state()
            return True
        except Exception as e:
            print(f"[ERROR] Move failed at {desc}: {e}")
            return False

    try:
        # CAB

        print("Step 1: Moving to tower B")
        if not safe_ik_move(0.25, -0.15, 0.025, "above tower B"):
            raise Exception("Can't move above tower B")
        
        print("Step 2: Lowering to pick from tower B")
        if not safe_ik_move(0.25, -0.15, 0.025, "pick from tower B"):
            raise Exception("Can't descend to tower B")

        print("Attaching mesh B")
        EefState = 1
        pub_EefState.publish(Bool(EefState))
        rospy.sleep(0.2)
        try:
            path_object.attach_mesh("B", "link5")
        except Exception as e:
            print(f"Visualization failed (attach B): {e}")

        print("Step 3: Lifting mesh B safely")
        if not safe_ik_move(0.25, -0.15, 0.15, "lift from tower B"):
            raise Exception("Can't lift from tower B")

        print("Step 4: Moving above tower A")
        if not safe_ik_move(0.25, 0.0, 0.15, "above tower A"):
            raise Exception("Can't move to tower A")
        
        print("Step 5: Lowering to place on tower A")
        if not safe_ik_move(0.25, 0.0, 0.04, "place on tower A"):
            raise Exception("Can't descend to tower A")

        print("Placing mesh B")
        EefState = 0
        pub_EefState.publish(Bool(EefState))
        rospy.sleep(0.2)
        try:
            path_object.detach_mesh("B", "link5")
        except Exception as e:
            print(f"Visualization failed (detach B): {e}")

        print("Step 6: Lifting arm to safe height")
        if not safe_ik_move(0.25, 0.0, 0.15, "lift from tower A"):
            raise Exception("Can't lift from tower A")

        print("Step 7: Moving above tower C")
        if not safe_ik_move(0.25, 0.15, 0.15, "above tower C"):
            raise Exception("Can't move to tower C")

        print("Step 8: Lowering to pick from tower C")
        if not safe_ik_move(0.25, 0.15, 0.025, "pick from tower C"):
            raise Exception("Can't descend to tower C")

        print("Attaching mesh C")
        EefState = 1
        pub_EefState.publish(Bool(EefState))
        rospy.sleep(0.2)
        try:
            path_object.attach_mesh("C", "link5")
        except Exception as e:
            print(f"Visualization failed (attach C): {e}")

        print("Step 9: Lifting from tower C")
        if not safe_ik_move(0.25, 0.15, 0.15, "lift from tower C"):
            raise Exception("Can't lift from tower C")

        print("Step 10: Moving above tower A")
        if not safe_ik_move(0.25, 0.0, 0.15, "above tower A again"):
            raise Exception("Can't move above tower A with C")

        print("Step 11: Lowering to place on top of B")
        if not safe_ik_move(0.25, 0.0, 0.06, "place on top of B"):
            raise Exception("Can't descend to place C")

        print("Placing mesh C")
        EefState = 0
        pub_EefState.publish(Bool(EefState))
        rospy.sleep(0.2)
        try:
            path_object.detach_mesh("C", "link5")
        except Exception as e:
            print(f"Visualization failed (detach C): {e}")
        
        print("Returning to original pose")
        path_object.joint_angles = [0.06161233, 0.56530055, 1.67981925, -0.67432347]
        path_object.go_to_joint_state()

        print("Case C execution completed successfully!")
        return True

    except Exception as e:
        print(f"[ABORTED] Case C failed: {e}")
        path_object.joint_angles = [0, -pi/2, pi/2, 0]
        path_object.go_to_joint_state()
        return False

def case_D(path_object):
    '''Solve B A C configuration'''
    from std_msgs.msg import Bool
    import rospy
    from math import pi
    global EefState, pub_EefState

    def safe_ik_move(x, y, z, desc=""):
        try:
            joint_solution = Your_IK(x, y, z)
            if not joint_solution or len(joint_solution) < 4:
                raise ValueError(f"IK failed at {desc} with ({x}, {y}, {z})")
            path_object.joint_angles = joint_solution
            path_object.go_to_joint_state()
            return True
        except Exception as e:
            print(f"[ERROR] Move failed at {desc}: {e}")
            return False

    try:
        # B A C

        print("Step 1: Moving to tower B")
        if not safe_ik_move(0.25, 0.15, 0.025, "above tower B"):
            raise Exception("Can't move above tower B")
        
        print("Step 2: Lowering to pick from tower B")
        if not safe_ik_move(0.25, 0.15, 0.025, "pick from tower B"):
            raise Exception("Can't descend to tower B")

        print("Attaching mesh B")
        EefState = 1
        pub_EefState.publish(Bool(EefState))
        rospy.sleep(0.2)
        try:
            path_object.attach_mesh("B", "link5")
        except Exception as e:
            print(f"Visualization failed (attach B): {e}")

        print("Step 3: Lifting mesh B safely")
        if not safe_ik_move(0.25, 0.15, 0.15, "lift from tower B"):
            raise Exception("Can't lift from tower B")

        print("Step 4: Moving above tower A")
        if not safe_ik_move(0.25, 0.0, 0.15, "above tower A"):
            raise Exception("Can't move to tower A")
        
        print("Step 5: Lowering to place on tower A")
        if not safe_ik_move(0.25, 0.0, 0.04, "place on tower A"):
            raise Exception("Can't descend to tower A")

        print("Placing mesh B")
        EefState = 0
        pub_EefState.publish(Bool(EefState))
        rospy.sleep(0.2)
        try:
            path_object.detach_mesh("B", "link5")
        except Exception as e:
            print(f"Visualization failed (detach B): {e}")

        print("Step 6: Lifting arm to safe height")
        if not safe_ik_move(0.25, 0.0, 0.15, "lift from tower A"):
            raise Exception("Can't lift from tower A")

        print("Step 7: Moving above tower C")
        if not safe_ik_move(0.25, -0.15, 0.15, "above tower C"):
            raise Exception("Can't move to tower C")

        print("Step 8: Lowering to pick from tower C")
        if not safe_ik_move(0.25, -0.15, 0.025, "pick from tower C"):
            raise Exception("Can't descend to tower C")

        print("Attaching mesh C")
        EefState = 1
        pub_EefState.publish(Bool(EefState))
        rospy.sleep(0.2)
        try:
            path_object.attach_mesh("C", "link5")
        except Exception as e:
            print(f"Visualization failed (attach C): {e}")

        print("Step 9: Lifting from tower C")
        if not safe_ik_move(0.25, -0.15, 0.15, "lift from tower C"):
            raise Exception("Can't lift from tower C")

        print("Step 10: Moving above tower A")
        if not safe_ik_move(0.25, 0.0, 0.15, "above tower A again"):
            raise Exception("Can't move above tower A with C")

        print("Step 11: Lowering to place on top of B")
        if not safe_ik_move(0.25, 0.0, 0.06, "place on top of B"):
            raise Exception("Can't descend to place C")

        print("Placing mesh C")
        EefState = 0
        pub_EefState.publish(Bool(EefState))
        rospy.sleep(0.2)
        try:
            path_object.detach_mesh("C", "link5")
        except Exception as e:
            print(f"Visualization failed (detach C): {e}")
        
        print("Returning to original pose")
        path_object.joint_angles = [0.06161233, 0.56530055, 1.67981925, -0.67432347]
        path_object.go_to_joint_state()

        print("Case C execution completed successfully!")
        return True

    except Exception as e:
        print(f"[ABORTED] Case C failed: {e}")
        path_object.joint_angles = [0, -pi/2, pi/2, 0]
        path_object.go_to_joint_state()
        return False

def case_E(path_object):
    '''Solve C B A configuration'''
    from std_msgs.msg import Bool
    import rospy
    from math import pi
    global EefState, pub_EefState

    def safe_ik_move(x, y, z, desc=""):
        try:
            joint_solution = Your_IK(x, y, z)
            if not joint_solution or len(joint_solution) < 4:
                raise ValueError(f"IK failed at {desc} with ({x}, {y}, {z})")
            path_object.joint_angles = joint_solution
            path_object.go_to_joint_state()
            return True
        except Exception as e:
            print(f"[ERROR] Move failed at {desc}: {e}")
            return False

    try:
        # C B A

        print("Step 1: Moving to tower B")
        if not safe_ik_move(0.25, 0.0, 0.025, "above tower B"):
            raise Exception("Can't move above tower B")
        
        print("Step 2: Lowering to pick from tower B")
        if not safe_ik_move(0.25, 0.0, 0.025, "pick from tower B"):
            raise Exception("Can't descend to tower B")

        print("Attaching mesh B")
        EefState = 1
        pub_EefState.publish(Bool(EefState))
        rospy.sleep(0.2)
        try:
            path_object.attach_mesh("B", "link5")
        except Exception as e:
            print(f"Visualization failed (attach B): {e}")

        print("Step 3: Lifting mesh B safely")
        if not safe_ik_move(0.25, 0.0, 0.15, "lift from tower B"):
            raise Exception("Can't lift from tower B")

        print("Step 4: Moving above tower A")
        if not safe_ik_move(0.25, -0.15, 0.15, "above tower A"):
            raise Exception("Can't move to tower A")
        
        print("Step 5: Lowering to place on tower A")
        if not safe_ik_move(0.25, -0.15, 0.04, "place on tower A"):
            raise Exception("Can't descend to tower A")

        print("Placing mesh B")
        EefState = 0
        pub_EefState.publish(Bool(EefState))
        rospy.sleep(0.2)
        try:
            path_object.detach_mesh("B", "link5")
        except Exception as e:
            print(f"Visualization failed (detach B): {e}")

        print("Step 6: Lifting arm to safe height")
        if not safe_ik_move(0.25, -0.15, 0.15, "lift from tower A"):
            raise Exception("Can't lift from tower A")

        print("Step 7: Moving above tower C")
        if not safe_ik_move(0.25, 0.15, 0.15, "above tower C"):
            raise Exception("Can't move to tower C")

        print("Step 8: Lowering to pick from tower C")
        if not safe_ik_move(0.25, 0.15, 0.025, "pick from tower C"):
            raise Exception("Can't descend to tower C")

        print("Attaching mesh C")
        EefState = 1
        pub_EefState.publish(Bool(EefState))
        rospy.sleep(0.2)
        try:
            path_object.attach_mesh("C", "link5")
        except Exception as e:
            print(f"Visualization failed (attach C): {e}")

        print("Step 9: Lifting from tower C")
        if not safe_ik_move(0.25, 0.15, 0.15, "lift from tower C"):
            raise Exception("Can't lift from tower C")

        print("Step 10: Moving above tower A")
        if not safe_ik_move(0.25, -0.15, 0.15, "above tower A again"):
            raise Exception("Can't move above tower A with C")

        print("Step 11: Lowering to place on top of B")
        if not safe_ik_move(0.25, -0.15, 0.06, "place on top of B"):
            raise Exception("Can't descend to place C")

        print("Placing mesh C")
        EefState = 0
        pub_EefState.publish(Bool(EefState))
        rospy.sleep(0.2)
        try:
            path_object.detach_mesh("C", "link5")
        except Exception as e:
            print(f"Visualization failed (detach C): {e}")
        
        print("Returning to original pose")
        path_object.joint_angles = [0.06161233, 0.56530055, 1.67981925, -0.67432347]
        path_object.go_to_joint_state()

        print("Case C execution completed successfully!")
        return True

    except Exception as e:
        print(f"[ABORTED] Case C failed: {e}")
        path_object.joint_angles = [0, -pi/2, pi/2, 0]
        path_object.go_to_joint_state()
        return False

def case_F(path_object):
    '''Solve B C A configuration'''
    from std_msgs.msg import Bool
    import rospy
    from math import pi
    global EefState, pub_EefState

    def safe_ik_move(x, y, z, desc=""):
        try:
            joint_solution = Your_IK(x, y, z)
            if not joint_solution or len(joint_solution) < 4:
                raise ValueError(f"IK failed at {desc} with ({x}, {y}, {z})")
            path_object.joint_angles = joint_solution
            path_object.go_to_joint_state()
            return True
        except Exception as e:
            print(f"[ERROR] Move failed at {desc}: {e}")
            return False

    try:
        # B C A

        print("Step 1: Moving to tower B")
        if not safe_ik_move(0.25, 0.15, 0.025, "above tower B"):
            raise Exception("Can't move above tower B")
        
        print("Step 2: Lowering to pick from tower B")
        if not safe_ik_move(0.25, 0.15, 0.025, "pick from tower B"):
            raise Exception("Can't descend to tower B")

        print("Attaching mesh B")
        EefState = 1
        pub_EefState.publish(Bool(EefState))
        rospy.sleep(0.2)
        try:
            path_object.attach_mesh("B", "link5")
        except Exception as e:
            print(f"Visualization failed (attach B): {e}")

        print("Step 3: Lifting mesh B safely")
        if not safe_ik_move(0.25, 0.15, 0.15, "lift from tower B"):
            raise Exception("Can't lift from tower B")

        print("Step 4: Moving above tower A")
        if not safe_ik_move(0.25, -0.15, 0.15, "above tower A"):
            raise Exception("Can't move to tower A")
        
        print("Step 5: Lowering to place on tower A")
        if not safe_ik_move(0.25, -0.15, 0.04, "place on tower A"):
            raise Exception("Can't descend to tower A")

        print("Placing mesh B")
        EefState = 0
        pub_EefState.publish(Bool(EefState))
        rospy.sleep(0.2)
        try:
            path_object.detach_mesh("B", "link5")
        except Exception as e:
            print(f"Visualization failed (detach B): {e}")

        print("Step 6: Lifting arm to safe height")
        if not safe_ik_move(0.25, -0.15, 0.15, "lift from tower A"):
            raise Exception("Can't lift from tower A")

        print("Step 7: Moving above tower C")
        if not safe_ik_move(0.25, 0.0, 0.15, "above tower C"):
            raise Exception("Can't move to tower C")

        print("Step 8: Lowering to pick from tower C")
        if not safe_ik_move(0.25, 0.0, 0.025, "pick from tower C"):
            raise Exception("Can't descend to tower C")

        print("Attaching mesh C")
        EefState = 1
        pub_EefState.publish(Bool(EefState))
        rospy.sleep(0.2)
        try:
            path_object.attach_mesh("C", "link5")
        except Exception as e:
            print(f"Visualization failed (attach C): {e}")

        print("Step 9: Lifting from tower C")
        if not safe_ik_move(0.25, 0.0, 0.15, "lift from tower C"):
            raise Exception("Can't lift from tower C")

        print("Step 10: Moving above tower A")
        if not safe_ik_move(0.25, -0.15, 0.15, "above tower A again"):
            raise Exception("Can't move above tower A with C")

        print("Step 11: Lowering to place on top of B")
        if not safe_ik_move(0.25, -0.15, 0.06, "place on top of B"):
            raise Exception("Can't descend to place C")

        print("Placing mesh C")
        EefState = 0
        pub_EefState.publish(Bool(EefState))
        rospy.sleep(0.2)
        try:
            path_object.detach_mesh("C", "link5")
        except Exception as e:
            print(f"Visualization failed (detach C): {e}")
        
        print("Returning to original pose")
        path_object.joint_angles = [0.06161233, 0.56530055, 1.67981925, -0.67432347]
        path_object.go_to_joint_state()

        print("Case C execution completed successfully!")
        return True

    except Exception as e:
        print(f"[ABORTED] Case C failed: {e}")
        path_object.joint_angles = [0, -pi/2, pi/2, 0]
        path_object.go_to_joint_state()
        return False


def case_1(path_object):
    # 1-2
    print("Step 1: Moving C")
    path_object.joint_angles = Your_IK(0.25, 0.15, 0.06)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.5)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("C", "link5")
        print("Mesh C attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    path_object.joint_angles = Your_IK(0.25, 0, 0.025)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("C", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    print("Step 2: Moving B")
    path_object.joint_angles = Your_IK(0.25, 0.15, 0.04)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("B", "link5")
        print("Mesh B attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    path_object.joint_angles = Your_IK(0.25, -0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("B", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    print("Step 3: Moving C")
    path_object.joint_angles = Your_IK(0.25, 0, 0.025)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("C", "link5")
        print("Mesh C attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    path_object.joint_angles = Your_IK(0.25, -0.15, 0.05)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("C", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    print("Step 4: Moving A")
    path_object.joint_angles = Your_IK(0.25, 0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("A", "link5")
        print("Mesh A attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    path_object.joint_angles = Your_IK(0.25, 0, 0.025)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("A", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    print("Step 5: Moving C")
    path_object.joint_angles = Your_IK(0.25, -0.15, 0.05)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("C", "link5")
        print("Mesh C attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    path_object.joint_angles = Your_IK(0.25, 0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("C", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    print("Step 6: Moving B")
    path_object.joint_angles = Your_IK(0.25, -0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("B", "link5")
        print("Mesh B attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    path_object.joint_angles = Your_IK(0.25, 0, 0.05)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("B", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    print("Step 7: Moving C")
    path_object.joint_angles = Your_IK(0.25, 0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("C", "link5")
        print("Mesh C attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    path_object.joint_angles = Your_IK(0.25, 0, 0.075)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("C", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

def case_2(path_object):
    # 1-3
    print("Step 1: Moving C")
    path_object.joint_angles = Your_IK(0.25, 0.15, 0.06)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.5)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("C", "link5")
        print("Mesh C attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, -0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("C", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    print("Step 2: Moving B")
    path_object.joint_angles = Your_IK(0.25, 0.15, 0.04)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("B", "link5")
        print("Mesh B attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, 0.0, 0.025)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("B", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    print("Step 3: Moving C")
    path_object.joint_angles = Your_IK(0.25, -0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("C", "link5")
        print("Mesh C attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, 0.0, 0.05)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("C", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    print("Step 4: Moving A")
    path_object.joint_angles = Your_IK(0.25, 0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("A", "link5")
        print("Mesh A attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, -0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("A", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    print("Step 5: Moving C")
    path_object.joint_angles = Your_IK(0.25, 0.0, 0.05)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("C", "link5")
        print("Mesh C attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, 0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("C", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    print("Step 6: Moving B")
    path_object.joint_angles = Your_IK(0.25, 0.0, 0.025)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("B", "link5")
        print("Mesh B attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, -0.15, 0.05)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("B", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    print("Step 7: Moving C")
    path_object.joint_angles = Your_IK(0.25, 0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("C", "link5")
        print("Mesh C attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, -0.15, 0.075)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("C", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
def case_3(path_object):
    # 2-1
    print("Step 1: Moving C")
    path_object.joint_angles = Your_IK(0.25, 0.0, 0.06)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.5)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("C", "link5")
        print("Mesh C attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, 0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("C", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    print("Step 2: Moving B")
    path_object.joint_angles = Your_IK(0.25, 0.0, 0.04)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("B", "link5")
        print("Mesh B attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, -0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("B", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    print("Step 3: Moving C")
    path_object.joint_angles = Your_IK(0.25, 0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("C", "link5")
        print("Mesh C attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, -0.15, 0.05)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("C", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    print("Step 4: Moving A")
    path_object.joint_angles = Your_IK(0.25, 0.0, 0.025)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("A", "link5")
        print("Mesh A attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, 0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("A", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    print("Step 5: Moving C")
    path_object.joint_angles = Your_IK(0.25, -0.15, 0.05)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("C", "link5")
        print("Mesh C attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, 0.0, 0.025)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("C", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")    

    print("Step 6: Moving B")
    path_object.joint_angles = Your_IK(0.25, -0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("B", "link5")
        print("Mesh B attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, 0.15, 0.05)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("B", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    print("Step 7: Moving C")
    path_object.joint_angles = Your_IK(0.25, 0.0, 0.025)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("C", "link5")
        print("Mesh C attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, 0.15, 0.075)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("C", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

def case_4(path_object):
    # 2-3
    print("Step 1: Moving C")
    path_object.joint_angles = Your_IK(0.25, 0.0, 0.06)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.5)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("C", "link5")
        print("Mesh C attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    path_object.joint_angles = Your_IK(0.25, -0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("C", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    print("Step 2: Moving B")
    path_object.joint_angles = Your_IK(0.25, 0.0, 0.04)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("B", "link5")
        print("Mesh B attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    path_object.joint_angles = Your_IK(0.25, 0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("B", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    print("Step 3: Moving C")
    path_object.joint_angles = Your_IK(0.25, -0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("C", "link5")
        print("Mesh C attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    path_object.joint_angles = Your_IK(0.25, 0.15, 0.05)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("C", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    print("Step 4: Moving A")
    path_object.joint_angles = Your_IK(0.25, 0.0, 0.025)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("A", "link5")
        print("Mesh A attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    path_object.joint_angles = Your_IK(0.25, -0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("A", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    print("Step 5: Moving C")
    path_object.joint_angles = Your_IK(0.25, 0.15, 0.05)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("C", "link5")
        print("Mesh C attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    path_object.joint_angles = Your_IK(0.25, 0.0, 0.025)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("C", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    print("Step 6: Moving B")
    path_object.joint_angles = Your_IK(0.25, 0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("B", "link5")
        print("Mesh B attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    path_object.joint_angles = Your_IK(0.25, -0.15, 0.05)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("B", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    print("Step 7: Moving C")
    path_object.joint_angles = Your_IK(0.25, 0.0, 0.025)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("C", "link5")
        print("Mesh C attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    path_object.joint_angles = Your_IK(0.25, -0.15, 0.075)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("C", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

def case_5(path_object):
    # 3-1
    print("Step 1: Moving C")
    path_object.joint_angles = Your_IK(0.25, -0.15, 0.06)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.5)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("C", "link5")
        print("Mesh C attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, 0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("C", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    print("Step 2: Moving B")
    path_object.joint_angles = Your_IK(0.25, -0.15, 0.04)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("B", "link5")
        print("Mesh B attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, 0.0, 0.025)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("B", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    print("Step 3: Moving C")
    path_object.joint_angles = Your_IK(0.25, 0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("C", "link5")
        print("Mesh C attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, 0.0, 0.05)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("C", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    print("Step 4: Moving A")
    path_object.joint_angles = Your_IK(0.25, -0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("A", "link5")
        print("Mesh A attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, 0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("A", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    print("Step 5: Moving C")
    path_object.joint_angles = Your_IK(0.25, 0.0, 0.05)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("C", "link5")
        print("Mesh C attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, -0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("C", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    print("Step 6: Moving B")
    path_object.joint_angles = Your_IK(0.25, 0.0, 0.025)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("B", "link5")
        print("Mesh B attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, 0.15, 0.05)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("B", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    print("Step 7: Moving C")
    path_object.joint_angles = Your_IK(0.25, -0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("C", "link5")
        print("Mesh C attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, 0.15, 0.075)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("C", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

def case_6(path_object):
    # 3-2
    print("Step 1: Moving C")
    path_object.joint_angles = Your_IK(0.25, -0.15, 0.06)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.5)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("C", "link5")
        print("Mesh C attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, 0, 0.025)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("C", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    print("Step 2: Moving B")
    path_object.joint_angles = Your_IK(0.25, -0.15, 0.04)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("B", "link5")
        print("Mesh B attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, 0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("B", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    print("Step 3: Moving C")
    path_object.joint_angles = Your_IK(0.25, 0, 0.025)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("C", "link5")
        print("Mesh C attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, 0.15, 0.05)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("C", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    print("Step 4: Moving A")
    path_object.joint_angles = Your_IK(0.25, -0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("A", "link5")
        print("Mesh A attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, 0, 0.025)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("A", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    print("Step 5: Moving C")
    path_object.joint_angles = Your_IK(0.25, 0.15, 0.05)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("C", "link5")
        print("Mesh C attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, -0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("C", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    print("Step 6: Moving B")
    path_object.joint_angles = Your_IK(0.25, 0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("B", "link5")
        print("Mesh B attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, 0, 0.05)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("B", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
    
    print("Step 7: Moving C")
    path_object.joint_angles = Your_IK(0.25, -0.15, 0.025)
    path_object.go_to_joint_state()

    EefState = 1  # Turn magnet on
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to activate
    # For visualization only
    try:
        path_object.attach_mesh("C", "link5")
        print("Mesh C attached to end effector")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")

    path_object.joint_angles = Your_IK(0.25, 0, 0.075)
    path_object.go_to_joint_state()

    EefState = 0  # Turn magnet off
    pub_EefState.publish(Bool(EefState))
    rospy.sleep(0.2)  # Wait for magnet to deactivate
    # For visualization only
    try:
        path_object.detach_mesh("C", "link5")
    except Exception as e:
        print(f"Note: Visualization failed but continuing: {e}")
# modified
def voice_callback(msg):
    global global_path_object
    case_map = {
        'case1': case_1,
        'case2': case_2,
        'case3': case_3,
        'case4': case_4,
        'case5': case_5,
        'case6': case_6,
    }
    key = msg.data.lower().replace(" ", "")
    print(f"Received voice command: {key}")
    
    if key in case_map:
        try:
            print(f"Executing {key}...")
            case_map[key](global_path_object)
            print(f"{key} execution completed!")
        except Exception as e:
            print(f"Error executing {key}: {e}")
    else:
        print(f"No matching case function for command: {key}")

def hanoi_tower(path_object, n_disks=3):
    pass


def main():
    global pub_EefState, EefState, global_path_object
    
    try:
        # Initialize the path planning object
        path_object = MoveGroupPythonIntefaceTutorial()
        global_path_object = path_object  # Set global reference for voice callback
        
        # Initialize end-effector publisher
        pub_EefState_to_arm()

        # Initialize voice command subscriber
        rospy.Subscriber('/voice_case_cmd', String, voice_callback)

        print("ctrl + z to close")
        
        print("Hanoi Tower Robot Controller")

        print("  1: Run Case_1, 1-2")
        print("  2: Run Case_2, 1-3")
        print("  3: Run Case_3, 2-1")
        print("  4: Run Case_4, 2-3")
        print("  5: Run Case_5, 3-1")
        print("  6: Run Case_6, 3-2")

        print("  m: Manual control")
        print("  q: Quit")
        
        def wait_for_voice_or_keyboard():
            """Wait for either voice command or keyboard input"""
            print("Listening for voice commands (case1-case6) or press Enter to continue with keyboard...")
            
            # Simple way to wait - you might want to implement a more sophisticated method
            try:
                user_input = input("Press Enter to return to keyboard control, or 'q' to quit: ").strip()
                if user_input.lower() == 'q':
                    return 'quit'
                return 'continue'
            except KeyboardInterrupt:
                return 'quit'
        image_path = capture_image_after_delay()
        if image_path:
            image = cv2.imread(image_path)
            vision_order = detect_color_positions(image)
            print(f"📷 Vision 辨識順序為: {vision_order}")
        vo = vision_order[::-1]
        while not rospy.is_shutdown():
            try:
                 # Visual Detection
                
                '''
                # Get the Start Location of Tower1
                index = vision_order.index(1)
                start_tower1 =  ['A', 'B', 'C'][index]

                # Add Mesh to RViz
                peg_order2 = [peg for _, peg in sorted(zip(vision_order, ['A', 'B', 'C']))]
                peg_order = ['A', 'B', 'C']
                tower_position = {size: peg_order[i] for i, size in enumerate(vision_order)}
                add_scene_objects(path_object.scene, peg_order2)
                print("✅ 場景物件已加入")
                '''
                #command = input("Enter command: ").strip()
                
                if vo== [1,2,3]:
                    print("Spawn A B C...")
                    spawn_hanoi_towers_a(path_object)
                    print("solve a")
                    case_A(path_object)
                    # Enable voice commands after solving
                    result = wait_for_voice_or_keyboard()
                    if result == 'quit':
                        break

                elif vo == [1,3,2]:
                    print("Spawn A C B..")
                    spawn_hanoi_towers_b(path_object)
                    print("solve b")
                    case_B(path_object)
                    # Enable voice commands after solving
                    result = wait_for_voice_or_keyboard()
                    if result == 'quit':
                        break

                elif vo== [3,1,2]:
                    print("Spawn C A B..")
                    spawn_hanoi_towers_c(path_object)
                    print("solve c")
                    case_C(path_object)
                    # Enable voice commands after solving
                    result = wait_for_voice_or_keyboard()
                    if result == 'quit':
                        break
                
                elif vo== [2,1,3]:
                    print("Spawn B A C..")
                    spawn_hanoi_towers_d(path_object)
                    print("solve d")
                    case_D(path_object)
                    # Enable voice commands after solving
                    result = wait_for_voice_or_keyboard()
                    if result == 'quit':
                        break

                elif vo== [3,2,1]:
                    print("Spawn C B A..")
                    spawn_hanoi_towers_e(path_object) 
                    print("solve e")
                    case_E(path_object)
                    # Enable voice commands after solving
                    result = wait_for_voice_or_keyboard()
                    if result == 'quit':
                        break
                    
                elif vo== [2,3,1]:
                    print("Spawn B C A..")
                    spawn_hanoi_towers_f(path_object)
                    print("solve f")
                    case_F(path_object)
                    # Enable voice commands after solving
                    result = wait_for_voice_or_keyboard()
                    if result == 'quit':
                        break
                
                elif command == 'm':
                    print("Manual control mode")
                    x_input = float(input("x: "))
                    y_input = float(input("y: "))
                    z_input = float(input("z: "))
                    eef_input = int(input("End-effector (1=on, 0=off): "))
                    
                    # Execute single command
                    path_object.joint_angles = Your_IK(x_input, y_input, z_input)
                    path_object.go_to_joint_state()
                    
                    EefState = eef_input
                    pub_EefState.publish(Bool(EefState))

                elif command == 'q':
                    break
                    
                else:
                    print("Invalid command")
                

            except ValueError:
                print("Invalid input. Please try again.")
            except Exception as e:
                print(f"Error: {e}")
                # Go back to home if weird input
                path_object.joint_angles = [0, -pi/2, pi/2, 0]
                path_object.go_to_joint_state()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()