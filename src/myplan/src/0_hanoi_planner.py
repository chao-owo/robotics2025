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
    
    # FIX: Convert numpy arrays to plain Python floats
    result = joint_angle.flatten().tolist()
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
def spawn_hanoi_towers(path_object, vo_list):
    '''Spawn the three Hanoi towers in the scene with positions determined by vo_list'''
    
    # Base tower positions (A, B, C correspond to indices 0, 1, 2)
    base_positions = [
        (p_Tower_x, p_Tower_y, 0.0),   # Position for tower A (index 0)
        (p_Tower_x, 0.0, 0.0),         # Position for tower B (index 1) 
        (p_Tower_x, -p_Tower_y, 0.0)  # Position for tower C (index 2)
    ]
    
    # Reorder positions based on vo_list
    # vo_list contains the desired order (1=A, 2=B, 3=C)
    # Convert to 0-based indices and reorder positions
    tower_positions = []
    for tower_id in vo_list:
        tower_positions.append(base_positions[tower_id - 1])  # Convert 1,2,3 to 0,1,2
    
    mesh_names = ['A', 'B', 'C']
    
    # Initialize tower_disks dictionary
    global tower_disks
    tower_disks = {'A': [],'B': [],'C': [] }
    
    # Update tower_coords to match the new positions
    global tower_coords
    tower_coords = {}
    for i, tower_name in enumerate(mesh_names):
        x, y, z = tower_positions[i]
        tower_coords[tower_name] = (x, y)
    
    # Spawn towers at reordered positions
    for i, (x, y, z) in enumerate(tower_positions):
        mesh_pose = geometry_msgs.msg.PoseStamped()
        mesh_pose.header.frame_id = 'world'
        mesh_pose.pose.position.x = x
        mesh_pose.pose.position.y = y
        mesh_pose.pose.position.z = z

        tower_name = mesh_names[i]
        path_object.add_mesh(tower_name, mesh_pose, MESH_FILE_PATH[i], (.00095, .00095, .00095))
        print(f"Added {tower_name} at position ({x}, {y}, {z})")

    # Add two tall tower-like boxes as obstacles (unchanged)
    tall_box_size = (0.010, 0.010, 0.4)  # width, depth, height

    box_positions = [
        (p_Tower_x, p_Tower_y / 2, 0.2),  # center at mid-height (0.2)
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


def solve(path_object, vo_list):
    '''
    Args:
        path_object: Robot path planning object
        vo_list: List of 3 integers [1,2,3] representing tower positions
                 where 1=left(0.15), 2=center(0.0), 3=right(-0.15)
    
    Returns:
        int: Position of tower A (vo_list[0])
    '''
    
    # Get tower positions from the global tower_coords set by spawn_hanoi_towers_a
    global tower_disks
    tower_positions = {
        'A': tower_coords['A'][1],  # Get y-coordinate for Tower A
        'B': tower_coords['B'][1],  # Get y-coordinate for Tower B  
        'C': tower_coords['C'][1]   # Get y-coordinate for Tower C
    }
    
    def safe_ik_move(x, y, z, desc=""):
        """Safe inverse kinematics move with error handling"""
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
    
    def move_disk(from_tower, to_tower, disk_name, target_height):
        """Move a disk from one tower to another at specified height"""
        from_y = tower_positions[from_tower]
        to_y = tower_positions[to_tower]
        
        # Heights for picking up and placing
        pick_height = 0.025  # Standard pick height
        safe_height = 0.15   # Safe travel height
        
        try:
            # Move above source tower
            print(f"Moving above tower {from_tower}")
            if not safe_ik_move(0.25, from_y, safe_height, f"above tower {from_tower}"):
                raise Exception(f"Can't move above tower {from_tower}")
            
            # Lower to pick up disk
            print(f"Lowering to pick {disk_name} from tower {from_tower}")
            if not safe_ik_move(0.25, from_y, pick_height, f"pick {disk_name} from tower {from_tower}"):
                raise Exception(f"Can't descend to tower {from_tower}")
            
            # Attach disk
            path_object.attach_mesh(disk_name, "link5")
            print(f"Attached {disk_name}")
            
            # Lift disk safely
            print(f"Lifting {disk_name} from tower {from_tower}")
            if not safe_ik_move(0.25, from_y, safe_height, f"lift {disk_name} from tower {from_tower}"):
                raise Exception(f"Can't lift from tower {from_tower}")
            
            # Move above destination tower
            print(f"Moving above tower {to_tower}")
            if not safe_ik_move(0.25, to_y, safe_height, f"above tower {to_tower}"):
                raise Exception(f"Can't move to tower {to_tower}")
            
            # Lower to place disk at target height
            print(f"Lowering to place {disk_name} on tower {to_tower} at height {target_height}")
            if not safe_ik_move(0.25, to_y, target_height, f"place {disk_name} on tower {to_tower}"):
                raise Exception(f"Can't descend to tower {to_tower}")
            
            # Detach disk
            path_object.detach_mesh(disk_name, "link5")
            print(f"Detached {disk_name}")
            
            # Lift to safe height
            print(f"Lifting from tower {to_tower}")
            if not safe_ik_move(0.25, to_y, safe_height, f"lift from tower {to_tower}"):
                raise Exception(f"Can't lift from tower {to_tower}")
            
            return True
            
        except Exception as e:
            print(f"[ERROR] Failed to move {disk_name} from {from_tower} to {to_tower}: {e}")
            return False
    
    try:
        print(f"Starting simplified Hanoi stacking with vo_list: {vo_list}")
        
        # Step 1: Move disk B from tower B to tower A (z = 0.05)
        print("Step 1: Moving disk B on top of A")
        success = move_disk('B', 'A', 'B', 0.05)
        
        if not success:
            raise Exception("Failed to move disk B to tower A")
        
        # Step 2: Move disk C from tower C to tower A on top of B (z = 0.075)
        print("Step 2: Moving disk C on top of B")
        success = move_disk('C', 'A', 'C', 0.075)
        
        if not success:
            raise Exception("Failed to move disk C to tower A")
        
        # Return to original pose
        print("Returning to original pose")
        path_object.joint_angles = [0.06161233, 0.56530055, 1.67981925, -0.67432347, 1]
        path_object.go_to_joint_state()
        
        print("Hanoi stacking completed successfully!")
        return vo_list[0]
            
    except Exception as e:
        print(f"[ABORTED] Hanoi stacking failed: {e}")
        # Go to safe position in case of error
        path_object.joint_angles = [0, -pi/2, pi/2, 0]
        path_object.go_to_joint_state()
        return None
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
            vo = vision_order[::-1]
            print(f"📷 Vision 辨識順序為: {vo}")
        spawn_hanoi_towers(path_object,vo)
        print("solve...")
        solve(path_object,vo)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()