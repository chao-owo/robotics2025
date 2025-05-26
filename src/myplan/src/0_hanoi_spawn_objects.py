#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import atan2, acos, asin, sqrt, sin, cos, pi
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import Bool
import rospkg 



'''Variable for end-effector'''
EefState = 0

'''Hanoi tower geometry'''
#You can measure these in Lab402
Tower_base = 0.0014     #Height of tower base
Tower_heitght = 0.025   #Height of each tower
Tower_overlap = 0.015   #Height of tower overlap

'''Hanoi tower position'''
#you may want to slightly change this
p_Tower_x = 0.25
p_Tower_y = 0.15 #(0.15, 0, -0,15) as lab4 shown

'''Hanoi tower mesh file path'''
rospack = rospkg.RosPack()
FILE_PATH = rospack.get_path('myplan')+ "/mesh"
MESH_FILE_PATH = [FILE_PATH +"/tower1.stl",FILE_PATH +"/tower2.stl",FILE_PATH +"/tower3.stl"]


'''Robot arm geometry'''
l0 = 0.06;l1 = 0.082;l2 = 0.132;l3 = 0.1664;l4 = 0.048;d4 = 0.004



'''
Hint:
    The output of your "Hanoi-Tower-Function" can be a series of [x, y, z, eef-state], where
    1.xyz in world frame
    2.eef-state: 1 for magnet on, 0 for off
'''



def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
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
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "ldsc_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    eef_link = 'link5'
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    # EefState_publisher =  rospy.Publisher('/SetEndEffector', Bool, queue_size=10)                                        

    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)


    move_group.set_workspace([-0.2984,-0.2984,0.0,0.2984,0.2984,0.4404])


    group_names = robot.get_group_names()
    # Misc variables
    
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    # self.EefState_publisher = EefState_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
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
    print("current pose:" , current_pose.position )
    return all_close(joint_goal, current_joints, 0.01)

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


  def attach_box(self, box_name, link_name):
    '''
    Description:
        1. Make sure the box has been added to rviz
        2. Attach a box to link_frame(usually 'link5'), and the box will move with the link_frame.
        3. Google scene.attach_box for more details
    '''
    scene = self.scene
    scene.attach_box(link_name, box_name, touch_links=[link_name])
    timeout=4
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


  def detach_box(self, box_name, link_name):
    '''
    Description: 
        1. Detach a box from link_frame(usually 'link5'), and the box will not move with the link_frame.
        2. An example is shown in the main function below.
        3. Google scene.detach_box for more details
    '''
    scene = self.scene
    scene.remove_attached_object(link_name, name=box_name)
    timeout=4
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_box(self, box_name):
    '''
    Description: 
        Remove a box from rviz.
    '''
    scene = self.scene
    scene.remove_world_object(box_name)
    ## **Note:** The object must be detached before we can remove it from the world
    timeout=4
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

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
    scene.add_mesh(mesh_name, mesh_pose, file_path, size=size_tuple)
    timeout=4
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def attach_mesh(self, mesh_name, link_name):
    '''
    Description: 
        1. Make sure the mesh has been added to rviz
        2. Attach a box to link_frame(usually 'link5'), and the box will move with the link_frame.
        3. An example is shown in the main function below.
    '''
    scene = self.scene
    scene.attach_mesh(link_name, mesh_name, touch_links=[link_name])
    timeout=4
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_mesh(self, mesh_name, link_name):
    '''
    Description: 
        1. Detach a box from link_frame(usually 'link5'), and the box will not move with the link_frame.
        2. An example is shown in the main function below.
    '''
    scene = self.scene
    scene.remove_attached_object(link_name, name=mesh_name)
    timeout=4
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_mesh(self, mesh_name):
    '''
    Description: 
        Remove a mesh from rviz.
    '''
    scene = self.scene
    scene.remove_world_object(mesh_name)
    ## **Note:** The object must be detached before we can remove it from the world
    # We wait for the planning scene to update.
    timeout=4
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

  def remove_world_object(self):
    '''
    Description: 
        Remove all objects from rviz.
    '''
    #remove all if no name specified
    self.scene.remove_world_object()
    timeout=4
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

def pub_EefState_to_arm():

    '''
    Description:
        Because moveit only plans the path, 
        you have to publish end-effector state for playing hanoi.
    '''
    global pub,rate
    pub = rospy.Publisher('/SetEndEffector', Bool, queue_size=10)
    rate = rospy.Rate(100) # 100hz




def main():
  try:
    pathPlanObject = MoveGroupPythonIntefaceTutorial()  #Declare the path-planning object

    print("Add a box to rviz. Press enter to continue.."); input()
    box_pose = geometry_msgs.msg.PoseStamped() # Set the parameter
    box_pose.header.frame_id = 'world'         # Put the box in 'world' frame
    box_pose.pose.orientation.w = 1.0          # Orieantion in quaterian
    box_pose.pose.position.x = 0.2             # Specify x of the box
    box_pose.pose.position.y = 0.0             # Specify y of the box
    box_pose.pose.position.z = 0.06/2          # Specify z of the box
    pathPlanObject.add_box('box_1', box_pose, (0.05, 0.05, 0.06)) #Specify box name, box pose, size in xyz
    
    print('Attach a box to a link_frame'); input()
    print("ttach box1 to link5.When arms move, box1 will stick to link5 frame")
    pathPlanObject.attach_box('box_1','link5') #Attach box1 to link5.When arms move, box1 will stick to link5 frame
    
    print('Detach a box from a link_frame'); input()
    pathPlanObject.detach_box('box_1','link5')

    print('Remove box1 from rviz'); input()
    pathPlanObject.remove_box('box_1')
    
    print('Add a mesh to rviz'); input()
    mesh_pose = geometry_msgs.msg.PoseStamped() # Set the parameter
    mesh_pose.header.frame_id = 'world'         # Put the mesh in 'world' frame
    mesh_pose.pose.position.x = 0.4            # Specify x of the mesh
    mesh_pose.pose.position.y = 0.0             # Specify y of the mesh
    mesh_pose.pose.position.z = 0.0             # Specify z of the mesh
    pathPlanObject.add_mesh('tower1', mesh_pose, MESH_FILE_PATH[0], (.00095,.00095,.00095))
    #Specify mesh name, mesh pose, factor_of_mesh_file(recommend not change)
    
    print('Attach a mesh to a link_frame'); input()
    print("Attach tower1 to link5.When arms move, tower1 will stick to link5 frame.")
    pathPlanObject.attach_mesh('tower1', 'link5')
    #Attach tower1 to link5.When arms move, tower1 will stick to link5 frame.

    print('Detach a mesh from a link_frame'); input()
    pathPlanObject.detach_mesh('tower1', 'link5')


    print('Remove mesh from rviz'); input()
    pathPlanObject.remove_mesh('tower1')

    # print('Remove all object from rviz'); input()
    # pathPlanObject.remove_world_object()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return
    
  

if __name__ == '__main__':
  main()

