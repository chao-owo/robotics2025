#!/usr/bin/env python3
import json
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math

pub = None

def callback(data):
    """
    Callback to handle incoming movement commands
    """
    rospy.loginfo("Received command: %s", data.data)

    try:
        command = json.loads(data.data)
        handle_action(command)
    except json.JSONDecodeError as e:
        rospy.logerr("JSON Decode Error: %s", e)
    except KeyError as e:
        rospy.logerr("Missing key in JSON data: %s", e)

def handle_action(command):
    """
    Handle different types of movement actions
    """
    global pub

    # Prioritize rectangle movements if present
    if "rectangle_movements" in command:
        rospy.loginfo(f"Executing rectangle with {len(command['rectangle_movements'])} movements")
        for i, movement in enumerate(command["rectangle_movements"], 1):
            rospy.loginfo(f"Executing movement step {i}: {movement}")
            execute_movement(movement)
    # Fallback to default movement if no rectangle movements
    else:
        execute_movement(command)

def execute_movement(movement):
    """
    Execute precise movement or rotation
    """
    global pub
    twist = Twist()
    rate = rospy.Rate(20)  # Increased rate for smoother control

    if movement["action"] == "move":
        # Extract movement parameters
        distance = movement.get("distance", 2.0)
        linear_speed = movement["speed"]["linear"]["x"]

        rospy.loginfo(f"Moving {distance} meters at speed {linear_speed} m/s")

        # Calculate precise movement duration
        move_duration = distance / linear_speed
        start_time = rospy.Time.now()
        end_time = start_time + rospy.Duration(move_duration)

        # Move forward
        while rospy.Time.now() < end_time:
            twist.linear.x = linear_speed
            pub.publish(twist)
            rate.sleep()

        # Stop the robot
        twist.linear.x = 0
        pub.publish(twist)
        rospy.sleep(0.5)  # Pause between movements

    elif movement["action"] == "rotate":
        # Precise 90-degree rotation
        angular_speed = math.pi / 2  # 90 degrees in radians

        rospy.loginfo(f"Rotating 90 degrees")

        # Rotate 90 degrees
        twist.angular.z = angular_speed
        start_time = rospy.Time.now()
        
        # Rotate for a fixed duration to ensure 90-degree turn
        rotation_duration = rospy.Duration(1)  # 1 second should be enough for 90-degree rotation
        while rospy.Time.now() < start_time + rotation_duration:
            pub.publish(twist)
            rate.sleep()

        # Stop rotation
        twist.angular.z = 0
        pub.publish(twist)
        rospy.sleep(0.5)  # Pause after rotation

def listener():
    """
    Set up ROS node and subscribers
    """
    global pub
    rospy.init_node('turtlesim_controller', anonymous=True)

    # Publisher for turtle movement
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    # Subscribe to voice commands
    rospy.Subscriber("gpt_reply_to_user", String, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()