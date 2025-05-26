#!/usr/bin/env python3
import speech_recognition as sr
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import httpx
import threading
import json
import re
import math
# =======================
# OpenAI (Azure) Configuration
# =======================
# Replace these values with your own settings.
api_key = "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"  # Your API key
api_base = "https://models.inference.ai.azure.com"  
deployment_id = "gpt-4o"  
api_version = "2023-03-15-preview" 

http_client = httpx.Client(
    base_url=api_base,
    follow_redirects=True,
)

class TurtlesimVoiceController:
    def __init__(self):
        # Speech Recognition Setup
        self.recognizer = sr.Recognizer()
        
        # ROS Setup
        rospy.init_node('voice_turtlesim_controller', anonymous=True)
        
        # Publisher for turtle movement
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Publisher for GPT replies
        self.gpt_reply_pub = rospy.Publisher("gpt_reply_to_user", String, queue_size=10)

    def extract_movement_parameters(self, user_text):
        """
        Extract movement parameters from user text with improved Chinese parsing
        """
        # Default parameters
        params = {
            "speed": 0.5,  # Default speed
            "distance": 2.0,  # Default distance
            "repeats": 1,  # Default repeats
            "path_type": "rectangle"
        }

        # Chinese number mapping
        chinese_to_arabic = {
            "零": 0, "一": 1, "兩": 2, "二": 2, "三": 3, "四": 4, "五": 5,
            "六": 6, "七": 7, "八": 8, "九": 9, "十": 10
        }

        # Extract speed
        speed_match = re.search(r'速度每秒(\d+)公尺', user_text)
        if speed_match:
            params["speed"] = float(speed_match.group(1))

        # Extract distance for square
        distance_match = re.search(r'(\d+)公尺正方形', user_text)
        if distance_match:
            params["distance"] = float(distance_match.group(1))

        # Extract number of repetitions
        repeats_match = re.search(r'([一二三四五六七八九十]+|[0-9]+)次', user_text)
        if repeats_match:
            # Convert Chinese numerals to Arabic numerals if needed
            repeats_str = repeats_match.group(1)
            if repeats_str in chinese_to_arabic:
                params["repeats"] = chinese_to_arabic[repeats_str]
            else:
                params["repeats"] = int(repeats_str)

        # Generate rectangle movements
        params["rectangle_movements"] = []
        for _ in range(4):
            # Move forward
            params["rectangle_movements"].append({
                "action": "move",
                "speed": {"linear": {"x": params["speed"], "y": 0}},
                "distance": params["distance"]
            })
            # Rotate 90 degrees
            params["rectangle_movements"].append({
                 "action": "rotate",
                "angular_speed": {"angular": {"z": math.pi / 2}}
            })
        return params

    def move_square_side(self, distance, speed):
        """
        Move forward for a specific distance at given speed
        """
        twist = Twist()
        rate = rospy.Rate(10)
        
        # Calculate move duration based on speed
        move_duration = rospy.Duration(distance / speed)
        start_time = rospy.Time.now()
        
        while rospy.Time.now() < start_time + move_duration:
            twist.linear.x = speed  # Use specified speed
            self.pub.publish(twist)
            rate.sleep()
        
        # Stop
        twist.linear.x = 0
        self.pub.publish(twist)
        rospy.sleep(0.5)

    def rotate_90_degrees(self):
        """
        Rotate 90 degrees
        """
        twist = Twist()
        rate = rospy.Rate(10)
        
        # Rotate for specific duration to achieve 90-degree turn
        rotate_duration = rospy.Duration(1)  # Adjust if needed
        start_time = rospy.Time.now()
        
        while rospy.Time.now() < start_time + rotate_duration:
            twist.angular.z = math.pi / 2  # 90 degrees in radians
            self.pub.publish(twist)
            rate.sleep()
        
        # Stop rotation
        twist.angular.z = 0
        self.pub.publish(twist)
        rospy.sleep(0.5)

    def handle_action(self, command):
        """
        Handle different types of movement actions
        """
        # Execute rectangle multiple times
        for _ in range(command.get("repeats", 1)):
            if "rectangle_movements" in command:
                rospy.loginfo(f"Executing rectangle with {len(command['rectangle_movements'])} movements")
                for i, movement in enumerate(command["rectangle_movements"], 1):
                    rospy.loginfo(f"Executing movement step {i}: {movement}")
                    
                    if movement["action"] == "move":
                        speed = movement["speed"]["linear"]["x"]
                        distance = movement.get("distance", 2.0)
                        self.move_square_side(distance, speed)
                    elif movement["action"] == "rotate":
                        self.rotate_90_degrees()

    def user_message_callback(self, user_text):
        """
        Process user voice input and publish movement commands
        """
        rospy.loginfo("Received user input: %s", user_text)

        try:
            # Extract movement parameters
            movement_params = self.extract_movement_parameters(user_text)

            # Convert to JSON for publishing
            gpt_reply = json.dumps(movement_params)

            rospy.loginfo("Published movement command: %s", gpt_reply)
            self.gpt_reply_pub.publish(gpt_reply)

            # Directly handle the action
            self.handle_action(movement_params)

        except Exception as e:
            rospy.logerr("Error processing user message: %s", e)

    def speech_recognition_loop(self):
        """
        Continuously performs speech recognition in a separate thread
        """
        while not rospy.is_shutdown():
            try:
                with sr.Microphone() as source:
                    rospy.loginfo("Please speak...")
                    audio = self.recognizer.listen(source)
                try:
                    # Use Google Speech Recognition for Chinese
                    recognized_text = self.recognizer.recognize_google(audio, language="zh-CN")
                    rospy.loginfo("Recognized text: %s", recognized_text)
                    self.user_message_callback(recognized_text)
                except sr.UnknownValueError:
                    rospy.logwarn("Could not understand the audio")
                except sr.RequestError as e:
                    rospy.logerr("Speech recognition error: %s", e)
            except Exception as e:
                rospy.logerr("Error during recording: %s", e)

    def run(self):
        # Start the speech recognition thread
        sr_thread = threading.Thread(target=self.speech_recognition_loop)
        sr_thread.daemon = True
        sr_thread.start()

        # Keep the ROS node running
        rospy.spin()

def main():
    controller = TurtlesimVoiceController()
    controller.run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass