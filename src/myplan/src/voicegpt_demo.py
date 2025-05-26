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


# Dictionary to map word numbers to digits
number_mapping = {
    "one": "1",
    "two": "2",
    "three": "3",
    "four": "4",
    "five": "5",
    "six": "6",
    "1": "1",
    "2": "2",
    "3": "3",
    "4": "4", 
    "5": "5",
    "6": "6"
}

def listen_and_publish():
    rospy.init_node('voice_command_node', anonymous=True)
    pub = rospy.Publisher('/voice_case_cmd', String, queue_size=10)

    recognizer = sr.Recognizer()
    mic = sr.Microphone()

    print("Voice recognizer is active. Say something like 'case one' through 'case six'...")

    while not rospy.is_shutdown():
        with mic as source:
            recognizer.adjust_for_ambient_noise(source)
            print("Listening...")
            try:
                audio = recognizer.listen(source, timeout=5, phrase_time_limit=5)
                text = recognizer.recognize_google(audio)
                print(f"You said: {text}")

                # Process for valid case commands - handle both "case 1" and "case one" formats
                text = text.lower().strip()
                
                # Match pattern "case X" where X can be a digit or word
                match = re.search(r"case\s+(one|two|three|four|five|six|[1-6])", text)
                if match:
                    number_word = match.group(1)
                    # Convert word to digit if needed
                    case_number = number_mapping.get(number_word, "")
                    
                    if case_number:
                        case_command = f"case{case_number}"
                        print(f"Publishing command: {case_command}")
                        pub.publish(case_command)
                    else:
                        print("Could not process number format.")
                else:
                    print("Not a valid case command. No command published.")

            except sr.UnknownValueError:
                print("Could not understand audio.")
            except sr.WaitTimeoutError:
                print("Timeout: no speech detected.")
            except sr.RequestError as e:
                print(f"Recognition service error: {e}")


if __name__ == '__main__':
    try:
        listen_and_publish()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("Exiting.")
