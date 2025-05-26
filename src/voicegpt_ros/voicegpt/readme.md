# LAB2

## Follow these steps to start the demo code

##### 1.Check if your Python version is Python 3. You can use "python -V" to check.

##### 2.Install pip3.

```
sudo apt update

sudo apt install python3-pip -y
```

##### 3.Install PyAudio.

```
sudo apt-get install portaudio-dev
```
##### 4.Install dependencies from requirements.txt.
Ensure that your current path is under ~/catkin_ws/src/voicegpt, then run:

```
pip3 install -r requirements.txt
```

##### 5.Build the workspace.
After installing the dependencies, unzip voicegpt.zip inside catkin_ws/src, then navigate to catkin_ws and run:
```
catkin_make 

source devel/setup.bash
```

##### 6.Set up your API key.
Remember to key your own api key inside the voicegpt_demo.py, and run:

```
rosrun voicegpt_ros voicegpt_demo.py

rosrun voicegpt_ros turtle_control.py
```

##### Now you can try say something to drive turtle. 
