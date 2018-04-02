# Lukas's Branch

Some meaningful text explaining my branch...

# 1. Requirements  

**Make sure you have all of these before moving on**  

- [Ubuntu 16.04](http://releases.ubuntu.com/16.04/)
- [Leap Motion SDK (v2 Tracking)](https://developer.leapmotion.com/sdk/v2)
- [Robot Operating System](http://www.ros.org/)
- [leap_motion](http://wiki.ros.org/leap_motion)
- [Universal Robot](http://wiki.ros.org/universal_robot) **Note: Use the catkin method! We are on Ubuntu 16.04 which is ABOVE 14.04**
- [MoveIt!](http://moveit.ros.org/install/)

# 2. leap_motion Setup

1. [Replace leapd.service](https://github.com/samisnotinsane/arq-teleop-robot/tree/lukas_development/modified_leapd)

2. Install the [SDK](https://developer.leapmotion.com/sdk/v2) and extract the **LeapSDK** folder in to your home folder  
If you're getting `Errors were encountered while processing: leap:i386` when you try to install (e.g. `sudo dpkg -i Leap-2.3.1+31549-x86.deb`) it means you have not done step 1.

3. After placing LeapSDK from the [SDK](https://developer.leapmotion.com/sdk/v2) archive in home directory:  
`export PYTHONPATH=$PYTHONPATH:$HOME/LeapSDK/lib:$HOME/LeapSDK/lib/x64`

4. Install the ros package [leap_motion](http://wiki.ros.org/leap_motion)  
`sudo apt-get install ros-kinetic-leap-motion`  

These instructions are also available on the ros wiki (check leap_motion under requirements)

# 3. Cloning this branch

1. Navigate to `~/catkin_ws/src` or wherever your `catkin_ws` folder is located.
2. `git clone https://github.com/samisnotinsane/arq-teleop-robot.git`
3. Navigate to `~/catkin_ws/src/arq-teleop-robot/launch` and continue with the below.

# 4. Starting

Please check `/modified_leapd/`  
  
Enable Leap Motion SDK:  
**Terminal** -> `sudo leapd`  
Launch the program:  
**Terminal** -> `source ~/catkin_ws/devel/setup.bash && roslaunch ur5_lm_move.launch`  (when in the launch directory)
Start roscore:  
**Terminal** -> `roscore`  
Enable the leap_motion sender:  
**Terminal** -> `rosrun leap_motion sender.py`
    
Control Panel from Leap Motion SDK:  
*(optional)* **Terminal** -> `LeapControlPanel`  
See the data being sent:  
*(optional)* **Terminal** -> `rostopic echo /leapmotion/data | grep "palmpos" -A 3`    
