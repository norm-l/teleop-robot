# Lukas's Branch

Some meaningful text explaining my branch...

# Starting

Please check `/modified_leapd/`  
  
**Terminal1** -> `roscore`  
**Terminal2** -> `sudo leapd`  
*(optional)* **Terminal3** -> `LeapControlPanel`  
**Terminal4** -> `source ~/catkin_ws/devel/setup.bash && roslaunch ur5_lm_move.launch`  
**Terminal5** -> `rosrun leap_motion sender.py`  
*(optional)* **Terminal6** -> `rostopic echo /leapmotion/data | grep "palmpos" -A 3`    

# Requirements
- [Ubuntu 16.04](http://releases.ubuntu.com/16.04/)
- [Leap Motion SDK (v2 Tracking)](https://developer.leapmotion.com/sdk/v2)
- [Robot Operating System](http://www.ros.org/)
- [leap_motion](http://wiki.ros.org/leap_motion)

# leap_motion Setup

1. [Replace leapd.service](https://github.com/samisnotinsane/arq-teleop-robot/tree/lukas_development/modified_leapd)

2. Install the [SDK](https://developer.leapmotion.com/sdk/v2) and extract the **LeapSDK** folder in to your home folder  
If you're getting `Errors were encountered while processing: leap:i386` when you try to install (e.g. `sudo dpkg -i Leap-2.3.1+31549-x86.deb`) it means you have not done step 1.

3. After placing LeapSDK from the [SDK](https://developer.leapmotion.com/sdk/v2) archive in home directory:  
`export PYTHONPATH=$PYTHONPATH:$HOME/LeapSDK/lib:$HOME/LeapSDK/lib/x64`

4. Install the ros package [leap_motion](http://wiki.ros.org/leap_motion)  
`sudo apt-get install ros-kinetic-leap-motion`  

These instructions are also available on the ros wiki (check leap_motion under requirements)

Note: When starting `leapd` there may be errors however for the most part they can be ignored. Here's an example of a successful run:

```
lukas@ubuntu:~/Downloads$ sudo leapd
Configuration file not found
Resetting /var//.Leap Motion/config.json.
[Critical] Secure WebSocket server failed to start
[Critical] Have you tried running as root/Administrator?
[Critical] WebSocket server failed to start
[Info] Leap Motion Controller detected: LP83535853740
[Info] Firmware is up to date.
```
