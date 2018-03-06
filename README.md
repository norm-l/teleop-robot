# Lukas's Branch

Some meaningful text explaining my branch...

# Starting

Please check `/modified_leapd/`  
  
**Terminal1** -> `roscore`  
**Terminal2** -> `sudo leapd`  
*(optional)* **Terminal3** -> `LeapControlPanel`  
**Terminal4** -> `source ~/catkin_ws/devel/setup.bash && roslaunch ur5_lm_move.launch`  
**Terminal5** -> `rosrun leap_motion sender.py`  
*(optional)* **Terminal6** -> `rostopic echo /leapmotion/data | grep "normal" -A 3`    

# Requirements

- [Leap Motion SDK (v2 Tracking)](https://developer.leapmotion.com/sdk/v2)
- [Robot Operating System](http://www.ros.org/)
- [leap_motion](http://wiki.ros.org/leap_motion)
