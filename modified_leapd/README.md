You will need sudo to replace both.

# leap_interface.py
This file has been replaced in: `/opt/ros/kinetic/lib/leap_motion`  
**Reason**: Removed the ability to send data when there are no hands detected by the Leap Motion.

# leapd.service
This file has been replaced in: `/lib/systemd/system`  
**Reason**: leapd service won't work until it is replaced. Check out the file for further instructions.

Also check out: https://d2beseu6pw5d2t.cloudfront.net/t/tip-ubuntu-systemd-and-leapd/2118/3
