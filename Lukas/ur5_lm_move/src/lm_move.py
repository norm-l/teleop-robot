#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
from leap_motion.msg import leapros

from std_msgs.msg import String

print('============ Starting setup')
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('lm_move', anonymous=True)

# This object is an interface to the world surrounding the robot.
# scene = moveit_commander.PlanningSceneInterface()

# This object is an interface to the robot as a whole.
robot = moveit_commander.RobotCommander()

# This object is an interface to one group of joints.
group = moveit_commander.MoveGroupCommander("manipulator")

print('============ Waiting for RVIZ...')
rospy.sleep(10)
print('============ Starting...')
print('============ Reference frame: %s' % group.get_planning_frame())
print('============ Reference frame: %s' % group.get_end_effector_link())
print('============ Robot Groups:')
print(robot.get_group_names())
print('============ Printing robot state')
print(robot.get_current_state())
print('============')


def lm_move(leap_msg):
    lm_palm_pos = leap_msg.palmpos
    rospy.loginfo("lm_palm_pos: x: %f y: %f z: %f", lm_palm_pos.x, lm_palm_pos.y, lm_palm_pos.z)


def lm_listener():
    rospy.Subscriber("/leapmotion/data", leapros, lm_move)
    rospy.spin()


if __name__ == '__main__':
    try:
        lm_listener()
    except rospy.ROSInterruptException:
        pass