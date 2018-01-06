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
# Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
scene = moveit_commander.PlanningSceneInterface()
# This object is an interface to one group of joints.
group = moveit_commander.MoveGroupCommander("manipulator")

display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=20)

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

def begin_plan():
	# group.clear_pose_targets()
	group_variable_values = group.get_current_joint_values()
	print "============ Joint values: ", group_variable_values
	# shoulder_pan_joint
	group_variable_values[0] = 0.0
	#shoulder_lift_joint
	group_variable_values[1] = 0.0
	#elbow_joint
	group_variable_values[2] = 0.0
	#wrist_1_joint
	group_variable_values[3] = 0.0
	#wrist_2_joint
	group_variable_values[4] = 2.57
	#wrist_3_joint
	group_variable_values[5] = -3.14
	group.set_joint_value_target(group_variable_values)
	plan = group.plan()
	print "============ Waiting while RVIZ displays the plan"
	rospy.sleep(5)
	print "============ Plan finished! Executing..."
	group.execute(plan)
	print "============ Execution finished!"
	print "============ Joint values: ", group_variable_values


# def lm_move(leap_msg):
# 	lm_palm_pos = leap_msg.palmpos
#     rospy.loginfo("lm_palm_pos: x: %f y: %f z: %f", lm_palm_pos.x, lm_palm_pos.y, lm_palm_pos.z)


# def lm_listener():
#     rospy.Subscriber("/leapmotion/data", leapros, lm_move)
#     rospy.spin()


if __name__ == '__main__':
    try:
        # lm_listener()
        begin_plan()
    except rospy.ROSInterruptException:
        pass
