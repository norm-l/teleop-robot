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

print('============ Waiting for RVIZ: WAITING!')
rospy.sleep(10)
print('============ Waiting for RVIZ: DONE!')

def begin_plan(palm_x, palm_y, palm_z):
    # group.clear_pose_targets()
    group_variable_values = group.get_current_joint_values()

    print "============ Generating: ", palm_x, palm_y, palm_z
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 0.0
    pose_target.position.x = palm_x
    pose_target.position.y = palm_y
    pose_target.position.z = palm_z
    group.set_pose_target(pose_target)
    plan = group.plan()

    rospy.sleep(5)
    group.execute(plan)
    print "============ Joint values: ", group_variable_values


def lm_move(leap_msg):
    # lm_palm_pos = leap_msg.palmpos
    lm_normal = leap_msg.normal
    print "GOT: ", lm_normal.x, lm_normal.y, lm_normal.z
    rospy.sleep(1)
    # print "GOT: ", lm_palm_pos.x, lm_palm_pos.y, lm_palm_pos.z
    # begin_plan(lm_palm_pos.x, lm_palm_pos.y, lm_palm_pos.z)


def lm_listener():
    rospy.Subscriber("/leapmotion/data", leapros, lm_move)
    rospy.spin()


if __name__ == '__main__':
    try:
        lm_listener()
    except rospy.ROSInterruptException:
        pass
