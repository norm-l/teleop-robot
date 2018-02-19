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

def begin_plan(new_pos):
    # group.clear_pose_targets()
    pose_target = geometry_msgs.msg.Pose()

    previous_pos = group.get_current_pose().pose.position
    previous_x = round(previous_pos.x, 4) == round(new_pos.x, 4)
    previous_y = round(previous_pos.y, 4) == round(new_pos.y, 4)
    previous_z = round(previous_pos.z, 4) == round(new_pos.z, 4)

    print "============ Previous: ", previous_pos.x, previous_pos.y, previous_pos.z

    if previous_x or previous_y or previous_z:
        print "============ ERROR: previous position is too close to new position!"
    else:
        print "============ INFO: Valid new position passed, attempting: ", new_pos.x, new_pos.y, new_pos.z
        # pose_target.orientation.w = 0.0 # For now, let's ignore orientation.
        pose_target.position.x = new_pos.x # 0.045308
        pose_target.position.y = new_pos.y
        pose_target.position.z = new_pos.z

        group.set_pose_target(pose_target)
        plan = group.plan()
        rospy.sleep(5)
        group.execute(plan)

        final_pos = group.get_current_pose().pose.position
        print "============ New Position: ", final_pos.x, final_pos.y, final_pos.z, "\n\n"


def lm_move(leap_msg):
    # lm_palm_pos = leap_msg.palmpos
    lm_normal = leap_msg.normal
    pos_list = [lm_normal.x, lm_normal.y, lm_normal.z]
    if any(x > 0.0 for x in pos_list): # Avoid passing 0.0 coords
        begin_plan(lm_normal)
    

def lm_listener():
    rospy.Subscriber("/leapmotion/data", leapros, lm_move, queue_size=1) # Subscribe to the topic and call lm_move each time we receive some input
    rospy.spin() # Do this infinite amount of times


if __name__ == '__main__':
    try:
        lm_listener() # Call the listener method
    except rospy.ROSInterruptException:
        pass
