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
                                    queue_size=1)

rospy.sleep(10) # Wait for rviz to initialise
print('============ Waiting for RVIZ: DONE!')

executing = False # flag to determine if we are currently executing a plan
prev_pos = geometry_msgs.msg.Pose().position # keep track of what the previous passed position was

def begin_plan(new_pos):
    global prev_pos
    global executing

    dp = 3 # decimal points to round for equality check
    curr_pos = transform_pos(new_pos) # need to transform leap motion input

    # there is probably a much nicer way to do this
    prev_pos_x = round(prev_pos.x, dp)
    curr_pos_x = round(curr_pos.x, dp)
    prev_pos_y = round(prev_pos.y, dp)
    curr_pos_y = round(curr_pos.y, dp)
    prev_pos_z = round(prev_pos.z, dp)
    curr_pos_z = round(curr_pos.z, dp)
    check_x = prev_pos_x == curr_pos_x
    check_y = prev_pos_y == curr_pos_y
    check_z = prev_pos_z == curr_pos_z

    if check_x or check_y or check_z:
        print "\n=[ ERROR: Coordinates too similar! ]=\n" \
            "Previous: ", "x:", prev_pos_x, "y:", prev_pos_y, "z:", prev_pos_z, \
            "\nNew:      ", "x:", curr_pos_x, "y:", curr_pos_y, "z:", curr_pos_z
        return

    prev_pos = curr_pos # keep track of this position for next iteration
    print "============ INFO: Valid new position passed, attempting: ", curr_pos.x, curr_pos.y, curr_pos.z

    executing = True # we are now executing
    waypoints = []
    waypoints.append(group.get_current_pose().pose)

    wpose = geometry_msgs.msg.Pose()
    # wpose.orientation.w = 1.0
    wpose.position.x = curr_pos.x
    wpose.position.y = curr_pos.z # we switch z and y because the leap motion is faced upwards
    wpose.position.z = curr_pos.y
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = group.compute_cartesian_path(
                            waypoints,   # waypoints to follow
                            0.01,        # eef_step
                            0.0)         # jump_threshold
    group.execute(plan)
    executing = False # we are no longer executing

def transform_pos(pos):
    conversion_value = 0.001 # lower leap motion values by this much
    pos.x = zero_pos.x + (pos.x * conversion_value)
    pos.y = zero_pos.y + (pos.y * conversion_value)
    pos.z = zero_pos.z + (pos.z * conversion_value)
    return pos


def lm_move(leap_msg):
    if not executing:
        lm_palm_pos = leap_msg.palmpos
        pos_list = [lm_palm_pos.x, lm_palm_pos.y, lm_palm_pos.z]
        if any(x > 0.0 for x in pos_list): # avoid passing (0.0,0.0,0.0)
            begin_plan(lm_palm_pos)


def lm_listener():
    rospy.Subscriber("/leapmotion/data", leapros, lm_move, queue_size=1) # Subscribe to the topic and call lm_move each time we receive some input
    rospy.spin() # Do this infinite amount of times


if __name__ == '__main__':
    try:
        # move the robot to an initial comfortable position
        values = [3.25,-3.30,1.44,-3.0,-0.11,0.70]
        group.set_joint_value_target(values)
        plan = group.plan()
        group.execute(plan)
        rospy.sleep(3)
        # save this position to be used later
        zero_pos = group.get_current_pose().pose.position
        # call the listener method
        lm_listener()
    except rospy.ROSInterruptException:
        pass