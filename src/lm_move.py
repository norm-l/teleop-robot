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
last_passed = 0.0 # to know what was the last passed value

def begin_plan(new_pos):
    global last_passed
    global executing

    previous_pos = group.get_current_pose().pose.position # get the current position of the robot
    rounding_value = 2 # how many decimal points to round
    conversion_value = 0.001 # lower leap motion values by this much

    if last_passed == round(zero_pos.z + (new_pos.y * conversion_value), rounding_value):
        print "last_passed: [", last_passed, "] \n", "new: [", round(zero_pos.z + (new_pos.y * conversion_value), rounding_value), "]"
        return

    last_passed = round(zero_pos.z + (new_pos.y * conversion_value), rounding_value)

    """ 
    1. z and y are switched from the input (leap motion) side because the leap motion is facing up
    2. We multiply the leap motion input by conversion_value to ensure the positions are not too high
    3. We check if the current robot position (x,y,z) == the new passed position (x,y,z), rounded to rounding_value 
    """
    previous_x = round(previous_pos.x, rounding_value) == round(zero_pos.x + (new_pos.x * conversion_value), rounding_value)
    previous_y = round(previous_pos.y, rounding_value) == round(zero_pos.y + (new_pos.z * conversion_value), rounding_value)
    previous_z = round(previous_pos.z, rounding_value) == round(zero_pos.z + (new_pos.y * conversion_value), rounding_value)

    if previous_x or previous_y or previous_z: # if any of these equality checks are true
        print "============ ERROR: Previous position is too close to new position!"
    else:
        print "============ INFO: Valid new position passed, attempting: ", (new_pos.x * conversion_value), (new_pos.y * conversion_value), (new_pos.z * conversion_value)

        executing = True # we are now executing
        waypoints = []
        waypoints.append(group.get_current_pose().pose)

        wpose = geometry_msgs.msg.Pose()
        # wpose.orientation.w = 1.0
        wpose.position.x = zero_pos.x + (new_pos.x * 0.001)
        wpose.position.y = zero_pos.y + (new_pos.z * 0.001)
        wpose.position.z = zero_pos.z + (new_pos.y * 0.001)
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold
        group.execute(plan)
        executing = False # we are no longer executing


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