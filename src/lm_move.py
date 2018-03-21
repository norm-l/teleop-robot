#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
from leap_motion.msg import leapros
from sympy.matrices import *

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
                                    queue_size=1)

print('============ Waiting for RVIZ: WAITING!')
# rospy.sleep(10)
print('============ Waiting for RVIZ: DONE!')

def begin_plan(new_pos):
    # group.clear_pose_targets()
    pose_target = geometry_msgs.msg.Pose()

    previous_pos = group.get_current_pose().pose.position
    previous_x = round(previous_pos.x, 4) == round(new_pos.x, 4)
    previous_y = round(previous_pos.y, 4) == round(new_pos.y, 4)
    previous_z = round(previous_pos.z, 4) == round(new_pos.z, 4)

    # print "============ Previous: ", previous_pos.x, previous_pos.y, previous_pos.z

    if previous_x or previous_y or previous_z:
        print "============ ERROR: previous position is too close to new position!"
    else:
        print "============ INFO: Valid new position passed, attempting: ", new_pos.x, new_pos.y, new_pos.z
        # pose_target.orientation.w = 0.0 # For now, let's ignore orientation.
        pose_target.position.x = new_pos.x # 0.0453
        pose_target.position.y = new_pos.y
        pose_target.position.z = new_pos.z     

        group.set_pose_target(pose_target)
        plan = group.plan()
        rospy.sleep(5)
        group.execute(plan)

        # final_pos = group.get_current_pose().pose.position
        # print "============ New Position: ", final_pos.x, final_pos.y, final_pos.z, "\n\n"


def lm_move(leap_msg):
    # lm_palm_pos = leap_msg.palmpos
    lm_normal = leap_msg.normal
    pos_list = [lm_normal.x, lm_normal.y, lm_normal.z]
    if any(x > 0.0 for x in pos_list): # Avoid passing 0.0 coords
        begin_plan(lm_normal)
    

def lm_listener():
    root_joint_model = group.get_joints()[0] # shoulder_pan_joint
    root_link_model = robot.get_link_names() # ['world', 'base_link', 'base', 'shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link', 'ee_link', 'tool0']
    reference_transform = robot.get_link("base")
    print reference_transform.pose()
    
    jacobian = zeros(6) # zero matrix
    
    # joint_transform = 
    # joint_axis = 

    print jacobian

    # rospy.Subscriber("/leapmotion/data", leapros, lm_move, queue_size=1) # Subscribe to the topic and call lm_move each time we receive some input
    # rospy.spin() # Do this infinite amount of times


if __name__ == '__main__':
    try:
        lm_listener() # Call the listener method
    except rospy.ROSInterruptException:
        pass



# https://books.google.co.uk/books?id=C_2zCgAAQBAJ&pg=PA142&lpg=PA142&dq=sympy+jacobian&source=bl&ots=rGX2-h1peQ&sig=9Bj4qUtBXAZs-iwZcgjrT4Xilm4&hl=en&sa=X&ved=0ahUKEwi3muXwktjZAhUJDsAKHeuMCvEQ6AEIlgEwCQ#v=onepage&q=sympy%20jacobian&f=false
# http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/pr2_tutorials/kinematics/src/doc/kinematic_model_tutorial.html?highlight=jacobian
# https://github.com/ros-planning/moveit_tutorials/blob/indigo-devel/doc/pr2_tutorials/kinematics/src/kinematic_model_tutorial.cpp


# http://docs.ros.org/jade/api/moveit_core/html/robot__state_8cpp_source.html#l01098
# ^ Actual jacobian

# Pseudo Code
# J=getJacobian();
# JJ= J.inverse();
# Vector qq;
# Vector xx;
# xx= [hand_vel_x hand_vel_y hand_vel_z 0 0 0 ];
# qq=JJ*xx;

# moveit_commander.
# velocityMove(1,qq[1]);
# velocityMove(2,qq[2]);

# -----

# f = Matrix(???) What function?
# v1 = Matrix([0.01, 0.02, 0.01, 0.01, 0.03, 0.04])
# J = f.jacobian(v1)
# print J

# J_inv = J.inv()
# curr_pos = Matrix([0.05, 0.01, 0.10, 0, 0, 0])
# new_pos = J_inv * curr_pos
# print new_pos

# J = Matrix(3,3, [0.0, 0.0, 0.0 ,0.0, 0.0, 0.0 ,0.0, 0.0, 0.0]) # 3x3 Matrix
# q = Matrix(1,6, [0.01, 0.02, 0.01, 0.01, 0.03, 0.04]) #1x6 Vector
# J = robot.jacobian(q)

# joint_names = Matrix(group.get_active_joints()) # Vector of joint names
# print robot.get_link()
# robot_state = Matrix(robot.get_link)
# J = robot_state.jacobian(joint_names)


#C++ Version:
# Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
# Eigen::MatrixXd jacobian;
# kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, jacobian);
# ROS_INFO_STREAM("Jacobian: " << jacobian);

#Python: 
#robot->getJacobian(group, robot->getLinkModel(group->getLinkModelNames().back()), reference_point_position, jacobian)

#group->getLinkModelNames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'ee_fixed_joint']
#.back() = 'ee_fixed_joint'

