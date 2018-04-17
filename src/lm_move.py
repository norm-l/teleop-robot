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
import Tkinter as tk
import tkMessageBox

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('lm_move', anonymous=True)

# This object is an interface to the robot as a whole.
robot = moveit_commander.RobotCommander()
# Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
scene = moveit_commander.PlanningSceneInterface()
# This object is an interface to one group of joints.
group = moveit_commander.MoveGroupCommander("manipulator")

display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=1) # we don't want to buffer any messages

rospy.sleep(10) # Wait for rviz to initialise
print "\n=[ INFO: Waiting for RVIZ: DONE! ]=\n"

sub = None # we need to have the subscriber as an object so we can unregister/register when paused/resumed
resumed_run = False # flag to determine if it is the first run after pressing 'resume'
paused = False # flag to determine if program is paused
executing = False # flag to determine if we are currently executing a plan
prev_pos = geometry_msgs.msg.Pose().position # keep track of what the previous passed position was
zero_pos = geometry_msgs.msg.Pose().position # keep track of the zero position
win_hidden = True # keep track if zero pos window is hidden or not


def begin_plan(new_pos):
    global prev_pos
    global executing
    global resumed_run
    global zero_pos

    # check if this is the first run after a resume
    if resumed_run:
        old_zeroPosLbl_text.set("Old Zero Position\nx: %.3f\ny: %.3f\nz: %.3f" % (zero_pos.x, zero_pos.y, zero_pos.z))
        zero_pos.x = group.get_current_pose().pose.position.x
        zero_pos.y = group.get_current_pose().pose.position.z
        zero_pos.z = group.get_current_pose().pose.position.y
        zeroPosLbl_text.set("Zero Position\nx: %.3f\ny: %.3f\nz: %.3f" % (zero_pos.x, zero_pos.y, zero_pos.z))
        resumed_run = False

    # boundaries
    max_xl = start_pos.x - 0.150 # x left
    max_xr = start_pos.x + 0.300 # x right
    max_y = start_pos.y + 0.300 # y up
    max_zd = start_pos.z + 0.150 # z down
    max_zu = start_pos.z - 0.110 # z up

    # need to transform leap motion input
    curr_pos = transform_pos(new_pos)

    dp = 3 # decimal points
    # we round the positions so it is much easier to carry out checks
    prev_pos_x = round(prev_pos.x, dp)
    curr_pos_x = round(curr_pos.x, dp)
    prev_pos_y = round(prev_pos.y, dp)
    curr_pos_y = round(curr_pos.y, dp)
    prev_pos_z = round(prev_pos.z, dp)
    curr_pos_z = round(curr_pos.z, dp)

    '''
    We check 3 things:
    1. If the passed position is very similar to the current one (avoid duplicates)
    2. If the difference is larger than pos_diff (avoid large movements)
    3. If the new position exceeds the boundaries set
    '''

    # check if duplicate
    ch_duplicate = prev_pos_x == curr_pos_x \
        or prev_pos_y == curr_pos_y \
        or prev_pos_z == curr_pos_z

    if ch_duplicate:
        errorLbl_text.set("Co-ordinates are too close!\n\
        Old Values: x: %.3f | y: %.3f | z: %.3f\n \
        New Values: x: %.3f | y: %.3f | z: %.3f\n" \
            % (prev_pos_x, prev_pos_y, prev_pos_z,
                curr_pos_x, curr_pos_y, curr_pos_z))
        return

    # check if exceeding boundaries
    ch_boundary_x = curr_pos_x > max_xr \
        or curr_pos_x < max_xl
    ch_boundary_y = curr_pos_y > max_y
    ch_boundary_z = curr_pos_z > max_zd \
        or curr_pos_z < max_zu

    if ch_boundary_x or ch_boundary_y or ch_boundary_z:
        posLbl_text.set("Current Position\nx: %.3f | %s \ny: %.3f | %s\nz: %.3f | %s" \
            % (curr_pos.x, ch_boundary_x, curr_pos.y, ch_boundary_y, curr_pos.z, ch_boundary_z))
        errorLbl_text.set("Boundaries reached!\ni.e. True next to x means the boundary for x was reached")
        return

    # check if difference too high
    arr_prevpos = [prev_pos_x, prev_pos_y, prev_pos_z]
    if any(x > 0.0 for x in arr_prevpos):
        pos_diff = 0.100
        ch_diff_x = abs(prev_pos_x - curr_pos_x) > pos_diff
        ch_diff_y = abs(prev_pos_y - curr_pos_y) > pos_diff
        ch_diff_z = abs(prev_pos_z - curr_pos_z) > pos_diff
        if ch_diff_x or ch_diff_y or ch_diff_z:
            errorLbl_text.set("Too large of a difference!\n\
            Old Values: x: %.3f | y: %.3f | z: %.3f\n \
            New Values: x: %.3f | y: %.3f | z: %.3f\n \
            Problem is with: x: %s | y: %s | z: %s" \
                % (prev_pos_x, prev_pos_y, prev_pos_z,
                    curr_pos_x, curr_pos_y, curr_pos_z,
                    ch_diff_x, ch_diff_y, ch_diff_z))
            return

    # clear any previous error set as in this run all checks are false
    errorLbl_text.set("")

    # keep track of previous position
    prev_pos = curr_pos
    # all checks equate to false thus this is a valid attempt
    print "\n=[ INFO: Valid new position passed, attempting: ", curr_pos_x, curr_pos_y, curr_pos_z, "]=\n"
    # we are now executing
    executing = True
    # append current pose to the waypoints array
    waypoints = []
    waypoints.append(group.get_current_pose().pose)
    # set the pose for x, y and z
    wpose = geometry_msgs.msg.Pose()
    wpose.position.x = curr_pos.x
    wpose.position.y = curr_pos.z # we switch z and y because the leap motion is faced upwards
    wpose.position.z = curr_pos.y
    waypoints.append(copy.deepcopy(wpose))
    # plan the movement
    (plan, fraction) = group.compute_cartesian_path(
                            waypoints,   # waypoints to follow
                            0.01,        # eef_step
                            0.0)         # jump_threshold
    # execute the plan
    group.execute(plan)
    # we are no longer executing
    executing = False
    # update label text
    posLbl_text.set("Current Position\nx: %.3f\ny: %.3f\nz: %.3f" % (curr_pos.x, curr_pos.y, curr_pos.z))
    zeroPosLbl_text.set("Zero Position\nx: %.3f\ny: %.3f\nz: %.3f" % (zero_pos.x, zero_pos.y, zero_pos.z))


def transform_pos(pos):
    global zero_pos
    # lower leap motion values by this much
    conversion_value = 0.001
    # adjust the offset and pass back the new coordinates
    pos.x = zero_pos.x - (pos.x * conversion_value) # inverted
    pos.y = zero_pos.y + (pos.y * conversion_value)
    pos.z = zero_pos.z + (pos.z * conversion_value)
    return pos


def lm_move(leap_msg):
    global paused
    if not executing and not paused:
        # store xyz information in a variable
        lm_palm_pos = leap_msg.palmpos
        # create an array of the xyz
        pos_list = [lm_palm_pos.x, lm_palm_pos.y, lm_palm_pos.z]
        # check if x,y,z is > than 0.0 (avoid passing 0.0 coordinates)
        if any(x > 0.0 for x in pos_list):
            begin_plan(lm_palm_pos)
        

def tracking_control(*ignore):
    global paused
    global sub
    global resumed_run
    if paused:
        # adjust button text/colour
        controlBtn_text.set("Pause")
        controlBtn.configure(bg="yellow")
        # adjust global flags
        paused = False
        resumed_run = True
        # register back to the topic
        lm_listener()
    else:
        # unregister from the topic
        sub.unregister()
        # adjust button text/colour
        controlBtn_text.set("Resume")
        controlBtn.configure(bg="green")
        # adjust global flag
        paused = True


def show_zeropos_win(*ignore):
    global win_hidden
    if win_hidden:
        window.deiconify()
        win_hidden = False
    else:
        window.withdraw()
        win_hidden = True


def reset_zeropos(*ignore):
    global zero_pos
    zero_pos = start_pos
    zeroPosLbl_text.set("Zero Position\nx: %.3f\ny: %.3f\nz: %.3f" % (zero_pos.x, zero_pos.y, zero_pos.z))


def lm_listener():
    global sub
    # register to the topic
    sub = rospy.Subscriber("/leapmotion/data", leapros, lm_move, queue_size=1)


if __name__ == '__main__':
    try:
        # move the robot to an initial comfortable position
        values = [4.111435176058169, 
                  2.715653621728593, 
                  0.7256647920137681, 
                  5.983459446005512, 
                  -5.682231515319553, 
                  -6.283185179581844]
        group.set_joint_value_target(values)
        plan = group.plan()
        group.execute(plan)
        rospy.sleep(3)

        # save these positions on run
        start_pos = group.get_current_pose().pose.position
        zero_pos = group.get_current_pose().pose.position

        # create a root window
        root = tk.Tk()
        root.title("LM_Move | Control Panel")
        root.geometry("350x180")
        root.resizable(False, False)
        
        # current pos
        posLbl_text = tk.StringVar()
        posLbl = tk.Label(root, textvariable=posLbl_text)
        posLbl_text.set("Current Position")
        posLbl.pack()

        # error label
        errorLbl_text = tk.StringVar()
        errorLbl = tk.Label(root, textvariable=errorLbl_text, fg="red")
        errorLbl.pack(side="bottom")

        # pause button
        controlBtn_text = tk.StringVar()
        controlBtn = tk.Button(root, textvariable=controlBtn_text, command=tracking_control, width=20)
        controlBtn_text.set("Pause")
        controlBtn.configure(bg="yellow")
        controlBtn.pack()

        # show zero pos window button
        showZeroPosBtn = tk.Button(root, text="Zero Pos Info", command=show_zeropos_win, width=20)
        showZeroPosBtn.pack()

        # window for zero pos
        window = tk.Toplevel()
        window.title("LM_Move | Zero Position Info")
        window.geometry("350x130")
        window.resizable(False, False)
        window.withdraw()

        # zero pos label
        zeroPosLbl_text = tk.StringVar()
        zeroPosLbl = tk.Label(window, textvariable=zeroPosLbl_text)
        zeroPosLbl_text.set("Zero Position\n\nNone!")
        zeroPosLbl.pack(side="left")

        # old zero pos label
        old_zeroPosLbl_text = tk.StringVar()
        old_zeroPosLbl = tk.Label(window, textvariable=old_zeroPosLbl_text)
        old_zeroPosLbl_text.set("Previous Zero Position\n\nNone!")
        old_zeroPosLbl.pack(side="right")

        # reset zero pos button
        resetZeroPosBtn = tk.Button(window, text="Reset", command=reset_zeropos, width=20)
        resetZeroPosBtn.configure(bg="orange")
        resetZeroPosBtn.pack(side="bottom")

        # subscribe to data
        root.after(1, lm_listener)
        root.mainloop()
    except rospy.ROSInterruptException:
        pass