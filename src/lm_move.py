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

subscriber = None # we need to have the subscriberscriber as an object so we can unregister/register when paused/resumed
resumedRun = False # flag to determine if it is the first run after pressing 'resume'
paused = True # flag to determine if program is paused
executing = False # flag to determine if we are currently executing a plan
gui_positionInfo_hidden = True # keep track if zero pos window is hidden or not
prev_desiredPos = geometry_msgs.msg.Pose().position # keep track of what the previous passed position was
robot_zeroPos = geometry_msgs.msg.Pose().position # keep track of the robot zero position
hand_zeroPos = geometry_msgs.msg.Pose().position # keep track of the hand zero position
robot_initialPos = geometry_msgs.msg.Pose().position # keep track of the robot starting position


def begin_plan(hand_pos):
    global prev_desiredPos
    global robot_zeroPos
    global hand_zeroPos
    global executing
    global resumedRun

    # check if this is the first run after a resume
    if resumedRun:
        # display previous zero positions
        prev_hand_zeroPosText.set("Previous Robot Zero Position\nx: %.3f\ny: %.3f\nz: %.3f" \
            % (robot_zeroPos.x, robot_zeroPos.y, robot_zeroPos.z))
        prev_robot_zeroPosText.set("Previous Hand Zero Position\nx: %.3f\ny: %.3f\nz: %.3f" \
            % (hand_zeroPos.x, hand_zeroPos.y, hand_zeroPos.z))
        # set the updated zero positions
        robot_zeroPos.x = group.get_current_pose().pose.position.x
        robot_zeroPos.y = group.get_current_pose().pose.position.z
        robot_zeroPos.z = group.get_current_pose().pose.position.y
        hand_zeroPos.x = hand_pos.x
        hand_zeroPos.y = hand_pos.y
        hand_zeroPos.z = hand_pos.z
        # display updated zero positions
        hand_zeroPosText.set("Hand Zero Position\nx: %.3f\ny: %.3f\nz: %.3f" \
            % (hand_zeroPos.x, hand_zeroPos.y, hand_zeroPos.z))
        robot_zeroPosText.set("Robot Zero Position\nx: %.3f\ny: %.3f\nz: %.3f" \
            % (robot_zeroPos.x, robot_zeroPos.y, robot_zeroPos.z))
        resumedRun = False

    # need to transform leap motion input
    desiredPos = getDesiredPos(hand_pos)

    # boundaries
    # initial coords: x: 0.603 y: 0.124 z: 0.566
    maxLeft = robot_initialPos.x - 0.070 # x left
    maxRight = robot_initialPos.x + 0.070 # x right
    maxHeight = robot_initialPos.y + 0.300 # y up
    maxUp = robot_initialPos.z + 0.100 # z up
    maxDown = robot_initialPos.z - 0.080 # z down

    dp = 3 # decimal points
    # we round the positions so it is much easier to carry out checks
    prev_desiredPos_x = round(prev_desiredPos.x, dp)
    desiredPos_x = round(desiredPos.x, dp)
    prev_desiredPos_y = round(prev_desiredPos.y, dp)
    desiredPos_y = round(desiredPos.y, dp)
    prev_desiredPos_z = round(prev_desiredPos.z, dp)
    desiredPos_z = round(desiredPos.z, dp)

    '''
    We check 3 things:
    1. If the passed position is very similar to the current one (avoid duplicates)
    2. If the difference is larger than maxDiff (avoid large movements)
    3. If the new position exceeds the boundaries set
    '''

    # check if duplicate
    duplicateCheck = prev_desiredPos_x == desiredPos_x \
        or prev_desiredPos_y == desiredPos_y \
        or prev_desiredPos_z == desiredPos_z

    if duplicateCheck:
        errorText.set("Co-ordinates are too close!\n\
        Old Values: x: %.3f | y: %.3f | z: %.3f\n \
        New Values: x: %.3f | y: %.3f | z: %.3f\n" \
            % (prev_desiredPos_x, prev_desiredPos_y, prev_desiredPos_z,
                desiredPos_x, desiredPos_y, desiredPos_z))
        return

    # check if exceeding boundaries
    boundaryCheck_x = desiredPos_x > maxRight \
        or desiredPos_x < maxLeft
    boundaryCheck_y = desiredPos_y > maxHeight
    boundaryCheck_z = desiredPos_z > maxUp \
        or desiredPos_z < maxDown

    if boundaryCheck_x or boundaryCheck_y or boundaryCheck_z:
        currentPosText.set("Current Position\nx: %.3f | %s \ny: %.3f | %s\nz: %.3f | %s" \
            % (desiredPos.x, boundaryCheck_x, desiredPos.y, boundaryCheck_y, desiredPos.z, boundaryCheck_z))
        errorText.set("Boundaries reached!\ni.e. True next to x means the boundary for x was reached")
        return

    # check if difference too high
    prevPosArray = [prev_desiredPos_x, prev_desiredPos_y, prev_desiredPos_z]
    # avoid checking if there is no previous position
    if any(x > 0.0 for x in prevPosArray):
        # maximum difference we will accept
        maxDiff = 0.100
        diffCheck_x = abs(prev_desiredPos_x - desiredPos_x) > maxDiff
        diffCheck_y = abs(prev_desiredPos_y - desiredPos_y) > maxDiff
        diffCheck_z = abs(prev_desiredPos_z - desiredPos_z) > maxDiff
        if diffCheck_x or diffCheck_y or diffCheck_z:
            errorText.set("Too large of a difference!\n\
            Old Values: x: %.3f | y: %.3f | z: %.3f\n \
            New Values: x: %.3f | y: %.3f | z: %.3f\n \
            Problem is with: x: %s | y: %s | z: %s" \
                % (prev_desiredPos_x, prev_desiredPos_y, prev_desiredPos_z,
                    desiredPos_x, desiredPos_y, desiredPos_z,
                    diffCheck_x, diffCheck_y, diffCheck_z))
            return

    # clear any previous error set as in this run all checks are false
    errorText.set("")
    # we are now executing
    executing = True
    # keep track of previous position
    prev_desiredPos.x = desiredPos.x
    prev_desiredPos.y = desiredPos.y
    prev_desiredPos.z = desiredPos.z
    # all checks equate to false thus this is a valid attempt
    print "\n===> ATTEMPT: ", desiredPos.x, desiredPos.y, desiredPos.z
    # update the current position
    currentPosText.set("Current Position\nx: %.3f\ny: %.3f\nz: %.3f" % (desiredPos.x, desiredPos.y, desiredPos.z))
    
    # append current pose to the waypoints array
    waypoints = []
    waypoints.append(group.get_current_pose().pose)
    # set the pose for x, y and z
    wpose = geometry_msgs.msg.Pose()
    wpose.position.x = desiredPos.x
    wpose.position.y = desiredPos.z # we switch z and y because the leap motion is faced upwards
    wpose.position.z = desiredPos.y
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


def getDesiredPos(hand_pos):
    global robot_zeroPos
    global hand_zeroPos
    # lower leap motion values by this much
    conversion_value = 0.001
    # adjust the offset and pass back the new coordinates
    desired_pos = geometry_msgs.msg.Pose().position
    desired_pos.x = robot_zeroPos.x - ((hand_pos.x - hand_zeroPos.x) * conversion_value) # inverted
    desired_pos.y = robot_zeroPos.y + ((hand_pos.y - hand_zeroPos.y) * conversion_value)
    desired_pos.z = robot_zeroPos.z + ((hand_pos.z - hand_zeroPos.z) * conversion_value)
    return desired_pos


def beginExecution(leap_msg):
    global paused
    if not executing and not paused:
        # store xyz information in a variable
        palmPos = leap_msg.palmpos
        # create an array of the xyz
        posArray = [palmPos.x, palmPos.y, palmPos.z]
        # check if x,y,z is > than 0.0 (avoid passing 0.0 coordinates)
        if any(x > 0.0 for x in posArray):
            begin_plan(palmPos)
        

def trackingControl(*ignore):
    global paused
    global subscriber
    global resumedRun
    if paused:
        # adjust button text/colour
        pauseButtonText.set("Pause")
        pauseButton.configure(bg="yellow")
        # adjust global flags
        paused = False
        resumedRun = True
        # register back to the topic
        subscribeToTopic()
    else:
        # unregister from the topic
        subscriber.unregister()
        # adjust button text/colour
        pauseButtonText.set("Resume")
        pauseButton.configure(bg="green")
        # adjust global flag
        paused = True


def toggleInfoWindow(*ignore):
    global gui_positionInfo_hidden
    if gui_positionInfo_hidden:
        gui_positionInfo.deiconify()
        gui_positionInfo_hidden = False
    else:
        gui_positionInfo.withdraw()
        gui_positionInfo_hidden = True


def onCloseHide(*ignore):
    global gui_positionInfo_hidden
    gui_positionInfo.withdraw()
    gui_positionInfo_hidden = True


def onCloseQuit(*ignore):
    if tkMessageBox.askokcancel("Quit", "Do you want to quit?"):
        gui_mainWindow.destroy()


def resetZeroPos(*ignore):
    global robot_zeroPos
    global robot_initialPos
    global paused
    if not paused:
        robot_zeroPos.x = robot_initialPos.x
        robot_zeroPos.y = robot_initialPos.y
        robot_zeroPos.z = robot_initialPos.z
        robot_zeroPosText.set("Zero Position\nx: %.3f\ny: %.3f\nz: %.3f" % (robot_zeroPos.x, robot_zeroPos.y, robot_zeroPos.z))
        tkMessageBox.showinfo("Information", "Zero position has been reset")
    else:
        tkMessageBox.showerror("Error", "Cannot reset zero position because the program is currently paused")


def subscribeToTopic():
    global subscriber
    # register to the topic
    subscriber = rospy.Subscriber("/leapmotion/data", leapros, beginExecution, queue_size=1)


if __name__ == '__main__':
    try:
        # move the robot to an initial comfortable position
        initialJointValues = [4.111435176058169, 
                  2.715653621728593, 
                  0.7256647920137681, 
                  5.983459446005512, 
                  -5.682231515319553, 
                  -6.283185179581844]
        group.set_joint_value_target(initialJointValues)
        plan = group.plan()
        group.execute(plan)
        rospy.sleep(3)

        # save these positions on run
        robot_initialPos.x = group.get_current_pose().pose.position.x
        robot_initialPos.y = group.get_current_pose().pose.position.z
        robot_initialPos.z = group.get_current_pose().pose.position.y
        robot_zeroPos.x = group.get_current_pose().pose.position.x
        robot_zeroPos.y = group.get_current_pose().pose.position.z
        robot_zeroPos.z = group.get_current_pose().pose.position.y

        '''
        -----------------------------------------
            gui_mainWindow
        -----------------------------------------
        '''

        # main GUI window
        gui_mainWindow = tk.Tk()
        gui_mainWindow.title("LM-Move | Control Panel")
        gui_mainWindow.geometry("350x180")
        gui_mainWindow.resizable(False, False)
        
        # current position label
        currentPosText = tk.StringVar()
        currentPosLabel = tk.Label(gui_mainWindow, textvariable=currentPosText)
        currentPosText.set("Current Position\n\nNone!\n")
        currentPosLabel.pack()

        # error label
        errorText = tk.StringVar()
        errorLabel = tk.Label(gui_mainWindow, textvariable=errorText, fg="red")
        errorLabel.pack(side="bottom")

        # pause button
        pauseButtonText = tk.StringVar()
        pauseButton = tk.Button(gui_mainWindow, textvariable=pauseButtonText, command=trackingControl, width=20)
        pauseButtonText.set("Resume")
        pauseButton.configure(bg="green")
        pauseButton.pack()

        # toggle zero pos information window
        toggleInfoButton = tk.Button(gui_mainWindow, text="Positional Info", command=toggleInfoWindow, width=20)
        toggleInfoButton.pack()

        '''
        -----------------------------------------
            gui_positionInfo
        -----------------------------------------
        '''

        # new window for zero pos information
        gui_positionInfo = tk.Toplevel()
        gui_positionInfo.title("LM-Move | Positional Info")
        gui_positionInfo.geometry("350x160")
        gui_positionInfo.resizable(False, False)
        gui_positionInfo.withdraw()

        # robot zero pos label
        robot_zeroPosText = tk.StringVar()
        robot_zeroPosLabel = tk.Label(gui_positionInfo, textvariable=robot_zeroPosText).grid(row=0, column=0)
        robot_zeroPosText.set("Robot Zero Position\n\nNone!")

        # hand zero pos label
        hand_zeroPosText = tk.StringVar()
        hand_zeroPosLabel = tk.Label(gui_positionInfo, textvariable=hand_zeroPosText).grid(row=0, column=1)
        hand_zeroPosText.set("Hand Zero Position\n\nNone!")

        # previous robot zero pos label
        prev_robot_zeroPosText = tk.StringVar()
        prev_robot_zeroPosLabel = tk.Label(gui_positionInfo, textvariable=prev_robot_zeroPosText).grid(row=1, column=0)
        prev_robot_zeroPosText.set("Previous Hand Zero Position\n\nNone!")

        # previous hand zero pos label
        prev_hand_zeroPosText = tk.StringVar()
        prev_hand_zeroPosLabel = tk.Label(gui_positionInfo, textvariable=prev_hand_zeroPosText).grid(row=1, column=1)
        prev_hand_zeroPosText.set("Previous Robot Zero Position\n\nNone!")

        # reset zero pos button
        resetButton = tk.Button(gui_positionInfo, text="Reset Robot Zero Position", command=resetZeroPos, width=20)
        resetButton.configure(bg="orange")
        resetButton.grid(row=2, columnspan=2)

        # subscribe to data
        gui_mainWindow.after(1, subscribeToTopic)
        # handle closing of GUI
        gui_mainWindow.protocol("WM_DELETE_WINDOW", onCloseQuit)
        gui_positionInfo.protocol("WM_DELETE_WINDOW", onCloseHide)
        # keep the GUI running
        gui_mainWindow.mainloop()
    except rospy.ROSInterruptException:
        pass