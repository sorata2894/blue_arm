#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, radians
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf import transformations as tf

ROTATION_AXIS = 'sxyz'

class BlueArmMoveGroup(object):
    def __init__(self):
        super(BlueArmMoveGroup, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the blue arm robot, so we set the group's name to "blue_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "blue_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print( "============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print( "============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print( "============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print( "============ Printing robot state")
        print( robot.get_current_state())
        print( "")
        ## END_SUB_TUTORIAL


        # Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.dof = 7

        #Add box under base
        box_pos = [0, 0, 0]
        box_euler = [0, 0, 0]
        box_size = [1, 1, 0.01]
        self.add_box('base_box', 'world', box_pos, box_euler, box_size)

    def all_close(self, goal, actual, tolerance):
        """
        Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
        @param: goal       A list of floats, a Pose or a PoseStamped
        @param: actual     A list of floats, a Pose or a PoseStamped
        @param: tolerance  A float
        @returns: bool
        """
        all_equal = True
        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    return False

        elif type(goal) is geometry_msgs.msg.PoseStamped:
            return self.all_close(goal.pose, actual.pose, tolerance)

        elif type(goal) is geometry_msgs.msg.Pose:
            return self.all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

        return True

    
    def go_to_joint_goal(self, joint_goal):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        if len(joint_goal) != self.dof:
            return False
        move_group = self.move_group
        # joint_goal_ = move_group.get_current_joint_values()
        # print(type(joint_goal_))
        # print(joint_goal_)
        # print(type(joint_goal))
        # print(joint_goal)
        # for i, value in enumerate(joint_goal):
        #     joint_goal_[i] = value

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
        ## thing we want to do is move it to a slightly better configuration.
        # We can get the joint values from the group and adjust some of the values:

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        # move_group.set_joint_value_target(joint_goal)
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()
        # move_group.clear_joint_value_target()
        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return self.all_close(joint_goal, current_joints, 0.05)

    
    def go_to_pose_goal(self, pos, euler):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        quaternion = tf.quaternion_from_euler(radians(euler[0]), radians(euler[1]), radians(euler[2]), axes=ROTATION_AXIS)
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]
        pose_goal.position.x = pos[0]
        pose_goal.position.y = pos[1]
        pose_goal.position.z = pos[2]

        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return self.all_close(pose_goal, current_pose, 0.05)

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)
        
        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Receieved
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL
    
    def add_box(self, box_name, parent_frame, box_pos, box_euler, box_size, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene at the location of the left finger:
        quaternion = tf.quaternion_from_euler(radians(box_euler[0]), radians(box_euler[1]), radians(box_euler[2]), axes=ROTATION_AXIS)
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = parent_frame
        box_pose.pose.orientation.x = quaternion[0]
        box_pose.pose.orientation.y = quaternion[1]
        box_pose.pose.orientation.z = quaternion[2]
        box_pose.pose.orientation.w = quaternion[3]
        box_pose.pose.position.x = box_pos[0]
        box_pose.pose.position.y = box_pos[1]
        box_pose.pose.position.z = box_pos[2]
        scene.add_box(box_name, box_pose, size=box_size)

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        return self.wait_for_state_update(box_name, box_is_known=True, timeout=timeout)

    def remove_box(self, box_name, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

def main():
    rospy.init_node('blue_arm_sample', anonymous=True)
    blue_arm = BlueArmMoveGroup()


    # # joint_goal = [0., pi/2, 0., pi/2, 0., pi/9, 0.]
    # joint_goal = [0, radians(10), radians(180), radians(-160), 0, radians(-10), 0]
    # if blue_arm.go_to_joint_goal(joint_goal) is False:
    #     rospy.logerr("Move Robot Failed!!")
    # rospy.sleep(3)

    # joint_goal = [0, 0, radians(180), radians(-90), 0, radians(-1), 0]
    # if blue_arm.go_to_joint_goal(joint_goal) is False:
    #     rospy.logerr("Move Robot Failed!!")
    # rospy.sleep(3)
 
    # # joint_goal = [0., pi/2, 0., pi/2, 0., pi/9, 0.]
    # joint_goal = [0, radians(90), 0, radians(-90), radians(-180), radians(20), 0]
    # if blue_arm.go_to_joint_goal(joint_goal) is False:
    #     rospy.logerr("Move Robot Failed!!")
    # rospy.sleep(3)


    # start point
    pos = [0.2, 0, 0.45]
    euler = [0, 0, 0]
    if blue_arm.go_to_pose_goal(pos, euler) is False:
        rospy.logerr("Move Robot Failed!!")
    rospy.sleep(6)


    pos = [0.3, 0, 0.3]
    euler = [0, 30, 0]
    if blue_arm.go_to_pose_goal(pos, euler) is False:
        rospy.logerr("Move Robot Failed!!")
    rospy.sleep(6)


    # down pocker_place
    pos = [0.34, 0, 0.08]
    euler = [0, 30, 0]
    if blue_arm.go_to_pose_goal(pos, euler) is False:
        rospy.logerr("Move Robot Failed!!")
    rospy.sleep(6)


    # down pocker_place_left
    pos = [0.34, 0.2, 0.08]
    euler = [0, 30, 0]
    if blue_arm.go_to_pose_goal(pos, euler) is False:
        rospy.logerr("Move Robot Failed!!")
    rospy.sleep(6)

    # down pocker_place_right
    pos = [0.34, -0.2, 0.08]
    euler = [0, 30, 0]
    if blue_arm.go_to_pose_goal(pos, euler) is False:
        rospy.logerr("Move Robot Failed!!")
    rospy.sleep(6)


    # upper pocker_place
    pos = [0.375, 0, 0.13]
    euler = [0, 30, 0]
    if blue_arm.go_to_pose_goal(pos, euler) is False:
        rospy.logerr("Move Robot Failed!!")
    rospy.sleep(6)

    # upper pocker_place_left
    pos = [0.375, 0.2, 0.13]
    euler = [0, 30, 0]
    if blue_arm.go_to_pose_goal(pos, euler) is False:
        rospy.logerr("Move Robot Failed!!")
    rospy.sleep(6)

    # upper pocker_place_right
    pos = [0.375, -0.2, 0.13]
    euler = [0, 30, 0]
    if blue_arm.go_to_pose_goal(pos, euler) is False:
        rospy.logerr("Move Robot Failed!!")
    rospy.sleep(6)

    
    
     
if __name__ == '__main__':
    main()
    

