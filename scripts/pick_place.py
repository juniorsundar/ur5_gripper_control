#!/usr/bin/env python3

import sys
import moveit_commander
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header
from moveit_msgs.msg import DisplayTrajectory
from std_srvs.srv import Empty
from ur5_gripper_control.srv import FilterWorkspace, FilterWorkspaceRequest

def update_octomap():
    """Update octomap in moveit planning scene.
    """
    
    # First clearing octomap
    clear_octomap.call()
    # Loop through available depth cameras and obtain pointclouds for octomap
    for camera in camera_topics:
        req = FilterWorkspaceRequest()
        req.pointcloud_topic.data = camera + '/depth/color/points/'
        req.image_topic.data = camera + '/color/image_raw/'
        publish_octomap.call(req)
    return

if __name__ == "__main__":
    try:
        # Initialise moveit_commander and rosnode
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pick_place', anonymous=False)
        
        # Octomap topics and services
        camera_topics = ['camera_1_depth', 'camera_2_depth']
        clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
        publish_octomap = rospy.ServiceProxy('/filter_workspace', FilterWorkspace)
        
        # Diagnostic publisher for waypoint poses
        pose_pub = rospy.Publisher('/checker',PoseStamped,latch=True,queue_size=5)
        
        # Initialise robot and move groups
        robot = moveit_commander.robot.RobotCommander()
        arm_group = moveit_commander.move_group.MoveGroupCommander("ur5_arm")
        gripper_group = moveit_commander.move_group.MoveGroupCommander("gripper")

        rospy.sleep(2)
        
        # Start at home position
        update_octomap()
        home_state = arm_group.get_current_state().joint_state
        home_state.name = list(home_state.name)[:6]
        home_state.position = [arm_group.get_named_target_values('home')['shoulder_pan_joint'],
                                arm_group.get_named_target_values('home')['shoulder_lift_joint'],
                                arm_group.get_named_target_values('home')['elbow_joint'],
                                arm_group.get_named_target_values('home')['wrist_1_joint'],
                                arm_group.get_named_target_values('home')['wrist_2_joint'],
                                arm_group.get_named_target_values('home')['wrist_3_joint']]
        plan = arm_group.plan(home_state)
        success = arm_group.execute(plan[1], wait=True)
        arm_group.stop()
        while not success:  # FALLBACK FOR SAFETY
            arm_group.stop()
            plan = arm_group.plan(home_state)
            success = arm_group.execute(plan[1], wait=True)
            arm_group.stop()

        
        rospy.sleep(1)
        
        # Open gripper
        update_octomap()
        open_gripper = [gripper_group.get_named_target_values('open')['robotiq_85_left_knuckle_joint']]
        gripper_group.go(open_gripper, wait=True)
        gripper_group.stop()

        # Move to block
        update_octomap()
        pose_goal = Pose()
        pose_goal.orientation = arm_group.get_current_pose().pose.orientation
        pose_goal.position.x = 0.6
        pose_goal.position.y = 0.0
        pose_goal.position.z = 0.34
        arm_group.set_pose_target(pose_goal)
        plan = arm_group.plan()
        success = arm_group.execute(plan[1], wait=True)
        arm_group.stop()
        arm_group.clear_pose_targets()
        while not success:  # FALLBACK FOR SAFETY
            arm_group.set_pose_target(pose_goal)
            plan = arm_group.plan()
            success = arm_group.execute(plan[1], wait=True)
            arm_group.stop()
            arm_group.clear_pose_targets()

        rospy.sleep(1)
        
        # Close gripper
        update_octomap()
        close_gripper = [gripper_group.get_named_target_values('closed')['robotiq_85_left_knuckle_joint']]
        gripper_group.go(close_gripper, wait=True)
        gripper_group.stop()        

        rospy.sleep(1)

        # Lift block
        update_octomap()
        pose_goal.position.z = 0.5
        arm_group.set_max_velocity_scaling_factor(0.1)
        arm_group.set_pose_target(pose_goal)
        plan = arm_group.plan()
        success = arm_group.execute(plan[1], wait=True)
        arm_group.stop()
        arm_group.clear_pose_targets()
        while not success:  # FALLBACK FOR SAFETY
            arm_group.set_pose_target(pose_goal)
            plan = arm_group.plan()
            success = arm_group.execute(plan[1], wait=True)
            arm_group.stop()
            arm_group.clear_pose_targets()

        rospy.sleep(1)

        # Go to opposite side
        update_octomap()
        pose_goal.position.x = -0.6
        arm_group.set_max_velocity_scaling_factor(0.2)
        arm_group.set_pose_target(pose_goal)
        plan = arm_group.plan()
        success = arm_group.execute(plan[1], wait=True)
        arm_group.stop()
        arm_group.clear_pose_targets()
        while not success:  # FALLBACK FOR SAFETY
            arm_group.set_pose_target(pose_goal)
            plan = arm_group.plan()
            success = arm_group.execute(plan[1], wait=True)
            arm_group.stop()
            arm_group.clear_pose_targets()

        rospy.sleep(1)

        # Lower block
        update_octomap()
        pose_goal.position.z = 0.34
        arm_group.set_pose_target(pose_goal)
        plan = arm_group.plan()
        success = arm_group.execute(plan[1], wait=True)
        arm_group.stop()
        arm_group.clear_pose_targets()
        while not success:  # FALLBACK FOR SAFETY
            arm_group.set_pose_target(pose_goal)
            plan = arm_group.plan()
            success = arm_group.execute(plan[1], wait=True)
            arm_group.stop()
            arm_group.clear_pose_targets()

        rospy.sleep(1)

        # Release
        update_octomap()
        gripper_group.go(open_gripper, wait=True)
        gripper_group.stop()
        gripper_group.go(open_gripper, wait=True)
        gripper_group.stop()
        
        rospy.sleep(1)

        # Return to home
        update_octomap()
        plan = arm_group.plan(home_state)
        success = arm_group.execute(plan[1], wait=True)
        arm_group.stop()
        while not success:  # FALLBACK FOR SAFETY
            arm_group.stop()
            plan = arm_group.plan(home_state)
            success = arm_group.execute(plan[1], wait=True)
            arm_group.stop()
            
        # rospy.spin()

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass