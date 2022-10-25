#!/usr/bin/env python3

import sys
import moveit_commander
import rospy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import DisplayTrajectory
from std_srvs.srv import Empty
from ur5_control.srv import FilterWorkspace, FilterWorkspaceRequest

def update_octomap():
    clear_octomap.call()

    for camera in camera_topics:
        req = FilterWorkspaceRequest()
        req.pointcloud_topic.data = camera + '/depth/color/points/'
        req.image_topic.data = camera + '/color/image_raw/'
        publish_octomap.call(req)
    
    return

if __name__ == "__main__":
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pick_place', anonymous=False)
        
        camera_topics = ['camera_1_depth', 'camera_2_depth']        
        clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
        publish_octomap = rospy.ServiceProxy('/filter_workspace', FilterWorkspace)
        
        robot = moveit_commander.robot.RobotCommander()
        
        arm_group = moveit_commander.move_group.MoveGroupCommander("ur5_arm")
        gripper_group = moveit_commander.move_group.MoveGroupCommander("gripper")
        display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", DisplayTrajectory, queue_size=20)

        # Start at home position
        update_octomap()
        joint_state = arm_group.get_current_state().joint_state
        go_home = [arm_group.get_named_target_values('home')['shoulder_pan_joint'],
                    arm_group.get_named_target_values('home')['shoulder_lift_joint'],
                    arm_group.get_named_target_values('home')['elbow_joint'],
                    arm_group.get_named_target_values('home')['wrist_1_joint'],
                    arm_group.get_named_target_values('home')['wrist_2_joint'],
                    arm_group.get_named_target_values('home')['wrist_3_joint']]
        joint_state.name = list(joint_state.name)[:6]
        joint_state.position = go_home
        plan = arm_group.plan(joint_state)
        success = arm_group.execute(plan[1], wait=True)
        if success:
            print('Success')
        else:
            print('Fail')
        arm_group.stop()
        
        
        # Open gripper
        update_octomap()
        open_gripper = [gripper_group.get_named_target_values('open')['robotiq_85_left_knuckle_joint']]
        # success, plan = arm_group.plan(open_gripper)
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
        # success = arm_group.go(wait=True)
        success = arm_group.execute(plan[1], wait=True)
        if success:
            print('Success')
        else:
            print('Fail')
        arm_group.stop()
        arm_group.clear_pose_targets()

        # Close gripper
        update_octomap()
        close_gripper = [gripper_group.get_named_target_values('closed')['robotiq_85_left_knuckle_joint']]
        gripper_group.go(close_gripper, wait=True)
        gripper_group.stop()        

        # Lift block
        update_octomap()
        pose_goal.position.z = 0.5
        arm_group.set_max_velocity_scaling_factor(0.1)
        arm_group.set_pose_target(pose_goal)
        plan = arm_group.plan()
        # success = arm_group.go(wait=True)
        success = arm_group.execute(plan[1], wait=True)
        if success:
            print('Success')
        else:
            print('Fail')
            plan = arm_group.plan()
            success = arm_group.execute(plan[1], wait=True)
        arm_group.stop()
        arm_group.clear_pose_targets()
        
        # # Go to opposite side
        # update_octomap()
        # pose_goal.position.x = -0.6
        # arm_group.set_max_velocity_scaling_factor(0.3)
        # arm_group.set_pose_target(pose_goal)
        # success = arm_group.go(wait=True)
        # arm_group.stop()
        # arm_group.clear_pose_targets()
        
        # # Lower block
        # update_octomap()
        # pose_goal.position.z = 0.34
        # arm_group.set_pose_target(pose_goal)
        # success = arm_group.go(wait=True)
        # arm_group.stop()
        # arm_group.clear_pose_targets()
        
        # # Release
        # update_octomap()
        # gripper_group.go(open_gripper, wait=True)
        # gripper_group.stop()
        
        # # Return to home
        # update_octomap()
        # arm_group.go(go_home, wait=True)
        # arm_group.stop()
        
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass