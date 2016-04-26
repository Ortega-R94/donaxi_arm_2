#!/usr/bin/env python

import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItDemo:
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_demo')
                
        right_arm = moveit_commander.MoveGroupCommander('right_arm')
        right_arm.set_named_target('resting')
        right_arm.go()   

        end_effector_link = right_arm.get_end_effector_link()         

        reference_frame = '/odom'
        
        right_arm.set_pose_reference_frame(reference_frame)
	rospy.loginfo(str(end_effector_link))
	rospy.loginfo(right_arm.get_planning_frame())
	rospy.loginfo(right_arm.get_pose_reference_frame())
	rospy.loginfo(right_arm.get_current_pose())
        
        right_arm.allow_replanning(True)

        right_arm.set_goal_position_tolerance(0.01)
        right_arm.set_goal_orientation_tolerance(0.05)

        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.441402636719
        target_pose.pose.position.y = -0.279191735455
        target_pose.pose.position.z = 0.863870298601
        target_pose.pose.orientation.x = -0.0281863349718
        target_pose.pose.orientation.y = 0.063611003310
        target_pose.pose.orientation.z = 0.765618014453
        target_pose.pose.orientation.w = 0.63952187353

        right_arm.set_start_state_to_current_state()

        right_arm.set_pose_target(target_pose, end_effector_link)

        traj = right_arm.plan()

        right_arm.execute(traj)

        rospy.sleep(1)

        right_arm.set_named_target('resting')

        right_arm.go()


        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItDemo()

    
    
