#!/usr/bin/env python3

import rospy
import actionlib
import roslaunch
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String

import os
import time

def loginfo_green(message):
    '''
    This function acts as a wrapper around rospy.loginfo to print messages in green color.
    '''
    green_start = '\033[92m'
    color_reset = '\033[0m'
    rospy.loginfo(green_start + str(message) + color_reset)

def movebase_client(pose_goal):
	# Create an action client
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

	# Wait for the action server to come up
	client.wait_for_server()

	# Define a goal to send to the move_base action server
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"  # Use "map" for global goals, "base_link" for local goals
	goal.target_pose.header.stamp = rospy.Time.now()

	# Set the goal position and orientation
	goal.target_pose.pose = pose_goal  # Change these values to your desired goal

	# Send the goal
	rospy.loginfo("Sending goal")
	client.send_goal(goal)

	# Wait for the result
	wait = client.wait_for_result()

	if not wait:
		rospy.logerr("Action server not available!")
	else:
		return client.get_result()


def bringup_sim(index, folder_name, sim_launch_path, nav_launch_path, pose_goal, syscommand_pub):
	logmsg = "=====RUN " + str(index) + ": starting====="
	loginfo_green(logmsg)
	rospy.sleep(5)
	# rospy.loginfo(logmsg)
	rospy.loginfo("GAZEBO LAUNCH STARTING")
	gazebo_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(gazebo_uuid)
	gazebo_launch = roslaunch.parent.ROSLaunchParent(gazebo_uuid, [sim_launch_path])
	gazebo_launch.start()
	rospy.loginfo("GAZEBO LAUNCH STARTED")

	rospy.sleep(5)

	rospy.loginfo("NAV LAUNCH STARTING")
	nav_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(nav_uuid)
	nav_launch = roslaunch.scriptapi.ROSLaunch()
	nav_launch.parent = roslaunch.parent.ROSLaunchParent(nav_uuid, [nav_launch_path])
	nav_launch.start()
	rospy.loginfo("NAV LAUNCH STARTED")

	bags_folder = "/home/kriskappel/catkin_ws/src/wheelchair_comfortable_social_nav/comfort_social_nav/bags/" + folder_name
	images_folder= "/home/kriskappel/catkin_ws/src/wheelchair_comfortable_social_nav/comfort_social_nav/images/" + folder_name

	if not os.path.exists(bags_folder):
		os.makedirs(bags_folder)
		# loginfo_green(logmsg)
		rospy.loginfo(f"Folder '{bags_folder}' created.")
	if not os.path.exists(images_folder):
		os.makedirs(images_folder)
		# loginfo_green(logmsg)
		rospy.loginfo(f"Folder '{images_folder}' created.")

	bag_args = "record -O " + bags_folder + "/" + folder_name + str(index) + ".bag /amcl_pose /cmd_vel /initialpose /move_base/cancel /move_base/current_goal /move_base/feedback /move_base/global_costmap/comfort_layer/current_value /move_base/global_costmap/social_navigation_layer/current_value /move_base/goal /move_base/local_costmap/social_navigation_layer/current_value /move_base/local_costmap/vision_layer/current_value /move_base/recovery_status /move_base/result /move_base/status /move_base_simple/goal /odom /trajectory"

	rosbag_node = roslaunch.core.Node(package='rosbag', node_type='record', name='record_run', args=bag_args, output=None)
	nav_launch.launch(rosbag_node)

	logmsg = "=====RUN " + str(index) + ": recording bag====="
	loginfo_green(logmsg)
	# rospy.loginfo(logmsg)

	logmsg = "=====RUN " + str(index) + ": starting navigation====="
	loginfo_green(logmsg)
	# rospy.loginfo(logmsg)
	result = movebase_client(pose_goal)

	syscommand_pub.publish("savegeotiff")
	if result:
		logmsg = "=====RUN " + str(index) + ": navigation completed====="
		loginfo_green(logmsg)
		# rospy.loginfo(logmsg)
		gazebo_launch.shutdown()
		nav_launch.parent.shutdown()
	else:
		logmsg = "=====RUN " + str(index) + ": navigation failed====="
		rospy.logerr(logmsg)
		gazebo_launch.shutdown()
		nav_launch.parent.shutdown()

	try:
		gazebo_launch.spin()
		nav_launch.spin()
	finally:
		# After Ctrl+C, stop all nodes from running
		gazebo_launch.shutdown()
		nav_launch.parent.shutdown()


if __name__ == '__main__':

	rospy.init_node('launching_script', anonymous=True)
	# Create a publisher for /syscommand
	syscommand_pub = rospy.Publisher('/syscommand', String, queue_size=1)
	rospy.sleep(1)  # Give the publisher a moment to set up

	#---VARIABLES TO CHANGE---

	num_runs = 50

	bag_folder_name = "anglo_comfort_social_nav"

	# pose_goal = Pose(Point(3.0, -8.5, 0.0), Quaternion(0.0, 0.0, 1.0, 0.0)) #Corridor3m
	pose_goal = Pose(Point(32.0, -1.25, 0.0), Quaternion(0.0, 0.0, 0.0, 0.1)) #anglo

	# sim_launch_path = "/home/kriskappel/catkin_ws/src/wheelchair_comfortable_social_nav/nuric_wheelchair_model_02/launch/wheelchair_corridors3m_actor.launch"
	# nav_launch_path = "/home/kriskappel/catkin_ws/src/wheelchair_comfortable_social_nav/comfort_social_nav/launch/wheelchair_nav.launch"

	sim_launch_path = "/home/kriskappel/catkin_ws/src/wheelchair_comfortable_social_nav/nuric_wheelchair_model_02/launch/wheelchair_anglo_3_metade_1.launch"
	nav_launch_path = "/home/kriskappel/catkin_ws/src/wheelchair_comfortable_social_nav/comfort_social_nav/launch/wheelchair_nav_anglo.launch"

	#---------------

	try:
		for i in range(0, num_runs):
			bringup_sim(i, bag_folder_name, sim_launch_path, nav_launch_path, pose_goal, syscommand_pub)
			logmsg = "=====RUN " + str(i) + ": ended====="
			loginfo_green(logmsg)
			# rospy.loginfo(logmsg)
			

	except rospy.ROSInterruptException:
		pass
 	# try:
    #     result = movebase_client()
    #     if result:
    #         rospy.loginfo("Goal execution done!")
    # except rospy.ROSInterruptException:
    #     rospy.loginfo("Navigation test finished.")
