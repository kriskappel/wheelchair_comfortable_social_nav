import roslaunch 
import rospy 

rospy.init_node('test_roslaunch', anonymous=True)
gazebo_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(gazebo_uuid)
gazebo_launch = roslaunch.parent.ROSLaunchParent(gazebo_uuid, ["/home/kriskappel/catkin_ws/src/wheelchair_comfortable_social_nav/nuric_wheelchair_model_02/launch/wheelchair_corridors3m.launch"])
gazebo_launch.start()
rospy.loginfo("GAZEBO LAUNCH STARTED")

rospy.sleep(5)

nav_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(nav_uuid)
nav_launch = roslaunch.scriptapi.ROSLaunch()
nav_launch.parent = roslaunch.parent.ROSLaunchParent(nav_uuid, ["/home/kriskappel/catkin_ws/src/wheelchair_comfortable_social_nav/comfort_social_nav/launch/wheelchair_nav.launch"])
nav_launch.start()
rospy.loginfo("NAV LAUNCH STARTED")

rosbag_node = roslaunch.core.Node(package='rosbag', node_type='record', name='record_run', args="record -O /home/kriskappel/catkin_ws/src/wheelchair_comfortable_social_nav/comfort_social_nav/bags/test3.bag -a")
nav_launch.launch(rosbag_node)

try:
  gazebo_launch.spin()
  nav_launch.spin()
finally:
  # After Ctrl+C, stop all nodes from running
  gazebo_launch.shutdown()
  nav_launch.parent.shutdown()
# import roslaunch
# import rospy

# uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
# roslaunch.configure_logging(uuid)

# cli_args1 = ['pkg1', 'file1.launch', 'arg1:=arg1', 'arg2:=arg2']
# cli_args2 = ['pkg2', 'file2.launch', 'arg1:=arg1', 'arg2:=arg2']
# cli_args3 = ['pkg3', 'file3.launch']
# roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)
# roslaunch_args1 = cli_args1[2:]

# roslaunch_file2 = roslaunch.rlutil.resolve_launch_arguments(cli_args2)
# roslaunch_args2 = cli_args2[2:]

# roslaunch_file3 = roslaunch.rlutil.resolve_launch_arguments(cli_args3)

# launch_files = [(roslaunch_file1, roslaunch_args1), (roslaunch_file2, roslaunch_args2), roslaunch_file3]

# parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

# parent.start()