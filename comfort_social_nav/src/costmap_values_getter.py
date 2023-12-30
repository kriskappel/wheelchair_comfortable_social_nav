import rospy
from nav_msgs.msg import OccupancyGrid

def costmap_callback(msg):
    costmap_data = msg.data
    x_map, y_map = world_to_map_coordinates(msg, robot_x, robot_y)
    
    # Check bounds to avoid array out of bounds error
    if 0 <= x_map < msg.info.width and 0 <= y_map < msg.info.height:
        cost_at_robot_position = costmap_data[y_map * msg.info.width + x_map]
        rospy.loginfo(f"Cost at robot's position: {cost_at_robot_position}")
    else:
        rospy.logwarn("Robot is out of bounds in the costmap.")

def world_to_map_coordinates(costmap_msg, x_world, y_world):
    resolution = costmap_msg.info.resolution
    origin_x = costmap_msg.info.origin.position.x
    origin_y = costmap_msg.info.origin.position.y
    x_map = int((x_world - origin_x) / resolution)
    y_map = int((y_world - origin_y) / resolution)
    return x_map, y_map

rospy.init_node('costmap_reader')
rospy.Subscriber('/your_costmap_topic', OccupancyGrid, costmap_callback)
rospy.spin()