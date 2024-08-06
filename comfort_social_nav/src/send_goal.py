#!/usr/bin/env python3

# The values for the status of a goal are as follows:
# uint8 status
# uint8 PENDING         = 0   # The goal has yet to be processed by the action server
# uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
# uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
#                             #   and has since completed its execution (Terminal State)
# uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
# uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
#                             #    to some failure (Terminal State)
# uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
#                             #    because the goal was unattainable or invalid (Terminal State)
# uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
#                             #    and has not yet completed execution
# uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
#                             #    but the action server has not yet confirmed that the goal is canceled
# uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
#                             #    and was successfully cancelled (Terminal State)
# uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
#                             #    sent over the wire by an action server


import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from actionlib_msgs.msg import GoalStatus

def movebase_client():
    # Initialize a ROS node
    rospy.init_node('send_goal_py')

    # Create an action client
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Wait for the action server to come up
    client.wait_for_server()

    # Define a goal to send to the move_base action server
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"  # Use "map" for global goals, "base_link" for local goals
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set the goal position and orientation
    goal.target_pose.pose = Pose(Point(32.0, -1.25, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))  # Change these values to your desired goal

    # Send the goal
    rospy.loginfo("Sending goal")
    client.send_goal(goal)

    state = client.get_result()

        # if state == GoalStatus.SUCCEEDED:
        #     rospy.loginfo("Goal reached successfully!")
        # else:
        #     rospy.loginfo("Failed to reach the goal.")
    print(client.get_result())

    # Wait for the result
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        state = client.get_result()

        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached successfully!")
        else:
            rospy.loginfo(client.get_result())
            
            return client.get_result()



    rospy.loginfo("teste")

if __name__ == '__main__':
    try:
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
