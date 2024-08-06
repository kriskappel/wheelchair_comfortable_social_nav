#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from people_msgs.msg import People, Person
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import cos, sin


# Global variable to store robot position
robot_pos = None

def amcl_pose_callback(data):
    global robot_pos
    robot_pos = data.pose.pose.position

def model_states_callback(data):     # Callback function to handle /gazebo/modelstates messages

    pub_people = rospy.Publisher('/people', People, queue_size=10)

    people = People()
    people.header.frame_id = "map"

    if robot_pos is None:
        rospy.loginfo("Robot position not available")
        return

    person_near = False

    for i in range(len(data.name)):
        model_name = data.name[i]
        if "person" in model_name:
            model_pos = data.pose[i]

            # Check if the difference is less than 4 in x and y
            if abs(robot_pos.x - model_pos.position.x) < 4 and abs(robot_pos.y - model_pos.position.y) < 4:
                person_near=True
                (roll,pitch,yaw) = euler_from_quaternion([model_pos.orientation.x, model_pos.orientation.y, model_pos.orientation.z, model_pos.orientation.w])
                # mark_x = model_pos.position.x + sin(yaw)/4; 
                # mark_y = model_pos.position.y + cos(yaw)/4 ;
                mark_x = model_pos.position.x + sin(yaw)/2 ;
                mark_y = model_pos.position.y + cos(yaw)/2 ; 
                # model_twist = data.twist[i]
                # print("Model Name: {}".format(model_name))
                # print("Model Pose: {}".format(model_pos))
                # print("Model Twist: {}".format(model_twist))
                # print("\n")

                person = Person()
                person.name = model_name
                person.position = model_pos.position
                person.position.x=mark_x
                person.position.y=mark_y

                # person.velocity.x = model_pos.orientation.x
                # person.velocity.y = model_pos.orientation.y

                people.people.append(person)

    if person_near:
        pub_people.publish(people)





def model_states_listener():
    
    # Subscribe to /gazebo/modelstates topic
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
    # Subscribe to /amcl_pose topic
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)

    # Spin to keep the script alive
    rospy.spin()

# if __name__ == '__main__':
#     try:
#         model_states_listener()
#     except rospy.ROSInterruptException:
#         pass

# Initialize ROS node
rospy.init_node('model_states_listener', anonymous=True)


model_states_listener()

while not rospy.is_shutdown():
    pass