#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from people_msgs.msg import People, Person
from std_msgs.msg import Header


def model_states_callback(data):     # Callback function to handle /gazebo/modelstates messages

    pub_people = rospy.Publisher('/people', People, queue_size=10)

    people = People()
    people.header = Header(0,rospy.Time.now(),"map")


    for i in range(len(data.name)):
        model_name = data.name[i]
        if model_name == "person1":
            model_pos = data.pose[i]
            # model_twist = data.twist[i]

            # print("Model Name: {}".format(model_name))
            print("Model Pose: {}".format(model_pos))
            # print("Model Twist: {}".format(model_twist))
            print("\n")

            person = Person()
            person.name = model_name
            person.position = model_pos.position

            # person.velocity.x = model_pos.orientation.x
            # person.velocity.y = model_pos.orientation.y

            people.people.append(person)

    pub_people.publish(people)





def model_states_listener():
    
    # Subscribe to /gazebo/modelstates topic
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)

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