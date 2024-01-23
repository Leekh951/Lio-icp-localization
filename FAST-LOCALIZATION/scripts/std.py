#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    # Initialize the node with the name 'string_publisher'
    rospy.init_node('string_publisher')

    # Create a publisher. This publisher will publish messages of type String to the '/std' topic
    pub = rospy.Publisher('/std', String, queue_size=10)

    # Define the rate at which the messages will be published (e.g., 10 Hz)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        message = "stop"  # Here you can change the message you want to publish
        rospy.loginfo(message)  # This will log the message to the console
        pub.publish(message)  # This will publish the message to the '/std' topic

        rate.sleep()  # Sleeps just enough to maintain the desired rate

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass