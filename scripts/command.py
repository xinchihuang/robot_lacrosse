#!/usr/bin/env python3

import rospy
from std_msgs.msg import String


def command_publisher():
    rospy.init_node('command_publisher', anonymous=True)
    pub = rospy.Publisher('command', String, queue_size=10)

    while not rospy.is_shutdown():
        input_str = input("Input command: ")
        rospy.loginfo("Sending: %s", input_str)
        pub.publish(input_str)


if __name__ == '__main__':
    try:
        command_publisher()
    except rospy.ROSInterruptException:
        pass
