#!/usr/bin/env python
import rospy
from random import randint
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('/command_init', String, queue_size=10)
    pub2 = rospy.Publisher('/learning_progress', String, queue_size=10)
    pub3 = rospy.Publisher('/feature_progress', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    states = ['learn', 'detect']
    i = 0
    j = 0 % 100
    while not rospy.is_shutdown():
        if i == 0:
            i = 1
        else:
            i = 0
        pub.publish(states[i])
        pub2.publish(str(j))
        pub3.publish(str(j))
        j += 1
        rospy.sleep(5)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass