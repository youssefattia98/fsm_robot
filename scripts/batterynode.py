#!/usr/bin/env python
"""
.. module:: batterynode
    :platform: Unix
    :synopsis: Python module for publishing the battery state to the topic (batterylevel)

.. moduleauthor:: Youssef Attia youssef-attia@live.com
"""


import rospy
from std_msgs.msg import Bool

"""
Global Variables used to set the charged time and charging time of the battery which is 7:1
"""
charegedtime = 7
notcharegedtime = 1

def talker():
    pub = rospy.Publisher('batterylevel', Bool, queue_size=10)
    rospy.init_node('batterylevel_node', anonymous=True)
    while not rospy.is_shutdown():
        batt = 1
        pub.publish(batt)
        rospy.sleep(charegedtime)
        batt = 0
        pub.publish(batt)
        rospy.sleep(notcharegedtime)

if __name__ == '__main__':
    try:
        print('calling talker')
        talker()
    except rospy.ROSInterruptException:
        pass