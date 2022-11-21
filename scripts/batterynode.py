#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Bool

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