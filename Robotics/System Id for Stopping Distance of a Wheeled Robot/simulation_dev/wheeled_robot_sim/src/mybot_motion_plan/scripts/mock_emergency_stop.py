#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import sys

def talker():
    pub = rospy.Publisher('/emergency_stop', String, queue_size=1)
    rospy.init_node('mock_emergency_stop', anonymous=True)
    rate = rospy.Rate(10)

    initial_state = "False"
    rospy.loginfo(initial_state)
    pub.publish(initial_state)

    while not rospy.is_shutdown():
       print("Engage Emergency stop? (y or n)")
       try:
            input = raw_input()
            if input == "y":
                pubStr = "True"
            else:
                pubStr = "False"
       except:
            rospy.sleep(0.1)
            
       rospy.loginfo(pubStr)
       pub.publish(pubStr)
       rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
