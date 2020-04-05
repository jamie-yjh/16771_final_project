#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from std_msgs.msg import Float64MultiArray


def talk():
    pub = rospy.Publisher('/lw_leg_controller/command', Float64MultiArray , queue_size=10)
    rospy.init_node('talk_to_gaz', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    msg = Float64MultiArray()
    #msg.layout[0] = {label: '',size: 4,stride: 1}
    while not rospy.is_shutdown():
        msg.data = [1, 1, 1, 1]
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

    
 

if __name__ == '__main__':
    try:
        talk()
    except rospy.ROSInterruptException:
        pass




