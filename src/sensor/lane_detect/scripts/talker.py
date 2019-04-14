#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from little_ant_msgs.msg import Lane

def talker():
	rospy.init_node('talker', anonymous=True)

	pub = rospy.Publisher('/lane', Lane, queue_size=10)
	rate = rospy.Rate(10) # 10hz
	
	lane_msg = Lane()
	
	distance_from_center = 0.0
	
	sign = 1
	
	while not rospy.is_shutdown():
		lane_msg.header.stamp = rospy.Time.now()
		
		lane_msg.distance_from_center = distance_from_center
		
		distance_from_center = distance_from_center + 0.01 * sign
		
		if(distance_from_center>0.2):
			sign = -1
		elif(distance_from_center < -0.2):
			sign = 1
		
		pub.publish(lane_msg)
		
		rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
