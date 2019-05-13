#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool

def talker():
	pub=rospy.Publisher('box_chatter',Bool,queue_size=1000)
	rospy.init_node('try',anonymous=True)
	rate=rospy.Rate(10)
	while not rospy.is_shutdown():
		hello=False
		rospy.loginfo(hello)
		pub.publish(hello)
		rate.sleep()

if __name__=='__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
