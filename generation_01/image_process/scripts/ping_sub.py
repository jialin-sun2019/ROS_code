#! /usr/bin/env python
import rospy
from image.msg import ping


def pingInfoCallback(msg):
    rospy.loginfo("subscriber:    X:%d Y:%d R:%d", 
			 msg.x, msg.y, msg.r)
def ping_sub():
    rospy.init_node("ping_sub", anonymous=True)
    rospy.Subscriber("/ping_info", ping, pingInfoCallback)
    rospy.spin()

if __name__ == "__main__":
    ping_sub()