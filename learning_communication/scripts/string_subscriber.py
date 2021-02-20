#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将订阅/person_info话题，消息类型String

import rospy
from std_msgs.msg import String

def stringCallback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def person_subscriber():
	# ROS节点初始化
    rospy.init_node('listener', anonymous=True)

	# 创建一个Subscriber，订阅名为/chatter的topic，注册回调函数stringCallback
    rospy.Subscriber("chatter", String, stringCallback)

	# 循环等待回调函数
    rospy.spin()

if __name__ == '__main__':
    person_subscriber()
