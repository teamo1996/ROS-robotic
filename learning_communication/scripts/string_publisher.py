#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将发布chatter话题，消息类型String

import rospy
from std_msgs.msg import String

def velocity_publisher():
	# ROS节点初始化
    rospy.init_node('talker', anonymous=True)

	# 创建一个Publisher，发布名为/chatter的topic，消息类型为String，队列长度10
    pub = rospy.Publisher('chatter', String, queue_size=10)

	#设置循环的频率
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
		# 初始化PersonMsg类型的消息
        hello_str = "hello world %s" % rospy.get_time()

		# 发布消息
        pub.publish(hello_str)
        rospy.loginfo(hello_str)

		# 按照循环频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        pass
