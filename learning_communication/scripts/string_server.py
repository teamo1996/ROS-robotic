#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将提供print_string服务，std_srvs::SetBool

import rospy
from std_srvs.srv import SetBool, SetBoolResponse

def stringCallback(req):
	# 显示请求数据
    if req.data:
        rospy.loginfo("Hello ROS!")

		# 反馈数据
        return SetBoolResponse(True, "Print Successully")
    else:
        # 反馈数据
        return SetBoolResponse(False, "Print Failed")

def string_server():
	# ROS节点初始化
    rospy.init_node('string_server')

	# 创建一个名为/print_string的server，注册回调函数stringCallback
    s = rospy.Service('print_string', SetBool, stringCallback)

	# 循环等待回调函数
    print "Ready to print hello string."
    rospy.spin()

if __name__ == "__main__":
    string_server()


