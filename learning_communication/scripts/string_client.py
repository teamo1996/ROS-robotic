#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将请求print_string服务，std_srvs::SetBool

import sys
import rospy
from std_srvs.srv import SetBool, SetBoolRequest

def string_client():
	# ROS节点初始化
    rospy.init_node('string_client')

	# 发现print_string服务后，创建一个服务客户端，连接名为print_string的service
    rospy.wait_for_service('print_string')
    try:
        string_client = rospy.ServiceProxy('print_string', SetBool)

		# 请求服务调用，输入请求数据
        response = string_client(True)
        return response.success, response.message
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
	#服务调用并显示调用结果
    print "Response : [%s] %s" %(string_client())

