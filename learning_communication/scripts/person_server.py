#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将执行/show_person服务，服务数据类型learning_communication::PersonSrv

import rospy
from learning_communication.srv import PersonSrv, PersonSrvResponse

def personCallback(req):
	# 显示请求数据
    rospy.loginfo("Person: name:%s  age:%d  sex:%d", req.name, req.age, req.sex)

	# 反馈数据
    return PersonResponse("OK")

def person_server():
	# ROS节点初始化
    rospy.init_node('person_server')

	# 创建一个名为/show_person的server，注册回调函数personCallback
    s = rospy.Service('/show_person', PersonSrv, personCallback)

	# 循环等待回调函数
    print "Ready to show person informtion."
    rospy.spin()

if __name__ == "__main__":
    person_server()


