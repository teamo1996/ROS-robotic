/**
 * 该例程将请求/show_person服务，服务数据类型learning_communication::PersonSrv
 */

#include <ros/ros.h>
#include "learning_communication/PersonSrv.h"

int main(int argc, char** argv)
{
    // 初始化ROS节点
	ros::init(argc, argv, "person_client");

    // 创建节点句柄
	ros::NodeHandle node;

    // 发现/spawn服务后，创建一个服务客户端，连接名为/spawn的service
	ros::service::waitForService("/show_person");
	ros::ServiceClient person_client = node.serviceClient<learning_communication::PersonSrv>("/show_person");

    // 初始化learning_communication::Person的请求数据
	learning_communication::PersonSrv srv;
	srv.request.name = "Tom";
	srv.request.age  = 20;
	srv.request.sex  = learning_communication::PersonSrv::Request::male;

    // 请求服务调用
	ROS_INFO("Call service to show person[name:%s, age:%d, sex:%d]", 
			 srv.request.name.c_str(), srv.request.age, srv.request.sex);

	person_client.call(srv);

	// 显示服务调用结果
	ROS_INFO("Show person result : %s", srv.response.result.c_str());

	return 0;
};


