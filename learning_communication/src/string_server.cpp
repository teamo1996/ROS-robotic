/**
 * 该例程将提供print_string服务，std_srvs::SetBool
 */
 
#include "ros/ros.h"
#include "std_srvs/SetBool.h"

// service回调函数，输入参数req，输出参数res
bool print(std_srvs::SetBool::Request  &req,
         std_srvs::SetBool::Response &res)
{
    // 打印字符串
    if(req.data)
	{
		ROS_INFO("Hello ROS!");
		res.success = true;
		res.message = "Print Successully";
	}
	else
	{
		res.success = false;
		res.message = "Print Failed";
	}

    return true;
}

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "string_server");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个名为print_string的server，注册回调函数print()
    ros::ServiceServer service = n.advertiseService("print_string", print);

    // 循环等待回调函数
    ROS_INFO("Ready to print hello string.");
    ros::spin();

    return 0;
}


