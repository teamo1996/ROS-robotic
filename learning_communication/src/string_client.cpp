/**
 * 该例程将请求print_string服务，std_srvs::SetBool
 */
 
#include "ros/ros.h"
#include "std_srvs/SetBool.h"

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "string_client");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个client，service消息类型是std_srvs::SetBool
    ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>("print_string");

    // 创建std_srvs::SetBool类型的service消息
    std_srvs::SetBool srv;
    srv.request.data = true;

    // 发布service请求，等待应答结果
    if (client.call(srv))
    {
        ROS_INFO("Response : [%s] %s", srv.response.success?"True":"False", 
									   srv.response.message.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service print_string");
        return 1;
    }

    return 0;
}

