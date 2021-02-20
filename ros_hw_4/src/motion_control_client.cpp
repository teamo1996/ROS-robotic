#include "ros/ros.h"
#include "ros_hw_4/turtle_control.h"
#include "sstream"
#include "unistd.h"

int main(int argc,char** argv){
    
    //初始化节点
    ros::init(argc,argv,"motion_control_client");
    
    //创建节点句柄
    ros::NodeHandle n ;
    
    //创建客户端
    ros::ServiceClient client = n.serviceClient<ros_hw_4::turtle_control>("/motion_control");

    ros::service::waitForService("/motion_control");
    
    //构建服务
    ros_hw_4::turtle_control srv;
    srv.request.turtlename = argv[1];//海龟名称
    srv.request.x = atof(argv[3]);
    srv.request.z = atof(argv[4]);
    std::string begin = argv[2];
    
    if(begin == "begin"){
        srv.request.begin = true;
        ROS_INFO("Start motion:%s",argv[1]);
    }
    else{
        srv.request.begin = false;
        ROS_INFO("Stop motion:%s",argv[1]);
    }

    std::stringstream ss;
    ss << "/" << argv[1] << "/set_pen";
    
    ros::service::waitForService(ss.str());

    sleep((getpid()%10)/10);
    //调用服务
    client.call(srv);

    ROS_INFO("test");
    // if(srv.response.success == true){
    //     ROS_INFO("successfully control!");
    // }
    // else{
    //     ROS_INFO("control failed");
    // }
    
    return 0;
    
}

