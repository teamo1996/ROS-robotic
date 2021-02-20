#include "ros/ros.h"
#include "turtlesim/Spawn.h"
#include "sstream"

int main(int argc,char** argv){
    
    //初始化节点
    ros::init(argc,argv,"spawn_new_turtle");
    
    //创建节点句柄
    ros::NodeHandle n;
    
    //创建客户端
    ros::ServiceClient client = n.serviceClient<turtlesim::Spawn>("/spawn");
    
    //创建服务
    turtlesim::Spawn srv;
    
    //随机生成海龟
    srand(time(0));
    int turtle_id = rand();
    
    std::stringstream ss;
    ss << "turtle" << turtle_id;
    
    srv.request.name = ss.str();
    srv.request.x = rand()%12;
    srv.request.y = rand()%12;
    srv.request.theta = rand()%12;
    
    client.call(srv);
    
    ROS_INFO("successfully spwan new turtle:%s",srv.request.name.c_str());
    
    return 0;
    
}
