#include "ros/ros.h"
#include "turtlesim/Spawn.h"
#include "sstream"
#include "string"
#include "ros/master.h"
#include "unistd.h"

int main(int argc,char** argv){
    
    //初始化节点
    ros::init(argc,argv,"spawn_new_turtle");
    
    //创建节点句柄
    ros::NodeHandle n;
    
    //创建客户端
    ros::ServiceClient client = n.serviceClient<turtlesim::Spawn>("/spawn");
    
    //创建服务
    turtlesim::Spawn srv;


    ros::service::waitForService("/spawn");
    
    //随机生成海龟
    int turtle_id = getpid();
    ROS_INFO("current_pid:%d",turtle_id);
    srand(turtle_id);
/*     bool flag = true;

    while(flag){
        turtle_id = rand();
        ros::master::V_TopicInfo topics;
        ros::master::getTopics(topics);

        std::stringstream check_info;
        check_info << "/turtle" << turtle_id << "/pose";
        flag = false;

        for(ros::master::V_TopicInfo::iterator i = topics.begin();i!= topics.end();i++){
            const ros::master::TopicInfo& info = *i;
            if(strcmp(i->name.c_str(),check_info.str().c_str()) == 0){
                flag = true;
            }
        }
    } */

    std::stringstream ss;
    ss << "turtle" << turtle_id;

    srv.request.name = ss.str();
    srv.request.x = rand()%12;
    srv.request.y = rand()%12;
    srv.request.theta = rand()%12;

    if(client.call(srv) == false){
        return 0;
    }
        
    ROS_INFO("successfully spwan new turtle:%s",srv.request.name.c_str());

    return 0;
    
}
