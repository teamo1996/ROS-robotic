#include "ros/ros.h"
#include "ros_hw_3/turtle_control.h"
#include "geometry_msgs/Twist.h"
#include "sstream"
#include "map"

std::map<std::string,geometry_msgs::Twist> m_controls;

bool turtle_control(ros_hw_3::turtle_control::Request &req,
                    ros_hw_3::turtle_control::Response &res){
    
    //注册节点句柄
    ros::NodeHandle n;
    
    //注册发布者
    
    std::stringstream ss;
    
    //查找当前系统下是否已经存在对应小乌龟的控制消息
    ss << "/" << req.turtlename << "/cmd_vel";
    
    std::map<std::string,geometry_msgs::Twist>::iterator iter = m_controls.find(ss.str());
    
    if(iter != m_controls.end()){
        //如果已经存在
        if(req.begin == true)
        {
            iter->second.linear.x = req.x;
            iter->second.angular.z = req.z;
        }else{
            iter->second.linear.x = 0;
            iter->second.angular.z = 0;
        }

    }
    else{
        //如果不存在，则新建一个控制消息
        geometry_msgs::Twist control_msg;
        if(req.begin == true)
        {
            control_msg.linear.x = req.x;
            control_msg.angular.z = req.z;
        }else{
            control_msg.linear.x = 0;
            control_msg.angular.z = 0;
        }
        
        m_controls.insert(std::pair<std::string,geometry_msgs::Twist>(ss.str(),control_msg));
    }
    
    //构建发布者和消息队列
    std::vector<ros::Publisher> publist;
    std::vector<geometry_msgs::Twist> msglist;
    
    for(std::map<std::string,geometry_msgs::Twist>::iterator i = m_controls.begin();i != m_controls.end();i++){
            
            ros::Publisher pub = n.advertise<geometry_msgs::Twist>(i->first.c_str(),1000);
            std::string test = i->first;
            publist.push_back(pub);
            msglist.push_back(i->second);
    }
    
    ros::Rate loop_rate(10);
    
    res.success = true;
    
    //循环发布消息
    while(ros::ok()){
        for(int i = 0;i<publist.size();i++){
            publist[i].publish(msglist[i]);
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
     
    ros::shutdown(); 
    return true;
}


int main(int argc,char** argv){
    //初始化节点
    ros::init(argc,argv,"motion_control_service");
    
    //创建节点句柄
    ros::NodeHandle n ;
    
    //创建服务器
    ros::ServiceServer service = n.advertiseService("/motion_control",turtle_control);
    
    ROS_INFO("motion_control_service start");
    
    ros::spin();
    
    return 0;
}
