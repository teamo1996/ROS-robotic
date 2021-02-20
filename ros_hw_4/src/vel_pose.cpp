#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

//打印位置的回调函数
void printpose(const turtlesim::Pose::ConstPtr &pose)
{
    ROS_INFO("turtle_pose: x:%f,y:%f,theta:%f,linear_velocity:%f,angular_velocity:%f.\n",pose->x,pose->y,pose->theta,pose->linear_velocity,pose->angular_velocity);
    ROS_INFO("---");
}

int main(int argc, char** argv){
    
    //节点初始化
    ros::init(argc,argv,"vel_pose");
    
    //创建节点句柄
    ros::NodeHandle n;
    
    //创建发布者
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1000);
    //创建订阅者
    ros::Subscriber sub = n.subscribe("/turtle1/pose",1000,printpose);
    
    //消息发布延迟函数
    ros::Rate loop_rate(10);
    
    while(ros::ok()){
        //创建消息并发布
        geometry_msgs::Twist msg;
        msg.linear.x = 0.5;
        msg.angular.z = 0.5;
        
        pub.publish(msg);
        loop_rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
}
