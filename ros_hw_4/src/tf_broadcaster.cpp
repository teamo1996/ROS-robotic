#include "ros/ros.h"
#include "tf/transform_broadcaster.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_publisher");
  ros::NodeHandle n;

  ros::Rate loop_rate(100);

  tf::TransformBroadcaster broadcaster;
  
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0.1,0.0,0.2));
  tf::Quaternion q;
  q.setRPY(0.0,0.0,0.0);
  transform.setRotation(q);
  

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(transform,ros::Time::now(),"base_link", "base_laser"));
    loop_rate.sleep();
  }
}
