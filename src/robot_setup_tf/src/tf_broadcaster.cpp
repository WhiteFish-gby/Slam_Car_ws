#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

/*

发布各个传感器到机器人中心的坐标变换
parent为机器人base_link
child为laser_link 

*/

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0.0, 0.2)),
        ros::Time::now(),"base_link", "base_laser"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.13, 0.0, -0.05)),
        ros::Time::now(),"camera_link", "base_link"));
    r.sleep();
  }
}
