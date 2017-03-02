#include <ros/ros.h>  
#include <visualization_msgs/Marker.h>  

int main( int argc, char** argv )  
{  
  ros::init(argc, argv, "basic_shapes");  
  ros::NodeHandle n;  
  ros::Rate r(1);  
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);  
  
  // 初始化形状类型为CUBE  
  uint32_t shape = visualization_msgs::Marker::CUBE;  
    
  while (ros::ok())  
  {  
    visualization_msgs::Marker marker;  
    // 设置坐标系ID和时间戳。参考TF turtorials  
    marker.header.frame_id = "/my_frame";  
    marker.header.stamp = ros::Time::now();  
  
    // <span style="font-family: Consolas, 'Courier New', Courier, mono, serif; font-size: 12px; line-height: 18px;">为标志物设置名称空间和独一无二的ID</span>  
    // <span style="font-family: Consolas, 'Courier New', Courier, mono, serif; font-size: 12px; line-height: 18px; background-color: rgb(248, 248, 248);">所有的标志物设置相同的名称空间，而且id会覆盖旧的。</span>  
    marker.ns = "basic_shapes";  
    marker.id = 0;  
  
    // 设置标志物类型 Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER  
    marker.type = shape;  
  
    // 设置标志物的行为（action）   Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)  
    marker.action = visualization_msgs::Marker::ADD;  
  
    // 设置标志物的姿态.  This is a full 6DOF pose relative to the frame/time specified in the header  
    marker.pose.position.x = 0;  
    marker.pose.position.y = 0;  
    marker.pose.position.z = 0;  
    marker.pose.orientation.x = 0.0;  
    marker.pose.orientation.y = 0.0;  
    marker.pose.orientation.z = 0.0;  
    marker.pose.orientation.w = 1.0;  
  
    // <span style="font-family: Consolas, 'Courier New', Courier, mono, serif; font-size: 12px; line-height: 18px;">设置标记物的尺度</span> -- 1x1x1 here means 1m on a side  
    marker.scale.x = 1.0;  
    marker.scale.y = 1.0;  
    marker.scale.z = 1.0;  
  
    // 设置颜色 -- be sure to set alpha to something non-zero!  
    marker.color.r = 0.0f;  
    marker.color.g = 1.0f;  
    marker.color.b = 0.0f;  
    marker.color.a = 1.0;  
  
    marker.lifetime = ros::Duration();  
  
    // Publish the marker  
    while (marker_pub.getNumSubscribers() < 1)  
    {  
      if (!ros::ok())  
      {  
        return 0;  
      }  
      ROS_WARN_ONCE("Please create a subscriber to the marker");  
      sleep(1);  
    }  
    marker_pub.publish(marker);  
  
    // Cycle between different shapes  
    switch (shape)  
    {  
    case visualization_msgs::Marker::CUBE:  
      shape = visualization_msgs::Marker::SPHERE;  
      break;  
    case visualization_msgs::Marker::SPHERE:  
      shape = visualization_msgs::Marker::ARROW;  
      break;  
    case visualization_msgs::Marker::ARROW:  
      shape = visualization_msgs::Marker::CYLINDER;  
      break;  
    case visualization_msgs::Marker::CYLINDER:  
      shape = visualization_msgs::Marker::CUBE;  
      break;  
    }  
  
    r.sleep();  
  }  
}  