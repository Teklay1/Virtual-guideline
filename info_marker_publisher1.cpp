#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <string>
#include <math.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "info_marker_publisher1");
  ros::NodeHandle nh;

  // publisher
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 1);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.scale.x = 2;
    marker.scale.y = 2;
    marker.scale.z = 2;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    
     // Create the vertices for the points and lines
     for (uint32_t i = 0; i < 3; ++i)
     
     {
       //float y = 0 ;
       //float z = 0 ;
     
       //float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
       //float z = 5 * cos(f + i / 100.0f * 2 * M_PI);
 
       geometry_msgs::Point p;
       p.x = 0; //- 50;
       p.y = 2*(int32_t)i;
       p.z = 0;
 
       //points.points.push_back(p);
       //line_strip.points.push_back(p);
       //p.z += 1.0;
 /*marker.points.push_back(p);
       // The line list needs two points for each line
       //line_list.points.push_back(p);
       p.x += 2.0;
       p.y += 1;
       //p.z += 0;
       //line_list.points.push_back(p);
       marker.points.push_back(p);*/
     }
 
    
    marker_pub.publish(marker);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
