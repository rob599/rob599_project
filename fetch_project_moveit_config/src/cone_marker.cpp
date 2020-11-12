
#include <ros/ros.h>

// For visualizing things in rviz
#include <rviz_visual_tools/rviz_visual_tools.h>

// C++
#include <string>
#include <vector>

namespace rvt = rviz_visual_tools;

int main ( int argc, char** argv)
{
  ros::init(argc, argv, "cone_marker");
  ros::NodeHandle n;
  ros::Rate r(1);
  rvt::RvizVisualToolsPtr visual_tools_;
  std::string name_;

  visual_tools_.reset(new rvt::RvizVisualTools("gripper_link", "/cone_marker"));
  visual_tools_->loadMarkerPub();  // create publisher before waiting
  ROS_INFO("Sleeping 5 seconds before running demo");
  ros::Duration(5.0).sleep();
  // Clear messages
  visual_tools_->deleteAllMarkers();
  visual_tools_->enableBatchPublishing();

  // Create pose
  Eigen::Isometry3d pose_cone = Eigen::Isometry3d::Identity();
  pose_cone = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()); // rotate along X axis by 45 degrees
  pose_cone.translation() = Eigen::Vector3d( 0.0, 0.0, 0.0 ); // translate x,y,z

  //Publish Cone marker of pose
  ROS_INFO_STREAM_NAMED("test", "Publishing Cone");
  // while(ros::ok())
  while (ros::ok())
  {
  ros::Duration(.30).sleep();
  visual_tools_->deleteAllMarkers();
  visual_tools_->enableBatchPublishing();
  visual_tools_->publishCone(pose_cone,M_PI/2, rvt::CYAN, 0.3);
  visual_tools_->trigger();
  visual_tools_->setAlpha(.3);
  }
}
