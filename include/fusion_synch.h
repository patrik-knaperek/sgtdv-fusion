/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

#pragma once

#include <tf/transform_listener.h>
#include "fusion.h"

class FusionSynch
{
public:
  FusionSynch(ros::NodeHandle& handle);
  ~FusionSynch() = default;

#ifdef SGT_EXPORT_DATA_CSV
  void mapCallback(const visualization_msgs::MarkerArray::ConstPtr &msg) { fusion_obj_.writeMapToFile(msg); };
#endif

  void cameraCallback(const sgtdv_msgs::ConeStampedArr::ConstPtr &msg);
  void lidarCallback(const sgtdv_msgs::Point2DStampedArr::ConstPtr &msg);
  void poseCallback(const sgtdv_msgs::CarPose::ConstPtr &msg);
  geometry_msgs::PointStamped transformCoords(const geometry_msgs::PointStamped& coords_child_frame) const;

private:
  Fusion fusion_obj_;
  ros::Subscriber lidar_sub_;
  ros::Subscriber camera_sub_;
  ros::Subscriber pose_sub_;
#ifdef SGT_EXPORT_DATA_CSV
  ros::Subscriber map_marker_sub_;
#endif /* SGT_EXPORT_DATA_CSV */

  bool camera_ready_ = false; 
  bool lidar_ready_ = false;
  FusionMsg fusion_msg_;

  std::string base_frame_id_;
  tf::TransformListener listener_;
};
