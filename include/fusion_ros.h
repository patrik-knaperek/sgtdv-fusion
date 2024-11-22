/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

#pragma once

#include <tf/transform_listener.h>
#include "fusion.h"

class FusionROS
{
public:
  explicit FusionROS(ros::NodeHandle& handle);
  ~FusionROS() = default;

private:
  #ifdef SGT_EXPORT_DATA_CSV
  void mapCallback(const visualization_msgs::MarkerArray::ConstPtr &msg) { fusion_obj_.writeMapToFile(msg); };
#endif

  void cameraCallback(const sgtdv_msgs::ConeStampedArr::ConstPtr &msg);
  void lidarCallback(const sgtdv_msgs::Point2DStampedArr::ConstPtr &msg);
  void poseCallback(const sgtdv_msgs::CarPose::ConstPtr &msg);

  void loadParams(const ros::NodeHandle &handle);
  void getSensorFrameTF(Fusion::Params* params) const;
  
  
  Fusion fusion_obj_;
  
  ros::Publisher cones_pub_;
#ifdef SGT_DEBUG_STATE
  ros::Publisher vis_debug_pub_;
#endif		

  ros::Subscriber lidar_sub_;
  ros::Subscriber camera_sub_;
  ros::Subscriber pose_sub_;
#ifdef SGT_EXPORT_DATA_CSV
  ros::Subscriber map_marker_sub_;
#endif /* SGT_EXPORT_DATA_CSV */

  std::string base_frame_id_;
  tf::TransformListener listener_;
};
