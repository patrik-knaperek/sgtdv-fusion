/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

#pragma once

/* C++ */
#include <fstream>
#include <Eigen/Eigen>

/* ROS */
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

/* SGT */
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/ConeWithCovStampedArr.h>
#include "messages.h"
#include "SGT_Macros.h"
#include "SGT_Utils.h"
#include "fusion_kf.h"

class Fusion
{
public:
  struct Params
  {
    std::string base_frame_id;
    std::string camera_frame_id;
    std::string lidar_frame_id;
    double camera_static_tf_x;
    double lidar_static_tf_x;
    float dist_th;
    int n_of_models;
    Eigen::Matrix<double, Eigen::Dynamic, 4> camera_model;
    Eigen::Matrix<double, Eigen::Dynamic, 4> lidar_model;
    Utils::Range<float> camera_x;
    Utils::Range<float> camera_bearing;
    Utils::Range<float> lidar_x;
    Utils::Range<float> lidar_y;
    int vitality_score_init;
    int vitality_score_max;
    int validation_score_th;
  #ifdef SGT_EXPORT_DATA_CSV
    std::string data_filename;
    std::string map_frame_id;
  #endif
  };

public:
  Fusion(); 
  ~Fusion();
  
#ifdef SGT_EXPORT_DATA_CSV
  void openDataFiles(void);
  void writeMapToFile(const visualization_msgs::MarkerArray::ConstPtr &msg);
#endif

  void setParams(const Params& params)
  {
    params_ = params;
  };

  sgtdv_msgs::ConeWithCovStampedArr update(const FusionMsg &fusion_msg);

  void updatePose(const sgtdv_msgs::CarPose::ConstPtr &msg)
  {
    KF_obj_.updatePose(msg->position.x, msg->position.y, msg->yaw);
  };

private:
  struct TrackedCone
  {
    TrackedCone(const Eigen::Ref<const Eigen::Vector2d>& coords, const int vitality_init) :
      state(Eigen::Vector2d(coords(0), coords(1))),
      covariance(Eigen::Matrix2d::Identity()),
      vitality_score(vitality_init),
      validation_score(1)
    {
      covariance *= 1e10;
    };

    Eigen::Vector2d state;  // 2D cone coordinates
    Eigen::Matrix2d covariance;
    uint8_t color;			    // 'y' - yellow; 'b' - blue; 's' - orange small; 'g' - orange big
    ros::Time stamp;
    int vitality_score;		  // Value is incremented after each measurement association, decremented in each update cycle. Cones with score <= 0 are excluded from track list.
    int validation_score;	  // Value is incremented after each measurement association. Only cones with score > threshold are published.
  };

private:
  /*float MahalanDist(const Eigen::Ref<const Eigen::Vector2d> &set_mean, const Eigen::Ref<const Eigen::Matrix2d> &set_cov,
          const Eigen::Ref<const Eigen::Vector2d> &obs_mean, const Eigen::Ref<const Eigen::Matrix2d> &obs_cov);
          */
  bool findClosestTracked(const Eigen::Ref<const Eigen::Vector2d> &measurement, 
                          std::list<TrackedCone>::iterator *closest_it);
  
  FusionKF KF_obj_;
  Params params_;

  tf::TransformListener listener_;
  
  std::list<TrackedCone> tracked_cones_;
  int num_of_tracked_ = 0;
  
#ifdef SGT_EXPORT_DATA_CSV
  bool openFile(std::ofstream& file, const std::string& path);
  void writeToDataFile(int idx);
  Eigen::Vector2d transformCoords(const Eigen::Ref<const Eigen::Vector2d> &obs_base_frame, ros::Time stamp) const;
  Eigen::Vector2d transformCoords(const sgtdv_msgs::Point2DStamped &obs) const;

  std::vector<std::list<Eigen::Vector2d>> camera_data_, lidar_data_, fusion_data_;
  std::ofstream camera_data_file_, lidar_data_file_, fusion_data_file_, map_data_file_;
#endif /* SGT_EXPORT_DATA_CSV */
};
