/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#pragma once

#include <geometry_msgs/Pose2D.h>
#include <Eigen/Eigen>

class FusionKF
{
public:
  FusionKF();
  ~FusionKF() = default;

  void predict(Eigen::Ref<Eigen::Vector2d> tracked_cone_state, Eigen::Ref<Eigen::Matrix2d> tracked_cone_cov);
  void update(Eigen::Ref<Eigen::Vector2d> tracked_cone_state, Eigen::Ref<Eigen::Matrix2d> tracked_cone_cov,
              const Eigen::Ref<const Eigen::Vector2d> &measurement, 
              const Eigen::Ref<const Eigen::Matrix2d> &measurement_model) const;

  void updatePose(const double x, const double y, const double theta);
  void updateTimeAndPoseDelta();

private:
  Eigen::Matrix2d A_; // motion model matrix
  Eigen::Matrix2d H_; // observation model matrix
  Eigen::Matrix2d Q_; // noise covariance matrix

  geometry_msgs::Pose2D act_pose_;
  Eigen::Vector2d pos_prev_, pos_act_;
  Eigen::Rotation2Dd rot_prev_, rot_act_;
};

