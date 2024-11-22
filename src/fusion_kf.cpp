/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

/* ROS */
#include <ros/ros.h>

/* Header */
#include "fusion_kf.h"

FusionKF::FusionKF() :
  H_(Eigen::Matrix2d::Identity()),
  A_(Eigen::Matrix2d::Identity())
{
  Q_ << 1e-06, 0.,
        0., 1e-06;
}

/**
 * @brief Predict step of Linear Kalman Filter
 * 
 * @param tracked_cone_state x state vector
 * @param tracked_cone_cov P covariance matrix
 */
void FusionKF::predict(Eigen::Ref<Eigen::Vector2d> tracked_cone_state, Eigen::Ref<Eigen::Matrix2d> tracked_cone_cov)
{	
	tracked_cone_state = rot_act_.matrix().transpose() * rot_prev_.matrix() * tracked_cone_state 
										  - rot_act_.matrix().transpose() * (pos_act_ - pos_prev_);
	tracked_cone_cov = A_ * tracked_cone_cov * A_.transpose() + Q_;
}

/**
 * @brief Update step of Linear Kalman Filter
 * 
 * @param tracked_cone_state x state vector
 * @param tracked_cone_cov P covariance matrix
 * @param measurement z measurement vector
 * @param measurement_model R measurement covariance (uncertainty)
 */
void FusionKF::update(Eigen::Ref<Eigen::Vector2d> tracked_cone_state, Eigen::Ref<Eigen::Matrix2d> tracked_cone_cov,
                      const Eigen::Ref<const Eigen::Vector2d> &measurement, 
                      const Eigen::Ref<const Eigen::Matrix2d> &measurement_model) const
{
  static const auto I = Eigen::Matrix2d::Identity();

  ROS_DEBUG_STREAM("state before:\n" << tracked_cone_state);
  ROS_DEBUG_STREAM("covariance before:\n" << tracked_cone_cov);

  /* compute Kalman gain */
  Eigen::Matrix2d S = H_ * tracked_cone_cov * H_.transpose() + measurement_model;
  Eigen::Matrix2d K = tracked_cone_cov * H_.transpose() * S.inverse();

  ROS_DEBUG_STREAM("S\n" << S);
  ROS_DEBUG_STREAM("S^-1\n" << S.inverse());
  ROS_DEBUG_STREAM("K\n" << K);

  tracked_cone_state += K * (measurement - H_ * tracked_cone_state);
  tracked_cone_cov = (I - K * H_) * tracked_cone_cov;

  ROS_DEBUG_STREAM("state after:\n" << tracked_cone_state);
  ROS_DEBUG_STREAM("covariance after:\n" << tracked_cone_cov);
}

/// @brief Store latest pose data from topic message
/// @param x position coordinate
/// @param y position coordinate
/// @param theta (z-axis) rotation
void FusionKF::updatePose(const double x, const double y, const double theta)
{
  act_pose_.x = x;
  act_pose_.y = y;
  act_pose_.theta = theta;
}

/// @brief Update variables for differential prediction motion model
void FusionKF::updateTimeAndPoseDelta()
{
  pos_prev_ = pos_act_;
  rot_prev_ = rot_act_;
  pos_act_ << act_pose_.x, act_pose_.y;
  rot_act_ = Eigen::Rotation2Dd(act_pose_.theta);

  ROS_DEBUG_STREAM("\npos1:\n" << pos_prev_ << "\npos2:\n" << pos_act_);
  ROS_DEBUG_STREAM("\nrot1:\n" << rot_prev_.matrix() << "\nrot2:\n" << rot_act_.matrix());
}