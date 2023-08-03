#include "../include/FusionKF.h"

FusionKF::FusionKF()
{
	H_ = Eigen::Matrix2d::Identity();
	
	Q_ << 1e-06, 0.,
		   0., 1e-06;
	
	A_ = Eigen::Matrix2d::Identity();
}

/**
 * @brief Predict step of Linear Kalman Filter
 * 
 * @param trackedCone x state vector
 */
void FusionKF::predict(Eigen::Ref<Eigen::Vector2d> tracked_cone_state, Eigen::Ref<Eigen::Matrix2d> tracked_cone_cov)
{	
	tracked_cone_state = rot2_.matrix().transpose() * rot1_.matrix() * tracked_cone_state 
										- rot2_.matrix().transpose() * (pos2_ - pos1_);
	tracked_cone_cov = A_ * tracked_cone_cov * A_.transpose() + Q_;
}

/**
 * @brief Update step of Linear Kalman Filter
 * 
 * @param trackedConeIt x state vector
 * @param measurement z measurement vector
 * @param measurementModel R measurement covariance (uncertainty)
 */
void FusionKF::update(Eigen::Ref<Eigen::Vector2d> tracked_cone_state, Eigen::Ref<Eigen::Matrix2d> tracked_cone_cov,
					const Eigen::Ref<const Eigen::Vector2d> &measurement, 
					const Eigen::Ref<const Eigen::Matrix2d> &measurement_model) const
{
	static const auto I = Eigen::Matrix2d::Identity();
	
	ROS_DEBUG_STREAM("state before:\n" << tracked_cone_state);
	ROS_DEBUG_STREAM("covariance before:\n" << tracked_cone_cov);

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

void FusionKF::updatePose(const double x, const double y, const double theta)
{
	act_pose_.x = x;
	act_pose_.y = y;
	act_pose_.theta = theta;
}

void FusionKF::updateTimeAndPoseDelta()
{
	pos1_ = pos2_;
	rot1_ = rot2_;
	pos2_ << act_pose_.x, act_pose_.y;
	rot2_ = Eigen::Rotation2Dd(act_pose_.theta);

	ROS_DEBUG_STREAM("\npos1:\n" << pos1_ << "\npos2:\n" << pos2_);
	ROS_DEBUG_STREAM("\nrot1:\n" << rot1_.matrix() << "\nrot2:\n" << rot2_.matrix());
}