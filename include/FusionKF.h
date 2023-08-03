/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include <ros/ros.h>
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
		Eigen::Matrix2d A_;
		Eigen::Matrix2d H_;
		Eigen::Matrix2d Q_;

		geometry_msgs::Pose2D act_pose_;
		Eigen::Vector2d pos1_, pos2_;
		Eigen::Rotation2Dd rot1_, rot2_;
};

