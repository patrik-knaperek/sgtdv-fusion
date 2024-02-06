/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

#pragma once

// C++
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <array>
#include <Eigen/Eigen>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

// SGT
#include <sgtdv_msgs/ConeStampedArr.h>
#include <sgtdv_msgs/Point2DStampedArr.h>
#include <sgtdv_msgs/DebugState.h>
#include <sgtdv_msgs/CarPose.h>
#include "../include/Messages.h"
#include "../../SGT_Macros.h"
#include "../../SGT_Utils.h"
#include "../include/FusionKF.h"


#define VITALITY_SCORE_INIT 2
#define VITALITY_SCORE_MAX 6
#define VALIDATION_SCORE_TH 4   // validation score treshold

class Fusion
{
	public:
		Fusion(const ros::NodeHandle& handle, const ros::Publisher& publisher); 
		~Fusion();

		void loadParams(const ros::NodeHandle &handle);
		
	#ifdef SGT_EXPORT_DATA_CSV
		void openDataFiles(void);
		void writeMapToFile(const visualization_msgs::MarkerArray::ConstPtr &msg);
	#endif
	#ifdef SGT_DEBUG_STATE
		void setVisDebugPublisher(ros::Publisher publisher) { vis_debug_publisher_ = publisher; }
	#endif

		void update(const FusionMsg &fusionMsg);

		void updatePose(const sgtdv_msgs::CarPose::ConstPtr &msg)
		{
			KF_obj_.updatePose(msg->position.x, msg->position.y, msg->yaw);
		};
   
	private:
		struct TrackedCone
		{
			TrackedCone(const Eigen::Ref<const Eigen::Vector2d>& coords)
			: state(Eigen::Vector2d(coords(0), coords(1)))
			, covariance(Eigen::Matrix2d::Identity())
			, vitality_score(VITALITY_SCORE_INIT)
			, validation_score(1)
			{
				covariance *= 1e10;
			};
			Eigen::Vector2d state;	  // 2D cone coordinates
			Eigen::Matrix2d covariance;
			uint8_t color;			  // 'y' - yellow; 'b' - blue; 's' - orange small; 'g' - orange big
			ros::Time stamp;
			int vitality_score;		 // Value is incremented after each measurement association, decremented in each update cycle. Cones with score <= 0 are excluded from track list.
			int validation_score;	   // Value is incremented after each measurement association. Only cones with score > treshold are published.
		};

	private:
		void getSensorFrameTF(void);
		/*float MahalanDist(const Eigen::Ref<const Eigen::Vector2d> &setMean, const Eigen::Ref<const Eigen::Matrix2d> &setCov,
						const Eigen::Ref<const Eigen::Vector2d> &obsMean, const Eigen::Ref<const Eigen::Matrix2d> &obsCov);
						*/
		bool findClosestTracked(const Eigen::Ref<const Eigen::Vector2d> &measurement, 
														std::list<TrackedCone>::iterator *closest_it);
		
		FusionKF KF_obj_;
		Params params_;
		
		ros::Publisher publisher_;

		std::list<TrackedCone> tracked_cones_;
		int num_of_tracked_ = 0;

		double camera_frame_tf_x_, lidar_frame_tf_x_;
		tf::TransformListener listener_;
		
	#ifdef SGT_EXPORT_DATA_CSV
		bool openFile(std::ofstream& file, const std::string& path);
		void writeToDataFile(int Idx);
		Eigen::Vector2d transformCoords(const Eigen::Ref<const Eigen::Vector2d> &obs_base_frame, ros::Time stamp) const;
		Eigen::Vector2d transformCoords(const sgtdv_msgs::Point2DStamped &obs) const;

		std::vector<std::list<Eigen::Vector2d>> camera_data_, lidar_data_, fusion_data_;
		std::ofstream camera_data_file_, lidar_data_file_, fusion_data_file_, map_data_file_;
	#endif /* SGT_EXPORT_DATA_CSV */

	#ifdef SGT_DEBUG_STATE
		ros::Publisher vis_debug_publisher_;
	#endif		
};
