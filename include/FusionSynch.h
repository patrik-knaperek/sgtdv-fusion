/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

#pragma once

#include <tf/transform_listener.h>
#include "../include/Fusion.h"

class FusionSynch
{
	public:
		FusionSynch(const ros::NodeHandle& handle, const ros::Publisher& publisher);
		~FusionSynch() = default;

	#ifdef SGT_EXPORT_DATA_CSV
		void mapCallback(const visualization_msgs::MarkerArray::ConstPtr &msg) { fusion_obj_.writeMapToFile(msg); };
	#endif
	#ifdef SGT_DEBUG_STATE
		void setVisDebugPublisher(ros::Publisher publisher) { fusion_obj_.setVisDebugPublisher(publisher); }
	#endif

		void cameraCallback(const sgtdv_msgs::ConeStampedArr::ConstPtr &msg);
		void lidarCallback(const sgtdv_msgs::Point2DStampedArr::ConstPtr &msg);
		void poseCallback(const sgtdv_msgs::CarPose::ConstPtr &msg);
		geometry_msgs::PointStamped transformCoords(const geometry_msgs::PointStamped& coords_child_frame) const;

	private:
		Fusion fusion_obj_;
		bool camera_ready_ = false; 
		bool lidar_ready_ = false;
		FusionMsg fusion_msg_;

		std::string base_frame_id_;
		tf::TransformListener listener_;
};
