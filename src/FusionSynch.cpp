/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/


#include "../include/FusionSynch.h"

FusionSynch::FusionSynch(const ros::NodeHandle& handle, const ros::Publisher& publisher)
: fusion_obj_(handle, publisher)
{
	Utils::loadParam(handle, "/base_frame_id", &base_frame_id_);
}

void FusionSynch::cameraCallback(const sgtdv_msgs::ConeStampedArr::ConstPtr &msg)
{
	if (camera_ready_ && !lidar_ready_) return;

	const int cones_count = msg->cones.size();
	if (!cones_count) return;

	camera_ready_ = true;

	geometry_msgs::PointStamped coords_msg_frame, coords_base_frame;
	auto msg_base_frame = boost::make_shared<sgtdv_msgs::ConeStampedArr>();
	sgtdv_msgs::ConeStamped cone;
	msg_base_frame->cones.reserve(cones_count);

	for (const auto &cone_it : msg->cones)
	{
		if (std::isnan(cone_it.coords.x) || std::isnan(cone_it.coords.y))
			continue;
		
		coords_msg_frame.header = cone_it.coords.header;
		coords_msg_frame.point.x = cone_it.coords.x;
		coords_msg_frame.point.y = cone_it.coords.y;
		coords_msg_frame.point.z = 0;

		if (coords_msg_frame.header.frame_id != base_frame_id_)
			coords_base_frame = transformCoords(coords_msg_frame);
		else
			coords_base_frame = coords_msg_frame;
		
		cone.coords.header = coords_base_frame.header;
		cone.coords.x = coords_base_frame.point.x;
		cone.coords.y = coords_base_frame.point.y;
		cone.color = cone_it.color;
		msg_base_frame->cones.push_back(cone);
	}

	if (msg_base_frame->cones.size() > 0)
	{
		if (camera_ready_ && lidar_ready_)
		{
			camera_ready_ = false;
			lidar_ready_ = false;
			fusion_msg_.camera_data = msg_base_frame;
			ROS_DEBUG_STREAM("fusion msg lidar size: " << fusion_msg_.lidar_data->points.size());
			ROS_DEBUG_STREAM("fusion msg camera size: " << fusion_msg_.camera_data->cones.size());
			fusion_obj_.update(fusion_msg_);
		}
		else
		{
			fusion_msg_.camera_data = msg_base_frame;
		}	
	}
	else
	{
		camera_ready_ = false;
		lidar_ready_ = false;
	}
}

void FusionSynch::lidarCallback(const sgtdv_msgs::Point2DStampedArr::ConstPtr &msg)
{
	if (lidar_ready_ && !camera_ready_) return;

	const int points_count = msg->points.size();
	if (!points_count) return;

	lidar_ready_ = true;

	geometry_msgs::PointStamped coords_msg_frame, coords_base_frame;
	auto msg_base_frame = boost::make_shared<sgtdv_msgs::Point2DStampedArr>();
	sgtdv_msgs::Point2DStamped point;
	msg_base_frame->points.reserve(points_count);

	for (const auto &point_it : msg->points)
	{
		coords_msg_frame.header = point_it.header;
		coords_msg_frame.point.x = point_it.x;
		coords_msg_frame.point.y = point_it.y;
		coords_msg_frame.point.z = 0;

		if (coords_msg_frame.header.frame_id != base_frame_id_)
			coords_base_frame = transformCoords(coords_msg_frame);
		else
			coords_base_frame = coords_msg_frame;
		
		point.header = coords_base_frame.header;
		point.x = coords_base_frame.point.x;
		point.y = coords_base_frame.point.y;
		msg_base_frame->points.push_back(point);
	}

	if (camera_ready_ && lidar_ready_)
	{
		camera_ready_ = false;
		lidar_ready_ = false;
		fusion_msg_.lidar_data = msg_base_frame;
		ROS_DEBUG_STREAM("fusion msg lidar size: " << fusion_msg_.lidar_data->points.size());
		ROS_DEBUG_STREAM("fusion msg camera size: " << fusion_msg_.camera_data->cones.size());
		fusion_obj_.update(fusion_msg_);
	}
	else
	{
		fusion_msg_.lidar_data = msg_base_frame;
	}	
}

void FusionSynch::poseCallback(const sgtdv_msgs::CarPose::ConstPtr &msg)
{
	fusion_obj_.updatePose(msg);
}

geometry_msgs::PointStamped FusionSynch::transformCoords(const geometry_msgs::PointStamped& coords_child_frame) const
{
	geometry_msgs::PointStamped coords_parent_frame;
	try
	{
		listener_.transformPoint(base_frame_id_, coords_child_frame, coords_parent_frame);
	}
	catch (tf::TransformException &e)
	{
		std::cout << e.what();
	}
	return coords_parent_frame;
}
