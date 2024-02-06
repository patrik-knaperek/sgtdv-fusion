/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/

#pragma once

#include <sgtdv_msgs/ConeStampedArr.h>
#include <sgtdv_msgs/Point2DStampedArr.h>
#include "../../SGT_Macros.h"

struct FusionMsg
{
	sgtdv_msgs::ConeStampedArr::ConstPtr camera_data;
	sgtdv_msgs::Point2DStampedArr::ConstPtr lidar_data;
};

struct Params
{
	std::string base_frame_id;
	std::string camera_frame_id;
	std::string lidar_frame_id;
	float dist_th;
	int n_of_models;
	Eigen::Matrix<double, Eigen::Dynamic, 4> camera_model;
	Eigen::Matrix<double, Eigen::Dynamic, 4> lidar_model;
	float camera_x_min;
	float camera_x_max;
	float camera_bearing_min;
	float camera_bearing_max;
	float lidar_x_min;
	float lidar_x_max;
	float lidar_y_min;
	float lidar_y_max;
#ifdef SGT_EXPORT_DATA_CSV
	std::string data_filename;
	std::string map_frame_id;
#endif
};