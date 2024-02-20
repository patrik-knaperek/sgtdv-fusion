/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský
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