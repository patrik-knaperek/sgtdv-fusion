/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/

#pragma once

#include <sgtdv_msgs/ConeStampedArr.h>
#include <sgtdv_msgs/Point2DStampedArr.h>

struct FusionMsg
{
    sgtdv_msgs::ConeStampedArr::ConstPtr cameraData;
    sgtdv_msgs::Point2DStampedArr::ConstPtr lidarData;
};