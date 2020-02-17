#pragma once

#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/Point2DArr.h>

struct FusionMsg
{
    sgtdv_msgs::ConeArr::ConstPtr cameraData;
    sgtdv_msgs::Point2DArr::ConstPtr lidarData;
};