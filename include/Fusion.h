/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/

#pragma once

#include <ros/ros.h>
#include <cmath>
#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/Point2DArr.h>
#include "../include/Messages.h"
#include "../../SGT_Macros.h"
#include <sgtdv_msgs/DebugState.h>


class Fusion
{
public:
    Fusion();
    ~Fusion();

    void SetPublisher(ros::Publisher publisher);
    void Do(const FusionMsg &fusionMsg);

#ifdef DEBUG_STATE
    void SetVisDebugPublisher(ros::Publisher publisher) { m_visDebugPublisher = publisher; }
#endif

private:
    ros::Publisher m_publisher;
    float m_tol;

#ifdef DEBUG_STATE
    ros::Publisher m_visDebugPublisher;
#endif

    bool AreInSamePlace(const sgtdv_msgs::Point2D &p1, const sgtdv_msgs::Point2D &p2) const;
};