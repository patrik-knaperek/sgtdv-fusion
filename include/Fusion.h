#pragma once

#include <ros/ros.h>
#include <cmath>
#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/Point2DArr.h>
#include "../include/Messages.h"


class Fusion
{
public:
    Fusion();
    ~Fusion();

    void SetPublisher(ros::Publisher publisher);
    void Do(const FusionMsg &fusionMsg);

private:
    ros::Publisher m_publisher;
    float m_tol;

    bool AreInSamePlace(const sgtdv_msgs::Point2D &p1, const sgtdv_msgs::Point2D &p2) const;
};