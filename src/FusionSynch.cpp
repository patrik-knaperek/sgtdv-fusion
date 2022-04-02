/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include "../include/FusionSynch.h"

FusionSynch::FusionSynch()
{
    m_cameraReady = false;
    m_lidarReady = false;
}

FusionSynch::~FusionSynch()
{
    
}

void FusionSynch::DoCamera(const sgtdv_msgs::ConeArr::ConstPtr &msgSensorFrame)
{
    if (m_cameraReady && !m_lidarReady) return;

    int conesCount = msgSensorFrame->cones.size();
    if (conesCount == 0) return;

    m_cameraReady = true;

    geometry_msgs::PointStamped coordsSensorFrame = geometry_msgs::PointStamped();
    coordsSensorFrame.header.frame_id = m_cameraFrameId;
    coordsSensorFrame.header.stamp = ros::Time::now();
    
    sgtdv_msgs::ConeArrPtr msgFixedFrame(new sgtdv_msgs::ConeArr);
    sgtdv_msgs::Cone cone;
    msgFixedFrame->cones.reserve(conesCount);

    for (int i = 0; i < conesCount; i++)
    {
        if (std::isnan(msgSensorFrame->cones[i].coords.x) || std::isnan(msgSensorFrame->cones[i].coords.y))
            continue;
        
        coordsSensorFrame.point.x = msgSensorFrame->cones[i].coords.x;
        coordsSensorFrame.point.y = msgSensorFrame->cones[i].coords.y;
        coordsSensorFrame.point.z = 0;

        geometry_msgs::PointStamped coordsFixedFrame = TransformCoords(coordsSensorFrame);
        
        cone.coords.x = coordsFixedFrame.point.x;
        cone.coords.y = coordsFixedFrame.point.y;
        cone.color = msgSensorFrame->cones[i].color;
        msgFixedFrame->cones.push_back(cone);

    }

    if (msgFixedFrame->cones.size() > 0)
    {
        if (m_cameraReady && m_lidarReady)
        {
            m_cameraReady = false;
            m_lidarReady = false;
            m_fusionMsg.cameraData = msgFixedFrame;
            std::cout << "fusion msg lidar size: " << m_fusionMsg.lidarData->points.size() << std::endl;
            std::cout << "fusion msg camera size: " << m_fusionMsg.cameraData->cones.size() << std::endl;
            m_fusion.Do(m_fusionMsg);
        }
        else
        {
            m_fusionMsg.cameraData = msgFixedFrame;
        }    
    }
    else
    {
        m_cameraReady = false;
        m_lidarReady = false;
    }
}

void FusionSynch::DoLidar(const sgtdv_msgs::Point2DArr::ConstPtr &msgSensorFrame)
{
    if (m_lidarReady && !m_cameraReady) return;

    int pointsCount = msgSensorFrame->points.size();
    if (pointsCount == 0) return;

    m_lidarReady = true;

    geometry_msgs::PointStamped coordsSensorFrame = geometry_msgs::PointStamped();
    coordsSensorFrame.header.frame_id = m_lidarFrameId;
    coordsSensorFrame.header.stamp = ros::Time::now();
    
    sgtdv_msgs::Point2DArrPtr msgFixedFrame(new sgtdv_msgs::Point2DArr);
    sgtdv_msgs::Point2D point;
    msgFixedFrame->points.reserve(pointsCount);

     for (int i = 0; i < pointsCount; i++)
    {
        coordsSensorFrame.point.x = msgSensorFrame->points[i].x;
        coordsSensorFrame.point.y = msgSensorFrame->points[i].y;
        coordsSensorFrame.point.z = 0;

        geometry_msgs::PointStamped coordsFixedFrame = TransformCoords(coordsSensorFrame);
        
        point.x = coordsFixedFrame.point.x;
        point.y = coordsFixedFrame.point.y;
        msgFixedFrame->points.push_back(point);
    }

    if (m_cameraReady && m_lidarReady)
    {
        m_cameraReady = false;
        m_lidarReady = false;
        m_fusionMsg.lidarData = msgFixedFrame;
        std::cout << "fusion msg lidar size: " << m_fusionMsg.lidarData->points.size() << std::endl;
        std::cout << "fusion msg camera size: " << m_fusionMsg.cameraData->cones.size() << std::endl;
        m_fusion.Do(m_fusionMsg);
    }
    else
    {
        m_fusionMsg.lidarData = msgFixedFrame;
    }    
}

geometry_msgs::PointStamped FusionSynch::TransformCoords(geometry_msgs::PointStamped coordsChildFrame)
{
    geometry_msgs::PointStamped coordsParentFrame = geometry_msgs::PointStamped();
    try
    {
        m_listener.transformPoint(m_fixedFrameId, coordsChildFrame, coordsParentFrame);
    }
    catch (tf::TransformException &e)
    {
        std::cout << e.what();
    }
    return coordsParentFrame;
}