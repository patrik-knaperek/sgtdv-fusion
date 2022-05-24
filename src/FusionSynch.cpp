/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Patrik Knaperek
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

void FusionSynch::SetLidarFrameTF()
{
    geometry_msgs::PointStamped lidarZero;
    lidarZero.point.x = lidarZero.point.y = lidarZero.point.z = 0.0;
    lidarZero.header.frame_id = m_lidarFrameId;
    lidarZero.header.stamp = ros::Time::now();
    geometry_msgs::PointStamped lidarFrameTF = TransformCoords(lidarZero);
    m_fusionObj.SetLidarFrameTF(lidarFrameTF.point.x);
    m_lidarFrameId = std::string("done");
}

void FusionSynch::DoCamera(const sgtdv_msgs::ConeArr::ConstPtr &msg)
{
    if (m_cameraReady && !m_lidarReady) return;

    int conesCount = msg->cones.size();
    if (conesCount == 0) return;

    m_cameraReady = true;

    geometry_msgs::PointStamped coordsMsgFrame = geometry_msgs::PointStamped();
    geometry_msgs::PointStamped coordsBaseFrame = geometry_msgs::PointStamped();
    sgtdv_msgs::ConeArrPtr msgBaseFrame(new sgtdv_msgs::ConeArr);
    sgtdv_msgs::Cone cone;
    msgBaseFrame->cones.reserve(conesCount);

    for (size_t i = 0; i < conesCount; i++)
    {
        if (std::isnan(msg->cones[i].coords.x) || std::isnan(msg->cones[i].coords.y))
            continue;
        
        coordsMsgFrame.header = msg->cones[i].coords.header;
        coordsMsgFrame.point.x = msg->cones[i].coords.x;
        coordsMsgFrame.point.y = msg->cones[i].coords.y;
        coordsMsgFrame.point.z = 0;

        if (coordsMsgFrame.header.frame_id.compare(m_baseFrameId) != 0)
            coordsBaseFrame = TransformCoords(coordsMsgFrame);
        else
            coordsBaseFrame = coordsMsgFrame;
        
        cone.coords.header = coordsBaseFrame.header;
        cone.coords.x = coordsBaseFrame.point.x;
        cone.coords.y = coordsBaseFrame.point.y;
        cone.color = msg->cones[i].color;
        msgBaseFrame->cones.push_back(cone);

    }

    if (msgBaseFrame->cones.size() > 0)
    {
        if (m_cameraReady && m_lidarReady)
        {
            m_cameraReady = false;
            m_lidarReady = false;
            m_fusionMsg.cameraData = msgBaseFrame;
		#ifdef FUSION_CONSOLE_SHOW
            std::cout << "fusion msg lidar size: " << m_fusionMsg.lidarData->points.size() << std::endl;
            std::cout << "fusion msg camera size: " << m_fusionMsg.cameraData->cones.size() << std::endl;
		#endif
            m_fusionObj.Do(m_fusionMsg);
        }
        else
        {
            m_fusionMsg.cameraData = msgBaseFrame;
        }    
    }
    else
    {
        m_cameraReady = false;
        m_lidarReady = false;
    }
}

void FusionSynch::DoLidar(const sgtdv_msgs::Point2DArr::ConstPtr &msg)
{
    // set TF from lidar frame to base frame once
    if (m_lidarFrameId.compare(std::string("done")) != 0)
        this->SetLidarFrameTF();
    if (m_lidarReady && !m_cameraReady) return;

    int pointsCount = msg->points.size();
    if (pointsCount == 0) return;

    m_lidarReady = true;

    geometry_msgs::PointStamped coordsMsgFrame = geometry_msgs::PointStamped();
    geometry_msgs::PointStamped coordsBaseFrame = geometry_msgs::PointStamped();
    sgtdv_msgs::Point2DArrPtr msgBaseFrame(new sgtdv_msgs::Point2DArr);
    sgtdv_msgs::Point2D point;
    msgBaseFrame->points.reserve(pointsCount);

     for (size_t i = 0; i < pointsCount; i++)
    {
        coordsMsgFrame.header = msg->points[i].header;
        coordsMsgFrame.point.x = msg->points[i].x;
        coordsMsgFrame.point.y = msg->points[i].y;
        coordsMsgFrame.point.z = 0;

        if (coordsMsgFrame.header.frame_id.compare(m_baseFrameId) != 0)
            coordsBaseFrame = TransformCoords(coordsMsgFrame);
        else
            coordsBaseFrame = coordsMsgFrame;
        
        point.header = coordsBaseFrame.header;
        point.x = coordsBaseFrame.point.x;
        point.y = coordsBaseFrame.point.y;
        msgBaseFrame->points.push_back(point);
    }

    if (m_cameraReady && m_lidarReady)
    {
        m_cameraReady = false;
        m_lidarReady = false;
        m_fusionMsg.lidarData = msgBaseFrame;
	#ifdef FUSION_CONSOLE_SHOW
        std::cout << "fusion msg lidar size: " << m_fusionMsg.lidarData->points.size() << std::endl;
        std::cout << "fusion msg camera size: " << m_fusionMsg.cameraData->cones.size() << std::endl;
	#endif
        m_fusionObj.Do(m_fusionMsg);
    }
    else
    {
        m_fusionMsg.lidarData = msgBaseFrame;
    }    
}

geometry_msgs::PointStamped FusionSynch::TransformCoords(geometry_msgs::PointStamped coordsChildFrame)
{
    geometry_msgs::PointStamped coordsParentFrame = geometry_msgs::PointStamped();
    try
    {
        m_listener.transformPoint(m_baseFrameId, coordsChildFrame, coordsParentFrame);
    }
    catch (tf::TransformException &e)
    {
        std::cout << e.what();
    }
    return coordsParentFrame;
}
