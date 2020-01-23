#include "../include/FusionSynch.h"

FusionSynch::FusionSynch()
{
    m_cameraReady = false;
    m_lidarReady = false;
}

FusionSynch::~FusionSynch()
{

}

void FusionSynch::SetPublisher(ros::Publisher publisher)
{
    m_fusion.SetPublisher(publisher);
}

void FusionSynch::DoCamera(const sgtdv_msgs::ConeArr::ConstPtr &msg)
{
    m_cameraReady = true;

    if (m_cameraReady && m_lidarReady)
    {
        sgtdv_msgs::FusionMsgPtr fusionMsg( new sgtdv_msgs::FusionMsg );

        m_cameraReady = false;
        m_lidarReady = false;

        fusionMsg->cameraCones = *msg;
        fusionMsg->lidarCones = *m_lidarData;

        m_fusion.Do(fusionMsg);
    }
    else
    {
        m_cameraData = msg;                 // i am the FAST
    }    
}

void FusionSynch::DoLidar(const sgtdv_msgs::Point2DArr::ConstPtr &msg)
{
    m_lidarReady = true;

    if (m_cameraReady && m_lidarReady)
    {
        sgtdv_msgs::FusionMsgPtr fusionMsg( new sgtdv_msgs::FusionMsg );

        m_cameraReady = false;
        m_lidarReady = false;

        fusionMsg->cameraCones = *m_cameraData;
        fusionMsg->lidarCones = *msg;

        m_fusion.Do(fusionMsg);
    }
    else
    {
        m_lidarData = msg;
    }    
}