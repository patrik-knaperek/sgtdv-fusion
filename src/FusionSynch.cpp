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

void FusionSynch::SetPublisher(ros::Publisher publisher)
{
    m_fusion.SetPublisher(publisher);
}

void FusionSynch::DoCamera(const sgtdv_msgs::ConeArr::ConstPtr &msg)
{
    if (m_cameraReady && !m_lidarReady) return;

    m_cameraReady = true;

    if (m_cameraReady && m_lidarReady)
    {
        m_cameraReady = false;
        m_lidarReady = false;
        m_fusionMsg.cameraData = msg;
        m_fusion.Do(m_fusionMsg);
    }
    else
    {
        m_fusionMsg.cameraData = msg;
    }    
}

void FusionSynch::DoLidar(const sgtdv_msgs::Point2DArr::ConstPtr &msg)
{
    if (m_lidarReady && !m_cameraReady) return;

    m_lidarReady = true;

    if (m_cameraReady && m_lidarReady)
    {
        m_cameraReady = false;
        m_lidarReady = false;
        m_fusionMsg.lidarData = msg;
        m_fusion.Do(m_fusionMsg);
    }
    else
    {                                   //TODO
        m_fusionMsg.lidarData = msg;
    }    
}