/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include "../include/Fusion.h"

Fusion::Fusion()
{
    m_tol = 10.;
}

Fusion::~Fusion()
{

}

void Fusion::SetPublisher(ros::Publisher publisher)
{
    m_publisher = publisher;
}

void Fusion::Do(const FusionMsg &fusionMsg)
{   
    sgtdv_msgs::ConeArrPtr cones( new sgtdv_msgs::ConeArr );
    sgtdv_msgs::Cone cone;

    cones->cones.reserve(20);
    //TODO: Static transformations
    
    for(size_t i = 0; i < fusionMsg.lidarData->points.size(); i++)
    {
        for(size_t j = 0; j < fusionMsg.cameraData->cones.size(); j++)
        {
            if (AreInSamePlace(fusionMsg.lidarData->points[i], fusionMsg.cameraData->cones[j].coords))
            {
                cone.coords = fusionMsg.lidarData->points[i];
                cone.color = fusionMsg.cameraData->cones[j].color;

                cones->cones.push_back(cone);
            }
        }
    }

    m_publisher.publish(cones);
}

bool Fusion::AreInSamePlace(const sgtdv_msgs::Point2D &p1, const sgtdv_msgs::Point2D &p2) const
{
    if (sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y)) <= m_tol)       //norm of vector p1 - p2
    {
        return true;
    }

    return false;
}
