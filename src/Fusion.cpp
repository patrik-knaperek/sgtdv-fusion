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

void Fusion::Do(const sgtdv_msgs::FusionMsgPtr &msg)
{   
    sgtdv_msgs::ConeArrPtr cones( new sgtdv_msgs::ConeArr );
    sgtdv_msgs::Cone cone;

    cones->cones.reserve(20);
    //TODO: Static transformations
    
    for(size_t i = 0; i < msg->lidarCones.points.size(); i++)
    {
        for(size_t j = 0; j < msg->cameraCones.cones.size(); j++)
        {
            if (AreInSamePlace(msg->lidarCones.points[i], msg->cameraCones.cones[j].coords))
            {
                cone.coords = msg->lidarCones.points[i];
                cone.color = msg->cameraCones.cones[j].color;

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
