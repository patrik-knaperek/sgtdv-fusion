/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include "../include/Fusion.h"

Fusion::Fusion()
{
}

Fusion::~Fusion()
{

}

void Fusion::Do(const FusionMsg &fusionMsg)
{   
#ifdef SGT_DEBUG_STATE
    sgtdv_msgs::DebugState state;
    state.workingState = 1;
    m_visDebugPublisher.publish(state);
#endif

    sgtdv_msgs::ConeArrPtr fusedCones( new sgtdv_msgs::ConeArr );
    sgtdv_msgs::Cone cone;

    int conesCount;
    conesCount = fusionMsg.cameraData->cones.size();
    
    fusedCones->cones.reserve(conesCount);
    bool fused;

    for(size_t i = 0; i < conesCount; i++)
    {
        fused = false;
        for(size_t j = 0; j < fusionMsg.lidarData->points.size(); j++)
        {
            if (AreInSamePlace(fusionMsg.cameraData->cones[i].coords, fusionMsg.lidarData->points[j]))
            {
                cone.coords = fusionMsg.lidarData->points[j];
                fused = true;
                std::cout << "taking coords from lidar:\n" << cone.coords << std::endl; 
                break;
            }
        }

        if (!fused)
        {
            cone.coords = fusionMsg.cameraData->cones[i].coords;
            std::cout << "taking coords from camera:\n" << cone.coords << std::endl; 
        }

        cone.color = fusionMsg.cameraData->cones[i].color;
        std::cout << "CONE COLOR: " << cone.color << std::endl;

        fusedCones->cones.push_back(cone);
    }
    
    m_publisher.publish(fusedCones);

#ifdef SGT_DEBUG_STATE
    state.workingState = 0;
    state.numOfCones = static_cast<uint32_t>(fusedCones->cones.size());
    m_visDebugPublisher.publish(state);
#endif
}

bool Fusion::AreInSamePlace(const sgtdv_msgs::Point2D &p1, const sgtdv_msgs::Point2D &p2) const
{
    float e_dist = (float) sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y)); //norm of vector p1 - p2
    //std::cout << "euclid: " << e_dist << std::endl;
    
    if (e_dist <= m_tol)
    {
        return true;
    }
    
    return false;
}
