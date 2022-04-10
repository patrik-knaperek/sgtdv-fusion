/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include "../include/FusionSynch.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fusion");
    ros::NodeHandle handle;
    FusionSynch synchObj;

    ros::Publisher publisher = handle.advertise<sgtdv_msgs::ConeArr>("fusion_cones", 1);
    synchObj.SetPublisher(publisher);

    std::string baseFrame;
    if(!handle.getParam("/base_frame", baseFrame))
        ROS_ERROR("Failed to get parameter from server\n");
    synchObj.SetBaseFrameId(baseFrame);

    float tol;
    handle.param("/distance_tolerance", tol, 0.3f);
    synchObj.SetDistanceTol(tol);

#ifdef SGT_DEBUG_STATE
    ros::Publisher fusionDebugStatePublisher = handle.advertise<sgtdv_msgs::DebugState>("fusion_debug_state", 10);
    synchObj.SetVisDebugPublisher(fusionDebugStatePublisher);
#endif

    ros::Subscriber cameraSub = handle.subscribe("camera_cones", 1, &FusionSynch::DoCamera, &synchObj);
    ros::Subscriber lidarSub = handle.subscribe("lidar_cones", 1, &FusionSynch::DoLidar, &synchObj);

    ros::spin();

    return 0;
}
