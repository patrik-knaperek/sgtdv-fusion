/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include <ros/ros.h>
#include "../include/FusionSynch.h"
#include <sgtdv_msgs/ConeArr.h>
#include "../../SGT_Macros.h"
#include <sgtdv_msgs/DebugState.h>

int main(int argc, char** argv)
{
    FusionSynch synchObj;

    ros::init(argc, argv, "fusion");
    ros::NodeHandle handle;

    ros::Publisher publisher = handle.advertise<sgtdv_msgs::ConeArr>("fusion_cones", 1);

    synchObj.SetPublisher(publisher);

#ifdef DEBUG_STATE
    ros::Publisher fusionDebugStatePublisher = handle.advertise<sgtdv_msgs::DebugState>("fusion_debug_state", 10);
    synchObj.SetVisDebugPublisher(fusionDebugStatePublisher);
#endif

    ros::Subscriber cameraSub = handle.subscribe("camera_cones", 1, &FusionSynch::DoCamera, &synchObj);
    ros::Subscriber lidarSub = handle.subscribe("lidar_cones", 1, &FusionSynch::DoLidar, &synchObj);

    ros::spin();

    return 0;
}
