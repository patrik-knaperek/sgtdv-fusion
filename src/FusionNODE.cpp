/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

#include "../include/FusionSynch.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "fusion");
	ros::NodeHandle handle;

	ros::Publisher publisher = handle.advertise<sgtdv_msgs::ConeStampedArr>("fusion_cones", 1);
	FusionSynch synch_obj(handle,publisher);

#ifdef SGT_DEBUG_STATE
	ros::Publisher fusion_debug_state_publisher = handle.advertise<sgtdv_msgs::DebugState>("fusion_debug_state", 10);
	synch_obj.setVisDebugPublisher(fusion_debug_state_publisher);
#endif

	ros::Subscriber camera_sub = handle.subscribe("camera_cones", 1, &FusionSynch::cameraCallback, &synch_obj);
	ros::Subscriber lidar_sub = handle.subscribe("lidar_cones", 1, &FusionSynch::lidarCallback, &synch_obj);
	ros::Subscriber pose_sub = handle.subscribe("slam/pose", 1, &FusionSynch::poseCallback, &synch_obj);

#ifdef SGT_EXPORT_DATA_CSV
	ros::Subscriber map_sub = handle.subscribe("fssim/track/markers", 1, &FusionSynch::mapCallback, &synch_obj);
#endif /* SGT_EXPORT_DATA_CSV */

	ros::spin();

	return 0;
}
