/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

#include "fusion_ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fusion");
  ros::NodeHandle handle;

  FusionROS fusion_ros_obj(handle);

  ros::spin();

  return 0;
}
