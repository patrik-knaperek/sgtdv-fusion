/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

#include "fusion_synch.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fusion");
  ros::NodeHandle handle;

  FusionSynch synch_obj(handle);

  ros::spin();

  return 0;
}
