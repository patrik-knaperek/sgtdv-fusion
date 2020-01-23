#include <ros/ros.h>
#include <sgtdv_msgs/FusionMsg.h>
#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/Point2DArr.h>
#include "../include/Fusion.h"

class FusionSynch
{
public:
    FusionSynch();
    ~FusionSynch();

    void SetPublisher(ros::Publisher publisher);
    void DoCamera(const sgtdv_msgs::ConeArr::ConstPtr &msg);
    void DoLidar(const sgtdv_msgs::Point2DArr::ConstPtr &msg);

private:
    Fusion m_fusion;
    bool m_cameraReady;
    bool m_lidarReady;
    sgtdv_msgs::ConeArr::ConstPtr m_cameraData;
    sgtdv_msgs::Point2DArr::ConstPtr m_lidarData;
};