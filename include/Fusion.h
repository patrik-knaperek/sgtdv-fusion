#include <ros/ros.h>
#include <sgtdv_msgs/FusionMsg.h>
#include <cmath>

class Fusion
{
public:
    Fusion();
    ~Fusion();

    void SetPublisher(ros::Publisher publisher);
    void Do(const sgtdv_msgs::FusionMsgPtr &msg);

private:
    ros::Publisher m_publisher;
    float m_tol;

    bool AreInSamePlace(const sgtdv_msgs::Point2D &p1, const sgtdv_msgs::Point2D &p2) const;
};