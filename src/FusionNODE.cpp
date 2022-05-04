/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Patrik Knaperek
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

    // get sensor meassurement models from parameter server
    Eigen::Matrix2d cameraMM = Eigen::Matrix2d::Zero();
    XmlRpc::XmlRpcValue cameraMMParam;

    try
    {
        if (!handle.getParam("/camera_model", cameraMMParam))
            ROS_ERROR("Failed to get parameter from server \n");

        ROS_ASSERT(cameraMMParam.getType() == XmlRpc::XmlRpcValue::TypeArray);

        for (int j = 0; j < 4; j++)
        {
            try
            {
                std::ostringstream ostr;
                ostr << cameraMMParam[j];
                std::istringstream istr(ostr.str());
                istr >> cameraMM(j);
            }
            catch(XmlRpc::XmlRpcException &e)
            {
                throw e;
            }
            catch(...)
            {
                throw;
            }
                
        }
    }
    catch(XmlRpc::XmlRpcException &e)
    {
        ROS_ERROR_STREAM("ERROR reading from server: " <<
                        e.getMessage() <<
                        " for cones_coords (type: " <<
                        cameraMMParam.getType() << ")");
    }

    Eigen::Matrix2d lidarMM = Eigen::Matrix2d::Zero();
    XmlRpc::XmlRpcValue lidarMMParam;

    try
    {
        if (!handle.getParam("/lidar_model", lidarMMParam))
            ROS_ERROR("Failed to get parameter from server \n");

        ROS_ASSERT(lidarMMParam.getType() == XmlRpc::XmlRpcValue::TypeArray);

        for (int j = 0; j < 4; j++)
        {
            try
            {
                std::ostringstream ostr;
                ostr << lidarMMParam[j];
                std::istringstream istr(ostr.str());
                istr >> lidarMM(j);
            }
            catch(XmlRpc::XmlRpcException &e)
            {
                throw e;
            }
            catch(...)
            {
                throw;
            }
                
        }
    }
    catch(XmlRpc::XmlRpcException &e)
    {
        ROS_ERROR_STREAM("ERROR reading from server: " <<
                        e.getMessage() <<
                        " for cones_coords (type: " <<
                        lidarMMParam.getType() << ")");
    }
    synchObj.SetMeassurementModels(cameraMM, lidarMM);
    //std::cout << "here8" << std::endl;
#ifdef SGT_DEBUG_STATE
    ros::Publisher fusionDebugStatePublisher = handle.advertise<sgtdv_msgs::DebugState>("fusion_debug_state", 10);
    synchObj.SetVisDebugPublisher(fusionDebugStatePublisher);
#endif

    ros::Subscriber cameraSub = handle.subscribe("camera_cones", 1, &FusionSynch::DoCamera, &synchObj);
    ros::Subscriber lidarSub = handle.subscribe("lidar_cones", 1, &FusionSynch::DoLidar, &synchObj);

    ros::spin();

    return 0;
}
