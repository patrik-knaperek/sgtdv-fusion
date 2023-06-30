/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/


#include <XmlRpcException.h>
#include "../include/FusionSynch.h"


Eigen::ArrayXXd readArray(ros::NodeHandle handle, std::string paramName, int rows, int cols);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fusion");
    ros::NodeHandle handle;
    FusionSynch synchObj;

    ros::Publisher publisher = handle.advertise<sgtdv_msgs::ConeStampedArr>("fusion_cones", 1);
#ifdef SIMPLE_FUSION
    ros::Publisher simpleFusionPub = handle.advertise<sgtdv_msgs::ConeStampedArr>("fusion_simple_cones", 1);
#endif
    synchObj.SetPublisher(publisher
    #ifdef SIMPLE_FUSION
        , simpleFusionPub
    #endif
    );

    std::string baseFrame;
    if(!handle.getParam("/base_frame", baseFrame))
        ROS_ERROR("Failed to get parameter \"/base_frame\" from server\n");
    synchObj.SetBaseFrameId(baseFrame);
    

    std::string cameraFrame;
    if(!handle.getParam("/camera_frame", cameraFrame))
        ROS_ERROR("Failed to get parameter \"/camera_frame\" from server\n");
    synchObj.SetCameraFrameId(cameraFrame);

    std::string lidarFrame;
    if(!handle.getParam("/lidar_frame", lidarFrame))
        ROS_ERROR("Failed to get parameter \"/lidar_frame\" from server\n");
    synchObj.SetLidarFrameId(lidarFrame);

    float tol;
    handle.param("/distance_tolerance", tol, 0.3f);
    synchObj.SetDistanceTol(tol);

#ifdef SGT_EXPORT_DATA_CSV
    std::string dataFilename;
    if (!handle.getParam("/data_filename", dataFilename))
        ROS_ERROR("Failed to get parameter \"/data_filename\" from server\n");
    synchObj.OpenDataFile(dataFilename);

    std::string mapFrame;
    if (!handle.getParam("/map_frame", mapFrame))
        ROS_ERROR("Failed to get parameter \"/map_frame\" from server\n");
    synchObj.SetMapFrameId(mapFrame);

    ros::Subscriber mapSub = handle.subscribe("fssim/track/markers", 1, &FusionSynch::MapCallback, &synchObj);

#endif //SGT_EXPORT_DATA_CSV

    // get sensor measurement models from parameter server
    Eigen::Matrix<double, N_OF_MODELS, 4> cameraModel = Eigen::Matrix<double, N_OF_MODELS, 4>::Zero();
    cameraModel.block<N_OF_MODELS, 2>(0,0) = readArray(handle, std::string("/camera/offset"), N_OF_MODELS, 2);
    cameraModel.block<N_OF_MODELS, 2>(0,2) = readArray(handle, std::string("/camera/covariance"), N_OF_MODELS, 2);

    Eigen::Matrix<double, N_OF_MODELS, 4> lidarModel = Eigen::Matrix<double, N_OF_MODELS, 4>::Zero();
    lidarModel.block<N_OF_MODELS, 2>(0,0) = readArray(handle, std::string("/lidar/offset"), N_OF_MODELS, 2);
    lidarModel.block<N_OF_MODELS, 2>(0,2) = readArray(handle, std::string("/lidar/covariance"), N_OF_MODELS, 2);
    
    synchObj.SetMeassurementModels(cameraModel, lidarModel);

#ifdef SGT_DEBUG_STATE
    ros::Publisher fusionDebugStatePublisher = handle.advertise<sgtdv_msgs::DebugState>("fusion_debug_state", 10);
    synchObj.SetVisDebugPublisher(fusionDebugStatePublisher);
#endif

    ros::Subscriber cameraSub = handle.subscribe("camera_cones", 1, &FusionSynch::DoCamera, &synchObj);
    ros::Subscriber lidarSub = handle.subscribe("lidar_cones", 1, &FusionSynch::DoLidar, &synchObj);

    ros::spin();

    return 0;
}

Eigen::ArrayXXd readArray(ros::NodeHandle handle, std::string paramName, int rows, int cols)
{
    XmlRpc::XmlRpcValue paramValue;
    Eigen::ArrayXXd arrayValue = Eigen::ArrayXXd::Zero(rows, cols);
    if (!handle.getParam(paramName, paramValue))
        ROS_ERROR_STREAM("Failed to get parameter " << paramName << " from server\n");
    else
    {
        try
        {
            ROS_ASSERT(paramValue.getType() == XmlRpc::XmlRpcValue::TypeArray);
            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                    try
                    {
                        std::ostringstream ostr;
                        ostr << paramValue[cols * i  + j];
                        std::istringstream istr(ostr.str());
                        istr >> arrayValue(i, j);
                    }
                    catch(XmlRpc::XmlRpcException &e)
                    {
                        throw e;;
                    }
                    catch(...)
                    {
                        throw;
                    }
                }
            }
        }
        catch(XmlRpc::XmlRpcException &e)
        {
            ROS_ERROR_STREAM("ERROR reading from sercer: " << e.getMessage() <<
                            " for " << paramName << "(type: " << paramValue.getType() << ")");
        }
    }
    return arrayValue;
}
