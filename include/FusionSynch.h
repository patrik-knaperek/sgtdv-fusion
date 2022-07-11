/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

#pragma once

#include <tf/transform_listener.h>
#include "../include/Fusion.h"

class FusionSynch
{
    public:
        FusionSynch();
        ~FusionSynch();

        // Setters
        void SetBaseFrameId(std::string baseFrame) 
        { 
            m_baseFrameId = baseFrame;
            m_fusionObj.SetBaseFrameId(baseFrame);
        };
        void SetCameraFrameId(std::string cameraFrame) { m_cameraFrameId = cameraFrame; };
        void SetLidarFrameId(std::string lidarFrame) { m_lidarFrameId = lidarFrame; };
        float SetSensorFrameTF(std::string &sensorFrameId);

        void SetPublisher(ros::Publisher publisher
        #ifdef SIMPLE_FUSION
            , ros::Publisher simpleFusionPub
        #endif
        ) { m_fusionObj.SetPublisher(publisher
        #ifdef SIMPLE_FUSION
        , simpleFusionPub
        #endif
        ); };
        void SetDistanceTol(float tol) { m_fusionObj.SetDistanceTol(tol); };
        void SetMeassurementModels(Eigen::Matrix<double, N_OF_MODELS, 4> cameraModel, Eigen::Matrix<double, N_OF_MODELS, 4> lidarModel)
        {
            m_fusionObj.SetMeassurementModels(cameraModel, lidarModel);
        };
        
    #ifdef SGT_EXPORT_DATA_CSV
        void OpenDataFile(std::string dataFilename) { m_fusionObj.OpenDataFile(dataFilename); };
        void SetMapFrameId(std::string mapFrame) { m_fusionObj.SetMapFrameId(mapFrame); };
        void MapCallback(const visualization_msgs::MarkerArray::ConstPtr &msg) { m_fusionObj.WriteMapToFile(msg); };
    #endif
    #ifdef SGT_DEBUG_STATE
        void SetVisDebugPublisher(ros::Publisher publisher) { m_fusionObj.SetVisDebugPublisher(publisher); }
    #endif

        void DoCamera(const sgtdv_msgs::ConeArr::ConstPtr &msg);
        void DoLidar(const sgtdv_msgs::Point2DArr::ConstPtr &msg);
        geometry_msgs::PointStamped TransformCoords(geometry_msgs::PointStamped coordsChildFrame);

    private:
        Fusion m_fusionObj;
        bool m_cameraReady; 
        bool m_lidarReady;
        FusionMsg m_fusionMsg;

        std::string m_baseFrameId;
        std::string m_cameraFrameId;
        std::string m_lidarFrameId;

        tf::TransformListener m_listener;
};