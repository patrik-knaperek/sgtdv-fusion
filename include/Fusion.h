/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

#pragma once

// C++
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <array>
#include <Eigen/Eigen>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

// SGT
#include <sgtdv_msgs/ConeStampedArr.h>
#include <sgtdv_msgs/Point2DStampedArr.h>
#include <sgtdv_msgs/DebugState.h>
#include "../include/Messages.h"
#include "../../SGT_Macros.h"
#include "../include/FusionKF.h"


#define VITALITY_SCORE_INIT 2
#define VITALITY_SCORE_MAX 4       // maximalna hodnota skore vitality sledovanych kuzelov
#define MAX_TRACKED_CONES_N 20          // maximalny pocet sledovanych kuzelov
#define N_OF_MODELS 3                   // pocet regionov modelov merania
#define CAMERA_X_MIN 1.7
#define CAMERA_X_MAX 8.0
#define LIDAR_X_MIN 0.75
#define LIDAR_X_MAX 8
#define VALIDATION_SCORE_TH 2           // prahova hodnota validacneho skore pre publikovanie sledovaneho kuzela
#define ACCURACY_CORRECTION             // ci sa ma robit korekcia presnosti

#ifdef SGT_EXPORT_DATA_CSV
    #define DATA_SIZE_MAX 300
#endif




class Fusion
{
    public:
        Fusion(); 
        ~Fusion();

        // Setters
        void SetPublisher(ros::Publisher publisher
        #ifdef SIMPLE_FUSION
            , ros::Publisher simpleFusionPub
        #endif
        ) { m_publisher = publisher; 
        #ifdef SIMPLE_FUSION
            m_simpleFusionPub = simpleFusionPub; 
        #endif
        };
        void SetDistanceTol(float tol) { m_distTH = tol; };
        void SetMeassurementModels(Eigen::Matrix<double, N_OF_MODELS, 4> cameraModel, Eigen::Matrix<double, N_OF_MODELS, 4> lidarModel)
        {
            m_cameraModel = cameraModel;
            m_lidarModel = lidarModel;
        };
        void SetBaseFrameId(std::string baseFrame) { m_baseFrameId = baseFrame; };
        void SetCameraFrameId(std::string cameraFrame) { m_cameraFrameId = cameraFrame; };
        void SetLidarFrameId(std::string lidarFrame) { m_lidarFrameId = lidarFrame; };

    #ifdef SGT_EXPORT_DATA_CSV
        void OpenDataFile(std::string filename);
        void SetMapFrameId(std::string mapFrame) { m_mapFrameId = mapFrame; };
        void WriteMapToFile(const visualization_msgs::MarkerArray::ConstPtr &msg);
    #endif
    #ifdef SGT_DEBUG_STATE
        void SetVisDebugPublisher(ros::Publisher publisher) { m_visDebugPublisher = publisher; }
    #endif

        void Do(const FusionMsg &fusionMsg);
   
    private:
        void GetSensorFrameTF(void);
        /*float MahalanDist(const Eigen::Ref<const Eigen::Vector2d> &setMean, const Eigen::Ref<const Eigen::Matrix2d> &setCov,
                        const Eigen::Ref<const Eigen::Vector2d> &obsMean, const Eigen::Ref<const Eigen::Matrix2d> &obsCov);
                        */
        /*int MinDistIdx(const Eigen::Ref<const Eigen::Matrix2Xd> &measurementSetMean,const Eigen::Ref<const Eigen::MatrixX2d>&measurementSetCov,
                        int setSize, const Eigen::Ref<const Eigen::Vector2d> &measurementMean, const Eigen::Ref<const Eigen::Matrix2d> &measurementCov);
                        */
        int MinDistIdx(const Eigen::Ref<const Eigen::Matrix2Xd> &measurementSetMean, int setSize, 
        const Eigen::Ref<const Eigen::Vector2d> &measurementMean);

        FusionKF m_KF;
        
        ros::Publisher m_publisher;

    #ifdef SIMPLE_FUSION
        ros::Publisher m_simpleFusionPub;
    #endif

        float m_distTH;     // distance treshold for data association

        Eigen::Matrix<double, 2, MAX_TRACKED_CONES_N> m_fusionCones;    //tracked cones

    #ifdef SIMPLE_FUSION
        Eigen::Matrix<double, 2, MAX_TRACKED_CONES_N> m_fusionSimpleCones;
    #endif

        Eigen::Matrix<double, 2*MAX_TRACKED_CONES_N, 2> m_fusionConesCov;
        Eigen::Array<int, 1, MAX_TRACKED_CONES_N> m_vitalityScore;
        Eigen::Array<int, 1, MAX_TRACKED_CONES_N> m_validationScore;
        uint8_t m_colors[MAX_TRACKED_CONES_N];
        ros::Time m_stamps[MAX_TRACKED_CONES_N];
        int m_numOfCones;

        Eigen::Matrix<double, N_OF_MODELS, 4> m_cameraModel;
        Eigen::Matrix<double, N_OF_MODELS, 4> m_lidarModel;

        std::string m_baseFrameId;
        std::string m_cameraFrameId;
        std::string m_lidarFrameId;
        tf::StampedTransform m_cameraFrameTF;
        tf::StampedTransform m_lidarFrameTF;
        tf::TransformListener m_listener;

    #ifdef SGT_EXPORT_DATA_CSV
        void WriteToDataFile(int Idx);
        Eigen::Vector2d TransformCoords(const Eigen::Ref<const Eigen::Vector2d> &obsBaseFrame, ros::Time stamp);
        Eigen::Vector2d TransformCoords(const sgtdv_msgs::Point2D &obs);

        Eigen::Matrix<double, 2*MAX_TRACKED_CONES_N, DATA_SIZE_MAX> m_cameraData;
        Eigen::Matrix<double, 2*MAX_TRACKED_CONES_N, DATA_SIZE_MAX> m_lidarData;
        Eigen::Matrix<double, 2*MAX_TRACKED_CONES_N, DATA_SIZE_MAX> m_fusionData;
        Eigen::Array<int, MAX_TRACKED_CONES_N, 1> m_cameraDataCount;
        Eigen::Array<int, MAX_TRACKED_CONES_N, 1> m_lidarDataCount;
        Eigen::Array<int, MAX_TRACKED_CONES_N, 1> m_fusionDataCount;
        std::ofstream m_cameraDataFile;
        std::ofstream m_lidarDataFile;
        std::ofstream m_fusionDataFile;
        std::ofstream m_mapDataFile;

        std::string m_mapFrameId;

        #ifdef SIMPLE_FUSION
            Eigen::Matrix<double, 2*MAX_TRACKED_CONES_N, DATA_SIZE_MAX> m_simpleFusionData;
            Eigen::Array<int, MAX_TRACKED_CONES_N, 1> m_simpleFusionDataCount;
            std::ofstream m_simpleFusionDataFile;
        #endif // SIMPLE_FUSION
    #endif //SGT_EXPORT_DATA_CSV

    #ifdef SGT_DEBUG_STATE
        ros::Publisher m_visDebugPublisher;
    #endif        
};
