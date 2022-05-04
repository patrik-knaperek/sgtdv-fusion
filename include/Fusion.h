/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

#pragma once

#include <ros/ros.h>
#include <cmath>
#include <Eigen/Core>
#include <geometry_msgs/PointStamped.h>
#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/Point2DArr.h>
#include "../include/Messages.h"
#include "../../SGT_Macros.h"
#include <sgtdv_msgs/DebugState.h>

#include "../include/FusionKF.h"

using namespace Eigen;


class Fusion
{
    public:
        Fusion();
        ~Fusion();


        // Setters
        void SetPublisher(ros::Publisher publisher) { m_publisher = publisher; };
        void SetDistanceTol(float tol) { m_distTH = tol; };
        void SetMeassurementModels(Matrix2d cameraMeasModel, Matrix2d lidarMeasModel)
        {
            m_cameraMeasModel = cameraMeasModel;
            m_lidarMeasModel = lidarMeasModel;
        };

    #ifdef SGT_DEBUG_STATE
        void SetVisDebugPublisher(ros::Publisher publisher) { m_visDebugPublisher = publisher; }
    #endif

        void Do(const FusionMsg &fusionMsg);

    private:
        FusionKF m_KF;
        
        ros::Publisher m_publisher;
        ros::Time m_tAct;
        ros::Time m_tOld;

        float m_distTH;

        Matrix2Xd m_fusionCones;
        MatrixX2d m_fusionConesCov;
        ArrayXi m_modified;
        uint8_t m_colors[100];
        ros::Time m_stamps[100];
        int m_numOfCones;

        Matrix2d m_cameraMeasModel;
        Matrix2d m_lidarMeasModel;

    #ifdef SGT_DEBUG_STATE
        ros::Publisher m_visDebugPublisher;
    #endif

        //bool AreInSamePlace(const sgtdv_msgs::Point2D &p1, const sgtdv_msgs::Point2D &p2) const;
        float MahalanDist(const Ref<const Vector2d> &setMean, const Ref<const Matrix2d> &setCov,
                        const Ref<const Vector2d> &obsMean, const Ref<const Matrix2d> &obsCov);
        int MinDistIdx(const Ref<const Matrix2Xd> &meassurementSetMean,const Ref<const MatrixX2d>&meassurementSetCov,
                        int size, const Ref<const Vector2d> &meassurementMean, const Ref<const Matrix2d> &meassurementCov);
};