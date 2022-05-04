/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/


#include "../include/Fusion.h"


Fusion::Fusion()
{
    m_fusionCones = Matrix2Xd::Zero(2,100);
    m_fusionConesCov = MatrixX2d::Zero(200,2);
    m_modified = ArrayXi::Zero(100);
    //std::cout << "here1" << std::endl;
    m_numOfCones = 0;
}

Fusion::~Fusion()
{

}

void Fusion::Do(const FusionMsg &fusionMsg)
{   
#ifdef SGT_DEBUG_STATE
    sgtdv_msgs::DebugState state;
    state.workingState = 1;
    m_visDebugPublisher.publish(state);
#endif
    std::cout << "here2" << std::endl;
    //* Fusion EKF *//
    m_tAct = ros::Time::now();
    if (m_numOfCones > 0)
        m_modified -= 1;

    // extract data from input message
    int numOfCamObs = fusionMsg.cameraData->cones.size();
    int numOfLidObs = fusionMsg.lidarData->points.size();

    Eigen::Matrix2Xd cameraObs(2, numOfCamObs);
    Eigen::Matrix2Xd lidarObs(2, numOfLidObs);

    for (int i = 0; i < numOfCamObs; i++)
    {
        cameraObs(0, i) = fusionMsg.cameraData->cones[i].coords.x;
        cameraObs(1, i) = fusionMsg.cameraData->cones[i].coords.y;
    }

    for (int i = 0; i < numOfLidObs; i++)
    {
        lidarObs(0, i) = fusionMsg.lidarData->points[i].x;
        lidarObs(1, i) = fusionMsg.lidarData->points[i].y;
    }

    std::cout << "here12\n" << "m_numOfCones: " << m_numOfCones << std::endl;
    if (m_numOfCones > 0)
    {
        std::cout << "tracked cones before Predict:\n" << m_fusionCones << std::endl;
        std::cout << "time delta: " << m_tAct - m_tOld << std::endl;
        m_KF.Predict(m_fusionCones, m_fusionConesCov, m_numOfCones, m_tAct - m_tOld);
        for (int i = 0; i < m_numOfCones; i++)
        {
            m_stamps[i] = ros::Time::now();
        }
        std::cout << "tracked cones after Predict:\n" << m_fusionCones << std::endl;
        //m_stamps.setConstant(ros::Time::now());
    }
    // search in camera detections
    int fIdx;
    int lIdx;
    for (int cIdx = 0; cIdx < numOfCamObs; cIdx++)
    {
        // associate and ekf-update with tracked detections
        fIdx = MinDistIdx(m_fusionCones, m_fusionConesCov, m_numOfCones, cameraObs.col(cIdx), m_cameraMeasModel);
        std::cout << "fIdx: " << fIdx << std::endl;
        if (fIdx >= 0)
        {   // run ekf-update with new detection
            std::cout << "here 13" << std::endl;
            m_KF.Update(m_fusionCones, m_fusionConesCov, fIdx, cameraObs.col(cIdx), m_cameraMeasModel);
            m_modified(fIdx) += m_modified(fIdx) >=6 ? 0 : 1;
            m_stamps[fIdx] = fusionMsg.cameraData->cones[cIdx].coords.header.stamp;
        }
        else
        {   // add to tracked detections
            //std::cout << "here12\n" << "m_numOfCones: " << m_numOfCones
            //<< "\ncIdx: " << cIdx << std::endl;
            //std::cout << "cameraObs: \n" << cameraObs << std::endl;
            m_fusionCones.col(m_numOfCones) = cameraObs.col(cIdx);
            m_fusionConesCov.block<2,2>(2*cIdx, 0) = m_cameraMeasModel;
            //std::cout << "color: " << fusionMsg.cameraData->cones[cIdx].color << std::endl;
            m_colors[m_numOfCones] = fusionMsg.cameraData->cones[cIdx].color;
            m_stamps[m_numOfCones] = fusionMsg.cameraData->cones[cIdx].coords.header.stamp;
            m_modified(m_numOfCones) = 2;
            fIdx = m_numOfCones++;
            std::cout << "here11" << std::endl;
        }
    }
    //search in lidar detections
    for (size_t lIdx = 0; lIdx < numOfLidObs; lIdx++)
    {
        
        // associate and ekf-update with tracked detection
        fIdx = MinDistIdx(m_fusionCones, m_fusionConesCov, m_numOfCones, lidarObs.col(lIdx), m_lidarMeasModel);
        if (fIdx >= 0)
        {
            m_KF.Update(m_fusionCones, m_fusionConesCov, fIdx, lidarObs.col(lIdx), m_lidarMeasModel);
            m_modified(fIdx) += m_modified(fIdx) >=6 ? 0 : 1;
            m_stamps[fIdx] = fusionMsg.lidarData->points[lIdx].header.stamp;
        }
    }

    // update tracked detections
    // throw away detections that weren't updated several times in a row
    int pointer = 0;
    for (uint8_t i = 0; i < m_numOfCones; i++)
    {
        if (m_modified(i) > 0)
        {
            m_fusionCones.col(pointer) = m_fusionCones.col(i);
            m_fusionConesCov.block<2,2>(2*pointer,0) = m_fusionConesCov.block<2,2>(2*i,0);
            m_stamps[pointer] = m_stamps[i];
            m_colors[pointer] = m_colors[i];
            m_modified(pointer) = m_modified(i);
            pointer++;
        }
    }
    m_numOfCones = pointer + 1;

    m_tOld = m_tAct;

    std::cout << "here3" << std::endl;
    
    // create and publish Fusion message
    sgtdv_msgs::ConeArrPtr fusedCones( new sgtdv_msgs::ConeArr );
    sgtdv_msgs::Cone cone;
 
    fusedCones->cones.reserve(m_numOfCones);
    /*bool fused;*/

    for(size_t i = 0; i < m_numOfCones; i++)
    {
        /*fused = false;
        for(size_t j = 0; j < fusionMsg.lidarData->points.size(); j++)
        {
            if (AreInSamePlace(fusionMsg.cameraData->cones[i].coords, fusionMsg.lidarData->points[j]))
            {
                cone.coords = fusionMsg.lidarData->points[j];
                fused = true;
			#ifdef FUSION_CONSOLE_SHOW
                std::cout << "taking coords from lidar:\n" << cone.coords << std::endl;
			#endif
                break;
            }
        }

        if (!fused)
        {
            cone.coords = fusionMsg.cameraData->cones[i].coords;
		#ifdef FUSION_CONSOLE_SHOW
            std::cout << "taking coords from camera:\n" << cone.coords << std::endl; 
		#endif
        }

        cone.color = fusionMsg.cameraData->cones[i].color;
	#ifdef FUSION_CONSOLE_SHOW
        }*/

        cone.coords.header.frame_id = "base_link";
        cone.coords.header.seq = i;
        cone.coords.header.stamp = m_stamps[i];
        cone.coords.x = m_fusionCones(0, i);
        cone.coords.y = m_fusionCones(1,i);
        cone.color = m_colors[i];
        std::cout << "CONE COLOR: " << cone.color << std::endl;
	//#endif

        fusedCones->cones.push_back(cone);
    }
    
    m_publisher.publish(fusedCones);

#ifdef SGT_DEBUG_STATE
    state.workingState = 0;
    state.numOfCones = static_cast<uint32_t>(m_numOfCones);
    m_visDebugPublisher.publish(state);
#endif
}

/*bool Fusion::AreInSamePlace(const sgtdv_msgs::Point2D &p1, const sgtdv_msgs::Point2D &p2) const
{
    float e_dist = (float) sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y)); //norm of vector p1 - p2
    //std::cout << "euclid: " << e_dist << std::endl;
    
    if (e_dist <= m_tol)
    {
        return true;
    }
    
    return false;
}*/

int Fusion::MinDistIdx(const Ref<const Matrix2Xd> &meassurementSetMean,const Ref<const MatrixX2d>&meassurementSetCov,
                        int size, const Ref<const Vector2d> &meassurementMean, const Ref<const Matrix2d> &meassurementCov)
{
    if (size > 0)
    {
        for (uint8_t i; i < size; i++)
        {
            if (MahalanDist(meassurementSetMean.col(i), meassurementSetCov.block<2,2>(2*i,0), 
                meassurementMean, meassurementCov) <= m_distTH)
            {
                return i;
            }
            else
            {
                return -1;
            }

        }
    }
    return -1;
}

float Fusion::MahalanDist(const Ref<const Vector2d> &setMean, const Ref<const Matrix2d> &setCov,
                        const Ref<const Vector2d> &obsMean, const Ref<const Matrix2d> &obsCov)
{
    /*float mahDist = std::pow(p1Mean(0) - p2Mean(0), 2) / (std::pow(p1Cov(0,0),2) + std::pow(p2Cov(0,0),2)) +
                    std::pow(p1Mean(1) - p2Mean(1), 2) / (std::pow(p1Cov(1,1),2) + std::pow(p2Cov(1,1),2));*/
    Vector2d diff = obsMean - setMean;
    RowVector2d diffT = diff.transpose();
    //std::cout << "x1 - x2 = " << (obsMean(0) - setMean(0)) << "\ny1 - y2 = " << (obsMean(1) - setMean(1)) << std::endl;
    std::cout << "diff = " << diff << std::endl; 
    std::cout << "Cov = \n" << setCov << std::endl;
    
    RowVector2d temp = diffT * setCov.inverse();
    double mahDist2 = temp * diff;
    float mahDist = sqrt(mahDist2);
    float e_dist = (float) sqrt(pow((obsMean(0) - setMean(0)),2) + pow((obsMean(1) - setMean(1)),2)); //norm of vector p1 - p2
    std::cout << "euclid: " << e_dist << std::endl;
    std::cout << "ED = " << sqrt(diffT * diff) << std::endl;
    std::cout << "MD = " << mahDist << std::endl;
    return mahDist;
}
