/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/


#include "../include/Fusion.h"

Fusion::Fusion()
{
    m_fusionCones = Eigen::Matrix2Xd::Zero(2,MAX_TRACKED_CONES_N);
    m_fusionConesCov = Eigen::MatrixX2d::Zero(2*MAX_TRACKED_CONES_N,2);
    m_vitalityScore.setZero();
    m_validationScore.setZero();
    m_numOfCones = 0;

#ifdef SGT_EXPORT_DATA_CSV
    m_cameraData.setZero();
    m_lidarData.setZero();
    m_fusionData.setZero();
    m_cameraDataCount.setZero();
    m_lidarDataCount.setZero();
    m_fusionDataCount.setZero();

#ifdef SIMPLE_FUSION
    m_simpleFusionData.setZero();
    m_simpleFusionDataCount.setZero();
#endif

#endif // SGT_EXPORT_DATA_CSV
}

Fusion::~Fusion()
{
#ifdef SGT_EXPORT_DATA_CSV
    for (int i = 0; i < m_numOfCones; i++)
    {
        if (m_fusionDataCount(i) > 0)
            {
                WriteToDataFile(i); 
            }
    }

    m_cameraDataFile.close();
    m_lidarDataFile.close();
    m_fusionDataFile.close();
    m_mapDataFile.close();

#ifdef SIMPLE_FUSION
    m_simpleFusionDataFile.close();
#endif

#endif // SGT_EXPORT_DATA_CSV
}


void Fusion::Do(const FusionMsg &fusionMsg)
{   
#ifdef SGT_DEBUG_STATE
    sgtdv_msgs::DebugState state;
    state.workingState = 1;
    m_visDebugPublisher.publish(state);
#endif

    if (m_numOfCones > 0)
        m_vitalityScore -= 1;

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

    // KF prediction step for all tracked cones
    if (m_numOfCones > 0)
    {
        m_KF.Predict(m_fusionCones, m_fusionConesCov, m_numOfCones);
    }

    // search in CAMERA detections
    int fIdx;
    Eigen::Vector2d cameraObsAct;
    Eigen::Matrix2d cameraCovAct = Eigen::Matrix2d::Zero(2,2);
    Eigen::Vector2d lidarObsAct;
    Eigen::Matrix2d lidarCovAct = Eigen::Matrix2d::Zero(2,2);

#ifdef SIMPLE_FUSION
    sgtdv_msgs::ConeArrPtr simpleFusionCones (new sgtdv_msgs::ConeArr);
    sgtdv_msgs::Cone simpleFusionCone;
    simpleFusionCones->cones.reserve(numOfCamObs);
#endif
    for (int cIdx = 0; cIdx < numOfCamObs; cIdx++)
    {
        // filter measurement by x axis
        if (cameraObs(0, cIdx) < m_cameraFrameTF + CAMERA_X_MIN || cameraObs(0, cIdx) > m_cameraFrameTF + CAMERA_X_MAX)
            continue;
        
        // asign measurement model to measurement
        for(int model = 0; model < N_OF_MODELS; model++)
        {
            if (cameraObs(0, cIdx) < (CAMERA_X_MAX - CAMERA_X_MIN) / N_OF_MODELS * (model+1) + m_cameraFrameTF + CAMERA_X_MIN)
            {
                cameraObsAct = cameraObs.col(cIdx);
            #ifdef ACCURACY_CORRECTION
                cameraObsAct += m_cameraModel.block<1,2>(model,0).transpose();
            #endif
                cameraCovAct(0,0) = m_cameraModel(model,2);
                cameraCovAct(1,1) = m_cameraModel(model,3);
                break;
            }
        }
        
        // associate and ekf-update with tracked detections
        fIdx = MinDistIdx(m_fusionCones, m_numOfCones, cameraObsAct);
        if (fIdx >= 0)
        {   // run ekf-update with new detection
            m_KF.Update(m_fusionCones, m_fusionConesCov, fIdx, cameraObsAct, cameraCovAct);
            m_vitalityScore(fIdx) += m_vitalityScore(fIdx) >= MAX_TRACKED_CONES_SCORE ? 0 : 1;
            m_validationScore(fIdx) += m_validationScore(fIdx) > VALIDATION_SCORE_TH ? 0 : 1;
        }
        else
        {   
            // add to tracked detections
            fIdx = m_numOfCones++;
            m_fusionCones.col(fIdx) = cameraObsAct;
            m_fusionConesCov.block<2,2>(2*fIdx, 0) = cameraCovAct;
            m_vitalityScore(fIdx) = 2;
            m_validationScore(fIdx) = 1;
        }

        // update color and stamp information
        m_colors[fIdx] = fusionMsg.cameraData->cones[cIdx].color;
        m_stamps[fIdx] = fusionMsg.cameraData->cones[cIdx].coords.header.stamp;

    #ifdef SIMPLE_FUSION
        int lIdx = MinDistIdx(lidarObs, numOfLidObs, cameraObs.col(cIdx));
        if (lIdx >= 0)
        {
            simpleFusionCone.coords.header = fusionMsg.lidarData->points[lIdx].header;
            simpleFusionCone.coords.x 
                = (fusionMsg.cameraData->cones[cIdx].coords.x + fusionMsg.lidarData->points[lIdx].x) / 2;
            simpleFusionCone.coords.y 
                = (fusionMsg.cameraData->cones[cIdx].coords.y + fusionMsg.lidarData->points[lIdx].y) / 2;
        }
        else
        {
            simpleFusionCone.coords = fusionMsg.cameraData->cones[cIdx].coords;
        }
        //std::cout << "cIdx" << cIdx << std::endl;
        //std::cout << simpleFusionCone.coords << std::endl;
        simpleFusionCone.color = fusionMsg.cameraData->cones[cIdx].color;
        simpleFusionCones->cones.push_back(simpleFusionCone);
    #endif // SIMPLE_FUSION    


    #ifdef SGT_EXPORT_DATA_CSV

        Eigen::Vector2d cameraObsMap = TransformCoords(fusionMsg.cameraData->cones[cIdx].coords);
        if (cameraObsMap != Eigen::Vector2d::Zero())
        {
            std::cout << m_cameraDataCount(fIdx) << std::endl;
            m_cameraData.block<2, 1>(2*fIdx, m_cameraDataCount(fIdx)) = cameraObsMap; 
            m_cameraDataCount(fIdx)++;
        }

    #ifdef SIMPLE_FUSION
        Eigen::Vector2d simpleFusionMap = TransformCoords(simpleFusionCone.coords);
        if (simpleFusionMap != Eigen::Vector2d::Zero())
        {
            m_simpleFusionData.block<2, 1>(2*fIdx, m_simpleFusionDataCount(fIdx)) = simpleFusionMap;
            m_simpleFusionDataCount(fIdx)++;
        }
    #endif // SIMPLE_FUSION
    #endif // SGT_EXPORT_DATA_CSV
    }


    //search in LIDAR detections
    for (int lIdx = 0; lIdx < numOfLidObs; lIdx++)
    {
        //filter by x axis
        if (lidarObs(0, lIdx) < m_lidarFrameTF + LIDAR_X_MIN || lidarObs(0, lIdx) > m_lidarFrameTF + LIDAR_X_MAX)
            continue;

        // asign measurement model to measurement
        for(int model = 0; model < N_OF_MODELS; model++)
        {
            if (lidarObs(0, lIdx) < (LIDAR_X_MAX - LIDAR_X_MIN) / N_OF_MODELS * (model+1) + LIDAR_X_MIN + m_lidarFrameTF)
            {
                lidarObsAct = lidarObs.col(lIdx);
            #ifdef ACCURACY_CORRECTION
                lidarObsAct += m_lidarModel.block<1,2>(model,0).transpose();
            #endif
                lidarCovAct(0,0) = m_lidarModel(model,2);
                lidarCovAct(1,1) = m_lidarModel(model,3);
                break;
            }
        }
        
        // associate and ekf-update with tracked detection
        fIdx = MinDistIdx(m_fusionCones, m_numOfCones, lidarObsAct);
        if (fIdx >= 0)
        {
            m_KF.Update(m_fusionCones, m_fusionConesCov, fIdx, lidarObsAct, lidarCovAct);
            m_vitalityScore(fIdx) += m_vitalityScore(fIdx) >= MAX_TRACKED_CONES_SCORE ? 0 : 1;
            m_validationScore(fIdx) += m_validationScore(fIdx) > VALIDATION_SCORE_TH ? 0 : VALIDATION_SCORE_TH;
            m_stamps[fIdx] = fusionMsg.lidarData->points[lIdx].header.stamp;

        #ifdef SGT_EXPORT_DATA_CSV
            Eigen::Vector2d lidarObsMap = TransformCoords(fusionMsg.lidarData->points[lIdx]);
            if (lidarObsMap != Eigen::Vector2d::Zero())
            {
                m_lidarData.block<2, 1>(2*fIdx, m_lidarDataCount(fIdx)) = lidarObsMap; 
                m_lidarDataCount(fIdx)++;
            }
        #endif
        }
    }

    // update tracked detections
    // - throw away detections that weren't updated several times in a row
    int pointer = 0;
    for (int fIdx = 0; fIdx < m_numOfCones; fIdx++)
    {   
        if ((m_vitalityScore(fIdx)) > 0 && (m_fusionCones(0,fIdx) >= m_cameraFrameTF + CAMERA_X_MIN + m_cameraModel(0,0)))
        {
            m_fusionCones.col(pointer) = m_fusionCones.col(fIdx);
            m_fusionConesCov.block<2,2>(2*pointer,0) = m_fusionConesCov.block<2,2>(2*fIdx,0);
            m_stamps[pointer] = m_stamps[fIdx];
            m_colors[pointer] = m_colors[fIdx];
            m_vitalityScore(pointer) = m_vitalityScore(fIdx);
            m_validationScore(pointer) = m_validationScore(fIdx);
            

        #ifdef SGT_EXPORT_DATA_CSV
            Eigen::Vector2d fusionObsMap = TransformCoords(m_fusionCones.col(fIdx), m_stamps[fIdx]);
            if (fusionObsMap != Eigen::Vector2d::Zero() && m_validationScore(fIdx) > VALIDATION_SCORE_TH)
            {
                m_fusionData.block<2, 1>(2*fIdx, m_fusionDataCount(fIdx)) = fusionObsMap;
                m_fusionDataCount(fIdx)++;
            }
            
            m_cameraData.row(2*pointer) = m_cameraData.row(2*fIdx);
            m_cameraData.row(2*pointer+1) = m_cameraData.row(2*fIdx+1);
            m_cameraDataCount(pointer) = m_cameraDataCount(fIdx);

            m_lidarData.row(2*pointer) = m_lidarData.row(2*fIdx);
            m_lidarData.row(2*pointer+1) = m_lidarData.row(2*fIdx+1);
            m_lidarDataCount(pointer) = m_lidarDataCount(fIdx);
            
            m_fusionData.row(2*pointer) = m_fusionData.row(2*fIdx);
            m_fusionData.row(2*pointer+1) = m_fusionData.row(2*fIdx+1);
            m_fusionDataCount(pointer) = m_fusionDataCount(fIdx);

        #ifdef SIMPLE_FUSION
            m_simpleFusionData.row(2*pointer) = m_simpleFusionData.row(2*fIdx);
            m_simpleFusionData.row(2*pointer+1) = m_simpleFusionData.row(2*fIdx+1);
            m_simpleFusionDataCount(pointer) = m_simpleFusionDataCount(fIdx);
        #endif // SIMPLE_FUSION

        #endif // SGT_EXPORT_DATA_CSV
            
            pointer++;
        }
        else
        {
        #ifdef SGT_EXPORT_DATA_CSV
            if (m_cameraDataCount(fIdx) > 0)
            {
                WriteToDataFile(fIdx); 
            }   
        #endif
        }

        if (pointer <= fIdx)
        {   
            m_fusionCones.col(fIdx).setZero();
            m_fusionConesCov.block<2,2>(2*fIdx,0).setZero();
            m_vitalityScore(fIdx) = 0;
            m_validationScore(fIdx) = 0;

        #ifdef SGT_EXPORT_DATA_CSV
            m_cameraData.block<2, DATA_SIZE_MAX>(2*fIdx, 0).setZero();
            m_lidarData.block<2, DATA_SIZE_MAX>(2*fIdx, 0).setZero();
            m_fusionData.block<2, DATA_SIZE_MAX>(2*fIdx, 0).setZero();
                
            m_cameraDataCount(fIdx) = 0;
            m_lidarDataCount(fIdx) = 0;
            m_fusionDataCount(fIdx) = 0;

        #ifdef SIMPLE_FUSION

            m_simpleFusionData.block<2, DATA_SIZE_MAX>(2*fIdx, 0).setZero();
            m_simpleFusionDataCount(fIdx) = 0;
        #endif // SIMPLE_FUSION

        #endif // SGT_EXPORT_DATA_CSV
        }
    }
    
    m_numOfCones = pointer;

    // create and publish Fusion message
    sgtdv_msgs::ConeArrPtr fusedCones( new sgtdv_msgs::ConeArr );
    sgtdv_msgs::Cone cone;
 
    fusedCones->cones.reserve(m_numOfCones);
#ifdef FUSION_CONSOLE_SHOW
    std::cout << "number of cones: " << m_numOfCones << std::endl;
    std::cout << m_fusionCones << std:: endl;
    //std::cout << "validation score:\n" << m_validationScore << std::endl;
#endif

    // publish tracked detections
    for(int i = 0; i < m_numOfCones; i++)
    {
        if (m_validationScore(i) > VALIDATION_SCORE_TH)
        {
            cone.coords.header.frame_id = m_baseFrameId;
            cone.coords.header.seq = i;
            cone.coords.header.stamp = m_stamps[i];
            cone.coords.x = m_fusionCones(0, i);
            cone.coords.y = m_fusionCones(1,i);
            cone.color = m_colors[i];

            fusedCones->cones.push_back(cone);
        }
    }
    
    m_publisher.publish(fusedCones);

#ifdef SIMPLE_FUSION
	if (simpleFusionCones->cones.size() > 0)
	{
    	m_simpleFusionPub.publish(simpleFusionCones);
	}
#endif

#ifdef SGT_DEBUG_STATE
    state.workingState = 0;
    state.numOfCones = static_cast<uint32_t>(m_numOfCones);
    m_visDebugPublisher.publish(state);
#endif
}

/*int Fusion::MinDistIdx(const Eigen::Ref<const Eigen::Matrix2Xd> &measurementSetMean,const Eigen::Ref<const Eigen::MatrixX2d>&measurementSetCov,
                        int setSize, const Eigen::Ref<const Eigen::Vector2d> &measurementMean, const Eigen::Ref<const Eigen::Matrix2d> &measurementCov)
{
    int minIdx = -1;
    float minDist = 100.0;
    if (setSize > 0)
    {
        for (int i = 0; i < setSize; i++)
        {
            float dist = MahalanDist(measurementSetMean.col(i), measurementSetCov.block<2,2>(2*i,0),
                measurementMean, measurementCov);
            if (dist <= m_distTH && dist < minDist)
            {
                minIdx = i;
                minDist = dist;
            }
        }
    }
    return minIdx;
}*/


int Fusion::MinDistIdx(const Eigen::Ref<const Eigen::Matrix2Xd> &measurementSetMean, int setSize, 
        const Eigen::Ref<const Eigen::Vector2d> &measurementMean)
// returns index of the least distant measurement in a set
// of measurements from a single measurement
{
    int minIdx = -1;
    float minDist = 100.0;
    if (setSize > 0)
    {
        for (int i = 0; i < setSize; i++)
        {
            // euclidean distance of vectors
            float dist = (measurementSetMean.col(i) - measurementMean).norm();
            //std::cout << "dist: " << dist << std::endl;
            if (dist <= m_distTH && dist < minDist)
            {
                minIdx = i;
                minDist = dist;
            }
        }
    }
    //std::cout << "minDist: " << minDist << std::endl;
    //std::cout << "minIdx: " << minIdx << std::endl;
    return minIdx;
}

/*float Fusion::MahalanDist(const Eigen::Ref<const Eigen::Vector2d> &setMean, const Eigen::Ref<const Eigen::Matrix2d> &setCov,
                        const Eigen::Ref<const Eigen::Vector2d> &obsMean, const Eigen::Ref<const Eigen::Matrix2d> &obsCov)
{
    Eigen::Vector2d diff = obsMean - setMean;
    Eigen::RowVector2d diffT = diff.transpose();
    
    Eigen::RowVector2d temp = diffT * setCov.inverse();
    double mahDist2 = temp * diff;
    float mahDist = sqrt(mahDist2);

    //std::cout << "ED = " << sqrt(diffT * diff) << std::endl;
    //std::cout << "MD = " << mahDist << std::endl;
    return mahDist;
}*/

#ifdef SGT_EXPORT_DATA_CSV
void Fusion::OpenDataFile(std::string filename)
{
    std::string pathToPackage = ros::package::getPath("fusion");
    std::string pathToFileCamera = pathToPackage + std::string("/data/" + filename + "_camera.csv");
    std::string pathToFileLidar = pathToPackage + std::string("/data/" + filename + "_lidar.csv");
    std::string pathToFileFusion = pathToPackage + std::string("/data/" + filename + "_fusion.csv");
    std::string pathToFileMap = pathToPackage + std::string("/data/" + filename + "_map.csv");

    m_cameraDataFile.open(pathToFileCamera);
    if (!m_cameraDataFile.is_open())
        ROS_ERROR_STREAM("Could not open file " << pathToFileCamera << std::endl);
    else
	    std::cout << "File " << pathToFileCamera << " opened" << std::endl;

    m_lidarDataFile.open(pathToFileLidar);
    if (!m_lidarDataFile.is_open())
        ROS_ERROR_STREAM("Could not open file " << pathToFileLidar << std::endl);
    else
            std::cout << "File " << pathToFileLidar << " opened" << std::endl;
    
    m_fusionDataFile.open(pathToFileFusion);
    if (!m_fusionDataFile.is_open())
        ROS_ERROR_STREAM("Could not open file " << pathToFileFusion << std::endl);
    else
            std::cout << "File " << pathToFileFusion << " opened" << std::endl;
    
    m_mapDataFile.open(pathToFileMap);
    if (!m_mapDataFile.is_open())
        ROS_ERROR_STREAM("Could not open file " << pathToFileMap << std::endl);

#ifdef SIMPLE_FUSION
    std::string pathToFileSimpleFusion = pathToPackage + std::string("/data/" + filename + "_simple_fusion.csv");
    m_simpleFusionDataFile.open(pathToFileSimpleFusion);
    if (!m_simpleFusionDataFile.is_open())
        ROS_ERROR_STREAM("Could not open file " << pathToFileSimpleFusion << std::endl);
#endif
}

void Fusion::WriteToDataFile(int Idx)
{
    m_cameraDataFile << m_cameraDataCount(Idx);
    m_lidarDataFile << m_lidarDataCount(Idx);
    m_fusionDataFile << m_fusionDataCount(Idx);
#ifdef SIMPLE_FUSION
    m_simpleFusionDataFile << m_simpleFusionDataCount(Idx);
#endif
    for (int i = 0; i < DATA_SIZE_MAX; i++)
    {
        m_cameraDataFile << ", " << m_cameraData(2*Idx, i);
        m_lidarDataFile << ", " << m_lidarData(2*Idx, i);
        m_fusionDataFile << ", " << m_fusionData(2*Idx, i);
    #ifdef SIMPLE_FUSION
        m_simpleFusionDataFile << ", " << m_simpleFusionData(2*Idx, i);
    #endif
    }
    
    m_cameraDataFile << "\n" << m_cameraDataCount(Idx);
    m_lidarDataFile << "\n" << m_lidarDataCount(Idx);
    m_fusionDataFile << "\n" << m_fusionDataCount(Idx);
#ifdef SIMPLE_FUSION
    m_simpleFusionDataFile << "\n" << m_simpleFusionDataCount(Idx);
#endif
    for (int i = 0; i < DATA_SIZE_MAX; i++)
    {
        m_cameraDataFile << ", " << m_cameraData(2*Idx+1, i);
        m_lidarDataFile << ", " << m_lidarData(2*Idx+1, i);
        m_fusionDataFile << ", " << m_fusionData(2*Idx+1, i);
    #ifdef SIMPLE_FUSION
        m_simpleFusionDataFile << ", " << m_simpleFusionData(2*Idx+1, i);
    #endif    
    }
    m_cameraDataFile << std::endl;
    m_lidarDataFile << std::endl;
    m_fusionDataFile << std::endl;
#ifdef SIMPLE_FUSION
    m_simpleFusionDataFile << std::endl;
#endif    
}

void Fusion::WriteMapToFile(const visualization_msgs::MarkerArray::ConstPtr &msg)
{
    int size = msg->markers.size();
    double *mapX = new double[size];
    double *mapY = new double[size];

    for (int i = 0; i < size; i++)
    {
        mapX[i] = msg->markers[i].pose.position.x;
        mapY[i] = msg->markers[i].pose.position.y;
    }

    for (int i = 0; i < size; i++)
    {
        m_mapDataFile << mapX[i] << ", ";
    }
    m_mapDataFile << std::endl;

    for (int i = 0; i < size; i++)
    {
        m_mapDataFile << mapY[i] << ", ";
    }
    m_mapDataFile << std::endl;
}

Eigen::Vector2d Fusion::TransformCoords(const Eigen::Ref<const Eigen::Vector2d> &obsBaseFrame, ros::Time stamp)
{
    geometry_msgs::PointStamped coordsChildFrame = geometry_msgs::PointStamped();
    coordsChildFrame.header.frame_id = m_baseFrameId;
    coordsChildFrame.header.stamp = stamp;
    coordsChildFrame.point.x = obsBaseFrame(0);
    coordsChildFrame.point.y = obsBaseFrame(1);
    coordsChildFrame.point.z = 0.0;
    
    geometry_msgs::PointStamped coordsParentFrame = geometry_msgs::PointStamped();
    try
    {
        m_listener.transformPoint(m_mapFrameId, coordsChildFrame, coordsParentFrame);
    }
    catch (tf::TransformException &e)
    {
        std::cout << e.what();
    }
    Eigen::Vector2d obsMapFrame(coordsParentFrame.point.x, coordsParentFrame.point.y);
    return obsMapFrame;
}

Eigen::Vector2d Fusion::TransformCoords(const sgtdv_msgs::Point2D &obs)
{
    return TransformCoords(Eigen::Vector2d(obs.x, obs.y), obs.header.stamp);
}
#endif //SGT_EXPORT_DATA_CSV
