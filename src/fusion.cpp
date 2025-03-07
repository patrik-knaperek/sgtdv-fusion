/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

/* C++ */
#include <fstream>

/* ROS */
#include <ros/package.h>
#include <geometry_msgs/PointStamped.h>

/* Header */
#include "fusion.h"

Fusion::Fusion()
{  
#ifdef SGT_EXPORT_DATA_CSV
  openDataFiles();
#endif /* SGT_EXPORT_DATA_CSV */
}

Fusion::~Fusion()
{
#ifdef SGT_EXPORT_DATA_CSV
  for(int i = 0; i < num_of_tracked_; i++)
  {
    if(fusion_data_[i].size() > 0)
    {
      writeToDataFile(i); 
    }
  }

  camera_data_file_.close();
  lidar_data_file_.close();
  fusion_data_file_.close();
  map_data_file_.close();
#endif /* SGT_EXPORT_DATA_CSV */
}

sgtdv_msgs::ConeWithCovStampedArr Fusion::update(const FusionMsg &fusion_msg)
{ 
  /* KF prediction step for all tracked cones */
  ROS_DEBUG("KF Predict");
  KF_obj_.updatePoseDelta();
  if(num_of_tracked_ > 0)
  {
    for(auto& cone : tracked_cones_)
    {
      KF_obj_.predict(cone.state, cone.covariance);
      cone.vitality_score -=1;
    }
  }

  static std::list<TrackedCone>::iterator associate_it;
  static Eigen::Vector2d camera_obs_act, lidar_obs_act;
  static Eigen::Matrix2d camera_cov_act, lidar_cov_act;

  /* search in CAMERA detections */
  ROS_DEBUG("searching in camera detections");
  for(const auto& observation : fusion_msg.camera_data->cones)
  {
    ROS_DEBUG_STREAM("coords: " << observation.coords);
    /* filter measurement by x axis */
    if(observation.coords.x < params_.camera_static_tf_x + params_.camera_x.min 
      || observation.coords.x > params_.camera_static_tf_x + params_.camera_x.max)
      continue;

    /* filter by bearing */
    const auto bearing = std::atan2(observation.coords.y, observation.coords.x);
    ROS_DEBUG_STREAM("\nbearing: " << bearing << "\ncolor: " << observation.color);
    if(bearing > params_.camera_bearing.max || bearing < params_.camera_bearing.min)
      continue;
    
    /* asign measurement model to measurement */
    for(int model = 0; model < params_.n_of_models; model++)
    {
      if(observation.coords.x < 
        (params_.camera_x.max - params_.camera_x.min) / params_.n_of_models * (model+1) 
        + params_.camera_static_tf_x + params_.camera_x.min)
      {
        camera_obs_act << observation.coords.x, observation.coords.y;
        camera_obs_act += params_.camera_model.block(model,0,1,2).transpose();
        camera_cov_act << params_.camera_model(model,2), params_.camera_model(model,4), 
                          params_.camera_model(model,4), params_.camera_model(model,3);
        break;
      }
    }
    
    /* associate and KF-update with tracked detections */
    if(findClosestTracked(camera_obs_act, &associate_it))
    {
      /* run KF-update with new detection */
      ROS_DEBUG_STREAM("closest:\n" << associate_it->state);
      KF_obj_.update(associate_it->state, associate_it->covariance, camera_obs_act, camera_cov_act);
      associate_it->vitality_score += (associate_it->vitality_score >= params_.vitality_score_max) ? 0 : 1;
      associate_it->validation_score += (associate_it->validation_score > params_.validation_score_th) ? 0 : 1;
    }
    else
    {
      /* add to tracked detections */
      ROS_DEBUG("Adding new tracked cone");
      tracked_cones_.emplace_back(TrackedCone(camera_obs_act, params_.vitality_score_init));
      associate_it = --tracked_cones_.end();
    
    #ifdef SGT_EXPORT_DATA_CSV
      camera_data_.push_back(std::list<Eigen::Vector2d>());
      lidar_data_.push_back(std::list<Eigen::Vector2d>());
      fusion_data_.push_back(std::list<Eigen::Vector2d>());
    #endif /* SGT_EXPORT_DATA_CSV */
    }

    /* update color and stamp information */
    associate_it->color = observation.color;
    associate_it->stamp = observation.coords.header.stamp;

  #ifdef SGT_EXPORT_DATA_CSV
    Eigen::Vector2d camera_obs_map = transformCoords(observation.coords);
    if(camera_obs_map != Eigen::Vector2d::Zero())
    {
      camera_data_.at(std::distance(tracked_cones_.begin(), associate_it)).push_back(camera_obs_map);
    }
  #endif /* SGT_EXPORT_DATA_CSV */
	}

  /* search in LIDAR detections */
  ROS_DEBUG("searching in lidar detections");
  for(const auto& observation : fusion_msg.lidar_data->points)
  {
    /* filter by x axis */
    if(observation.x < params_.lidar_static_tf_x + params_.lidar_x.min 
      || observation.x > params_.lidar_static_tf_x + params_.lidar_x.max
      || observation.y < params_.lidar_y.min || observation.y > params_.lidar_y.max)
      continue;

    /* asign measurement model to measurement */
    for(int model = 0; model < params_.n_of_models; model++)
    {
      if(observation.x < 
        (params_.lidar_x.max - params_.lidar_x.min) / params_.n_of_models * (model+1) 
        + params_.lidar_x.min + params_.lidar_static_tf_x)
      {
        lidar_obs_act << observation.x, observation.y;
        lidar_obs_act += params_.lidar_model.block(model,0,1,2).transpose();
        lidar_cov_act << params_.lidar_model(model,2), params_.lidar_model(model,4),
                         params_.lidar_model(model,4), params_.lidar_model(model,3);
        break;
      }
    }
    
    /* associate and KF-update with tracked detection */
    if(findClosestTracked(lidar_obs_act, &associate_it))
    {
      ROS_DEBUG_STREAM("closest:\n" << associate_it->state);
      KF_obj_.update(associate_it->state, associate_it->covariance, lidar_obs_act, lidar_cov_act);
      associate_it->vitality_score += (associate_it->vitality_score >= params_.vitality_score_max) ? 0 : 1;
      associate_it->validation_score += 
        (associate_it->validation_score > params_.validation_score_th) ? 0 : params_.validation_score_th;
      associate_it->stamp = observation.header.stamp;

    #ifdef SGT_EXPORT_DATA_CSV
      Eigen::Vector2d lidar_obs_map = transformCoords(observation);
      ROS_DEBUG_STREAM("transformed coords: " << lidar_obs_map);
      if(lidar_obs_map != Eigen::Vector2d::Zero())
      {
        lidar_data_.at(std::distance(tracked_cones_.begin(), associate_it)).push_back(lidar_obs_map);
      }
    #endif
    }
  }

  /* update tracked detections */
  /* - throw away detections that weren't updated several times in a row */
  ROS_DEBUG("Tracked cones:");
  for(auto cone_it = tracked_cones_.begin(); cone_it != tracked_cones_.end(); cone_it++)
  {
    ROS_DEBUG_STREAM("\n" << cone_it->state);
    if(!(cone_it->vitality_score > 0)
      || cone_it->state(0) < params_.camera_static_tf_x + params_.camera_x.min + params_.camera_model(0,0))
      {
        auto coneItTemp = cone_it--;
        tracked_cones_.erase(coneItTemp);
        
    #ifdef SGT_EXPORT_DATA_CSV
      const int idx = std::distance(tracked_cones_.begin(), cone_it);
      if(camera_data_.at(idx).size() > 0)
      {
        writeToDataFile(idx); 
      }  

      camera_data_.erase(std::next(camera_data_.begin(), idx));
      lidar_data_.erase(std::next(lidar_data_.begin(), idx)); 
      fusion_data_.erase(std::next(fusion_data_.begin(), idx)); 
    #endif /* SGT_EXPORT_DATA_CSV */
      
    }
    else
    {
    #ifdef SGT_EXPORT_DATA_CSV
      Eigen::Vector2d fusion_obs_map = transformCoords(cone_it->state.head<2>(), cone_it->stamp);
      if(fusion_obs_map != Eigen::Vector2d::Zero() && cone_it->validation_score > params_.validation_score_th)
      {
        fusion_data_.at(std::distance(tracked_cones_.begin(), cone_it)).push_back(fusion_obs_map);
      }
    #endif /* SGT_EXPORT_DATA_CSV */
    }
  }

  num_of_tracked_ = tracked_cones_.size();

  /* create and publish Fusion message */
  sgtdv_msgs::ConeWithCovStampedArr fused_cones;
  static sgtdv_msgs::ConeWithCovStamped cone;

  try
  {
    fused_cones.cones.reserve(num_of_tracked_);
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }

  ROS_DEBUG_STREAM("number of cones: " << num_of_tracked_);

  /* sort by X axis */
    tracked_cones_.sort( 
          [&](TrackedCone &a, TrackedCone &b) {
            return a.state(0) < b.state(0);
          });

  /* publish tracked detections */
  int i = 0;
  for(const auto& tracked : tracked_cones_)
  {
    if(tracked.validation_score > params_.validation_score_th)
    {
      cone.coords.header.frame_id = params_.base_frame_id;
      cone.coords.header.seq = i++;
      cone.coords.header.stamp = tracked.stamp;
      cone.coords.x = tracked.state(0);
      cone.coords.y = tracked.state(1);
      cone.covariance = {tracked.covariance(0,0), tracked.covariance(0,1), tracked.covariance(1,0), tracked.covariance(1,1)};
      cone.color = tracked.color;

      fused_cones.cones.push_back(cone);
    }
  }
  return fused_cones;
}

bool Fusion::findClosestTracked(const Eigen::Ref<const Eigen::Vector2d> &measurement, 
                                std::list<TrackedCone>::iterator *closest_it)
{
  bool success = false;
  double min_dist = std::numeric_limits<double>::max();
  if(tracked_cones_.size() > 0)
  {
    for(auto cone_it = tracked_cones_.begin(); cone_it != tracked_cones_.end(); cone_it++)
    {
      /* eucliedean distance of vectors */
      auto dist = (cone_it->state.head<2>() - measurement).norm();
      if(dist <= params_.dist_th && dist < min_dist)
      {
        if(!success) success = true;
        *closest_it = cone_it;
        min_dist = dist;
      }
    }
  }
  ROS_DEBUG_STREAM("min_dist: " << min_dist);
  return success;
}

/* TODO */
/*float Fusion::MahalanDist(const Eigen::Ref<const Eigen::Vector2d> &set_mean, const Eigen::Ref<const Eigen::Matrix2d> &set_cov,
                            const Eigen::Ref<const Eigen::Vector2d> &obs_mean, const Eigen::Ref<const Eigen::Matrix2d> &obs_cov)
{
  Eigen::Vector2d diff = obs_mean - set_mean;
  Eigen::RowVector2d diff_T = diff.transpose();

  Eigen::RowVector2d temp = diff_T * set_cov.inverse();
  double mah_dist2 = temp * diff;
  float mah_dist = sqrt(mah_dist2);

  //std::cout << "ED = " << sqrt(diffT * diff) << std::endl;
  //std::cout << "MD = " << mah_dist << std::endl;
  return mah_dist;
}*/

#ifdef SGT_EXPORT_DATA_CSV
void Fusion::openDataFiles(void)
{
  const auto path_to_package = ros::package::getPath("fusion");
  const auto path_to_camera_file = path_to_package + std::string("/data/" + params_.data_filename + "_camera.csv");
  const auto path_to_lidar_file = path_to_package + std::string("/data/" + params_.data_filename + "_lidar.csv");
  const auto path_to_fusion_file = path_to_package + std::string("/data/" + params_.data_filename + "_fusion.csv");
  const auto path_to_map_file = path_to_package + std::string("/data/" + params_.data_filename + "_map.csv");

  openFile(camera_data_file_, path_to_camera_file);
  openFile(lidar_data_file_,path_to_lidar_file);
  openFile(fusion_data_file_, path_to_fusion_file);
  openFile(map_data_file_, path_to_map_file);
}

bool Fusion::openFile(std::ofstream& file, const std::string& path)
{
  file.open(path);
  if(!file.is_open())
  {
    ROS_ERROR_STREAM("Could not open file " << path);
    return false;
  }
  else
  {
    ROS_INFO_STREAM("File " << path << " opened");
    return true;
  }
}

void Fusion::writeToDataFile(int idx)
{
  ROS_DEBUG("Writing to file");
  camera_data_file_ << camera_data_.at(idx).size();
  lidar_data_file_ << lidar_data_.at(idx).size();
  fusion_data_file_ << fusion_data_.at(idx).size();

  for(const auto& data : camera_data_.at(idx)) camera_data_file_ << ", " << data(0);
  for(const auto& data : lidar_data_.at(idx)) lidar_data_file_ << ", " << data(0);
  for(const auto& data : fusion_data_.at(idx)) fusion_data_file_ << ", " << data(0);

  camera_data_file_ << "\n" << camera_data_.at(idx).size();;
  lidar_data_file_ << "\n" << lidar_data_.at(idx).size();;
  fusion_data_file_ << "\n" << fusion_data_.at(idx).size();;

  for(const auto& data : camera_data_.at(idx)) camera_data_file_ << ", " << data(1);
  for(const auto& data : lidar_data_.at(idx)) lidar_data_file_ << ", " << data(1);
  for(const auto& data : fusion_data_.at(idx)) fusion_data_file_ << ", " << data(1);

  camera_data_file_ << std::endl;
  lidar_data_file_ << std::endl;
  fusion_data_file_ << std::endl;
}

void Fusion::writeMapToFile(const visualization_msgs::MarkerArray::ConstPtr &msg)
{
  const int size = msg->markers.size();
  std::vector<double> map_x, map_y;
  map_x.reserve(size);
  map_y.reserve(size);

  for(const auto& marker : msg->markers)
  {
    map_x.push_back(marker.pose.position.x);
    map_y.push_back(marker.pose.position.y);
  }

  for(const auto& i : map_x) map_data_file_ << i << ", ";
  map_data_file_ << std::endl;

  for(const auto& i : map_y) map_data_file_ << i << ", ";
  map_data_file_ << std::endl;
}

Eigen::Vector2d Fusion::transformCoords(const Eigen::Ref<const Eigen::Vector2d> &obs_base_frame, ros::Time stamp) const
{
  geometry_msgs::PointStamped coords_child_frame;
  coords_child_frame.header.frame_id = params_.base_frame_id;
  coords_child_frame.header.stamp = stamp;
  coords_child_frame.point.x = obs_base_frame(0);
  coords_child_frame.point.y = obs_base_frame(1);

  geometry_msgs::PointStamped coords_parent_frame;
  try
  {
    listener_.transformPoint(params_.map_frame_id, coords_child_frame, coords_parent_frame);
  }
  catch(tf::TransformException &e)
  {
    std::cout << e.what();
  }

  return Eigen::Vector2d(coords_parent_frame.point.x, coords_parent_frame.point.y);
}

Eigen::Vector2d Fusion::transformCoords(const sgtdv_msgs::Point2DStamped &obs) const
{
  return transformCoords(Eigen::Vector2d(obs.x, obs.y), obs.header.stamp);
}
#endif /* SGT_EXPORT_DATA_CSV */
