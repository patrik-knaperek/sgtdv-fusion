/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

/* SGT */
#include <sgtdv_msgs/DebugState.h>
#include "SGT_Utils.h"

/* Header */
#include "fusion_ros.h"

FusionROS::FusionROS(ros::NodeHandle& handle)
  /* ROS interface init */
  : cones_pub_(handle.advertise<sgtdv_msgs::ConeWithCovStampedArr>("fusion/cones", 1))
#ifdef SGT_DEBUG_STATE
  , vis_debug_pub_(handle.advertise<sgtdv_msgs::DebugState>("fusion/debug_state", 2))
#endif /* SGT_DEBUG_STATE */
#ifdef SGT_EXPORT_DATA_CSV
  , map_marker_sub_(handle.subscribe("fssim/track/markers", 1, &FusionROS::mapCallback, this))
#endif /* SGT_EXPORT_DATA_CSV */ 
  , lidar_sub_(handle.subscribe("lidar/cones", 1, &FusionROS::lidarCallback, this))
  , camera_sub_(handle.subscribe("camera/cones", 1, &FusionROS::cameraCallback, this))
  , pose_sub_(handle.subscribe("odometry/pose", 1, &FusionROS::poseCallback, this))
{
  loadParams(handle);
}

void FusionROS::loadParams(const ros::NodeHandle& handle)
{
  ROS_DEBUG("LOADING PARAMETERS");

  Fusion::Params params;

  Utils::loadParam(handle, "/base_frame_id", &base_frame_id_);
  params.base_frame_id = base_frame_id_;

  Utils::loadParam(handle, "/camera/frame_id", &params.camera_frame_id);
  Utils::loadParam(handle, "/camera/fov/x/min", &params.camera_x.min);
  Utils::loadParam(handle, "/camera/fov/x/max", &params.camera_x.max);
  Utils::loadParam(handle, "/camera/fov/bearing/max", &params.camera_bearing.max);
  Utils::loadParam(handle, "/camera/fov/bearing/min", &params.camera_bearing.min);
  Utils::loadParam(handle, "/lidar/frame_id", &params.lidar_frame_id);
  Utils::loadParam(handle, "/lidar/fov/x/min", &params.lidar_x.min);
  Utils::loadParam(handle, "/lidar/fov/x/max", &params.lidar_x.max);
  Utils::loadParam(handle, "/distance_tolerance", &params.dist_th);
  Utils::loadParam(handle,"/number_of_models", &params.n_of_models);
  Utils::loadParam(handle, "/vitality_score/init", &params.vitality_score_init);
  Utils::loadParam(handle, "/vitality_score/max", &params.vitality_score_max);
  Utils::loadParam(handle, "/validation_score_treshold", &params.validation_score_th);

  params.camera_model = Eigen::MatrixXd::Zero(params.n_of_models, 5);
  params.camera_model.block(0,0,params.n_of_models,2) 
    = Utils::loadArray(handle, std::string("/camera/offset"), params.n_of_models, 2);
  params.camera_model.block(0,2,params.n_of_models,2) 
    = Utils::loadArray(handle, std::string("/camera/covariance"), params.n_of_models, 3);

  params.lidar_model = Eigen::MatrixXd::Zero(params.n_of_models, 5);
  params.lidar_model.block(0,0,params.n_of_models,2) 
    = Utils::loadArray(handle, std::string("/lidar/offset"), params.n_of_models, 2);
  params.lidar_model.block(0,2,params.n_of_models,2) 
    = Utils::loadArray(handle, std::string("/lidar/covariance"), params.n_of_models, 3);

#ifdef SGT_EXPORT_DATA_CSV
  Utils::loadParam(handle, "/data_filename", &params.data_filename);
  Utils::loadParam(handle, "/map_frame", &params.map_frame_id);
#endif /* SGT_EXPORT_DATA_CSV */

getSensorFrameTF(&params);

fusion_obj_.setParams(params);
}

void FusionROS::getSensorFrameTF(Fusion::Params* params) const
{	
  /* get camera frame static TF*/
  if(params->base_frame_id != params->camera_frame_id)
  {
    tf::StampedTransform camera_frame_tf;
    try
    {
      listener_.lookupTransform(params->base_frame_id, params->camera_frame_id, ros::Time::now(), camera_frame_tf);
    }
    catch(const std::exception& e)
    {
      std::cout << e.what();
    }
    params->camera_static_tf_x = camera_frame_tf.getOrigin().getX();
  }
  else 
    params->camera_static_tf_x = 0.;

  /* getlidar frame static TF */
  if(params->base_frame_id != params->lidar_frame_id)
  {
    tf::StampedTransform lidar_frame_tf;
    try
    {
      listener_.lookupTransform(params->base_frame_id, params->lidar_frame_id, ros::Time::now(), lidar_frame_tf);
    }
    catch(const std::exception& e)
    {
      std::cout << e.what();
    }
    params->lidar_static_tf_x = lidar_frame_tf.getOrigin().getX();
  }
  else
    params->lidar_static_tf_x = 0.;

  ROS_DEBUG_STREAM("camera TF:\n" << params->camera_static_tf_x);
}

void FusionROS::cameraCallback(const sgtdv_msgs::ConeStampedArr::ConstPtr &msg)
{
  const int cones_count = msg->cones.size();
  if(!cones_count) return;

  geometry_msgs::PointStamped coords_msg_frame, coords_base_frame;
  sgtdv_msgs::ConeStampedArr cones_base_frame;
  sgtdv_msgs::ConeStamped cone;
  cones_base_frame.cones.reserve(cones_count);

  for(const auto &cone_it : msg->cones)
  {
    if(std::isnan(cone_it.coords.x) || std::isnan(cone_it.coords.y))
      continue;
    
    coords_msg_frame.header = cone_it.coords.header;
    coords_msg_frame.point.x = cone_it.coords.x;
    coords_msg_frame.point.y = cone_it.coords.y;

    if(coords_msg_frame.header.frame_id != base_frame_id_)
      coords_base_frame 
      = Utils::transformCoords(listener_, base_frame_id_, coords_msg_frame);
    else
      coords_base_frame = coords_msg_frame;
    
    cone.coords.header = coords_base_frame.header;
    cone.coords.x = coords_base_frame.point.x;
    cone.coords.y = coords_base_frame.point.y;
    cone.color = cone_it.color;
    cones_base_frame.cones.push_back(cone);
  }

  if(cones_base_frame.cones.size() > 0)
  {
  #ifdef SGT_DEBUG_STATE
    sgtdv_msgs::DebugState state;
    state.stamp = ros::Time::now();
    state.working_state = 1;
    vis_debug_pub_.publish(state);
  #endif

    const auto fused_cones = fusion_obj_.updateCamera(cones_base_frame);

    cones_pub_.publish(fused_cones);

  #ifdef SGT_DEBUG_STATE
    state.stamp = ros::Time::now();
    state.working_state = 0;
    state.num_of_cones = static_cast<uint32_t>(fused_cones.cones.size());
    vis_debug_pub_.publish(state);
  #endif
  }
}

void FusionROS::lidarCallback(const sgtdv_msgs::Point2DStampedArr::ConstPtr &msg)
  {
  const int points_count = msg->points.size();
  if(!points_count) return;

  geometry_msgs::PointStamped coords_msg_frame, coords_base_frame;
  sgtdv_msgs::Point2DStampedArr points_base_frame;
  sgtdv_msgs::Point2DStamped point;
  points_base_frame.points.reserve(points_count);

  for(const auto &point_it : msg->points)
  {
    coords_msg_frame.header = point_it.header;
    coords_msg_frame.point.x = point_it.x;
    coords_msg_frame.point.y = point_it.y;

    if(coords_msg_frame.header.frame_id != base_frame_id_)
      coords_base_frame 
        = Utils::transformCoords(listener_, base_frame_id_, coords_msg_frame);
    else
      coords_base_frame = coords_msg_frame;
    
    point.header = coords_base_frame.header;
    point.x = coords_base_frame.point.x;
    point.y = coords_base_frame.point.y;
    points_base_frame.points.push_back(point);
  }
  #ifdef SGT_DEBUG_STATE
    sgtdv_msgs::DebugState state;
    state.stamp = ros::Time::now();
    state.working_state = 1;
    vis_debug_pub_.publish(state);
  #endif

    const auto fused_cones = fusion_obj_.updateLidar(points_base_frame);

    cones_pub_.publish(fused_cones);

  #ifdef SGT_DEBUG_STATE
    state.stamp = ros::Time::now();
    state.working_state = 0;
    state.num_of_cones = static_cast<uint32_t>(fused_cones.cones.size());
    vis_debug_pub_.publish(state);
  #endif
}

void FusionROS::poseCallback(const sgtdv_msgs::CarPose::ConstPtr &msg)
{
  fusion_obj_.updatePose(msg);
}
