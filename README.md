# **fusion package**

___

&copy; **SGT Driverless**

**Authors:** Juraj Krasňanský, Patrik Knaperek

**Objective:** Fusion of cone detections from camera and lidar.

___

## Overview

Task of `fusion` node is cone detection fusion - combining input data streams from a stereocamera and a LiDAR into a single output stream of detected cones. The input data isn't equivalent, as the LiDAR cone detection currently doesn't provide cone classification. Therefore **only cone position is the subject of data fusion**. Data fusion is based on **Linear Kalman Filter** (KF), using estimation of sensors' measurement models. They allow for current measurement noise characteristics assumption which has a decisive influence on the resulting position estimation.

As the vehicle moves, a database of tracked cones that are currently seen in the field of view is updated. Each cone in the database represents a single instance of KF with 2D position state vector. The fusion cycle consists of these steps:
  1. **KF Predict** - computing KF prediction step for each cone in the database using constant velocity motion model - the current position estimate of tracked cone ($\bm x_k$) is a space transformation of the delta car-pose vector (position $\bm p$, rotation $\bm R$) as follows:
    $\begin{align}
      \hat{\bm x}_{k} & = \bm R_{k-1}^T \bm R_k \bm x_{k-1} - \bm R_k (\bm p_k - \bm p_{k-1}) \nonumber\\
      \hat{\bm P}_{k} & = {\bm A} {\bm P}_{k-1} {\bm A}^T + {\bm Q}_k \nonumber
    \end{align}$
  2. **Measurement model assignment** - based on measured 2D position of a cone ($\bm z_k'$), noise characteristics (systematic error $e$ and random error $\bm R$) are assigned to the measurement based on the measurement model. Subsequently, systematic error correction is applied:
    $\begin{equation}
      \bm z_k = \bm z_k' - \bm e \nonumber
    \end{equation}$
  3. **Tracked cone association** - based on 2D coordinates, we try to find a tracked cone corresponding to the current measurement. If unsuccessful, we add a new cone to the database (in case of camera measurement) or throw up the measurement (in case of LiDAR measurement).
  4. **KF update** - computing KF correction step, using new measurement ($\bm z_k$) and random noise from measurement model ($\bm R$)
    $\begin{align}
      {\bm K} & = ({\bm H} \hat{\bm P}_{k} {\bm H}^T + {\bm R})^{-1} {\hat{\bm P}_{k} {\bm H}^T} \nonumber\\ 
      {\bm x}_{k} & = \hat{\bm x}_{k} + {\bm K} \left({\bm z_k} - {\bm H} \hat{\bm x}_{k} \right) \nonumber\\
      {\bm P}_{k} & = (\bm I - \bm{KH}) \hat{\bm P}_{k} \nonumber
    \end{align}$
  5. **Tracked cones database update** - filter out cones that came out of FOV (based on *vitality score*)
  6. **Tracked cones validation** - publish only cones that were associated several times (based on *validation score*)

### Measurement models

Before running cone detection fusion, we need to know the characteristics of the measurements (measurement models) for stereocamera and LiDAR. The process of getting measurement models has to be done every time after a significant change in cone detection was made, especially:
  - reality: new sensor, new NN training in camera cone detection or new position estimation algorithm in lidar cone detection,
  - FSSIM: change in parameters for detections generation (file `sensors_1.yaml`).

For this purpose, use the `measurement_models` package. Move the output stored in `measurement_models/params` to `fusion/params`. 

**Before launching the fusion package, make sure there are valid configuration files in that folder which match the files included in the fusion launchfile!**

### ROS Interface

**Subscribes topics:**
* `/camera/cones` [[`sgtdv_msgs/ConeStampedArr`](../sgtdv_msgs/msg/ConeStampedArr.msg)] : detected cones from stereocamera in `camera_center` frame
* `/lidar/cones` [[`sgtdv_msgs/Point2DStampedArr`](../sgtdv_msgs/msg/Point2DStampedArr.msg)] : detected cones from LiDAR in `lidar` frame
* `/odometry/pose` [[`sgtdv_msgs/CarPose`](../sgtdv_msgs/msg/CarPose.msg)] : current car pose

*If `SGT_EXPORT_DATA_CSV` macro enabled*
* `/fssim/track/markers` [[`visualization_msgs/MarkerArray`](/opt/ros/noetic/share/visualization_msgs/msg/MarkerArray.msg)] : map (cones) coordinates (FSSIM setup)

**Published topics:**
* `/fusion/cones` [[`sgtdv_msgs/ConeWithCovStampedArr`](../sgtdv_msgs/msg/ConeWithCovStampedArr.msg)] : fused cones in `base_link` frame

*If `SGT_DEBUG_STATE` macro enabled*
* `/fusion/debug_state` [[`sgtdv_msgs/DebugState](../sgtdv_msgs/msg/DebugState.msg)] : node lifecycle information (active/inactive, number of fused cones)

**Parameters:**
* `/distance_tolerance` : [m] threshold value of distance between two measurement to be associated
* `/base_frame_id` : vehicle coordinate frame ID
* `/camera/frame_id` : camera detections coordinate frame ID
* `/camera/fov/x/min`: [m] minimum valid x position coordinate for camera detections
* `/camera/fov/x/max`: [m] maximum valid x position coordinate for camera detections
* `/camera/fov/bearing/min`: [rad] minimum valid bearing angle for camera detections
* `/camera/fov/bearing/max`: [rad] maximum valid bearing angle for camera detections
* `/lidar/frame_id` : lidar detections coordinate frame ID
* `/lidar/fov/x/min`: minimum valid x position coordinate for lidar detections
* `/lidar/fov/x/max`: maximum valid x position coordinate for lidar detections
* `/lidar/fov/y/min`: minimum valid y position coordinate for lidar detections
* `/lidar/fov/y/max`: maximum valid y position coordinate for lidar detections
* `/number_of_models`: type of measurement model
* `/vitality_score/init` : initial value of vitality score
* `/vitality_score/max` : maximum value of vitality score for cones in database (see Vitality score diagram)
* `/validation_score_th` : threshold value of validation score for tracked cone publishing

*If `SGT_EXPORT_DATA_CSV` macro enabled*
* `/data_filename` : filename core of exported CSV files
* `/map_frame` : inertial coordinate frame ID

### Related packages
  * [`camera_driver`](../camera_driver/README.md) - `/camera/cones` publisher
  * [`lidar_cone_detection`](../lidar_cone_detection/README.md) - `/lidar/cones` publisher
  * [`cone_detection_si`](../simulation_interface/cone_detection_si/README.md) - (FSSIM setup) `lidar/cones` and `/camera/cones` publisher
  * [`odometry_interface`](../odometry_interface/README.md) - `/odometry/pose` publisher
  * [`control_si`](../simulation_interface/control_si/) - (FSSIM setup) `/odometry/pose` publisher
  * [`measurement_models`](../measurement_models/README.md) - sensor data processing for computing measurement characteristics

## Compilation
* standalone
```sh
  $ catkin build fusion -DCMAKE_BUILD_TYPE=Release
```
* FSSIM setup
```sh
  $ source ${SGT_ROOT}/scripts/build_sim.sh
```
* RC car setup
```sh
  $ source ${SGT_ROOT}/scripts/build_rc.sh
```

### Compilation configuration

* [`SGT_Macros.h`](../SGT_Macros.h) :
  - `SGT_EXPORT_DATA_CSV` : export data (camera detections, lidar detections, fused detections, (FSSIM) real cone coordinates) in map frame into folder `fusion/data/`
  - `SGT_DEBUG_STATE` : publish node lifecycle information

## Launch

### FSSIM setup
```sh
  $ source ros_implementation/devel/setup.bash
  $ roslaunch fusion fusion_sim.launch
```
Besides `fusion`, following nodes will be launched:
  - `cone_detection_si` : translantes cone detection messages from FSSIM into the same types as our cone detection nodes publish (see Data flow diagram)

Requires running:
  - [`control_si`](../simulation_interface/control_si/README.md)

(check [FSSIM testing](../../doc/FSSIM_testing.md) manual for more info)

### RC car setup
```sh
  $ source ros_implementation/devel/setup.bash
  $ roslaunch fusion fusion_rc.launch
```
Besides `fusion`, following nodes will be launched:
  - `camera_driver`
  - `lidar_cone_detection`
  <!-- - `visual_odometry` (see [README for `camera_driver`](../camera_driver/README.md)) -->

Requires running:
  - [`robot_localization`](../robot_localization/README.md)
  - [`odometry_interface`](../odometry_interface/README.md)

 Notes:
  - if launching on rosbag data, the `"use_sim_time"` parameter set has to be uncommented
  - transformations from sensor frames to vehicle frame (`base_link`) must be set according to real setup; format: `args="tx ty tz rx ry rz parent_frame child_frame"`
  - transformation from `camera_center` has to have `tz=0`

### Launch configuration
* `fusion_rc.yaml`: RC car setup
* `fusion_sim.yaml` : FSSIM setup

### RViz visualization
* FSSIM setup
```sh
  $ roslaunch data_visualization data_visualization_sim.launch
```
* RC car setup
```sh
  $ roslaunch data_visualization data_visualization_rc.launch
```

## Diagrams and flowcharts

<figcaption align = "center">Fusion algorithm flowchart</figcaption>
<p align="center">
    <img src="./doc/SW flowcharts-Cone Detection Fusion.svg" width="900">
</p>

<figcaption align = "center">Cone detection data flow diagram</figcaption>
<p align="center">
    <img src="./doc/DV_architecture-cone_detection_data_flow.svg" width="900">
</p>

<figcaption align = "center">Vitality score diagram</figcaption>
<p align="center">
  <img src="./doc/SW flowcharts-Cone Detection Fusion__vitality_score.svg" height="600">
</p>