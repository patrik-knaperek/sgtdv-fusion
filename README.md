# **Fusion package**

___

&copy; **SGT Driverless**

**Authors:** Juraj Krasňanský, Patrik Knaperek

**Objective:** Fusion of cone detections from camera and lidar.

___

### Related packages
  * `fusion_sim_interface`
  * `sensors_visualizator`
  * `calibration`
  * `camera_cone_detection`
  * `lidar_cone_detection`

### Measurement models

Before running cone detection fusion, we need to know characteristics of the measurements (measurement models) for camera and lidar. The process of getting measurement models has to be done every time after a significant change in cone detection was made, especially:
  - reality: new CNN training in camera cone detection or new position estimation algorithm in lidar cone detection,
  - FSSIM: change in parameters for detections generation (file `sensors_1.yaml`).

For this purpose, use the `calibration` package. The output is saved in `calibration/params`. **Before launching the fusion package, make sure there are valid configuration files in that folder which match the files included in the fusion launchfile!**

## Compilation
The following packages have to be built at first:
  * `sgtdv_msgs`
  * `camera_cone_detection`
  * `visual_odometry`
  * `velodyne`
  * `lidar_cone_detection`
  * `calibration`

In folder `ros_implementation/src/` run:
```
$ catkin build fusion
```

### Compilation configuration

* `SGT_Macros.h`:
  - `FUSION_CONSOLE_SHOW` : print fusion results in terminal
  - `SIMPLE_FUSION` : parallel computing of simple fusion for comparison with the main algorithm
  - `SGT_EXPORT_DATA_CSV` : export data (camera detections, lidar detections, fused detections, (FSSIM) real cone coordinates, (SIMPLE_FUSION) simple fusion detections) in map frame into folder `fusion/data/`
* `Fusion.h`
  - `MAX_TRACKED_CONES_SCORE` : maximum value of vitality score for cones in database (see Vitality score diagram)
  - `MAX_TRACKED_CONES_N` : maximum number of cones in database. In case of fusion node crashing, try to increase the number.
  - `N_OF_MODELS` : number of regions of measurement models
  - `CAMERA_X_MIN` : minimum x position coordinate for camera detections
  - `CAMERA_X_MAX` : maximum x position coordinate for camera detections
  - `LIDAR_X_MIN` : minimum x position coordinate for lidar detections
  - `LIDAR_X_MAX` : maximum x position coordinate for lidar detections
  - `VALIDATION_SCORE_TH` : treshold value of validation score for tracked cone publishing
  - `ACCURACY_CORRECTION` : correction of systematic error in position measurements based on assigned measurement model. 


## Launch
#### **FSSIM**
```
  $ source ros_implementation/devel/setup.bash
  $ roslaunch fusion fusion_sim.launch
```
Besides `fusion`, following nodes will be launched:
  - `fusion_sim_interface` : translantes cone detection messages from FSSIM into the same types as our cone detection nodes publish (see Data flow diagram)
  - `sensors_visualizator` : visualization of detections in RViz

#### **Real sensors on RC car**
```
  $ source ros_implementation/devel/setup.bash
  $ roslaunch fusion fusion_rc.launch
```
Besides `fusion`, following nodes will be launched:
  - `camera_cone_detection`
  - `lidar_cone_detection`
  - `visual_odometry` (see [README for `camera_cone_detection`](../camera_cone_detection/README.md))

 Notes:
  - if launching on rosbag data, the `"use_sim_time"` parameter set has to be uncommented
  - transformations from sensor frames to vehicle frame (`base_link`) must be set according to real setup; format: `args="tx ty tz rx ry rz parent_frame child_frame"`
  - transformation from `camera_center` has to have `tz=0`

### Launch configuration
* `fusion_rc.yaml`, `fusion_sim.yaml` :
  - `distance_tolerance` : treshold value of distance [m] between two measurement to be associated
  - `base_frame` : vehicle coordinate frame ID
  - `camera_frame` : camera detections coordinate frame ID
  - `lidar_frame` : lidar detections coordinate frame ID
  - `data_filename` : filename core of exported CSV files
  - `map_frame` : inertial coordinate frame ID


## Documentation

### Cone detection data flow diagram

<p align="left">
    <img src="./doc/fusion_diagrams-cone_detection_en.svg" width="900">
</p>

### Fusion algorithm diagram

<p align="left">
    <img src="./doc/fusion_diagrams-fusion_KF_en.svg" width="600">
</p>

### Simple fusion algorithm diagram
<p align="left">
  <img src="./doc/fusion_diagrams-fusion_simple_en.svg" width="300">
</p>

### Fusion synchronization diagram
<p align="left">
  <img src="./doc/fusion_diagrams-fusion_synch_en.svg" width="300">
</p>

### Fusion message decomposition
<p align="left">
  <img src="./doc/fusion_diagrams-fusion_msg.svg" width="500">
</p>

### Vitality score diagram
<p align="left">
  <img src="./doc/fusion_diagrams-vitality_score.svg" width="500">
</p>