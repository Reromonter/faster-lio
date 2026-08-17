## FasterLIO (ROS 2 port)

This is the code repository of FasterLIO by Chunge Bai, Tao Xiang, Yajie Chen, Haoqian Wang, Fang Zhang and Xiang Gao.

Faster-LIO is a light-weight Lidar-inertial odometry for lidar pose tracking and point cloud mapping. It is developed on
the base of [FastLIO2](https://github.com/hku-mars/FAST_LIO) and provides about 1.5-2x speed increase. It achieves
almost 1k-2k hz for solid-state lidars and over 100 hz for a typical 32 lines spinning lidar. Please refer to [our paper](./doc/faster-lio.pdf)
for more details.

**This fork is a ROS 2 (Humble) port.** The estimator itself is untouched; everything that talked to ROS 1 was
rewritten. See [Changes in this fork](#changes-in-this-fork) for the full list. If you are looking for the original
catkin/ROS 1 version, use [upstream](https://github.com/gaoxiang12/faster-lio).

Below is the FPS tested with avia and velodyne 32 spinning lidar from FastLIO2 and [NCLT](http://robots.engin.umich.edu/nclt/), where "AMD" is tested with  AMD R7 5800X and "Intel" tested with Intel Xeon Gold 5218.

<div  align="center">  
<img src="https://github.com/gaoxiang12/faster-lio/blob/main/doc/fps-fasterlio.png" width = "1080" align=center />
</div>

<div  align="center">  
<img src="https://github.com/gaoxiang12/faster-lio/blob/main/doc/faster-lio-nclt.png" width = "1080" align=center />
</div>

## Dependency

Tested on Ubuntu 22.04 with ROS 2 Humble.

1. ROS 2 (humble), plus `ros-humble-pcl-conversions`, `ros-humble-tf2-ros`, `ros-humble-rosbag2-cpp`
   (and `ros-humble-rosbag2-storage-mcap` if your bags are MCAP)
2. glog: ```sudo apt-get install libgoogle-glog-dev```
3. gflags: ```sudo apt-get install libgflags-dev```
4. eigen: ```sudo apt-get install libeigen3-dev```
5. pcl: ```sudo apt-get install libpcl-dev```
6. tbb: ```sudo apt-get install libtbb-dev```
7. yaml-cpp: ```sudo apt-get install libyaml-cpp-dev```

## Compile

The package is `ament_cmake` and builds with colcon. The bundled Livox message definitions
(`thirdparty/livox_ros_driver`) are a separate ROS 2 interface package, and **colcon does not descend into a package
directory**, so it has to be moved next to `faster_lio` in the workspace rather than left nested inside it:

```bash
mkdir -p ~/ros2_ws/src
cp -r faster-lio ~/ros2_ws/src/faster_lio
mv ~/ros2_ws/src/faster_lio/thirdparty/livox_ros_driver ~/ros2_ws/src/livox_ros_driver
rm -rf ~/ros2_ws/src/faster_lio/thirdparty

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to faster_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`laser_mapping.cc` instantiates deep Eigen/ikfom template chains and is memory-hungry to compile; on small machines (or
under qemu for cross-arch images) pass `--parallel-workers 1`.

Build options:

- `-DWITH_IVOX_NODE_TYPE_PHC=ON` — build with PHC iVox instead of the default linear iVox. The iVox type is fixed at
  compile time.
- `-DFASTER_LIO_DEBUG_SYMBOLS=ON` — re-enable `-g -ggdb` in Release builds. Upstream had this on unconditionally, which
  makes the templates in `laser_mapping.cc` emit ~150 MB of debug info and pushes link time into the tens of minutes.
  It is off by default here; turn it on only when you actually need to debug the estimator.

After the build you get `libfaster_lio.so` in `install/faster_lio/lib` and two executables in
`install/faster_lio/lib/faster_lio`, runnable with `ros2 run`.

## Prepare the datasets

Download the avia/nclt bags in your computer:

- [avia bags](https://drive.google.com/drive/folders/1YL5MQVYgAM8oAWUm7e3OGXZBPKkanmY1?usp=sharing)
- [nclt bags](https://drive.google.com/drive/folders/1VBK5idI1oyW0GC_I_Hxh63aqam3nocNK)

The link of the dataset is outdated.  Please use the NCLT bags in SAD datasets: 
BaiduYun: https://pan.baidu.com/s/1ELOcF1UTKdfiKBAaXnE8sQ?pwd=feky access code: feky
OneDrive：https://1drv.ms/u/s!AgNFVSzSYXMahcEZejoUwCaHRcactQ?e=YsOYy2

These are ROS 1 bags — convert them to rosbag2 (e.g. with `rosbags-convert`) before use.

## Run FasterLIO

The two entry points keep their upstream split, but they are configured differently: **offline reads a plain YAML file
directly, online reads ROS 2 parameters.**

### Offline mode

`run_mapping_offline` opens a rosbag2 with `rosbag2_cpp::Reader` and replays it through the estimator as fast as it can.
It reads the flat YAML files in `config/` via yaml-cpp (unchanged from upstream), *not* a ROS 2 params file. `--bag_file`
is a rosbag2 **directory** (the one containing `metadata.yaml`):

```bash
ros2 run faster_lio run_mapping_offline --bag_file /path/to/your_avia_bag --config_file ./config/avia.yaml
```

for avia. For NCLT, just run:

```bash
ros2 run faster_lio run_mapping_offline --bag_file /path/to/your_nclt_bag --config_file ./config/velodyne.yaml
```

Messages are dispatched by topic *type* (`livox_ros_driver/msg/CustomMsg`, `sensor_msgs/msg/PointCloud2`,
`sensor_msgs/msg/Imu`), so the topic names in the bag do not matter. FasterLIO will print the FPS and time usage when it
quits:

```shell
I0216 17:16:05.286536 26492 run_mapping_offline.cc:89] Faster LIO average FPS: 1884.6
I0216 17:16:05.286549 26492 run_mapping_offline.cc:91] save trajectory to: ./src/fast_lio2/Log/faster_lio/20120615.tum
I0216 17:16:05.286706 26492 utils.h:52] >>> ===== Printing run time =====
I0216 17:16:05.286711 26492 utils.h:54] > [     IVox Add Points ] average time usage: 0.0147311 ms , called times: 6373
I0216 17:16:05.286721 26492 utils.h:54] > [     Incremental Mapping ] average time usage: 0.0271787 ms , called times: 6373
I0216 17:16:05.286731 26492 utils.h:54] > [     ObsModel (IEKF Build Jacobian) ] average time usage: 0.00745852 ms , called times: 25040
I0216 17:16:05.286752 26492 utils.h:54] > [     ObsModel (Lidar Match) ] average time usage: 0.0298004 ms , called times: 25040
I0216 17:16:05.286775 26492 utils.h:54] > [ Downsample PointCloud ] average time usage: 0.0224052 ms , called times: 6373
I0216 17:16:05.286784 26492 utils.h:54] > [ IEKF Solve and Update ] average time usage: 0.342008 ms , called times: 6373
I0216 17:16:05.286792 26492 utils.h:54] > [ Laser Mapping Single Run ] average time usage: 0.530618 ms , called times: 6387
I0216 17:16:05.286800 26492 utils.h:54] > [ Preprocess (Livox) ] average time usage: 0.0267813 ms , called times: 6387
I0216 17:16:05.286808 26492 utils.h:54] > [ Undistort Pcl ] average time usage: 0.0810455 ms , called times: 6375
I0216 17:16:05.286816 26492 utils.h:59] >>> ===== Printing run time end =====
```

Point clouds will be saved to PCD/scans.pcd by default. Note that these output paths are relative to `ROOT_DIR`, which
CMake bakes in as the source directory at build time — keep `Log/` and `PCD/` around, or pass absolute paths.

### Online mode

`run_mapping_online` is a normal ROS 2 node (node name `laser_mapping`). Parameters come from the node's parameter
interface, so pass them with `--ros-args --params-file`; gflags flags are still accepted and must come *before*
`--ros-args`:

```bash
ros2 run faster_lio run_mapping_online \
  -traj_log_file=/tmp/traj.txt \
  --ros-args --params-file /path/to/your_params.yaml
```

Then play a rosbag2 in another terminal with `ros2 bag play`.

No ROS 2 params files are shipped here — the `config/*.yaml` files are the offline format and cannot be passed to
`--params-file`. A params file mirrors their layout, but nested keys become dot-separated parameter names
(`common: {lid_topic: ...}` → `common.lid_topic`) and the whole thing must be wrapped in the ROS 2 preamble:

```yaml
/**:
  ros__parameters:
    common:
      lid_topic: /livox/lidar
      imu_topic: /livox/imu
      time_sync_en: false
    preprocess:
      lidar_type: 1
      scan_line: 6
      blind: 4.0
      time_scale: 1.0e-3
    # ...
```

Two things to watch when converting a `config/*.yaml` by hand:

- **Types are strict.** ROS 2 has no float parameter type, so everything upstream declared as `float` is declared as a
  `double` here (`esti_plane_threshold`, `mapping.det_range`, `preprocess.time_scale`, `ivox_grid_resolution`). Write
  `det_range: 50.0`, not `50` — an int where a double is declared is a hard failure at startup.
- **Topics are absolute, outputs are relative.** `common.lid_topic` / `common.imu_topic` are used verbatim, while the
  publishers use relative names and therefore land under the node's namespace.

Published topics (relative — upstream published these as absolute `/cloud_registered`, `/Odometry`, `/path`, …):

| topic | type |
| --- | --- |
| `cloud_registered` | `sensor_msgs/msg/PointCloud2` |
| `cloud_registered_body` | `sensor_msgs/msg/PointCloud2` |
| `cloud_registered_effect_world` | `sensor_msgs/msg/PointCloud2` |
| `odometry` (was `/Odometry`) | `nav_msgs/msg/Odometry` |
| `path` | `nav_msgs/msg/Path` |

Subscriptions use `rclcpp::SensorDataQoS()` (best effort, keep-last 200 for lidar / 2000 for IMU), which is compatible
with both reliable and best-effort publishers.

The node also broadcasts `tf_world_frame_` → `tf_imu_frame_` on `/tf`. Set the new `publish.tf_en: false` parameter when
something else in your system already owns that edge.

## Changes in this fork

Build system:

- catkin → `ament_cmake`; `cmake/packages.cmake`, `src/CMakeLists.txt` and `app/CMakeLists.txt` folded into one
  top-level `CMakeLists.txt`. Plain-cmake (non-ROS) builds are no longer supported.
- `thirdparty/livox_ros_driver` converted to a `rosidl_generate_interfaces` message-only package (`std_msgs/Header` in
  `CustomMsg.msg`, `member_of_group` rosidl_interface_packages). Must be built as a sibling package, see
  [Compile](#compile).
- `-g -ggdb` removed from the default Release flags, behind the `FASTER_LIO_DEBUG_SYMBOLS` option.
- Installs the library, both executables (into `lib/faster_lio`) and `config/`.

ROS 1 → ROS 2 API:

- `LaserMapping` now derives from `rclcpp::Node`; `InitROS(ros::NodeHandle&)` → `InitROS()`, and the publisher arguments
  dropped off the `Publish*` methods since they use the members.
- Parameters read via `declare_parameter` instead of `nh.param`, with `/` replaced by `.` in every name. New parameter:
  `publish.tf_en`.
- `ros::Subscriber`/`ros::Publisher` → typed `rclcpp::Subscription`/`rclcpp::Publisher`. The lidar subscription is split
  into `sub_pcl_livox_` / `sub_pcl_std_` because the two are now distinct types.
- `tf::TransformBroadcaster` (a function-local static) → an owned `tf2_ros::TransformBroadcaster` member.
- `msg/Pose6D.msg` deleted — it was only ever used internally, so it is a plain `struct faster_lio::Pose6D` now.
- `ros::Time::toSec()` / `ros::Time().fromSec()` → `common::ToSec()` / `common::FromSec()` helpers on
  `builtin_interfaces::msg::Time`.
- `boost::array` → `std::array`; dropped the `eigen_conversions` and `unsupported/Eigen/ArpackSupport` includes.
- `ROS_WARN` → `LOG(WARNING)`, `ROS_ASSERT` → `assert`.
- Offline replay moved from `rosbag::View` to `rosbag2_cpp::Reader` with explicit `rclcpp::Serialization`
  deserialization per message type.
- Both executables call `rclcpp::init_and_remove_ros_arguments()` first and hand the *remaining* argv to gflags.
  Parsing raw argv with gflags instead makes it consume or reject `--ros-args`, and the node silently comes up with
  default parameters.

Fixes made along the way:

- `imu_processing.hpp` and `use-ikfom.hpp` define free functions and out-of-class methods in headers; they are `inline`
  now, which is what the multi-TU ament build requires.
- `SyncPackages()` takes `mtx_buffer_` — it reads the buffers the subscription callbacks write.
- `HesaiHandler` returns early on an empty cloud; the loops below it index `points[0]` and `points[plsize - 1]`.
- `PublishOdometry` fills the covariance before publishing, not after.

Not ported: `launch/*.launch` and `docker/` are the upstream ROS 1 / noetic files. They are left in the tree for
reference but are neither installed nor usable as-is.

# Acknowledgements

- We thank the authors of [FastLIO2](https://github.com/hku-mars/FAST_LIO), LOAM for their great jobs.
- Please cite our work if you are using FasterLIO in academic work. Bibtex is provided here (early-access version):

```
@ARTICLE{9718203,  
author={Bai, Chunge and Xiao, Tao and Chen, Yajie and Wang, Haoqian and Zhang, Fang and Gao, Xiang},  
journal={IEEE Robotics and Automation Letters},   
title={Faster-LIO: Lightweight Tightly Coupled Lidar-Inertial Odometry Using Parallel Sparse Incremental Voxels},   
year={2022},  
volume={7},  
number={2},  
pages={4861-4868},  
doi={10.1109/LRA.2022.3152830}}
```


- This work is supported by Idriver+ Technologies Co. Ltd.

# Known Issues

- iVox is somehow sensitive to the voxel size. Please enlarge the voxel size if FasterLIO is not stable in your
  application.
- set -march=native in cmake may cause a core dump in the PCL point cloud deconstruction step (Ubuntu 20.04, gcc 11, AMD
  5800X)(Ubuntu 18.04, gcc 9, i7-10750H), see
  https://stackoverflow.com/questions/61278204/segmentation-fault-when-deallocating-pclpointcloudpclpointxyzptr
