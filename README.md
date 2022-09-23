# ORB-SLAM3-ROS

**Ongoing development**
A ROS implementation of [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) V1.0, which focuses on the ROS part similar to the [orb_slam_2_ros](https://github.com/appliedAI-Initiative/orb_slam_2_ros) package.

This package uses ```catkin build```. Tested on Ubuntu 20.04.
## 1. Prerequisites
### Eigen3
```
sudo apt install libeigen3-dev
```
### Pangolin (might be removed in the future)
```
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake ..
make
sudo make install
```
### OpenCV
Check the OpenCV version on your computer (required at leat 3.0 as stated in the original `README.md`):
```
python3 -c "import cv2; print(cv2.__version__)" 
```
On a freshly installed Ubuntu 20.04.4 LTS with desktop image, OpenCV 4.2.0 is included so we can skip to the next step. If a newer version is required (>= 3.0), follow [installation instruction](https://docs.opencv.org/4.x/d0/d3d/tutorial_general_install.html).


## 2. Installation
```
# Clone the repo:
cd ~/catkin_ws/src
git clone https://github.com/thien94/orb_slam_3_ros.git

# Build
cd ../
catkin build
```

## 3. Run
### [EuRoC dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets):
```
# Download MH_01_easy.bag
cd ~/some/path
wget -c http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag
```
```
# Run in mono-inertial mode
roslaunch orb_slam_3_ros euroc_mono_inertial.launch

# Playback the bag file
rosbag play MH_01_easy.bag
```

## 4. Topics
### Subscribed topics
- `/camera/left/image_raw`
- `/camera/right/image_raw`
- `/imu`
### Published topics
- `/orb_slam3/camera/pose`
- `/orb_slam3/tracking_image`
- `/orb_slam3/map_points`

## To-do:
- ~~Publish basic topics (camera pose, tracking image and point cloud)~~
- Publish more topics (keyframe, loop closure etc.)
- Add RGB-D and AR nodes
- Replace Pangolin (dynamic reconfigure?)
- Remove unneccessary parts