#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <orb_slam3_ros/SaveMap.h> // This file is created automatically, see here http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv

// ORB-SLAM3-specific libraries
#include "System.h"
#include "ImuTypes.h"

extern ORB_SLAM3::System* pSLAM;
extern ORB_SLAM3::System::eSensor sensor_type;

extern std::string world_frame_id, cam_frame_id, imu_frame_id;

extern ros::Publisher pose_pub, odom_pub, kf_markers_pub;
extern ros::Publisher tracked_mappoints_pub, all_mappoints_pub;
extern image_transport::Publisher tracking_img_pub;

void setup_services(ros::NodeHandle&, std::string);
void setup_publishers(ros::NodeHandle&, image_transport::ImageTransport&, std::string);
void publish_topics(ros::Time, Eigen::Vector3f = Eigen::Vector3f::Zero());

void publish_camera_pose(Sophus::SE3f, ros::Time);
void publish_tracking_img(cv::Mat, ros::Time);
void publish_tracked_points(std::vector<ORB_SLAM3::MapPoint*>, ros::Time);
void publish_all_points(std::vector<ORB_SLAM3::MapPoint*>, ros::Time);
void publish_tf_transform(Sophus::SE3f, string, string, ros::Time);
void publish_body_odom(Sophus::SE3f, Eigen::Vector3f, Eigen::Vector3f, ros::Time);
void publish_kf_markers(std::vector<Sophus::SE3f>, ros::Time);

bool save_map_srv(orb_slam3_ros::SaveMap::Request&, orb_slam3_ros::SaveMap::Response&);
bool save_traj_srv(orb_slam3_ros::SaveMap::Request&, orb_slam3_ros::SaveMap::Response&);

cv::Mat SE3f_to_cvMat(Sophus::SE3f);
tf::Transform SE3f_to_tfTransform(Sophus::SE3f);
sensor_msgs::PointCloud2 mappoint_to_pointcloud(std::vector<ORB_SLAM3::MapPoint*>, ros::Time);