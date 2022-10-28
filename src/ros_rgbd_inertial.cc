/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_mono_inertial.cc and ros_rgbd.cc
*
*/

#include "common.h"

using namespace std;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ImuGrabber *pImuGb): mpImuGb(pImuGb){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> imgRGBBuf, imgDBuf;
    std::mutex mBufMutex;
    ImuGrabber *mpImuGb;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD_Inertial");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

    std::string node_name = ros::this_node::getName();

    ros::NodeHandle node_handler;
    image_transport::ImageTransport image_transport(node_handler);

    std::string voc_file, settings_file;
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");       
        ros::shutdown();
        return 1;
    }

    bool enable_pangolin;
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, true);

    node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "map");
    node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");
    node_handler.param<std::string>(node_name + "/imu_frame_id", imu_frame_id, "imu");

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::IMU_RGBD;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);

    ImuGrabber imugb;
    ImageGrabber igb(&imugb);

    ros::Subscriber sub_imu = node_handler.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb);

    message_filters::Subscriber<sensor_msgs::Image> sub_rgb_img(node_handler, "/camera/rgb/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> sub_depth_img(node_handler, "/camera/depth_registered/image_raw", 100);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), sub_rgb_img, sub_depth_img);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

    setup_publishers(node_handler, image_transport, node_name);
    setup_services(node_handler, node_name);

    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    ros::spin();

    // Stop all threads
    pSLAM->Shutdown();
    ros::shutdown();

    return 0;
}

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    mBufMutex.lock();

    if (!imgRGBBuf.empty())
        imgRGBBuf.pop();
    imgRGBBuf.push(msgRGB);

    if (!imgDBuf.empty())
        imgDBuf.pop();
    imgDBuf.push(msgD);

    mBufMutex.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    return cv_ptr->image.clone();
}

void ImageGrabber::SyncWithImu()
{
    while(1)
    {
        if (!imgRGBBuf.empty() && !mpImuGb->imuBuf.empty())
        {
            cv::Mat im, depth;
            double tIm = 0;

            tIm = imgRGBBuf.front()->header.stamp.toSec();
            if (tIm > mpImuGb->imuBuf.back()->header.stamp.toSec())
                continue;
            
            this->mBufMutex.lock();
            ros::Time msg_time = imgRGBBuf.front()->header.stamp;
            im = GetImage(imgRGBBuf.front());
            imgRGBBuf.pop();
            depth = GetImage(imgDBuf.front());
            imgDBuf.pop();
            this->mBufMutex.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            vImuMeas.clear();
            Eigen::Vector3f Wbb;
            mpImuGb->mBufMutex.lock();
            if (!mpImuGb->imuBuf.empty())
            {
                // Load imu measurements from buffer
                while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec() <= tIm)
                {
                    double t = mpImuGb->imuBuf.front()->header.stamp.toSec();

                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
                    
                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);

                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    
                    Wbb << mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z;

                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();

            // ORB-SLAM3 runs in TrackRGBD()
            Sophus::SE3f Tcw = pSLAM->TrackRGBD(im, depth, tIm, vImuMeas);
            
            publish_topics(msg_time, Wbb);
        }

        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();

    return;
}