/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_mono_inertial.cc
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

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> img0Buf;
    std::mutex mBufMutex;
    ImuGrabber *mpImuGb;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono_Inertial");
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
    sensor_type = ORB_SLAM3::System::IMU_MONOCULAR;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);

    ImuGrabber imugb;
    ImageGrabber igb(&imugb);

    ros::Subscriber sub_imu = node_handler.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb); 
    ros::Subscriber sub_img = node_handler.subscribe("/camera/image_raw", 100, &ImageGrabber::GrabImage, &igb);

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

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    mBufMutex.lock();
    if (!img0Buf.empty())
        img0Buf.pop();
    img0Buf.push(img_msg);
    mBufMutex.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    
    if(cv_ptr->image.type()==0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cout << "Error type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void ImageGrabber::SyncWithImu()
{
    while(1)
    {
        if (!img0Buf.empty()&&!mpImuGb->imuBuf.empty())
        {
            cv::Mat im;
            double tIm = 0;

            tIm = img0Buf.front()->header.stamp.toSec();
            if(tIm>mpImuGb->imuBuf.back()->header.stamp.toSec())
                continue;
            
            this->mBufMutex.lock();
            im = GetImage(img0Buf.front());
            ros::Time msg_time = img0Buf.front()->header.stamp;
            img0Buf.pop();
            this->mBufMutex.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            Eigen::Vector3f Wbb;
            mpImuGb->mBufMutex.lock();
            if (!mpImuGb->imuBuf.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
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

            // ORB-SLAM3 runs in TrackMonocular()
            Sophus::SE3f Tcw = pSLAM->TrackMonocular(im, tIm, vImuMeas);
            
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