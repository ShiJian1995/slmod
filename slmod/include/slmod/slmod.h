#ifndef SLMOD_H_
#define SLMOD_H_
#include <ros/ros.h>
#include <Eigen/Core>
#include <string>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <livox_ros_driver/CustomMsg.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <yolov5_ros/Detection2DArray.h>  // 目标检测结果
#include "BYTETracker.h"
#include <map>
#include <vector>
#include "tools_thread_pool.hpp"
#include <cv_bridge/cv_bridge.h>
#include "feature_tracker.h"
#include "PinholeCamera.h"

typedef pcl::PointXYZINormal PointType;

struct DetectedObjects{

    std::vector<Object> objects;
    double stamp; // sec
};

struct SensorData{

    std::vector<pcl::PointCloud<PointType>::Ptr> lidar_vec;
    std::vector<sensor_msgs::CompressedImageConstPtr> compress_img_vec;
    std::vector<sensor_msgs::Imu::ConstPtr> imu_vec;
    std::vector<DetectedObjects> detected_object_vec;

    double lidar_begin_time, lidar_end_time;
    double imu_begin_time, imu_end_time;
    double compress_img_beign_time, compress_img_end_time;
    double detected_object_begin_time, detected_object_end_time;

    bool imu_ready = false;
    bool lidar_ready = false;
    bool compress_img_ready = false;
    bool detected_object_ready = false;

    void clear_all(){

        compress_img_vec.clear();
        imu_vec.clear();
        detected_object_vec.clear();
        // 初始化激光雷达数据
        lidar_vec.clear();

        lidar_ready = false;
        imu_ready = false;
        compress_img_ready = false;
        detected_object_ready = false;
    }

    // all ready
    bool all_ready(){

        if(lidar_ready && imu_ready && detected_object_ready && compress_img_ready){ // 所有数据就绪

            return true;
        }
        else{

            return false;
        }
    }
};


// lidar-visual-imu odometry
class SLMOD{

    public:

    // common
    ros::NodeHandle nh;
    std::map<std::string, int> dynamic_obj{{"person", 0}, {"bicycle", 1}, {"car", 2}, 
                                   {"motorcycle", 3}, {"bus", 5}, {"truck", 7}};

    std::shared_ptr<Common_tools::ThreadPool> thread_pool_ptr;
    std::mutex buffer_mutex;
    SensorData sensor_data; // 一次数据的结构体

    // lidar
    ros::Subscriber sub_lidar;
    std::string lidar_topic;
    void lidar_callback(const livox_ros_driver::CustomMsg::ConstPtr &msg); // 回调函数声明
    std::deque<pcl::PointCloud<PointType>::Ptr> lidar_buffer;
    pcl::PointCloud<PointType> pl_full; // 平面点云
    int N_SCANS;
    void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
    void publish_cloud(const ros::Publisher &pub_cloud);
    ros::Publisher pub_raw_lidar;
    int lidar_hz;

    // visual
    ros::Subscriber sub_compress_img;
    std::string compress_img_topic;
    void compress_img_callback(const sensor_msgs::CompressedImageConstPtr &msg);
    std::deque<sensor_msgs::CompressedImageConstPtr> compress_img_buffer;
    int img_width, img_height;
    int compress_img_hz;
    double compress_img_buffer_front_stamp;
    // Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_intrinsic;
    // Eigen::Matrix<double, 5, 1> camera_dist_coffes;
    bool equalize;
    int img_solved_num = 0;
    PinholeCameraPtr camera;
    bool init_feature = false;
    int WINDOW_SIZE = 20;
    ros::Publisher pub_match;


    // inertial
    ros::Subscriber sub_imu;
    std::string imu_topic;
    void imu_callback(const sensor_msgs::Imu::ConstPtr &msg); // 回调函数声明
    std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
    int imu_hz;
    double imu_buffer_front_stamp;
    int imu_solved_num = 0;

    // object
    ros::Subscriber sub_object;
    std::string object_topic;
    void object_callback(const yolov5_ros::Detection2DArrayConstPtr &msg);
    DetectedObjects detected_objects;
    std::deque<DetectedObjects> detected_object_buffer;
    Object bbox_transfom(yolov5_ros::Detection2D det);
    double detected_object_buffer_front_stamp;
    
    
    // slmod
    SLMOD();
    ~SLMOD(){};
    void sync_multi_sensor();
    void vio_odom();
    void lio_odom();
    void img_feature_track();

};

#endif