#ifndef SLMOD_H_
#define SLMOD_H_
#include <ros/ros.h>
#include <Eigen/Core>
#include <string>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
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

typedef pcl::PointXYZINormal PointType;

struct SensorData{

    pcl::PointCloud<PointType>::Ptr lidar;
    std::vector<sensor_msgs::CompressedImageConstPtr> compress_img_vec;
    std::vector<sensor_msgs::Imu::ConstPtr> imu_vec;
    std::vector<std::vector<Object>> detected_object_vec;

    double lidar_begin_time, lidar_end_time;

    void clear_all(){

        compress_img_vec.clear();
        imu_vec.clear();
        detected_object_vec.clear();
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

    // visual
    ros::Subscriber sub_compress_img;
    std::string compress_img_topic;
    void compress_img_callback(const sensor_msgs::CompressedImageConstPtr &msg);
    std::deque<sensor_msgs::CompressedImageConstPtr> compress_img_buffer;
    int img_width, img_height;


    // inertial
    ros::Subscriber sub_imu;
    std::string imu_topic;
    void imu_callback(const sensor_msgs::Imu::ConstPtr &msg); // 回调函数声明
    std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
    int imu_hz;

    // object
    ros::Subscriber sub_object;
    std::string object_topic;
    void object_callback(const yolov5_ros::Detection2DArrayConstPtr &msg);
    std::vector<Object> detected_object;
    std::deque<std::vector<Object>> detected_object_buffer;
    Object bbox_transfom(yolov5_ros::Detection2D det);
    
    
    // slmod
    SLMOD();
    ~SLMOD(){};
    void sync_multi_sensor();

};

#endif