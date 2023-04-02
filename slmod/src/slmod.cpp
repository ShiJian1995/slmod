#include "slmod.h"


SLMOD::SLMOD(){

    // 参数服务
    nh.param<std::string>("imu/topic", imu_topic, "/livox/imu");
    nh.param<std::string>("lidar/topic", lidar_topic, "/livox/lidar");
    nh.param<int>("/lidar/N_SCANS", N_SCANS, 6);
    nh.param<std::string>("/camera/topic", compress_img_topic, "/camera/image_color/compressed");
    nh.param<std::string>("/yolov5_ros/objects", object_topic, "/yolov5_ros/objects");
    nh.param<int>("/camera/img_width", img_width, 1280);
    nh.param<int>("/camera/img_height", img_height, 1024);

    // 回调函数
    sub_imu = nh.subscribe(imu_topic, 2000, &SLMOD::imu_callback, this, ros::TransportHints().tcpNoDelay());
    sub_lidar = nh.subscribe(lidar_topic, 100, &SLMOD::lidar_callback, this, ros::TransportHints().tcpNoDelay());
    sub_compress_img = nh.subscribe(compress_img_topic, 100, &SLMOD::compress_img_callback, this, ros::TransportHints().tcpNoDelay());
    sub_object = nh.subscribe(object_topic, 100, &SLMOD::object_callback, this, ros::TransportHints().tcpNoDelay());
    

    // 发布器
    pub_raw_lidar = nh.advertise<sensor_msgs::PointCloud2>("/cloud_raw", 100);

    // 线程池：数据对齐,数据预处理，LIO，VIO，pose graph
    thread_pool_ptr = std::make_shared<Common_tools::ThreadPool>(6, true, false); // 6 threads
    thread_pool_ptr->commit_task(&SLMOD::sync_multi_sensor, this); // 数据同步线程
    

}

void SLMOD::sync_multi_sensor(){
    
    // 数据头对齐
    
}