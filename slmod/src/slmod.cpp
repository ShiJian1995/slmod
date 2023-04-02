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
    nh.param<int>("/imu/hz", imu_hz, 200);

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
    ros::Rate rate(2000);
    while (ros::ok())
    {
        
        if(!lidar_buffer.empty() && !imu_buffer.empty() &&
            !compress_img_buffer.empty() && !detected_object_buffer.empty()){ // 所有消息队列均不为空

            buffer_mutex.lock();
            // 激光雷达起始时间
            double lidar_begin_time = lidar_buffer.front()->points.front().curvature / double (1000);
            double lidar_end_time = lidar_buffer.front()->points.back().curvature / double (1000);

            // 查询camera和IMU
            while(!imu_buffer.empty()){

                if(imu_buffer.front()->header.stamp.toSec() > lidar_end_time){

                    lidar_buffer.pop_front();
                    break;
                }
                if(imu_buffer.back()->header.stamp.toSec() < lidar_begin_time - (1.0 / imu_hz)){

                    imu_buffer.clear();
                    break;
                }

                if(imu_buffer.front()->header.stamp.toSec() < lidar_begin_time - (1.0 / imu_hz)){

                    imu_buffer.pop_front();
                    continue;
                }
                else{
                    sensor_data.imu_vec.push_back(imu_buffer.front());
                    imu_buffer.pop_back();
                }
            }

            while(!compress_img_buffer.empty()){

                if(compress_img_buffer.front()->header.stamp.toSec() > lidar_end_time){

                    lidar_buffer.pop_front();
                    break; 
                }

                if(compress_img_buffer.back()->header.stamp.toSec() < lidar_begin_time){

                    compress_img_buffer.clear();
                    break;
                }
                if(compress_img_buffer.front()->header.stamp.toSec() < lidar_begin_time){

                    compress_img_buffer.pop_front();
                    continue;
                }
                else{
                    sensor_data.compress_img_vec.push_back(compress_img_buffer.front());
                    compress_img_buffer.pop_front();
                }
            }

            // jixu

            buffer_mutex.unlock();
        }
        
        std::cout << "sensor data size " << std::endl;
        std::cout << sensor_data.compress_img_vec.size() << std::endl;
        rate.sleep();

    }
    

}