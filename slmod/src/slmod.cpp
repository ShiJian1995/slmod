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
    nh.param<int>("/camera/hz", compress_img_hz, 30);

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
    bool is_aligned = false; // 传感器起始数据是否对齐
    while (ros::ok())
    {
        
        if(!lidar_buffer.empty() && !is_aligned){ // 激光雷达队列不为空

            buffer_mutex.lock();
            // 激光雷达起始时间
            double lidar_begin_time = lidar_buffer.front()->points.front().curvature / double (1000);
            double lidar_end_time = lidar_buffer.front()->points.back().curvature / double (1000);

            
            // 查询camera和IMU
            while(!imu_buffer.empty() && !compress_img_buffer.empty()){ // 将IMU中早于激光雷达的数据全部清除

                double imu_front_time = imu_buffer.front()->header.stamp.toSec();
                double imu_back_time = imu_buffer.back()->header.stamp.toSec();

                if(imu_front_time > lidar_end_time){ // IMU起始时间大于激光雷达点云最后时刻

                    lidar_buffer.pop_front();
                    break;
                }

                if(imu_back_time < lidar_begin_time){ // IMU 全部在激光雷达之前，全部丢弃

                    imu_buffer.clear();
                    break;
                }

                if(imu_front_time < lidar_begin_time){ // 第一个数据在激光雷达之前，丢弃该数据

                    imu_buffer.pop_front();
                    continue;
                }

                double compress_img_front_time = compress_img_buffer.front()->header.stamp.toSec();
                double compress_img_back_time = compress_img_buffer.back()->header.stamp.toSec();

                if(compress_img_front_time > lidar_end_time){

                    lidar_buffer.pop_front();
                    break; 
                }

                if(compress_img_back_time < lidar_begin_time){

                    compress_img_buffer.clear();
                    break;
                }
                if(compress_img_front_time < lidar_begin_time){

                    compress_img_buffer.pop_front();
                    continue;
                }

                is_aligned = true; // 数据对齐
            }
            //  

            buffer_mutex.unlock();
        }

        if(is_aligned){
            
        }


        rate.sleep();
    }
    

}