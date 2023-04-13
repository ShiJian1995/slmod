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
    g_cam_mod.COL = img_width; 
    g_cam_mod.ROW = img_height;
    nh.param<int>("/imu/hz", imu_hz, 200);
    nh.param<int>("/camera/hz", compress_img_hz, 30); // 相机频率尽量高
    nh.param<int>("/lidar/hz", lidar_hz, 10); // 激光雷达默认10hz
    nh.param<bool>("/camera/equalize", equalize, 1);

    std::vector<double> camera_intrinsic_vec, camera_dist_coeffs_vec;
    nh.getParam("/camera/camera_intrinsic", camera_intrinsic_vec); // 相机内参
    nh.getParam("/camera/camera_dist_coeffs", camera_dist_coeffs_vec);


    g_cam_mod.camera_intrinsic = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(camera_intrinsic_vec.data());
    g_cam_mod.camera_dist_coffes = Eigen::Map<Eigen::Matrix<double, 5, 1>>(camera_dist_coeffs_vec.data());
    

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

    thread_pool_ptr->commit_task(&SLMOD::vio_odom, this); // 视觉惯性里程计 线程
    thread_pool_ptr->commit_task(&SLMOD::lio_odom, this); // 激光雷达惯性里程计 线程
    thread_pool_ptr->commit_task(&SLMOD::img_feature_track, this); // 视觉特征提取 线程
    

}

void SLMOD::sync_multi_sensor(){
    
    // 数据头对齐
    // ros::Rate rate(2000);
    bool is_aligned = false; // 传感器起始数据是否对齐

    std::cout << "sync multi sensor " << std::endl;
    while (ros::ok())
    {
        buffer_mutex.lock();

        // 将所有传感器的起始时间点对齐
        if(!is_aligned){

            while(!lidar_buffer.empty() && 
                !imu_buffer.empty() && 
                !compress_img_buffer.empty() &&
                !detected_object_buffer.empty()){ // 激光雷达队列不为空

                // 激光雷达起始时间
                double lidar_begin_time = lidar_buffer.front()->header.stamp / double (1e9);
                std::cout << "lidar begin time is " << lidar_begin_time << std::endl;
                double imu_front_time = imu_buffer.front()->header.stamp.toSec();
                double compress_img_front_time = compress_img_buffer.front()->header.stamp.toSec();
                double detected_object_front_time = detected_object_buffer.front().stamp;

                // 如果激光雷达时间最早，则完成初始化，否则将比激光雷达点起始点早的传感器数据清除
                if(lidar_begin_time > imu_front_time && lidar_begin_time > compress_img_front_time
                    && lidar_begin_time > detected_object_front_time){

                    double imu_back_time = imu_buffer.back()->header.stamp.toSec();
                    double compress_img_back_time = compress_img_buffer.back()->header.stamp.toSec();
                    double detected_object_back_time = detected_object_buffer.back().stamp;

                    if(imu_back_time < lidar_begin_time || compress_img_back_time < lidar_begin_time
                        || detected_object_back_time < lidar_begin_time){ // 等待数据

                        break;
                    }
                    else{

                        while(lidar_begin_time > imu_front_time){

                            imu_buffer.pop_front();
                            imu_front_time = imu_buffer.front()->header.stamp.toSec();
                        }

                        while(lidar_begin_time > compress_img_front_time){
                        
                            compress_img_buffer.pop_front();
                            compress_img_front_time = compress_img_buffer.front()->header.stamp.toSec();

                        }

                        while(lidar_begin_time > detected_object_front_time){

                            detected_object_buffer.pop_front();
                            detected_object_front_time = detected_object_buffer.front().stamp;
                        }
                        is_aligned = true;
                    }
                    
                }
                else{ // 弹出起始的激光雷达数据

                    lidar_buffer.pop_front();
                }
            }
        }
        else{ // 数据的起始点已经对齐

            // 提取激光雷达数据
            if(!lidar_buffer.empty()){

                sensor_data.lidar = lidar_buffer.front();
                sensor_data.lidar_ready = true;
                sensor_data.lidar_begin_time = sensor_data.lidar->header.stamp / double (1e9);
                sensor_data.lidar_end_time = sensor_data.lidar_begin_time + 
                                            sensor_data.lidar->points.end()->curvature / (1e3);
                
            }

            // 提取IMU数据
            while(!imu_buffer.empty() && !sensor_data.imu_ready){

                imu_buffer_front_stamp = imu_buffer.front()->header.stamp.toSec();
                
                if(sensor_data.imu_vec.empty()){

                    sensor_data.imu_begin_time = imu_buffer_front_stamp;
                    // sensor_data.imu_last_time = imu_buffer_front_stamp;
                    sensor_data.imu_end_time = imu_buffer_front_stamp;
                    sensor_data.imu_vec.push_back(imu_buffer.front());
                    imu_buffer.pop_front();
                }
                else{

                    // double imu_buffer_front_stamp = imu_buffer.front()->header.stamp.toSec();
                    // sensor_data.imu_end_time = sensor_data.imu_vec.back()->header.stamp.toSec();
                    // std::cout << "imu_buffer_front_stamp - sensor_imu_vec_back_stamp " << imu_buffer_front_stamp - sensor_data.imu_begin_time << std::endl;

                    if((imu_buffer_front_stamp - sensor_data.imu_begin_time) < (1.0 / lidar_hz - (0.5 / imu_hz))){

                        if((imu_buffer_front_stamp - sensor_data.imu_end_time) < (0.8 / imu_hz) || 
                            (imu_buffer_front_stamp - sensor_data.imu_end_time) > (1.2 / imu_hz)){
                        
                            ROS_ERROR_STREAM("imu data rate wrong !!");
                            // imu_buffer.pop_front();
                        }

                        sensor_data.imu_vec.push_back(imu_buffer.front());
                        imu_buffer.pop_front();
                        sensor_data.imu_end_time = imu_buffer_front_stamp;
                        // sensor_data.imu_last_time = imu_buffer_front_stamp;
                        
                    }
                    else{
                            
                        // std::cout << "imu ready !!!!!" << std::endl;
                        sensor_data.imu_ready = true;
                        std::cout << "imu data size is " << sensor_data.imu_vec.size() << std::endl;
                        // if(sensor_data.imu_vec.size() != (imu_hz / lidar_hz)){

                        //     ROS_WARN_STREAM("IMU LOST !!!");
                        // }
                    }
                    
                }


            }

            // 同步compress_img数据
            // std::cout << "compress img empty " << compress_img_buffer.empty() << std::endl;
            while(!compress_img_buffer.empty() && !sensor_data.compress_img_ready){

                compress_img_buffer_front_stamp = compress_img_buffer.front()->header.stamp.toSec();
                if(sensor_data.compress_img_vec.empty()){

                    sensor_data.compress_img_beign_time = compress_img_buffer_front_stamp;
                    sensor_data.compress_img_end_time = compress_img_buffer_front_stamp;

                    sensor_data.compress_img_vec.push_back(compress_img_buffer.front());
                    compress_img_buffer.pop_front();

                }
                else{

                    if(compress_img_buffer_front_stamp - sensor_data.compress_img_beign_time < (1.0 / lidar_hz - (0.5 / compress_img_hz))){

                        if(compress_img_buffer_front_stamp - sensor_data.compress_img_end_time < (0.8 / compress_img_hz) ||
                           compress_img_buffer_front_stamp - sensor_data.compress_img_end_time > (1.2 / compress_img_hz) ){

                            ROS_ERROR_STREAM("image data rate wrong !!");
                        }

                        sensor_data.compress_img_vec.push_back(compress_img_buffer.front());
                        compress_img_buffer.pop_front();
                        sensor_data.compress_img_end_time = compress_img_buffer_front_stamp;
                    }
                    else{

                        sensor_data.compress_img_ready = true;
                        std::cout << "compress img size " << sensor_data.compress_img_vec.size() << std::endl;
                    }
                }
            }

            // 添加目标缓存队列
            while(!detected_object_buffer.empty() && !sensor_data.detected_object_ready){

                detected_object_buffer_front_stamp = detected_object_buffer.front().stamp;

                if(sensor_data.detected_object_vec.empty()){

                    sensor_data.detected_object_begin_time = detected_object_buffer_front_stamp;
                    sensor_data.detected_object_end_time = detected_object_buffer_front_stamp;

                    sensor_data.detected_object_vec.push_back(detected_object_buffer.front());
                    detected_object_buffer.pop_front();
                }
                else{

                    if(detected_object_buffer_front_stamp - sensor_data.detected_object_begin_time < (1.0 / lidar_hz - (0.5 / compress_img_hz))){

                        if(detected_object_buffer_front_stamp - sensor_data.detected_object_end_time < (0.8 / compress_img_hz) ||
                           detected_object_buffer_front_stamp - sensor_data.detected_object_end_time > (1.2 / compress_img_hz) ){

                            ROS_ERROR_STREAM("detected object data rate wrong !!");
                        }

                        sensor_data.detected_object_vec.push_back(detected_object_buffer.front());
                        detected_object_buffer.pop_front();
                        sensor_data.detected_object_end_time = detected_object_buffer_front_stamp;
                    }
                    else{

                        sensor_data.detected_object_ready = true;
                        std::cout << "detected object size is " << sensor_data.detected_object_vec.size() << std::endl;
                    }
                }
            }
        }

        buffer_mutex.unlock();
        // 休眠 
        std::this_thread::sleep_for( std::chrono::milliseconds(1));
    }
    
}




