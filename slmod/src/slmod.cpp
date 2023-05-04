#include "slmod/slmod.h"


SLMOD::SLMOD(){
    // std::cout << "-----------------------------" << std::endl;
    // 参数服务
    nh.param<std::string>("imu/topic", imu_topic, "/livox/imu");
    nh.param<std::string>("lidar/topic", lidar_topic, "/livox/lidar");
    nh.param<int>("/lidar/N_SCANS", N_SCANS, 6);
    nh.param<std::string>("/camera/topic", compress_img_topic, "/camera/image_color/compressed");
    nh.param<std::string>("/yolov5_ros/objects", object_topic, "/yolov5_ros/objects");
    nh.param<int>("/camera/img_width", img_width, 1280);
    nh.param<int>("/camera/img_height", img_height, 1024);

    nh.param<int>("/imu/hz", imu_hz, 200);
    nh.param<int>("/camera/hz", compress_img_hz, 30); // 相机频率尽量高
    nh.param<int>("/lidar/hz", lidar_hz, 10); // 激光雷达默认10hz
    nh.param<bool>("/camera/equalize", equalize, 1);

    std::vector<double> camera_intrinsic_vec, camera_dist_coeffs_vec;
    nh.getParam("/camera/camera_intrinsic", camera_intrinsic_vec); // 相机内参
    nh.getParam("/camera/camera_dist_coeffs", camera_dist_coeffs_vec);

    std::vector<double> camera_2_inertial_t_vec, camera_2_inertial_R_vec;
    nh.getParam("/slmod_vio/camera_2_inertial_R", camera_2_inertial_R_vec);
    nh.getParam("/slmod_vio/camera_2_inertial_t", camera_2_inertial_t_vec);
    camera_2_inertial_R = Eigen::Map<Eigen::Matrix<double, 3, 3>>(camera_2_inertial_R_vec.data());
    camera_2_inertial_t = Eigen::Map<Eigen::Vector3d>(camera_2_inertial_t_vec.data());

    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_intrinsic = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(camera_intrinsic_vec.data());
    Eigen::Matrix<double, 4, 1> camera_dist_coffes = Eigen::Map<Eigen::Matrix<double, 4, 1>>(camera_dist_coeffs_vec.data());

    double k1, k2, p1, p2, fx, fy, cx, cy;
    k1 = camera_dist_coffes(0); k2 = camera_dist_coffes(1); p1 = camera_dist_coffes(2); p2 = camera_dist_coffes(3); // 畸变
    fx = camera_intrinsic(0, 0); fy = camera_intrinsic(1, 1); cx = camera_intrinsic(0, 2); cy = camera_intrinsic(1, 2); // 内参
    PinholeCamera::Parameters params(img_width, img_height, equalize, k1, k2, p1, p2, fx, fy, cx, cy);
    camera = boost::make_shared<PinholeCamera>(PinholeCamera(params)); // 针孔相机模型

    downSizeFilterSurf.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min); // 设置降采样网格大小

//     // 回调函数
    sub_imu = nh.subscribe(imu_topic, 2000, &SLMOD::imu_callback, this, ros::TransportHints().tcpNoDelay());
    sub_lidar = nh.subscribe(lidar_topic, 100, &SLMOD::lidar_callback, this, ros::TransportHints().tcpNoDelay());
    sub_compress_img = nh.subscribe(compress_img_topic, 100, &SLMOD::compress_img_callback, this, ros::TransportHints().tcpNoDelay());
    sub_object = nh.subscribe(object_topic, 100, &SLMOD::object_callback, this, ros::TransportHints().tcpNoDelay());
// std::cout <<"-------------------------------------------" << std::endl;
    laserCloudFullRes2 = boost::make_shared<PointCloudXYZINormal>();
    laserCloudFullResColor = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    
    // std::cout <<"-------------------------------------------" << std::endl;
    // 发布器
    pub_raw_lidar = nh.advertise<sensor_msgs::PointCloud2>("/cloud_raw", 100);
    //发布feature_img，实例ptr，跟踪的特征点图，给RVIZ和调试用
    pub_match = nh.advertise<sensor_msgs::Image>("feature_img",1000);

    pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);

    // 线程池：数据对齐,数据预处理，LIO，VIO，pose graph
    thread_pool_ptr = std::make_shared<Common_tools::ThreadPool>(6, true, false); // 6 threads
    thread_pool_ptr->commit_task(&SLMOD::sync_multi_sensor, this); // 数据同步线程

    thread_pool_ptr->commit_task(&SLMOD::vio_odom, this); // 视觉惯性里程计 线程
    thread_pool_ptr->commit_task(&SLMOD::lio_odom, this); // 激光雷达惯性里程计 线程
    thread_pool_ptr->commit_task(&SLMOD::img_feature_track, this); // 视觉特征提取 线程
    // std::cout <<"-------------------------------------------" << std::endl;
    
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

            while((!lidar_buffer.empty()) && 
                (!imu_buffer.empty()) && 
                (!compress_img_buffer.empty()) &&
                (!detected_object_buffer.empty())){ // 激光雷达队列不为空

                // 激光雷达起始时间
                double lidar_begin_time = lidar_buffer.front()->header.stamp * NS2S;
                double lidar_end_time = lidar_begin_time + 1.0 / lidar_hz;
                // std::cout << "lidar begin time is " << lidar_begin_time << std::endl;
                double imu_front_time = imu_buffer.front()->header.stamp.toSec();
                double compress_img_front_time = compress_img_buffer.front()->header.stamp.toSec();
                double detected_object_front_time = detected_object_buffer.front().stamp;

                // 如果激光雷达时间最早，则完成初始化，否则将比激光雷达起始点早的传感器数据清除
                if((lidar_begin_time > imu_front_time) && (lidar_begin_time > compress_img_front_time)
                    && (lidar_begin_time > detected_object_front_time)){

                    double imu_back_time = imu_buffer.back()->header.stamp.toSec();
                    double compress_img_back_time = compress_img_buffer.back()->header.stamp.toSec();
                    double detected_object_back_time = detected_object_buffer.back().stamp;

                    if(imu_back_time < lidar_end_time || compress_img_back_time < lidar_end_time
                        || detected_object_back_time < lidar_end_time){ // 等待数据

                        break;
                    }
                    else
                    {

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
                        last_lidar_begin_time = lidar_begin_time; // 使用上一时刻的起始时间作为参考
                        is_aligned = true;
                        break;
                    }
                    
                }
                else{ // 弹出起始的激光雷达数据

                    lidar_buffer.pop_front();
                }
            }
        }
        else{ // 数据的起始点已经对齐

            // std::cout << "-------------------" << std::endl;

            // 提取激光雷达数据
            if(!lidar_buffer.empty() && !sensor_data.lidar_ready){

                sensor_data.lidar_vec.push_back(lidar_buffer.front());
                sensor_data.lidar_ready = true;
                sensor_data.lidar_begin_time = lidar_buffer.front()->header.stamp * NS2S;
                sensor_data.lidar_end_time = sensor_data.lidar_begin_time + 
                                            lidar_buffer.front()->points.end()->curvature * MS2S;
                
                lidar_buffer.pop_front();
            }

            // 提取IMU数据
            while(!imu_buffer.empty() && !sensor_data.imu_ready){

                imu_buffer_front_stamp = imu_buffer.front()->header.stamp.toSec();
                
                if(sensor_data.imu_lio_vec.empty()){ // 当IMU数据为空时
                    
                    sensor_data.imu_begin_time = imu_buffer_front_stamp;
                    // sensor_data.imu_last_time = imu_buffer_front_stamp;
                    sensor_data.imu_end_time = imu_buffer_front_stamp;
                    sensor_data.imu_lio_vec.push_back(imu_buffer.front()); // 用于 lio 预积分
                    sensor_data.imu_vio_vec.push_back(imu_buffer.front()); // 用于 vio 预积分
                    imu_buffer.pop_front();
                    
                }
                else{

                    // double imu_buffer_front_stamp = imu_buffer.front()->header.stamp.toSec();
                    // sensor_data.imu_end_time = sensor_data.imu_vec.back()->header.stamp.toSec();
                    // std::cout << "imu_buffer_front_stamp - sensor_imu_vec_back_stamp " << imu_buffer_front_stamp - sensor_data.imu_begin_time << std::endl;
                    // std::cout << std::setprecision(15) << "imu buffer time is " << imu_buffer_front_stamp << std::endl;
                    // std::cout << std::setprecision(15) << "last_lidar_end_time " << (last_lidar_begin_time ) << std::endl;
                    if((imu_buffer_front_stamp - last_lidar_begin_time) < (2.0 / lidar_hz)){

                        if((imu_buffer_front_stamp - sensor_data.imu_end_time) < (0.8 / imu_hz) || 
                            (imu_buffer_front_stamp - sensor_data.imu_end_time) > (1.2 / imu_hz)){

                            // ROS_ERROR_STREAM("imu data rate wrong !!");
                            // imu_buffer.pop_front();
                        }

                        sensor_data.imu_lio_vec.push_back(imu_buffer.front());
                        sensor_data.imu_vio_vec.push_back(imu_buffer.front());
                        imu_buffer.pop_front();
                        sensor_data.imu_end_time = imu_buffer_front_stamp;
                        // sensor_data.imu_last_time = imu_buffer_front_stamp;
                        
                    }
                    else{
                            
                        // std::cout << "imu ready !!!!!" << std::endl;
                        sensor_data.imu_ready = true;
                        // std::cout << "imu data size is " << sensor_data.imu_lio_vec.size() << std::endl;
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

                    if(compress_img_buffer_front_stamp - last_lidar_begin_time < (2.0 / lidar_hz )){

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
                        // std::cout << "compress img size " << sensor_data.compress_img_vec.size() << std::endl;
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

                    if(detected_object_buffer_front_stamp - last_lidar_begin_time < (2.0 / lidar_hz)){

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
                        // std::cout << "detected object size is " << sensor_data.detected_object_vec.size() << std::endl;
                    }
                }
            }
        }

        buffer_mutex.unlock();
        // 休眠 
        std::this_thread::yield(); // 添加之后，算法可以实时运行
        std::this_thread::sleep_for( std::chrono::milliseconds(1));

    }
    
}