#include "slmod.h"


void SLMOD::lio_odom(){

    while(ros::ok()){

        pcl::PointCloud<PointType>::Ptr lidar_data;
        sensor_msgs::CompressedImageConstPtr image_data;

        buffer_mutex.lock();
        
        if(sensor_data.all_ready()){ // 所有数据就绪

          sensor_data.clear_all();  // 清除已经收集的数据
          img_solved_num = 0; // 特征点提取标志

          lidar_data = sensor_data.lidar_vec.front();

        }
        buffer_mutex.unlock();
        // 休眠 
        std::this_thread::sleep_for( std::chrono::milliseconds(1));
    }
}