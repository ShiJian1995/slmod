#include "slmod.h"


// vio: 同时使用特征点和投影点，提升估计结果
void SLMOD::vio_odom(){


}


// 图像特征跟踪
void SLMOD::img_feature_track(){

    cv_bridge::CvImageConstPtr ptr;
    FeatureTracker trackerData(g_cam_mod); // vins feature tracker

    // FeatureTrackerParam params(equalize, img_height, img_width);

    while(ros::ok()){

        bool data_exist = false;
        buffer_mutex.lock(); // 缓存队列锁

        if(sensor_data.compress_img_vec.size() > img_solved_num){

            ptr = cv_bridge::toCvCopy(sensor_data.compress_img_vec.at(img_solved_num), 
                                        sensor_msgs::image_encodings::MONO8);
            // 计数
            img_solved_num++;
            data_exist = true;
        }

        buffer_mutex.unlock();
        
        if(data_exist){

            trackerData.readImage(ptr->image, ptr->header.stamp.toSec());
        }

        // 休眠 
        std::this_thread::sleep_for( std::chrono::milliseconds(1));

    }
    
}