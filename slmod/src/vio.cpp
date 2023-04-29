#include "slmod/slmod.h"


// vio: 同时使用特征点和投影点，提升估计结果
void SLMOD::vio_odom(){


}


// 图像特征跟踪
void SLMOD::img_feature_track(){

    cv_bridge::CvImageConstPtr ptr;
    FeatureTracker trackerData(camera); // vins feature tracker

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

            cv::Mat show_img = ptr->image;
            trackerData.readImage(ptr->image, ptr->header.stamp.toSec());

            //更新全局ID
            for (unsigned int i = 0;; i++)
            {
                bool completed = trackerData.updateID(i);
                if (!completed)
                    break;
            }

            sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
            sensor_msgs::ChannelFloat32 id_of_point;
            sensor_msgs::ChannelFloat32 u_of_point;
            sensor_msgs::ChannelFloat32 v_of_point;
            sensor_msgs::ChannelFloat32 velocity_x_of_point;
            sensor_msgs::ChannelFloat32 velocity_y_of_point;

            feature_points->header = ptr->header;
            feature_points->header.frame_id = "world";

            set<int> hash_ids;
            
            auto &un_pts = trackerData.cur_un_pts;
            auto &cur_pts = trackerData.cur_pts;
            auto &ids = trackerData.ids;
            auto &pts_velocity = trackerData.pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData.track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids.insert(p_id);
                    geometry_msgs::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(p_id);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                }
            }
            
            feature_points->channels.push_back(id_of_point);
            feature_points->channels.push_back(u_of_point);
            feature_points->channels.push_back(v_of_point);
            feature_points->channels.push_back(velocity_x_of_point);
            feature_points->channels.push_back(velocity_y_of_point);
            // ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());
            
            // skip the first image; since no optical speed on frist image
            if (!init_feature)//第一帧不发布
            {
                init_feature = true;
            }
            else{

                if (1)
                {
                    ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
                    //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
                    cv::Mat tmp_img = ptr->image;

                    cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);
                    //显示追踪状态，越红越好，越蓝越不行
                    for (unsigned int j = 0; j < trackerData.cur_pts.size(); j++)
                    {
                        double len = std::min(1.0, 1.0 * trackerData.track_cnt[j] / WINDOW_SIZE);
                        cv::circle(tmp_img, trackerData.cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                        //draw speed line
                        /*
                        Vector2d tmp_cur_un_pts (trackerData[i].cur_un_pts[j].x, trackerData[i].cur_un_pts[j].y);
                        Vector2d tmp_pts_velocity (trackerData[i].pts_velocity[j].x, trackerData[i].pts_velocity[j].y);
                        Vector3d tmp_prev_un_pts;
                        tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                        tmp_prev_un_pts.z() = 1;
                        Vector2d tmp_prev_uv;
                        trackerData[i].m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                        cv::line(tmp_img, trackerData[i].cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
                        */
                        //char name[10];
                        //sprintf(name, "%d", trackerData[i].ids[j]);
                        //cv::putText(tmp_img, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
                    }
                    
                    //cv::imshow("vis", stereo_img);
                    //cv::waitKey(5);
                    pub_match.publish(ptr->toImageMsg());
                }


                // 利用上述的特征点求解 VIO


            }   

        }

        // 休眠 
        std::this_thread::sleep_for( std::chrono::milliseconds(1));

    }
    
}