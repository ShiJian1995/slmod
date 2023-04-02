#include "slmod.h"


void SLMOD::compress_img_callback(const sensor_msgs::CompressedImageConstPtr &msg){

    compress_img_buffer_mutex.lock();
    compress_img_buffer.push_back(msg);
    compress_img_buffer_mutex.unlock();

    // 显示图像
    // cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 );
    // cv::Mat img_temp = cv_ptr_compressed->image;
    // cv::imshow("feature point image", img_temp);
    // cv::waitKey(1);
}

void SLMOD::object_callback(const yolov5_ros::Detection2DArrayConstPtr &msg){

    detected_object.clear();

    for(auto det : msg->detections){

        detected_object.push_back(bbox_transfom(det));
    }
    detected_object_buffer.push_back(detected_object);
}

Object SLMOD::bbox_transfom(yolov5_ros::Detection2D det){

    Object obj;
    obj.label = dynamic_obj[det.name];
    obj.prob = det.score;
    // obj.rect = cv::Rect2f(det.bbox.center.x - det.bbox.size_x / 2.0, 
    //                         det.bbox.center.y - det.bbox.size_y / 2.0, 
    //                         det.bbox.center.x + det.bbox.size_x / 2.0, 
    //                         det.bbox.center.y + det.bbox.size_y / 2.0);

    obj.rect = cv::Rect2f(det.bbox.center.x - det.bbox.size_x / 2.0, 
                        det.bbox.center.y - det.bbox.size_y / 2.0, 
                        det.bbox.size_x, det.bbox.size_y);

    int point_size = det.segments.size() / 2;

    std::vector<cv::Point> seg_point(point_size);
    // std::cout << "point size is " << point_size << std::endl;
    for(int i = 0; i < point_size; i++){
        
        seg_point[i] = cv::Point(det.segments[2 * i] * img_width, det.segments[2 * i + 1] * img_height);

        // std::cout << seg_point[i] << std::endl;
    }
    std::vector<std::vector<cv::Point>> seg_points;
    seg_points.push_back(seg_point);

    obj.segs = seg_points;
    return obj;
}