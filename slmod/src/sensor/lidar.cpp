#include "slmod.h"

void SLMOD::lidar_callback(const livox_ros_driver::CustomMsg::ConstPtr &msg_raw)
{
    pcl::PointCloud<PointType>::Ptr msg(new pcl::PointCloud<PointType>());

    avia_handler(msg_raw);
    *msg = pl_full;

    // 显示点云数据
    // publish_cloud(pub_raw_lidar);

    buffer_mutex.lock();
    lidar_buffer.push_back(msg);
    buffer_mutex.unlock();
}

void SLMOD::avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    pl_full.clear();
    int plsize = msg->point_num;
    pl_full.resize(plsize);
    uint valid_num = 0;
    for(uint i=1; i<plsize; i++)
    {
        if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
        {
            valid_num ++;
    
            pl_full[i].x = msg->points[i].x;
            pl_full[i].y = msg->points[i].y;
            pl_full[i].z = msg->points[i].z;
            pl_full[i].intensity = msg->points[i].reflectivity;
            pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms
        }
    }
}

void SLMOD::publish_cloud(const ros::Publisher & pub_cloud)
{
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(pl_full, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time::now();
    laserCloudmsg.header.frame_id = "livox";
    pub_cloud.publish(laserCloudmsg);
}