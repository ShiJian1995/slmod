#include "slmod.h"

void SLMOD::imu_callback( const sensor_msgs::Imu::ConstPtr &msg){

    // sensor_msgs::Imu::Ptr msg( new sensor_msgs::Imu( *msg_raw ) );
    imu_buffer_mutex.lock();
    imu_buffer.push_back(msg);
    imu_buffer_mutex.unlock();

}