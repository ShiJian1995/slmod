#include "slmod.h"

void SLMOD::imu_callback( const sensor_msgs::Imu::ConstPtr &msg){

    // sensor_msgs::Imu::Ptr msg( new sensor_msgs::Imu( *msg_raw ) );
    buffer_mutex.lock();
    imu_buffer.push_back(msg);
    buffer_mutex.unlock();

}