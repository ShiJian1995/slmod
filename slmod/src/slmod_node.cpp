// 巨人从此起飞，2023.3.30

#include "slmod/slmod.h"

int main(int argc, char** argv){

    Eigen::initParallel(); // eigen 多线程初始化
    ros::init(argc, argv, "slmod");
    SLMOD slmod;  // livo

    ros::Rate rate(2000);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}