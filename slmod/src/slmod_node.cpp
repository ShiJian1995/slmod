// 巨人从此起飞，2023.3.30

#include "slmod/slmod.h"

KD_TREE ikdtree; // ikd-tree

int main(int argc, char** argv){

    Eigen::initParallel(); // eigen 多线程初始化
    ros::init(argc, argv, "slmod");
    // ros::NodeHandle nh;
    SLMOD* slmod = new SLMOD();  // livo

    ros::Rate rate(1000);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}