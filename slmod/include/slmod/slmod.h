#ifndef SLMOD_H_
#define SLMOD_H_
#include <ros/ros.h>
#include <Eigen/Core>
#include <string>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <yolov5_ros/Detection2DArray.h>  // 目标检测结果
#include "MOTtrack/BYTETracker.h"
#include <map>
#include <vector>
#include "tools/tools_thread_pool.hpp"
#include <cv_bridge/cv_bridge.h>
#include "feature_tracker/feature_tracker.h"
#include "camera_models/PinholeCamera.h"
// #include "color.hpp"
#include "tools/so3_math.h"
#include <nav_msgs/Path.h>
#include "tools/tools_eigen.hpp"
#include "common/common.h"
#include "kd_tree/ikd_Tree.h"


const int laserCloudWidth = 48;
const int laserCloudHeight = 48;
const int laserCloudDepth = 48;
const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth; // 这个是什么意思？

float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Process(const LIMeasureGroup &meas, StatesGroup &state, PointCloudXYZINormal::Ptr pcl_un_);
  void Reset();
  void IMU_Initial(const LIMeasureGroup &meas, StatesGroup &state, int &N);

  // Eigen::Matrix3d Exp(const Eigen::Vector3d &ang_vel, const double &dt);

  void IntegrateGyr(const std::vector<sensor_msgs::Imu::ConstPtr> &v_imu);

  void UndistortPcl(const LIMeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZINormal &pcl_in_out);
  void lic_state_propagate(const LIMeasureGroup &meas, StatesGroup &state_inout);
  void lic_point_cloud_undistort(const LIMeasureGroup &meas,  const StatesGroup &state_inout, PointCloudXYZINormal &pcl_out);
  StatesGroup imu_preintegration(const StatesGroup & state_inout, std::deque<sensor_msgs::Imu::ConstPtr> & v_imu,  double end_pose_dt = 0);
  ros::NodeHandle nh;

  void Integrate(const sensor_msgs::ImuConstPtr &imu);
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);

  Eigen::Vector3d angvel_last;
  Eigen::Vector3d acc_s_last;

//   Eigen::Matrix<double,DIM_OF_PROC_N,1> cov_proc_noise;

  Eigen::Vector3d cov_acc;
  Eigen::Vector3d cov_gyr;

  // std::ofstream fout;

 public:
  /*** Whether is the first frame, init for first frame ***/
  bool b_first_frame_ = true;
  bool imu_need_init_ = true;

  int init_iter_num = 1;
  Eigen::Vector3d mean_acc;
  Eigen::Vector3d mean_gyr;

  /*** Undistorted pointcloud ***/
  PointCloudXYZINormal::Ptr cur_pcl_un_;

  //// For timestamp usage
  sensor_msgs::ImuConstPtr last_imu_;

  /*** For gyroscope integration ***/
  double start_timestamp_;
  /// Making sure the equal size: v_imu_ and v_rot_
  std::deque<sensor_msgs::ImuConstPtr> v_imu_;
  std::vector<Eigen::Matrix3d> v_rot_pcl_;
  std::vector<Pose6D> IMU_pose;
};


// lidar-visual-imu odometry
class SLMOD{

    public:

    // common
    ros::NodeHandle nh;
    std::map<std::string, int> dynamic_obj{{"person", 0}, {"bicycle", 1}, {"car", 2}, 
                                   {"motorcycle", 3}, {"bus", 5}, {"truck", 7}};

    std::shared_ptr<Common_tools::ThreadPool> thread_pool_ptr;
    std::mutex buffer_mutex;
    SensorData sensor_data; // 一次数据的结构体
    StatesGroup g_state;
    std::map<uint64_t, StatesGroup> g_states_vec; // 保留一段时间内的数据
    void set_initial_state_cov(StatesGroup &stat);
    void lasermap_fov_segment();
    bool Localmap_Initialized = false;
    BoxPointType LocalMap_Points;
    void points_cache_collect();
    KD_TREE ikdtree; // ikd-tree
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    double filter_size_map_min = 0.3;
    
    
    // lidar
    ros::Subscriber sub_lidar;
    std::string lidar_topic;
    void lidar_callback(const livox_ros_driver::CustomMsg::ConstPtr &msg); // 回调函数声明
    std::deque<pcl::PointCloud<PointType>::Ptr> lidar_buffer;
    pcl::PointCloud<PointType> pl_full; // 平面点云
    int N_SCANS;
    void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
    void publish_cloud(const ros::Publisher &pub_cloud);
    ros::Publisher pub_raw_lidar;
    int lidar_hz;
    bool project_3d_lidar_point_to_image(Eigen::Vector3d& temp_pt, Eigen::Matrix3d& extric_R, 
                                            Eigen::Vector3d& extric_t, double& u, double& v);
    
    double cube_len = 1000.0; // from fast-lio

    int laserCloudCenWidth = 24;
    int laserCloudCenHeight = 24;
    int laserCloudCenDepth = 24;
    int FOV_RANGE = 4; // range of FOV = FOV_RANGE * cube_len

    std::vector<BoxPointType> cub_needrm;
    std::vector<BoxPointType> cub_needad;

    // visual
    ros::Subscriber sub_compress_img;
    std::string compress_img_topic;
    void compress_img_callback(const sensor_msgs::CompressedImageConstPtr &msg);
    std::deque<sensor_msgs::CompressedImageConstPtr> compress_img_buffer;
    int img_width, img_height;
    int compress_img_hz;
    double compress_img_buffer_front_stamp;
    // Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_intrinsic;
    // Eigen::Matrix<double, 5, 1> camera_dist_coffes;
    bool equalize;
    int img_solved_num = 0;
    PinholeCameraPtr camera;
    bool init_feature = false;
    int WINDOW_SIZE = 20;
    ros::Publisher pub_match;


    // inertial
    ros::Subscriber sub_imu;
    std::string imu_topic;
    void imu_callback(const sensor_msgs::Imu::ConstPtr &msg); // 回调函数声明
    std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
    int imu_hz;
    double imu_buffer_front_stamp;
    int imu_solved_num = 0;

    // object
    ros::Subscriber sub_object;
    std::string object_topic;
    void object_callback(const yolov5_ros::Detection2DArrayConstPtr &msg);
    DetectedObjects detected_objects;
    std::deque<DetectedObjects> detected_object_buffer;
    Object bbox_transfom(yolov5_ros::Detection2D det);
    double detected_object_buffer_front_stamp;
    
    
    // slmod
    SLMOD();
    ~SLMOD(){};
    void sync_multi_sensor();
    void vio_odom();
    void lio_odom();
    void img_feature_track();

    Eigen::Matrix<double, 3, 3> camera_2_inertial_R; // 相机坐标系到惯性坐标系
    Eigen::Vector3d camera_2_inertial_t;

};

#endif