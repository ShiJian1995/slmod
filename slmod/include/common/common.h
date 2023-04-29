#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>




#define USE_ikdtree
typedef pcl::PointXYZINormal PointType;
#define DIM_OF_STATES (18) // For faster speed.
#define INIT_COV (0.0001)
#define NS2S (1e-9)
#define MS2S (1e-3)
#define COV_START_ACC_DIAG 1e-1
#define COV_START_GYRO_DIAG 1e-1 // 此处需要重新设置
#define MAX_INI_COUNT (20)
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZINormal;
const inline bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};
static const Eigen::Vector3d Lidar_offset_to_IMU(0.04165, 0.02326, -0.0284); // Avia
#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]

static const Eigen::Matrix3d Eye3d(Eigen::Matrix3d::Identity());
static const Eigen::Matrix3f Eye3f(Eigen::Matrix3f::Identity());
static const Eigen::Vector3d Zero3d(0, 0, 0);
static const Eigen::Vector3f Zero3f(0, 0, 0);

#define COV_OMEGA_NOISE_DIAG 1e-1
#define COV_ACC_NOISE_DIAG 0.4
#define COV_GYRO_NOISE_DIAG 0.2

#define COV_BIAS_ACC_NOISE_DIAG 0.05
#define COV_BIAS_GYRO_NOISE_DIAG 0.1 // 需要进行相应的设置

struct DetectedObjects{

    std::vector<Object> objects;
    double stamp; // sec
};

struct SensorData{

    std::vector<pcl::PointCloud<PointType>::Ptr> lidar_vec;
    std::vector<sensor_msgs::CompressedImageConstPtr> compress_img_vec;
    std::deque<sensor_msgs::Imu::ConstPtr> imu_lio_vec; // 存放 lio 预积分 的IMU
    std::deque<sensor_msgs::Imu::ConstPtr> imu_vio_vec; // 存放 vio 预积分 的IMU
    std::vector<DetectedObjects> detected_object_vec;

    double lidar_begin_time, lidar_end_time;
    double imu_begin_time, imu_end_time;
    double compress_img_beign_time, compress_img_end_time;
    double detected_object_begin_time, detected_object_end_time;

    bool imu_ready = false;
    bool lidar_ready = false;
    bool compress_img_ready = false;
    bool detected_object_ready = false;

    void clear_all(){

        compress_img_vec.clear();
        imu_lio_vec.clear();
        imu_vio_vec.clear();
        detected_object_vec.clear();
        // 初始化激光雷达数据
        lidar_vec.clear();

        lidar_ready = false;
        imu_ready = false;
        compress_img_ready = false;
        detected_object_ready = false;
    }

    // all ready
    bool all_ready(){

        if(lidar_ready && imu_ready && detected_object_ready && compress_img_ready){ // 所有数据就绪

            return true;
        }
        else{

            return false;
        }
    }
};

struct LIMeasureGroup // Lidar data and imu dates for the curent process
{
    LIMeasureGroup()
    {
        this->lidar.reset(new PointCloudXYZINormal());
    };
    double lidar_beg_time;
    double lidar_end_time;
    PointCloudXYZINormal::Ptr lidar;
    std::deque<sensor_msgs::Imu::ConstPtr> imu;
};

struct Pose6D
{
    typedef double data_type;
    data_type offset_time;
    data_type rot[9];
    data_type acc[3];
    data_type vel[3];
    data_type pos[3];
    data_type gyr[3];
};

struct StatesGroup
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix3d rot_end;                                 // [0-2] the estimated attitude (rotation matrix) at the end lidar point
    Eigen::Vector3d pos_end;                                 // [3-5] the estimated position at the end lidar point (world frame)
    Eigen::Vector3d vel_end;                                 // [6-8] the estimated velocity at the end lidar point (world frame)
    Eigen::Vector3d bias_g;                                  // [9-11] gyroscope bias
    Eigen::Vector3d bias_a;                                  // [12-14] accelerator bias
    Eigen::Vector3d gravity;                                 // [15-17] the estimated gravity acceleration

    Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> cov; // states covariance
    double last_update_time = 0;
    double          td_ext_i2c;
    StatesGroup()
    {
        rot_end = Eigen::Matrix3d::Identity();
        pos_end = Eigen::Vector3d::Zero();
        vel_end = Eigen::Vector3d::Zero();
        bias_g = Eigen::Vector3d::Zero();
        bias_a = Eigen::Vector3d::Zero();
        gravity = Eigen::Vector3d(0.0, 0.0, 9.805);
        // gravity = Eigen::Vector3d(0.0, 9.805, 0.0);

        cov = Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Identity() * INIT_COV;
        // cov.block(18, 18, 6,6) *= 0.1;
        last_update_time = 0;
        td_ext_i2c = 0;
    }

    ~StatesGroup(){}

    StatesGroup operator+(const Eigen::Matrix<double, DIM_OF_STATES, 1> &state_add)
    {
        StatesGroup a = *this;
        // a.rot_end = this->rot_end * Sophus::SO3d::exp(vec_3(state_add(0, 0), state_add(1, 0), state_add(2, 0) ) );
        a.rot_end = this->rot_end * Exp(state_add(0), state_add(1), state_add(2));
        a.pos_end = this->pos_end + state_add.block<3, 1>(3, 0);
        a.vel_end = this->vel_end + state_add.block<3, 1>(6, 0);
        a.bias_g = this->bias_g + state_add.block<3, 1>(9, 0);
        a.bias_a = this->bias_a + state_add.block<3, 1>(12, 0);
#if ESTIMATE_GRAVITY
        a.gravity = this->gravity + state_add.block<3, 1>(15, 0);
#endif

        a.cov = this->cov;
        a.last_update_time = this->last_update_time;
#if ENABLE_CAMERA_OBS                
        //Ext camera w.r.t. IMU
        a.rot_ext_i2c = this->rot_ext_i2c * Exp(  state_add(18), state_add(19), state_add(20) );
        a.pos_ext_i2c = this->pos_ext_i2c + state_add.block<3,1>( 21, 0 );
        a.td_ext_i2c_delta = this->td_ext_i2c_delta + state_add(24);
        a.cam_intrinsic = this->cam_intrinsic + state_add.block(25, 0, 4, 1);
#endif
        return a;
    }

    StatesGroup &operator+=(const Eigen::Matrix<double, DIM_OF_STATES, 1> &state_add)
    {
        this->rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
        this->pos_end += state_add.block<3, 1>(3, 0);
        this->vel_end += state_add.block<3, 1>(6, 0);
        this->bias_g += state_add.block<3, 1>(9, 0);
        this->bias_a += state_add.block<3, 1>(12, 0);
#if ESTIMATE_GRAVITY
        this->gravity += state_add.block<3, 1>(15, 0);
#endif
#if ENABLE_CAMERA_OBS        
        //Ext camera w.r.t. IMU
        this->rot_ext_i2c = this->rot_ext_i2c * Exp(  state_add(18), state_add(19), state_add(20));
        this->pos_ext_i2c = this->pos_ext_i2c + state_add.block<3,1>( 21, 0 );
        this->td_ext_i2c_delta = this->td_ext_i2c_delta + state_add(24);
        this->cam_intrinsic = this->cam_intrinsic + state_add.block(25, 0, 4, 1);   
#endif
        return *this;
    }

    Eigen::Matrix<double, DIM_OF_STATES, 1> operator-(const StatesGroup &b)
    {
        Eigen::Matrix<double, DIM_OF_STATES, 1> a;
        Eigen::Matrix3d rotd(b.rot_end.transpose() * this->rot_end);
        a.block<3, 1>(0, 0) = SO3_LOG(rotd);
        a.block<3, 1>(3, 0) = this->pos_end - b.pos_end;
        a.block<3, 1>(6, 0) = this->vel_end - b.vel_end;
        a.block<3, 1>(9, 0) = this->bias_g - b.bias_g;
        a.block<3, 1>(12, 0) = this->bias_a - b.bias_a;
        a.block<3, 1>(15, 0) = this->gravity - b.gravity;

#if ENABLE_CAMERA_OBS    
        //Ext camera w.r.t. IMU
        Eigen::Matrix3d rotd_ext_i2c(b.rot_ext_i2c.transpose() * this->rot_ext_i2c);
        a.block<3, 1>(18, 0) = SO3_LOG(rotd_ext_i2c);
        a.block<3, 1>(21, 0) = this->pos_ext_i2c - b.pos_ext_i2c;
        a(24) = this->td_ext_i2c_delta - b.td_ext_i2c_delta;
        a.block<4, 1>(25, 0) = this->cam_intrinsic - b.cam_intrinsic;
#endif
        return a;
    }

    static void display(const StatesGroup &state, std::string str = std::string("State: "))
    {
        Eigen::Vector3d angle_axis = SO3_LOG(state.rot_end) * 57.3;
        printf("%s |", str.c_str());
        printf("[%.5f] | ", state.last_update_time);
        printf("(%.3f, %.3f, %.3f) | ", angle_axis(0), angle_axis(1), angle_axis(2));
        printf("(%.3f, %.3f, %.3f) | ", state.pos_end(0), state.pos_end(1), state.pos_end(2));
        printf("(%.3f, %.3f, %.3f) | ", state.vel_end(0), state.vel_end(1), state.vel_end(2));
        printf("(%.3f, %.3f, %.3f) | ", state.bias_g(0), state.bias_g(1), state.bias_g(2));
        printf("(%.3f, %.3f, %.3f) \r\n", state.bias_a(0), state.bias_a(1), state.bias_a(2));
    }
};

template <typename T>
auto set_pose6d(const double t, const Eigen::Matrix<T, 3, 1> &a, const Eigen::Matrix<T, 3, 1> &g,
                const Eigen::Matrix<T, 3, 1> &v, const Eigen::Matrix<T, 3, 1> &p, const Eigen::Matrix<T, 3, 3> &R)
{
    Pose6D rot_kp;
    rot_kp.offset_time = t;
    for (int i = 0; i < 3; i++)
    {
        rot_kp.acc[i] = a(i);
        rot_kp.gyr[i] = g(i);
        rot_kp.vel[i] = v(i);
        rot_kp.pos[i] = p(i);
        for (int j = 0; j < 3; j++)
            rot_kp.rot[i * 3 + j] = R(i, j);
    }
    // Eigen::Map<Eigen::Matrix3d>(rot_kp.rot, 3,3) = R;
    return std::move(rot_kp);
}

template <typename T>
void pointBodyToWorld(const Eigen::Matrix<T, 3, 3>& rot_end, const Eigen::Matrix<T, 3, 1>& pos_end, 
                    const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po)
{
    Eigen::Vector3d p_body(pi[0], pi[1], pi[2]);
    Eigen::Vector3d p_global(rot_end * (p_body + Lidar_offset_to_IMU) + pos_end);
    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

#endif