#include "slmod/slmod.h"


// // 激光雷达投影到图像
// bool SLMOD::project_3d_lidar_point_to_image(Eigen::Vector3d& temp_pt, Eigen::Matrix3d& extric_R, 
//                                             Eigen::Vector3d& extric_t, double& u, double& v){

//     // 3D point lidar -> camera 
//     Eigen::Vector3d pt_cam( extric_R * ( temp_pt ) + extric_t);

//     if(pt_cam(2) < 0.001){
//         return false;
//     }

//     PinholeCamera::Parameters params = camera->getParameters();
//     double fx = params.fx();
//     double fy = params.fy();
//     double cx = params.cx();
//     double cy = params.cy();
//     u = pt_cam(0) * fx / pt_cam(2) + cx;
//     v = pt_cam(1) * fy / pt_cam(2) + cy;

//     double used_fov_margin = 0.005;
    
//     // 如果点在图像的边缘,舍弃
//     if ((u  >= (used_fov_margin * img_width + 1)) && (std::ceil(u) < ((1 - used_fov_margin) * img_width)) &&
//         (v >= (used_fov_margin * img_height + 1)) && (std::ceil(v) < ((1 - used_fov_margin) * img_height)))
//     {
//         return true;
//     }
//     else
//     {
//         return false;
//     }

// }

void SLMOD::set_initial_state_cov(StatesGroup &state)
{
    // Set cov
    state.cov = state.cov.setIdentity() * INIT_COV;
    // state.cov.block(18, 18, 6 , 6 ) = state.cov.block(18, 18, 6 , 6 ) .setIdentity() * 0.1;
    // state.cov.block(24, 24, 5 , 5 ) = state.cov.block(24, 24, 5 , 5 ).setIdentity() * 0.001;
    state.cov.block( 0, 0, 3, 3 ) = mat_3_3::Identity() * 1e-5;   // R
    state.cov.block( 3, 3, 3, 3 ) = mat_3_3::Identity() * 1e-5;   // T
    state.cov.block( 6, 6, 3, 3 ) = mat_3_3::Identity() * 1e-5;   // vel
    state.cov.block( 9, 9, 3, 3 ) = mat_3_3::Identity() * 1e-3;   // bias_g
    state.cov.block( 12, 12, 3, 3 ) = mat_3_3::Identity() * 1e-1; // bias_a
    state.cov.block( 15, 15, 3, 3 ) = mat_3_3::Identity() * 1e-5; // Gravity
}

void SLMOD::lasermap_fov_segment()
{
    cub_needrm.clear();  
    // pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = g_state.pos_end(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = g_state.pos_end(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(g_state.pos_end(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(g_state.pos_end(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    // points_cache_collect();

    if(cub_needrm.size() > 0) 
        ikdtree.Delete_Point_Boxes(cub_needrm);

}

// project lidar frame to world
void SLMOD::pointBodyToWorld( PointType const *const pi, PointType *const po )
{
    Eigen::Vector3d p_body( pi->x, pi->y, pi->z );
    Eigen::Vector3d p_global( g_state.rot_end * ( p_body + Lidar_offset_to_IMU ) + g_state.pos_end );

    po->x = p_global( 0 );
    po->y = p_global( 1 );
    po->z = p_global( 2 );
    po->intensity = pi->intensity;
}

void SLMOD::RGBpointBodyToWorld( PointType const *const pi, pcl::PointXYZI *const po )
{
    Eigen::Vector3d p_body( pi->x, pi->y, pi->z );
    Eigen::Vector3d p_global( g_state.rot_end * ( p_body + Lidar_offset_to_IMU ) + g_state.pos_end );

    po->x = p_global( 0 );
    po->y = p_global( 1 );
    po->z = p_global( 2 );
    po->intensity = pi->intensity;

    float intensity = pi->intensity;
    intensity = intensity - std::floor( intensity );

    int reflection_map = intensity * 10000;
}

void SLMOD::lio_odom(){

    nav_msgs::Path path; // LIO 输出的路径
    // path.header.stamp = ros::Time::now(); // 后续处理时添加时间戳
    path.header.frame_id = "/world";
    /*** variables definition ***/
    Eigen::Matrix< double, DIM_OF_STATES, DIM_OF_STATES > G, H_T_H, I_STATE;
    G.setZero();
    H_T_H.setZero();
    I_STATE.setIdentity();
    cv::Mat matA1( 3, 3, CV_32F, cv::Scalar::all( 0 ) );
    cv::Mat matD1( 1, 3, CV_32F, cv::Scalar::all( 0 ) );
    cv::Mat matV1( 3, 3, CV_32F, cv::Scalar::all( 0 ) );
    cv::Mat matP( 6, 6, CV_32F, cv::Scalar::all( 0 ) );
    PointCloudXYZINormal::Ptr feats_undistort( new PointCloudXYZINormal() );
    PointCloudXYZINormal::Ptr feats_down( new PointCloudXYZINormal() );
    PointCloudXYZINormal::Ptr laserCloudOri( new PointCloudXYZINormal() );
    PointCloudXYZINormal::Ptr coeffSel( new PointCloudXYZINormal() );

    // PointCloudXYZINormal::Ptr featsArray[laserCloudNum];

    // for ( int i = 0; i < laserCloudNum; i++ )
    // {
    //     featsArray[ i ].reset( new PointCloudXYZINormal() ); // 分配内存
    // }

    std::shared_ptr<ImuProcess> p_imu(new ImuProcess());

    set_initial_state_cov(g_state);

    bool first_lio_process = true;
    LIMeasureGroup li_measures;

    // bool flg_map_initialized = false;
    bool flg_EKF_converged = false;
    double deltaR = 0.0;
    double deltaT = 0.0;
    int iterCount = 0;
    int NUM_MAX_ITERATIONS = 4; // 修改 综合考虑处理时间和精度
    int m_lio_update_point_step = 4; // 降采样 
    double m_maximum_pt_kdtree_dis = 0.5;
    double m_planar_check_dis = 0.05;
    double m_long_rang_pt_dis = 500.0;
    double m_maximum_res_dis = 0.3;
    int laserCloudSelNum = 0;
    double res_mean_last = 0;
    bool dense_map_en = true;


    while(ros::ok()){

        // 休眠 
        std::this_thread::yield();
        std::this_thread::sleep_for( std::chrono::milliseconds(1));

        // std::cout << "------------------" << std::endl;

        // 提取激光雷达和IMU数据
        pcl::PointCloud<PointType>::Ptr lidar_temp;
        std::deque<sensor_msgs::Imu::ConstPtr> imu_vec_temp;

        buffer_mutex.lock();

        // std::cout << "all ready " << sensor_data.all_ready() << std::endl;
        // std::cout << "sensor_data.compress_img_vec.size() " << sensor_data.compress_img_vec.size() << std::endl;
        // std::cout << "img_solved_num " << img_solved_num << std::endl;
        
        if(sensor_data.all_ready() && sensor_data.compress_img_vec.size() == img_solved_num){ // 所有数据就绪

          lidar_temp = sensor_data.lidar_vec.front();
          imu_vec_temp = sensor_data.imu_lio_vec;

          last_lidar_begin_time = sensor_data.lidar_begin_time;

          sensor_data.clear_all();  // 清除已经收集的数据
          img_solved_num = 0; // 特征点提取标志

        //   std::cout << "--------------------------" << std::endl;

        }
        buffer_mutex.unlock();

        if((lidar_temp != nullptr) && (imu_vec_temp.size() > 0)){ // 接收到数据

            li_measures.lidar = lidar_temp;

            // std::cout << "lidar temp size is " << li_measures.lidar->points.size() << std::endl;

            li_measures.lidar_beg_time = lidar_temp->header.stamp * NS2S; // sec
            
            // std::cout << "cuvature is " << li_measures.lidar->points.back().curvature << std::endl;
            li_measures.lidar_end_time = li_measures.lidar_beg_time + li_measures.lidar->points.back().curvature * MS2S;
            // std::cout << std::setprecision(15) << "lidar begin time is " << li_measures.lidar_beg_time << std::endl;
            // std::cout << std::setprecision(15) << "lidar end time is " << li_measures.lidar_end_time << std::endl;

            // for(auto imu_one : imu_vec_temp){

            //     std::cout << std::setprecision(15) << "imu timestamp is " << imu_one->header.stamp.toSec() << std::endl;
            // }
            li_measures.imu.clear();
            for(auto imu_one : imu_vec_temp){
                
                li_measures.imu.push_back(imu_one);
            }

            if(first_lio_process){
                
                g_states_vec.insert(std::pair<uint64_t, StatesGroup>(lidar_temp->header.stamp, g_state)); // 添加初始状态
                path.header.stamp.fromNSec(lidar_temp->header.stamp); // 确定初始的时间戳
                first_lio_process = false;
            }

            // LIO 
            // std::cout << "li measures lidar size is " << li_measures.lidar->points.size() << std::endl;
            p_imu->Process(li_measures, g_state, feats_undistort); // IMU预积分，点云去畸变
            // std::cout << "feats_undistort " << feats_undistort->size() << std::endl;

            // g_state.display(g_state, "g_state");

            StatesGroup state_propagate( g_state ); // 用于状态传播

            /*** Compute the euler angle ***/
            Eigen::Vector3d euler_cur = RotMtoEuler( g_state.rot_end );

            lasermap_fov_segment(); // 删减 ikd-tree

            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down); // 降采样

            if ( ( feats_down->points.size() > 1 ) && ( ikdtree.Root_Node == nullptr ) )
            {
                // std::vector<PointType> points_init = feats_down->points;
                ikdtree.set_downsample_param( filter_size_map_min );
                ikdtree.Build( feats_down->points ); // 初始静止
                // flg_map_initialized = true; // 该值并没有使用
                continue;
            }

            int featsFromMapNum = ikdtree.size(); // 地图点数目
            int feats_down_size = feats_down->points.size();

            /*** ICP and iterated Kalman filter update ***/
            PointCloudXYZINormal::Ptr coeffSel_tmpt( new PointCloudXYZINormal( *feats_down ) );
            PointCloudXYZINormal::Ptr feats_down_updated( new PointCloudXYZINormal( *feats_down ) );
            std::vector< double >     res_last( feats_down_size, 1000.0 ); // initial

            if ( featsFromMapNum >= 5 ) // 地图点数据大于5
            {
                // t1 = omp_get_wtime();

                // if ( m_if_publish_feature_map )
                // {
                //     PointVector().swap( ikdtree.PCL_Storage );
                //     ikdtree.flatten( ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD );
                //     featsFromMap->clear();
                //     featsFromMap->points = ikdtree.PCL_Storage;

                //     sensor_msgs::PointCloud2 laserCloudMap;
                //     pcl::toROSMsg( *featsFromMap, laserCloudMap );
                //     laserCloudMap.header.stamp = ros::Time::now(); // ros::Time().fromSec(last_timestamp_lidar);
                //     // laserCloudMap.header.stamp.fromSec(Measures.lidar_end_time); // ros::Time().fromSec(last_timestamp_lidar);
                //     laserCloudMap.header.frame_id = "world";
                //     pubLaserCloudMap.publish( laserCloudMap );
                // }

                std::vector< bool >               point_selected_surf( feats_down_size, true );
                std::vector< std::vector< int > > pointSearchInd_surf( feats_down_size );
                std::vector< PointVector >        Nearest_Points( feats_down_size );

                int  rematch_num = 0;
                bool rematch_en = false;
                flg_EKF_converged = false;
                deltaR = 0.0;
                deltaT = 0.0;
                // t2 = omp_get_wtime();
                double maximum_pt_range = 0.0;
                // cout <<"Preprocess 2 cost time: " << tim.toc("Preprocess") << endl;
                for ( iterCount = 0; iterCount < NUM_MAX_ITERATIONS; iterCount++ ) // 迭代卡尔曼滤波
                {
                    // tim.tic( "Iter" );
                    // match_start = omp_get_wtime();
                    laserCloudOri->clear();
                    coeffSel->clear();

                    /** closest surface search and residual computation **/
                    for ( int i = 0; i < feats_down_size; i += m_lio_update_point_step )
                    {
                        // double     search_start = omp_get_wtime();
                        PointType &pointOri_tmpt = feats_down->points[ i ];
                        double     ori_pt_dis =
                            sqrt( pointOri_tmpt.x * pointOri_tmpt.x + pointOri_tmpt.y * pointOri_tmpt.y + pointOri_tmpt.z * pointOri_tmpt.z );
                        maximum_pt_range = std::max( ori_pt_dis, maximum_pt_range );
                        PointType &pointSel_tmpt = feats_down_updated->points[ i ];

                        /* transform to world frame */
                        pointBodyToWorld(&pointOri_tmpt, &pointSel_tmpt ); // 转换到世界坐标系下
                        std::vector< float > pointSearchSqDis_surf;

                        auto &points_near = Nearest_Points[ i ];

                        if ( iterCount == 0 || rematch_en )
                        {
                            point_selected_surf[ i ] = true;
                            /** Find the closest surfaces in the map **/
                            ikdtree.Nearest_Search( pointSel_tmpt, NUM_MATCH_POINTS, points_near, pointSearchSqDis_surf );
                            float max_distance = pointSearchSqDis_surf[ NUM_MATCH_POINTS - 1 ]; // 5个点里面的最远点
                            //  max_distance to add residuals
                            // ANCHOR - Long range pt stragetry
                            if ( max_distance > m_maximum_pt_kdtree_dis ) // 距离大于阈值，不选择
                            {
                                point_selected_surf[ i ] = false;
                            }
                        }

                        // kdtree_search_time += omp_get_wtime() - search_start;
                        if ( point_selected_surf[ i ] == false )
                            continue;

                        // match_time += omp_get_wtime() - match_start;
                        // double pca_start = omp_get_wtime();
                        /// PCA (using minimum square method)
                        cv::Mat matA0( NUM_MATCH_POINTS, 3, CV_32F, cv::Scalar::all( 0 ) );
                        cv::Mat matB0( NUM_MATCH_POINTS, 1, CV_32F, cv::Scalar::all( -1 ) );
                        cv::Mat matX0( NUM_MATCH_POINTS, 1, CV_32F, cv::Scalar::all( 0 ) );

                        for ( int j = 0; j < NUM_MATCH_POINTS; j++ )
                        {
                            matA0.at< float >( j, 0 ) = points_near[ j ].x;
                            matA0.at< float >( j, 1 ) = points_near[ j ].y;
                            matA0.at< float >( j, 2 ) = points_near[ j ].z;
                        }

                        cv::solve( matA0, matB0, matX0, cv::DECOMP_QR ); // TODO

                        float pa = matX0.at< float >( 0, 0 );
                        float pb = matX0.at< float >( 1, 0 );
                        float pc = matX0.at< float >( 2, 0 );
                        float pd = 1;

                        float ps = sqrt( pa * pa + pb * pb + pc * pc );
                        pa /= ps;
                        pb /= ps;
                        pc /= ps;
                        pd /= ps;

                        bool planeValid = true;
                        for ( int j = 0; j < NUM_MATCH_POINTS; j++ )
                        {
                            // ANCHOR -  Planar check
                            if ( fabs( pa * points_near[ j ].x + pb * points_near[ j ].y + pc * points_near[ j ].z + pd ) >
                                 m_planar_check_dis ) // Raw 0.05
                            {
                                // ANCHOR - Far distance pt processing
                                if ( ori_pt_dis < maximum_pt_range * 0.90 || ( ori_pt_dis < m_long_rang_pt_dis ) )
                                // if(1)
                                {
                                    planeValid = false;
                                    point_selected_surf[ i ] = false;
                                    break;
                                }
                            }
                        }

                        if ( planeValid ) // 附近5个点在一个平面上
                        {
                            float pd2 = pa * pointSel_tmpt.x + pb * pointSel_tmpt.y + pc * pointSel_tmpt.z + pd;
                            float s = 1 - 0.9 * fabs( pd2 ) /
                                              sqrt( sqrt( pointSel_tmpt.x * pointSel_tmpt.x + pointSel_tmpt.y * pointSel_tmpt.y +
                                                          pointSel_tmpt.z * pointSel_tmpt.z ) );
                            // ANCHOR -  Point to plane distance
                            double acc_distance = ( ori_pt_dis < m_long_rang_pt_dis ) ? m_maximum_res_dis : 1.0;
                            if ( pd2 < acc_distance ) // 小于一定值，表明该点在该平面上，但是由于状态初始值不准，点到面存在一定的距离
                            {
                                // if(std::abs(pd2) > 5 * res_mean_last)
                                // {
                                //     point_selected_surf[i] = false;
                                //     res_last[i] = 0.0;
                                //     continue;
                                // }
                                point_selected_surf[ i ] = true;
                                coeffSel_tmpt->points[ i ].x = pa;
                                coeffSel_tmpt->points[ i ].y = pb;
                                coeffSel_tmpt->points[ i ].z = pc;
                                coeffSel_tmpt->points[ i ].intensity = pd2;
                                res_last[ i ] = std::abs( pd2 );
                            }
                            else
                            {
                                point_selected_surf[ i ] = false;
                            }
                        }
                        // pca_time += omp_get_wtime() - pca_start;
                    }
                    // tim.tic( "Stack" );
                    double total_residual = 0.0;
                    laserCloudSelNum = 0;

                    for ( int i = 0; i < coeffSel_tmpt->points.size(); i++ )
                    {
                        if ( point_selected_surf[ i ] && ( res_last[ i ] <= 2.0 ) ) // 一定是小于2.0的
                        {
                            laserCloudOri->push_back( feats_down->points[ i ] );
                            coeffSel->push_back( coeffSel_tmpt->points[ i ] );
                            total_residual += res_last[ i ];
                            laserCloudSelNum++;
                        }
                    }
                    res_mean_last = total_residual / laserCloudSelNum;

                    // match_time += omp_get_wtime() - match_start;
                    // solve_start = omp_get_wtime();

                    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
                    Eigen::MatrixXd Hsub( laserCloudSelNum, 6 );
                    Eigen::VectorXd meas_vec( laserCloudSelNum );
                    Hsub.setZero();

                    for ( int i = 0; i < laserCloudSelNum; i++ )
                    {
                        const PointType &laser_p = laserCloudOri->points[ i ];
                        Eigen::Vector3d  point_this( laser_p.x, laser_p.y, laser_p.z );
                        point_this += Lidar_offset_to_IMU; // 在IMU坐标系下的坐标
                        Eigen::Matrix3d point_crossmat;
                        point_crossmat << SKEW_SYM_MATRIX( point_this );

                        /*** get the normal vector of closest surface/corner ***/
                        const PointType &norm_p = coeffSel->points[ i ];
                        Eigen::Vector3d  norm_vec( norm_p.x, norm_p.y, norm_p.z );

                        /*** calculate the Measuremnt Jacobian matrix H ***/
                        Eigen::Vector3d A( point_crossmat * g_state.rot_end.transpose() * norm_vec );
                        Hsub.row( i ) << VEC_FROM_ARRAY( A ), norm_p.x, norm_p.y, norm_p.z;

                        /*** Measuremnt: distance to the closest surface/corner ***/
                        meas_vec( i ) = -norm_p.intensity;
                    }

                    Eigen::Vector3d                           rot_add, t_add, v_add, bg_add, ba_add, g_add;
                    Eigen::Matrix< double, DIM_OF_STATES, 1 > solution;
                    Eigen::MatrixXd                           K( DIM_OF_STATES, laserCloudSelNum );

                    // cout << ANSI_COLOR_RED_BOLD << "Run EKF uph" << ANSI_COLOR_RESET << endl;
                    auto &&Hsub_T = Hsub.transpose();
                    H_T_H.block< 6, 6 >( 0, 0 ) = Hsub_T * Hsub;
                    Eigen::Matrix< double, DIM_OF_STATES, DIM_OF_STATES > &&K_1 =
                        ( H_T_H + ( g_state.cov / LASER_POINT_COV ).inverse() ).inverse();
                    K = K_1.block< DIM_OF_STATES, 6 >( 0, 0 ) * Hsub_T;

                    auto vec = state_propagate - g_state;
                    solution = K * ( meas_vec - Hsub * vec.block< 6, 1 >( 0, 0 ) );
                    // double speed_delta = solution.block( 0, 6, 3, 1 ).norm();
                    // if(solution.block( 0, 6, 3, 1 ).norm() > 0.05 )
                    // {
                    //     solution.block( 0, 6, 3, 1 ) = solution.block( 0, 6, 3, 1 ) / speed_delta * 0.05;
                    // }

                    g_state = state_propagate + solution;
                    // print_dash_board();
                    // cout << ANSI_COLOR_RED_BOLD << "Run EKF uph, vec = " << vec.head<9>().transpose() << ANSI_COLOR_RESET << endl;
                    rot_add = solution.block< 3, 1 >( 0, 0 );
                    t_add = solution.block< 3, 1 >( 3, 0 );
                    flg_EKF_converged = false;
                    if ( ( ( rot_add.norm() * 57.3 - deltaR ) < 0.01 ) && ( ( t_add.norm() * 100 - deltaT ) < 0.015 ) )
                    {
                        flg_EKF_converged = true;
                    }

                    deltaR = rot_add.norm() * 57.3;
                    deltaT = t_add.norm() * 100;
                    

                    // printf_line;
                    g_state.last_update_time = li_measures.lidar_end_time;
                    euler_cur = RotMtoEuler( g_state.rot_end );
                    // dump_lio_state_to_log( m_lio_state_fp );

                    /*** Rematch Judgement ***/
                    rematch_en = false;
                    if ( flg_EKF_converged || ( ( rematch_num == 0 ) && ( iterCount == ( NUM_MAX_ITERATIONS - 2 ) ) ) )
                    {
                        rematch_en = true;
                        rematch_num++;
                    }

                    /*** Convergence Judgements and Covariance Update ***/
                    // if (rematch_num >= 10 || (iterCount == NUM_MAX_ITERATIONS - 1))
                    if ( rematch_num >= 2 || ( iterCount == NUM_MAX_ITERATIONS - 1 ) ) // Fast lio ori version.
                    {
                        
                        /*** Covariance Update ***/
                        G.block< DIM_OF_STATES, 6 >( 0, 0 ) = K * Hsub;
                        g_state.cov = ( I_STATE - G ) * g_state.cov;
                        // total_distance += ( g_state.pos_end - position_last ).norm();
                        // position_last = g_lio_state.pos_end;

                        // std::cout << "position: " << g_lio_state.pos_end.transpose() << " total distance: " << total_distance << std::endl;
                        
                        // solve_time += omp_get_wtime() - solve_start;
                        break;
                    }
                    // solve_time += omp_get_wtime() - solve_start;
                    // cout << "Match cost time: " << match_time * 1000.0
                    //      << ", search cost time: " << kdtree_search_time*1000.0
                    //      << ", PCA cost time: " << pca_time*1000.0
                    //      << ", solver_cost: " << solve_time * 1000.0 << endl;
                    // cout <<"Iter cost time: " << tim.toc("Iter") << endl;
                }

                // t3 = omp_get_wtime();

                /*** add new frame points to map ikdtree ***/
                PointVector points_history;
                ikdtree.acquire_removed_points( points_history );

                // memset( cube_updated, 0, sizeof( cube_updated ) );

                // for ( int i = 0; i < points_history.size(); i++ )
                // {
                //     PointType &pointSel = points_history[ i ];

                //     int cubeI = int( ( pointSel.x + 0.5 * cube_len ) / cube_len ) + laserCloudCenWidth;
                //     int cubeJ = int( ( pointSel.y + 0.5 * cube_len ) / cube_len ) + laserCloudCenHeight;
                //     int cubeK = int( ( pointSel.z + 0.5 * cube_len ) / cube_len ) + laserCloudCenDepth;

                //     if ( pointSel.x + 0.5 * cube_len < 0 )
                //         cubeI--;
                //     if ( pointSel.y + 0.5 * cube_len < 0 )
                //         cubeJ--;
                //     if ( pointSel.z + 0.5 * cube_len < 0 )
                //         cubeK--;

                //     if ( cubeI >= 0 && cubeI < laserCloudWidth && cubeJ >= 0 && cubeJ < laserCloudHeight && cubeK >= 0 && cubeK < laserCloudDepth )
                //     {
                //         int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                //         featsArray[ cubeInd ]->push_back( pointSel );
                //     }
                // }

                for ( int i = 0; i < feats_down_size; i++ )
                {
                    /* transform to world frame */
                    pointBodyToWorld( &( feats_down->points[ i ] ), &( feats_down_updated->points[ i ] ) );
                }
                // t4 = omp_get_wtime();
               
                ikdtree.Add_Points( feats_down_updated->points, true );
                
                // kdtree_incremental_time = omp_get_wtime() - t4 + readd_time + readd_box_time + delete_box_time;
                // t5 = omp_get_wtime();
            }

            // 发布数据
            /******* Publish current frame points in world coordinates:  *******/
            laserCloudFullRes2->clear();
            *laserCloudFullRes2 = dense_map_en ? ( *feats_undistort ) : ( *feats_down );

            int laserCloudFullResNum = laserCloudFullRes2->points.size();
            // std::cout << "laserCloudFullResNum " << laserCloudFullResNum << std::endl;

            pcl::PointXYZI temp_point;
            laserCloudFullResColor->clear();
            
            for ( int i = 0; i < laserCloudFullResNum; i++ )
            {
                RGBpointBodyToWorld( &laserCloudFullRes2->points[ i ], &temp_point );
                laserCloudFullResColor->push_back( temp_point );
            }
            sensor_msgs::PointCloud2 laserCloudFullRes3;
            pcl::toROSMsg( *laserCloudFullResColor, laserCloudFullRes3 );
            // laserCloudFullRes3.header.stamp = ros::Time::now(); //.fromSec(last_timestamp_lidar);
            laserCloudFullRes3.header.stamp = ros::Time().fromSec( li_measures.lidar_end_time );
            laserCloudFullRes3.header.frame_id = "world"; // world; camera_init
            pubLaserCloudFullRes.publish( laserCloudFullRes3 );
            
        }
    }
}

// void SLMOD::points_cache_collect()
// {
//     PointVector points_history;
//     ikdtree.acquire_removed_points(points_history);
//     // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
// }