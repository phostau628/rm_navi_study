//1：缓存点云,将最近一次的点云缓存起来,用于后续的激光雷达FOV分割,缓存的点云数量为points_cache_size,缓存的点云为points_history,当需要更新本地地图时,将points_history中的点云加入删除列表,最后调用kdtree的删除函数删除点云
void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    points_cache_size = points_history.size();
}

//2：激光雷达FOV分割,根据当前位姿,分割出当前FOV内的点云,并更新本地地图,如果当前FOV边界距离当前位姿小于某个阈值,则更新本地地图,并将需要删除的点云加入删除列表,最后调用kdtree的删除函数删除点云
void lasermap_fov_segment()
{
    cub_needrm.shrink_to_fit();
    V3D pos_LiD;
    if (use_imu_as_input)
    {
        pos_LiD = kf_input.x_.pos + kf_input.x_.rot * Lidar_T_wrt_IMU;
    }
    else
    {
        pos_LiD = kf_output.x_.pos + kf_output.x_.rot * Lidar_T_wrt_IMU;
    }
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
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
            cub_needrm.emplace_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.emplace_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;
    points_cache_collect();
    if(cub_needrm.size() > 0) int kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
}
//3：加载点云文件,将点云文件加载到内存中,并转换为ROS消息格式,返回点云指针
PointCloudXYZI::Ptr loadPointcloudFromPcd(const std::string &filename){

    pcl::io::loadPCDFile(filename, cloudBlob);
    pcl::fromPCLPointCloud2(cloudBlob, *cloud);
    pcl::toROSMsg(*cloud,pcd_map_);
    return cloud;
}

void publish_init_kdtree(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes)
{
    int size_init_ikdtree = ikdtree.size();
    PointCloudXYZI::Ptr   laserCloudInit(new PointCloudXYZI(size_init_ikdtree, 1));
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    PointVector ().swap(ikdtree.PCL_Storage);
    ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
    laserCloudInit->points = ikdtree.PCL_Storage;
    pcl::toROSMsg(*laserCloudInit, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "odom";
    pubLaserCloudFullRes->publish(laserCloudmsg);
}

void publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes)
{
    if (scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(feats_down_body);
        int size = laserCloudFullRes->points.size();

        PointCloudXYZI::Ptr   laserCloudWorld(new PointCloudXYZI(size, 1));
        
        for (int i = 0; i < size; i++)
        {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x;
            laserCloudWorld->points[i].y = feats_down_world->points[i].y;
            laserCloudWorld->points[i].z = feats_down_world->points[i].z;
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity;
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = "odom";
        pubLaserCloudFullRes->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }
    if (pcd_save_en)
    {
        int size = feats_down_world->points.size();
        PointCloudXYZI::Ptr   laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x;
            laserCloudWorld->points[i].y = feats_down_world->points[i].y;
            laserCloudWorld->points[i].z = feats_down_world->points[i].z;
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity;
        }
        *pcl_wait_save += *laserCloudWorld;
        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            // pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudIMUBody(new pcl::PointCloud<pcl::PointXYZINormal>(size, 1));
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    for (int i = 0; i < size; i++)
    {
        pointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "lidar_link";
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap)//发布Lasermap
{
    bool dense_pub_en = false;
    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);//feats_down_body特征点
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        pointBodyToWorld(&laserCloudFullRes->points[i], \
                            &laserCloudWorld->points[i]);
    }
    if(is_first_frame__)
    {
        *pcl_wait_pub += *cloud;
        is_first_frame__ = false;
    }
    *pcl_wait_pub += *laserCloudWorld;

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "odom";
    int point_num = laserCloudmsg.row_step / laserCloudmsg.point_step;
    // std::cout << "point_num: " << point_num << std::endl;
    double stamp_ = (float)laserCloudmsg.header.stamp.sec;
    pcd_map_.header.stamp = get_ros_time(lidar_end_time);
    pcd_map_.header.frame_id = "odom";
    // pubLaserCloudMap->publish(pcd_map_);
    pubLaserCloudMap->publish(laserCloudmsg);
}

void set_posestamp(T & out)
{
    if (!use_imu_as_input)
    {   
        if(is_first_kf_)
        {
            if(task == "A")
            {
                kf_output.x_.pos(0) = init_x_;
                kf_output.x_.pos(1) = init_y_;
                kf_output.x_.pos(2) = init_z_;
                out.position.x = kf_output.x_.pos(0);
                out.position.y = kf_output.x_.pos(1);
                out.position.z = kf_output.x_.pos(2);
                // Eigen::Quaterniond q(kf_output.x_.rot);
                // out.orientation.x = q.coeffs()[0];
                // out.orientation.y = q.coeffs()[1];
                // out.orientation.z = q.coeffs()[2];
                // out.orientation.w = q.coeffs()[3];
                // is_first_kf_ = false;
                Eigen::Quaterniond q(cos(M_PI / 2), 0, 0, sin(M_PI / 2));
                // 将四元数转换为旋转矩阵
                Eigen::Matrix3d rot_matrix = q.normalized().toRotationMatrix();
                // 将旋转矩阵转换为 MTK::SO3<double> 类型
                MTK::SO3<double> so3_rot(rot_matrix);
                // 将转换后的旋转矩阵赋值给 kf_output.x_.rot
                kf_output.x_.rot = so3_rot;
                // Eigen::Quaterniond q(kf_output.x_.rot);
                // out.orientation.x = q.coeffs()[0];
                // out.orientation.y = q.coeffs()[1];
                // out.orientation.z = q.coeffs()[2];
                // out.orientation.w = q.coeffs()[3];
                Eigen::Quaterniond q_updated(so3_rot); // 使用更新后的 kf_output.x_.rot 构造四元数
                out.orientation.x = q_updated.coeffs()[0];
                out.orientation.y = q_updated.coeffs()[1];
                out.orientation.z = q_updated.coeffs()[2];
                out.orientation.w = q_updated.coeffs()[3];
                is_first_kf_ = false; 

            }else if(task == "B")
            {
                kf_output.x_.pos(0) = init_x_;
                kf_output.x_.pos(1) = init_y_;
                kf_output.x_.pos(2) = init_z_;
                out.position.x = kf_output.x_.pos(0);
                out.position.y = kf_output.x_.pos(1);
                out.position.z = kf_output.x_.pos(2);
                Eigen::Quaterniond q(kf_output.x_.rot);
                out.orientation.x = q.coeffs()[0];
                out.orientation.y = q.coeffs()[1];
                out.orientation.z = q.coeffs()[2];
                out.orientation.w = q.coeffs()[3];
                is_first_kf_ = false; 
            }
        }
        else
        {
            out.position.x = kf_output.x_.pos(0);
            out.position.y = kf_output.x_.pos(1);
            out.position.z = kf_output.x_.pos(2);
            Eigen::Quaterniond q(kf_output.x_.rot);
            out.orientation.x = q.coeffs()[0];
            out.orientation.y = q.coeffs()[1];
            out.orientation.z = q.coeffs()[2];
            out.orientation.w = q.coeffs()[3];
        }
    }
    else
    {
        out.position.x = kf_input.x_.pos(0);
        out.position.y = kf_input.x_.pos(1);
        out.position.z = kf_input.x_.pos(2);
        Eigen::Quaterniond q(kf_input.x_.rot);
        out.orientation.x = q.coeffs()[0];
        out.orientation.y = q.coeffs()[1];
        out.orientation.z = q.coeffs()[2];
        out.orientation.w = q.coeffs()[3];
    }
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br)
{
    odomAftMapped.header.frame_id = "odom";
    odomAftMapped.child_frame_id = "lidar_link";
    if (publish_odometry_without_downsample)
    {
        odomAftMapped.header.stamp = get_ros_time(time_current);
    }
    else
    {
        odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    }
    set_posestamp(odomAftMapped.pose.pose);
    pubOdomAftMapped->publish(odomAftMapped);
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.transform.translation.x = odomAftMapped.pose.pose.position.x;
    transformStamped.transform.translation.y = odomAftMapped.pose.pose.position.y;
    transformStamped.transform.translation.z = odomAftMapped.pose.pose.position.z;
    transformStamped.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    transformStamped.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    transformStamped.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    transformStamped.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    transformStamped.header.stamp = rclcpp::Time(odomAftMapped.header.stamp);
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "lidar_link";
    // std::cout << "a"<<std::endl;
    tf_br->sendTransform(transformStamped);
}

void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose.pose);
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
    msg_body_pose.header.frame_id = "odom";
    static int jjj = 0;
    jjj++;
    {
        path.poses.emplace_back(msg_body_pose);
        pubPath->publish(path);
    }
}        

# Fast-LIO 核心算法深度解析：EKF、ikd-Tree 与 IMU-雷达紧耦合融合

这份解析将**完全结合你提供的代码**，从理论到实践，逐层拆解 Fast-LIO 的三大核心：**迭代误差状态卡尔曼滤波 (IESKF)**、**增量式 KD-Tree (ikd-Tree)** 以及 **IMU-激光雷达紧耦合融合**。

---

## 目录
1.  **第一部分：迭代误差状态卡尔曼滤波 (IESKF) 预测与更新**
    *   1.1 状态向量定义（结合 `state_input` / `state_output`）
    *   1.2 预测阶段（Predict）：`get_f_*` 与 `df_dx_*`
    *   1.3 更新阶段（Update）：`h_model_*` 与点云配准
2.  **第二部分：增量式 KD-Tree (ikd-Tree) 原理与代码应用**
    *   2.1 ikd-Tree 解决的核心问题
    *   2.2 在代码中的三个核心调用
3.  **第三部分：IMU-激光雷达紧耦合融合全流程**
    *   3.1 为什么是“紧耦合”？
    *   3.2 结合 `timer_callback` 的完整数据流

---

## 第一部分：迭代误差状态卡尔曼滤波 (IESKF)

Fast-LIO 没有用标准的 EKF，而是用了 **IESKF (Iterated Error State Kalman Filter)**。它的核心思想是：**在误差状态空间里做滤波，在观测更新时进行多次迭代，直到收敛**。

### 1.1 状态向量定义（Manifold 上的状态）

Fast-LIO 使用了 `IKFoM` 库，在流形 (Manifold) 上定义状态。看你的 `Estimator.h`：

#### 1.1.1 `state_input`（24 维，用于 IMU 高频预测）
```cpp
MTK_BUILD_MANIFOLD(state_input,
((vect3, pos))        // 0-2: 位置 (世界坐标系)
((SO3, rot))           // 3-5: 旋转 (世界坐标系 -> 机体坐标系)
((SO3, offset_R_L_I))  // 6-8: 雷达->IMU 外参旋转 (可选在线标定)
((vect3, offset_T_L_I)) // 9-11: 雷达->IMU 外参平移
((vect3, vel))         // 12-14: 速度 (世界坐标系)
((vect3, bg))          // 15-17: 陀螺仪零偏
((vect3, ba))          // 18-20: 加速度计零偏
((vect3, gravity))     // 21-23: 重力向量 (世界坐标系)
);
```

#### 1.1.2 `state_output`（30 维，用于激光低频更新）
```cpp
MTK_BUILD_MANIFOLD(state_output,
((vect3, pos))
((SO3, rot))
((SO3, offset_R_L_I))
((vect3, offset_T_L_I))
((vect3, vel))
((vect3, omg))         // 新增：角速度 (机体坐标系)
((vect3, acc))         // 新增：加速度 (机体坐标系)
((vect3, gravity))
((vect3, bg))
((vect3, ba))
);
```
**设计目的**：`state_output` 把 IMU 的原始测量 (`omg`, `acc`) 也放进了状态里，这样可以在激光更新时，同时修正 IMU 的测量值。

---

### 1.2 预测阶段 (Predict)：IMU 积分

预测阶段只靠 IMU，高频运行（通常 200Hz-1000Hz）。

#### 1.2.1 过程模型：`get_f_input` / `get_f_output`
看 `Estimator.cpp` 里的 `get_f_input`：
```cpp
Eigen::Matrix<double, 24, 1> get_f_input(state_input &s, const input_ikfom &in)
{
    // ...
    // 1. 角速度补偿零偏：omega = 测量角速度 - 陀螺仪零偏
    vect3 omega;
    in.gyro.boxminus(omega, s.bg);

    // 2. 加速度补偿零偏并转到世界坐标系：a_world = R * (a_meas - ba) + g
    vect3 a_inertial = s.rot * (in.acc - s.ba);

    // 3. 状态导数
    res(i) = s.vel[i];              // 位置导数 = 速度
    res(i + 3) = omega[i];          // 旋转导数 = 角速度
    res(i + 12) = a_inertial[i] + s.gravity[i]; // 速度导数 = 加速度 + 重力
    // ...
}
```
**物理意义**：这就是标准的 IMU 运动学方程。

#### 1.2.2 状态转移矩阵的雅可比：`df_dx_input`
卡尔曼滤波需要计算状态转移矩阵的雅可比，用来传播协方差。看 `df_dx_input`：
```cpp
cov.template block<3, 3>(12, 3) = -s.rot * MTK::hat(acc_); 
// 核心项：加速度的反对称矩阵，描述旋转对加速度的影响
```
这里的 `MTK::hat(acc_)` 就是把向量变成反对称矩阵，对应李代数里的左乘雅可比。

#### 1.2.3 过程噪声协方差：`process_noise_cov_input`
```cpp
cov.block<3, 3>(3, 3).diagonal() << gyr_cov_input, ...;  // 陀螺仪噪声
cov.block<3, 3>(12, 12).diagonal() << acc_cov_input, ...; // 加速度计噪声
cov.block<3, 3>(15, 15).diagonal() << b_gyr_cov, ...;    // 陀螺仪零偏随机游走
cov.block<3, 3>(18, 18).diagonal() << b_acc_cov, ...;    // 加速度计零偏随机游走
```
这些参数都是在 `laser_mapping.cpp` 的 `readParameters()` 里读的配置文件。

---

### 1.3 更新阶段 (Update)：激光点云配准（核心！）

这是 Fast-LIO 最精彩的部分：**把点云配准问题转化为卡尔曼滤波的观测更新问题**，并且进行**迭代**直到收敛。

核心函数是 `h_model_output`（对应 `use_imu_as_input = false` 的模式）。

#### 1.3.1 第一步：ikd-Tree 最近邻搜索
在 `h_model_output` 里：
```cpp
// 1. 把当前帧点投影到世界坐标系
pointBodyToWorld(&point_body_j, &point_world_j);

// 2. 在 ikd-Tree 里找最近的 5 个点
ikdtree.Nearest_Search(point_world_j, NUM_MATCH_POINTS, points_near, pointSearchSqDis, 2.236);
```

#### 1.3.2 第二步：平面拟合 (Plane Fitting)
```cpp
// 用最近的 5 个点拟合一个平面 ax + by + cz + d = 0
if (esti_plane(pabcd, points_near, plane_thr))
{
    // 计算当前点到这个平面的距离
    float pd2 = pabcd(0) * point_world_j.x + ... + pabcd(3);
    
    // 检查点是否有效（距离不能太大，点不能太靠近原点）
    if (p_body.norm() > match_s * pd2 * pd2)
    {
        // 有效点！记录平面法向量
        normvec->points[j].x = pabcd(0); // 法向量 nx
        normvec->points[j].y = pabcd(1); // 法向量 ny
        normvec->points[j].z = pabcd(2); // 法向量 nz
        normvec->points[j].intensity = pabcd(3); // 平面距离 d
    }
}
```
**关键创新**：Fast-LIO 不用 ICP 的点到点距离，而是用 **点到平面的距离**作为观测残差，这比点到点更鲁棒，尤其是在结构化环境（走廊、房间）里。

#### 1.3.3 第三步：构建观测残差和观测雅可比
```cpp
// 观测残差 z：点到平面的距离
ekfom_data.z(m) = -nx * x - ny * y - nz * z - d;

// 观测雅可比 H_x：残差对状态的导数
M3D point_crossmat = crossmat_list[idx+j+1]; // 点的反对称矩阵
V3D C(s.rot.transpose() * norm_vec);          // 法向量转到机体坐标系
V3D A(point_crossmat * C);                     // 旋转部分的雅可比

// 填充 H 矩阵
ekfom_data.h_x.block<1, 12>(m, 0) << 
    norm_vec(0), norm_vec(1), norm_vec(2),  // 对位置 pos 的导数 (nx, ny, nz)
    VEC_FROM_ARRAY(A),                        // 对旋转 rot 的导数
    0.0, 0.0, 0.0, ...;                      // 其他项
```
**IESKF 的“迭代”体现在哪**：
在 `kf_output.update_iterated_dyn_share_modified()` 内部，会：
1. 用当前状态计算残差和雅可比
2. 计算卡尔曼增益，更新状态
3. **用更新后的状态，重新投影点云，重新找最近邻，重新拟合平面**
4. 重复上述步骤，直到状态变化小于阈值或达到最大迭代次数

---

## 第二部分：增量式 KD-Tree (ikd-Tree)

传统的 PCL KD-Tree 有一个致命问题：**如果要增量式加点，或者删除旧点，必须重建整个树，速度极慢**。

ikd-Tree (Incremental KD-Tree) 是 Fast-LIO 的核心配套数据结构，专门解决这个问题。

### 2.1 ikd-Tree 的三个核心特性
1.  **增量式插入 (Incremental Insertion)**：不用重建整棵树，直接插入新点。
2.  **懒删除 (Lazy Deletion)**：要删除旧点时，先做个“标记”，不真的删除，等后面重构时再处理。
3.  **并行重建 (Parallel Re-Balancing)**：当树变得不平衡时，在后台并行重建，不阻塞主线程。

### 2.2 在你代码里的三个核心调用

#### 1. 初始化建图
```cpp
// 在 laser_mapping.cpp 的 timer_callback 里
if(!init_map)
{
    // ...
    if(use_pcd_map_)
    {
        // 加载先验地图并建树
        ikdtree.Build(map_cloud_->points); 
    }
    else
    {
        // 用第一帧点云初始化树
        ikdtree.Build(init_feats_world->points);
    }
}
```

#### 2. 最近邻搜索 (配准用)
```cpp
// 在 Estimator.cpp 的 h_model_output 里
// 为当前点找最近的 NUM_MATCH_POINTS (通常是5个) 地图点
// 最后一个参数 2.236 是最大搜索距离 sqrt(5)
ikdtree.Nearest_Search(point_world_j, NUM_MATCH_POINTS, points_near, pointSearchSqDis, 2.236);
```

#### 3. 增量式地图更新 (滑动窗口)
```cpp
// 在 laser_mapping.cpp 的 map_incremental() 里
// 1. 先把新点加进去
int no_need_down_num_ = ikdtree.Add_Points(PointNoNeedDownsample, false);

// 配合 lasermap_fov_segment() 做滑动窗口
// 当机器人走出当前立方体区域时
int kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
// 删除视野外的旧点（懒删除）
```

---

## 第三部分：IMU-激光雷达紧耦合融合全流程

现在我们把所有东西串起来，看 `laser_mapping.cpp` 的 `timer_callback` 是如何把 IMU 和激光雷达“焊”在一起的。

### 3.1 为什么叫“紧耦合”？
| 类型 | 特点 | 例子 |
|------|------|------|
| **松耦合 (Loosely Coupled)** | 先单独用 IMU 算一个位姿，单独用激光算一个位姿，最后把两个位姿做融合。 | LOAM (Lidar Odometry and Mapping) |
| **紧耦合 (Tightly Coupled)** | **不单独算位姿**，把 IMU 的原始测量和激光的原始点云，一起扔进一个优化器/滤波器里，一次性算出最优位姿。 | **Fast-LIO (这份代码)**、LIO-SAM |

**Fast-LIO 的紧耦合体现在**：
激光点云的配准残差，直接用来修正 IMU 的状态（位置、速度、旋转、零偏、重力），而不是修正一个“激光位姿”。

### 3.2 完整数据流（结合 `timer_callback`）

我们以 `use_imu_as_input = false`（默认模式，也是最强模式）为例：

1.  **数据同步**：`sync_packages(Measures)`
    *   确保一帧激光对应的时间范围内，有完整的 IMU 数据覆盖。

2.  **IMU 预处理与初始化**：`p_imu->Process(...)`
    *   初始静止时，用 IMU 估计重力方向和初始零偏。

3.  **点云降采样**：`downSizeFilterSurf.filter(...)`
    *   减少计算量。

4.  **IMU 高频预测 (Predict)**：
    *   在激光帧的时间间隔内，只要有新的 IMU 数据来，就调用 `kf_output.predict(...)`。
    *   这一步是**高频**的（IMU 频率），保证位姿的平滑性。

5.  **激光低频更新 (Update & Iterate)**：
    *   当一整帧激光准备好后，调用 `kf_output.update_iterated_dyn_share_modified()`。
    *   内部调用 `h_model_output`：
        *   投影点云到世界坐标系。
        *   **ikd-Tree 最近邻搜索**。
        *   **平面拟合**。
        *   **计算点到平面残差**。
        *   **迭代更新状态**。
    *   这一步是**低频**的（激光频率），保证位姿的精度。

6.  **发布结果**：
    *   `publish_odometry(...)`：发布里程计和 `odom -> lidar_link` TF。
    *   `map_incremental()`：把当前帧点云加入 ikd-Tree。

---

## 总结
这份 Fast-LIO 代码的核心三板斧是：
1.  **IESKF**：在误差状态空间滤波，观测更新时迭代收敛，精度比标准 EKF 高。
2.  **ikd-Tree**：增量式增删点，保证地图管理的效率。
3.  **点到平面的紧耦合**：把激光配准残差直接用来修正 IMU 状态，充分利用两种传感器的互补特性。

需要我针对其中某一部分（比如 IESKF 的具体数学推导、ikd-Tree 的数据结构细节）做更深入的讲解吗？