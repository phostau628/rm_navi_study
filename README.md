已完成内容：
3.26  lidox_driver--MID360_config.json


3.27 完成 lidox_driver--msg_MID360_launch.py 启动文件
3.27 Point-LIO_cxr/config/RMUC.yaml 配置文件完成
1》init_x / init_y / init_z
= 机器人在 map 坐标系 下的初始坐标

id_topic:  "/livox/lidar" 

            # imu_topic:  "/imu/data"  #from imu_complementary_filter

            # 订阅 IMU 话题（这里用雷达自带的 IMU）
            imu_topic:  "/livox/imu" 这两个经常换设备，或切仿真，环境不同，话题不同，需要根据实际情况修改，所以写在yaml好维护


3.27 完成 Point-LIO_cxr/launch/RMUC.launch.py 启动文件
知识点：
1.*declare 只是 “定义参数名字”
真正加载 yaml 的是「节点的 parameters」
  
  declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', ......)

  declare_rviz_cmd = DeclareLaunchArgument(
  ......      
    )
   point_lio_node = Node(
        package='point_lio_cxr',
        executable='pointlio_mapping',
        # 真正加载 yaml 的是「节点的 parameters」！
        parameters=[
            config_path,          # 1. yaml 配置文件
            {'use_sim_time': use_sim_time}  # 2. 官方参数：是否用仿真时间
        ],

 2.只有launch启动节点，没有节点启动节点
