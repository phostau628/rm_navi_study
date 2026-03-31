# 导包：路径处理 + ROS2 launch 核心工具
import os.path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument          # 声明启动参数
from launch.substitutions import LaunchConfiguration      # 读取启动参数
from launch.conditions import IfCondition                 # 条件判断（true/false 生效）
from launch_ros.actions import Node, SetUseSimTime        # 启动节点


def generate_launch_description():
    # ====================== 路径拼接（固定写法）======================
    # 获取功能包的共享目录（绝对路径）
    package_path = get_package_share_directory('point_lio_cxr')
    
    # 拼接 yaml 配置文件路径：包路径/config/RMUC.yaml
    default_config_path = os.path.join(package_path, 'config', 'RMUC.yaml')
    
    # 拼接 rviz 配置文件路径：包路径/rviz/fastlio.rviz
    default_rviz_config_path = os.path.join(package_path, 'rviz', 'fastlio.rviz')


    # ====================== 【重点 1：读取声明的参数】 ======================
    # 作用：把 DeclareLaunchArgument 声明的参数 取出来用
    use_sim_time = LaunchConfiguration('use_sim_time')    # 官方内置参数：仿真时间
    config_path = LaunchConfiguration('config_path')      # 自定义：配置文件路径
    rviz_use = LaunchConfiguration('rviz')                # 自定义：rviz 开关（true/false）
    rviz_cfg = LaunchConfiguration('rviz_cfg')            # 自定义：rviz 配置文件路径


    # ====================== 【重点 2：声明启动参数】 ======================
    # 声明：是否使用仿真时间（官方自带，必须叫这个名）
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',  # 默认不使用仿真时间
        description='Use simulation (Gazebo) clock if true'
    )

    # 声明：yaml 配置文件路径（自定义名，默认值=上面拼的路径,描述）
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )

    # 声明：rviz 开关（自定义名，默认 true=启动 rviz）
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )

    # 声明：rviz 配置文件路径（自定义名）
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )


    # ====================== 【重点 3：启动主节点】 ======================
    point_lio_node = Node(
        package='point_lio_cxr',
        executable='pointlio_mapping',
        
        # 传给节点的参数：
        parameters=[
            config_path,          # 1. yaml 配置文件
            {'use_sim_time': use_sim_time}  # 2. 官方参数：是否用仿真时间
        ],
        
        output='screen',         # 日志输出到终端
        respawn=True,            # 挂了自动重启
        respawn_delay=1,         # 延迟 1s 重启
    )


    # ====================== 【重点 4：条件启动 RViz】 ======================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],  # 加载 rviz 配置文件
        
        # 关键：只有 rviz_use = true 时才启动 RViz
        condition=IfCondition(rviz_use)
    )


    # ====================== 把所有动作加入启动文件 ======================
    ld = LaunchDescription()
    
    # 1. 先加参数声明
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    
    # 2. 再加节点
    ld.add_action(point_lio_node)
    # ld.add_action(rviz_node)  # 这里被注释了 → 所以 rviz 不会启动！

    return ld