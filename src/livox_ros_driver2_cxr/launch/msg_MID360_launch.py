import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch

################### user configure parameters for ros2 start ###################
xfer_format   = 4    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
# 0 是 ROS2 标准的 Pointcloud2 格式（兼容 RVIZ2、PCL 等工具），1 是览沃自定义格式（一般不用）
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 30.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc. 10hz->20000 20hz->10000 30hz->6624
output_type   = 0
frame_id      = 'lidar_link'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

# 获取当前 launch 文件所在的绝对路径
cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
# 拼接得到 config 配置文件夹的路径
cur_config_path = cur_path + '../config'
# 最终得到雷达配置文件 MID360_config.json 的完整路径
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
################### user configure parameters for ros2 end #####################

# 给下面的 parameters=livox_ros2_params
livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]



def generate_launch_description():
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params,  # 将上面定义的参数列表传递给节点
        respawn=True,
        respawn_delay=1,
        )

    return LaunchDescription([
        livox_driver,
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=livox_rviz,
        #         on_exit=[
        #             launch.actions.EmitEvent(event=launch.events.Shutdown()),
        #         ]
        #     )
        # )
    ])
