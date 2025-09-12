#include "serial_driver/serial_driver_node.hpp"

namespace rm_auto_aim
{

SerialDriverNode::SerialDriverNode(const rclcpp::NodeOptions& options) : Node("serial_driver", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting SerialDriverNode!");

  std::string device_name = this->declare_parameter("device_name", "/dev/robomaster");
  int baud_rate = this->declare_parameter("baud_rate", 921600);
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  current_v = this->declare_parameter("current_v",25.0);
  s_bias = this->declare_parameter("s_bias",0.03);
  z_bias = this->declare_parameter("z_bias",0.0);
  pitch_bias = this->declare_parameter("pitch_bias",0.005);
  k = this->declare_parameter("k",0.038);



  //由device_ptr_接管串口相关初始化与协议解包功能
  device_ptr_ = std::make_shared<::robomaster::SerialDevice>(device_name, baud_rate);
  if (!device_ptr_->Init())
  {
    // return false;
    RCLCPP_ERROR(this->get_logger(), "fail to init SerialDevice!");
  }

  recv_buff_ = std::unique_ptr<uint8_t[]>(new uint8_t[BUFF_LENGTH]);
  send_buff_ = std::unique_ptr<uint8_t[]>(new uint8_t[BUFF_LENGTH_SEND]);  // debug

  memset(&frame_recv_header_, 0, sizeof(frame_header_struct_t));
  memset(&frame_send_header_, 0, sizeof(frame_header_struct_t));

  gimbal_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);  // test_common

  gimbal_fdb_pub_ = this->create_publisher<auto_aim_interfaces::msg::GimbalFdb>("/gimbal_fdb", 10);

  gimbal_ctrl_sub_ = this->create_subscription<auto_aim_interfaces::msg::GimbalCtrl>(
      "gimbal_ctrl", rclcpp::SensorDataQoS(),
      std::bind(&SerialDriverNode::gimbalCtrlCallback, this, std::placeholders::_1));

  fdb_sub_ = this->create_subscription<auto_aim_interfaces::msg::Fdb>(
      "fdb", rclcpp::SensorDataQoS(),
      std::bind(&SerialDriverNode::naviCallback, this, std::placeholders::_1));
  // initial timer
  timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1)),
                                   std::bind(&SerialDriverNode::timerCallback, this));

  trajectory_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("trojectory_markers", 10);
}

void SerialDriverNode::gimbalCtrlCallback(const auto_aim_interfaces::msg::GimbalCtrl::SharedPtr msg)
{
  // RCLCPP_INFO_STREAM(this->get_logger(), std::oct << "=======================" << std::endl
  //                                                 << "Get GimbalCtrl msg!!!!!" << std::endl
  //                                                 << "----------------------" << std::endl
  //                                                 << "yaw: " << msg->yaw << std::endl
  //                                                 << "pitch: " << msg->pit << std::endl
  //                                                 << "dis: " << msg->dis << std::endl
  //                                                 << "tof: " << msg->tof << std::endl
  //                                                 << "pos: " << msg->pos << std::endl
  //                                                 << "empty: " << msg->empty << std::endl;);

  vision_rx_data_.data.yaw = msg->yaw;
  vision_rx_data_.data.pit = msg->pit;
  vision_rx_data_.data.shoot_yaw_tole = msg->shoot_yaw_tole;
  vision_rx_data_.data.tof = msg->fire_flag;
  vision_rx_data_.data.pos = msg->pos;
  vision_rx_data_.data.empty = msg->empty;
  //msg->x = -16;
  vision_rx_data_.data.base_dx = (msg->x > 0?msg->x:-msg->x); 

  // tempData = msg->cnt;
  // tempData<<1;
  // tempData = tempData && msg-
  vision_rx_data_.data.flag = ((((msg->cnt & 0x1f)) | ((msg->x > 0?1:0) << 5) | (msg->ist_flag << 6)) | (msg->aim_flag << 7));
  vision_rx_data_.data.eof = VISION_EOF;  // EOF

  uint16_t send_length = senderPackSolveEOF((uint8_t*)&vision_rx_data_, sizeof(vision_rx_t), send_buff_.get());

  device_ptr_->Write(send_buff_.get(), send_length);
}

void SerialDriverNode::timerCallback()
{
  static int a = 0;
  static int flag = 0;
  static int last_len = 0;
  static int temp_frame_length = 25;  // frame 's length

  last_len = device_ptr_->ReadUntil2(recv_buff_.get(), END1_SOF, END2_SOF, temp_frame_length);

  if (last_len > 1)
  {
    RCLCPP_ERROR(get_logger(), "too long msg!!! length: %d", last_len);  //, length: %d",temp_frame_length);
            std::cout<<"+++++++++++++++++:"  <<std::endl;//debug
  }
  else if (last_len == 0)
  {
    RCLCPP_ERROR(get_logger(), "wrong msg!!!");
  }

  while (flag == 0 && last_len == 1)
  {
    // RCLCPP_INFO(get_logger(),"ciiiiiiiiiiiiiiii");//debug
    // std::cout << "ciiiiiiiiiiiiiiii" << std::endl;//debug
    if ((recv_buff_[a] == END1_SOF) && (recv_buff_[a + 1] == END2_SOF))
    {
      flag = 1;
      searchFrameSOF(recv_buff_.get(), a);
    }
    a++;
  }
  flag = 0;
  a = 0;

  usleep(1);
}

void SerialDriverNode::naviCallback(const auto_aim_interfaces::msg::Fdb::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(),"a:%f",msg->yaw);
  fdb_rx_data_.data.yaw = msg->yaw;
  fdb_rx_data_.data.pitch = msg->pitch;
  std::cout<<"yaw:"<<msg->yaw<<std::endl<<"pitch:"<<msg->pitch<<std::endl;
  // fdb_rx_data_.data.yaw = 0;
  // fdb_rx_data_.data.pitch = 1;
  // fdb_rx_data_.data.meiyoude = 2;
  fdb_rx_data_.data.eof = VISION_EOF_FDB;  // EOF

  uint16_t send_length = senderPackSolveEOF((uint8_t*)&fdb_rx_data_, sizeof(fdb_rx_t), send_buff_.get());

  device_ptr_->Write(send_buff_.get(), send_length);
}
uint16_t SerialDriverNode::senderPackSolveEOF(uint8_t* data, uint16_t data_length, uint8_t* send_buf)
{
  uint8_t index = 0;

  memcpy(send_buf + index, data, data_length);

  // Append_CRC16_Check_Sum(send_buf, data_length + 9);

  return data_length + 0;
}

void SerialDriverNode::searchFrameSOF(uint8_t* frame, uint16_t total_len)
{
  static vision_tx_t vision_tx_data;  // debug receive
  uint16_t i;

  for (i = 0; i < total_len;)
  {
    if (*frame == HEADER_SOF)
    {
      // RCLCPP_INFO(get_logger(), "header get!");  //debug

      memcpy(&vision_tx_data.buff, frame, sizeof(vision_tx_data));

      // pub tf    test_common
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = this->get_clock()->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
      transform.header.frame_id = "world";
      transform.child_frame_id = "gimbal_link";
      tf2::Quaternion q;
      vision_tx_data.data.imu_pit = -vision_tx_data.data.imu_pit / 57.29578;
      vision_tx_data.data.imu_yaw = vision_tx_data.data.imu_yaw / 57.29578;
      q.setRPY(0.0, vision_tx_data.data.imu_pit, vision_tx_data.data.imu_yaw);
      transform.transform.rotation = tf2::toMsg(q);

      gimbal_tf_broadcaster->sendTransform(transform);

      publishTrajectoryMarker(vision_tx_data.data.imu_pit, vision_tx_data.data.imu_yaw);

      auto_aim_interfaces::msg::GimbalFdb gimbal_fdb;
      gimbal_fdb.imu_pit = vision_tx_data.data.imu_pit;
      gimbal_fdb.imu_pit_spd = vision_tx_data.data.imu_pit_spd;
      gimbal_fdb.imu_yaw = vision_tx_data.data.imu_yaw;
      gimbal_fdb.imu_yaw_spd = vision_tx_data.data.imu_yaw_spd;
      gimbal_fdb.camp = vision_tx_data.data.mode_msg.camp;  //机器人阵营  0：红方  1：蓝方
      gimbal_fdb.priority_number = vision_tx_data.data.priority_number;
      gimbal_fdb.aiming_mode = vision_tx_data.data.mode_msg.aiming_mode; //选择相机
      gimbal_fdb.shooter_speed = vision_tx_data.data.shooter_speed; //射速
      gimbal_fdb.lidar_mode = vision_tx_data.data.mode_msg.lidar_mode; //基地血量判断雷达

      //gimbal_fdb.aiming_mode = 2;
      gimbal_fdb_pub_->publish(gimbal_fdb);
            // std::cout<<gimbal_fdb.aiming_mode<<std::endl;

      i = total_len;
    }
    else
    {
      frame++;
      i++;
      // debug
      RCLCPP_WARN(get_logger(), "bad header!");  // debug
    }
  }
}

// publish trajectory marker
void SerialDriverNode::publishTrajectoryMarker(float pitch, float yaw)
{
  
  current_v = get_parameter("current_v").as_double();
  s_bias = get_parameter("s_bias").as_double();
  z_bias = get_parameter("z_bias").as_double();
  pitch_bias = get_parameter("pitch_bias").as_double();
  k = get_parameter("k").as_double();

  pitch +=pitch_bias;
  // 创建MarkerArray消息对象
  auto marker_array_msg = std::make_unique<visualization_msgs::msg::MarkerArray>();

  // 创建线条标记...
  auto line_strip_marker = std::make_unique<visualization_msgs::msg::Marker>();
  line_strip_marker->header.frame_id = "world";
  line_strip_marker->type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_strip_marker->action = visualization_msgs::msg::Marker::ADD;
  line_strip_marker->scale.x = 0.01;
  line_strip_marker->pose.orientation.w = 1.0;
  line_strip_marker->color.a = 1.0;
  line_strip_marker->color.r = 1.0;

  // 生成弹道点...
  const double GRAVITY = 9.78;
  
  for (int t = 0; t <= 20; t++)
  {
    const double t0 = static_cast<double>(t) / 10.0;
    const double s = (std::log(t0 * k * current_v * std::cos(-pitch)) / k)+s_bias;
    const double x = s * std::cos(yaw);
    const double y = s * std::sin(yaw);
    const double z = (current_v * std::sin(-pitch) * t0 - GRAVITY * t0 * t0 / 2.0)+z_bias;
    // RCLCPP_INFO_STREAM(this->get_logger(), "position :" << std::endl
    //                         << "x: " << x << std::endl
    //                         << "y: " << y << std::endl
    //                         << "z: " << z << std::endl
    //                         << "t0: " << t0 << std::endl
    //                         <<"speed_: " << speed  << std::endl
    //                         << "speed_z: " << speed * std::sin(-pitch) << std::endl);

    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    line_strip_marker->points.push_back(point);
  }

  // 将标记添加到MarkerArray消息对象中...
  marker_array_msg->markers.push_back(*line_strip_marker);

  trajectory_marker_pub_->publish(std::move(marker_array_msg));
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::SerialDriverNode)
