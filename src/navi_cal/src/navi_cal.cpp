#include "navi_cal/navi_cal.hpp"
namespace taurus
{
    Navi_cal::Navi_cal(const rclcpp::NodeOptions &options) : Node("Navi_cal",options)
    {   
        // RCLCPP_INFO(this->get_logger(),"Navi_cal");
        auto period = std::chrono::milliseconds(40);
        timer_ = this->create_wall_timer(period,std::bind(&Navi_cal::timerCallback,this));
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        fdb_pub_ = this->create_publisher<auto_aim_interfaces::msg::Fdb>("/fdb", 10);
        gimbal_fdb_sub_ = this->create_subscription<auto_aim_interfaces::msg::GimbalFdb>(
        "gimbal_fdb", rclcpp::SensorDataQoS(),
        std::bind(&Navi_cal::GimbalFdbCallback, this, std::placeholders::_1));
        ParamInit();
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&Navi_cal::paramCallback, this, std::placeholders::_1));
    }
    Navi_cal::~Navi_cal()
    {
    }
    void Navi_cal::GimbalFdbCallback(const auto_aim_interfaces::msg::GimbalFdb::SharedPtr msg)
    {
        lidar_mode = msg->lidar_mode;
        aiming_mode = msg->aiming_mode;
    }
    float Navi_cal::pitchTrajectoryCompensation(float s, float z, float v)
    {   
    auto dist_vertical = z;
    auto vertical_tmp = dist_vertical;
    auto dist_horizonal = s;
    auto pitch = atan(dist_vertical / dist_horizonal) * 180 / PI;
    auto pitch_new = pitch;
    for (int i = 0; i < 10; i++)
    {
        auto x = 0.0;
        auto y = 0.0;
        auto p = tan(pitch_new / 180 * PI);
        auto u = v / sqrt(1 + pow(p, 2));
        auto delta_x = dist_horizonal / R_K_iter;
        for (int j = 0; j < R_K_iter; j++)
        {
            auto k1_u = -k * u * sqrt(1 + pow(p, 2));
            auto k1_p = -GRAVITY / pow(u, 2);
            auto k1_u_sum = u + k1_u * (delta_x / 2);
            auto k1_p_sum = p + k1_p * (delta_x / 2);

            auto k2_u = -k * k1_u_sum * sqrt(1 + pow(k1_p_sum, 2));
            auto k2_p = -GRAVITY / pow(k1_u_sum, 2);
            auto k2_u_sum = u + k2_u * (delta_x / 2);
            auto k2_p_sum = p + k2_p * (delta_x / 2);

            auto k3_u = -k * k2_u_sum * sqrt(1 + pow(k2_p_sum, 2));
            auto k3_p = -GRAVITY / pow(k2_u_sum, 2);
            auto k3_u_sum = u + k3_u * delta_x;  
            auto k3_p_sum = p + k3_p * delta_x;  

            auto k4_u = -k * k3_u_sum * sqrt(1 + pow(k3_p_sum, 2));
            auto k4_p = -GRAVITY / pow(k3_u_sum, 2);

            u += (delta_x / 6) * (k1_u + 2 * k2_u + 2 * k3_u + k4_u);
            p += (delta_x / 6) * (k1_p + 2 * k2_p + 2 * k3_p + k4_p);

            x += delta_x;
            y += p * delta_x;
        }
        auto error = dist_vertical - y;
        if (abs(error) <= 0.0005)
        {
            break;
        }
        else
        {
            vertical_tmp += error;
            pitch_new = atan(vertical_tmp / dist_horizonal) * 180 / PI;
        }
    }
    return pitch_new;
    }
    void Navi_cal::ParamInit()
    {
        this->declare_parameter<double>("pit_bias", 0.0);
        this->get_parameter_or<double>("pit_bias", pit_bias, 0.0);
    }
    rcl_interfaces::msg::SetParametersResult Navi_cal::paramCallback(
        const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        
        for (const auto &param : params) {
            if (param.get_name() == "pit_bias") {
                pit_bias = param.as_double();
                RCLCPP_INFO(this->get_logger(), "参数更新: pit_bias = %.3f", pit_bias);
            }
        }
        return result;
    }

    void Navi_cal::timerCallback()
    {
        RCLCPP_INFO(this->get_logger(),"timerCallback");
        try{
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("map","lidar_link",tf2::TimePointZero);
            lidar_pose_.position.x = transform.transform.translation.x;
            lidar_pose_.position.y = transform.transform.translation.y;
            lidar_pose_.position.z = transform.transform.translation.z;
            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            );
            tf2::Matrix3x3 m(q);
            m.getRPY(lidar_roll_,lidar_pitch_,lidar_yaw_);
            RCLCPP_INFO(this->get_logger(),"lidar yaw: %f, lidar x: %f, lidar y: %f,lidar z: %f,",lidar_yaw_,lidar_pose_.position.x,lidar_pose_.position.y,lidar_pose_.position.z);
        }catch(tf2::TransformException  &ex){
            RCLCPP_ERROR(this->get_logger(),"Failure %s",ex.what());
        }
        try{
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("map","cloud_link",tf2::TimePointZero);
            robot_pose_.position.x = transform.transform.translation.x;
            robot_pose_.position.y = transform.transform.translation.y;
            robot_pose_.position.z = transform.transform.translation.z;
            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            );
            tf2::Matrix3x3 m(q);
            m.getRPY(robot_roll_,robot_pitch_,robot_yaw_);
            RCLCPP_INFO(this->get_logger(),"Current yaw: %f, Current x: %f, Current y: %f,Current z: %f,",robot_yaw_,robot_pose_.position.x,robot_pose_.position.y,robot_pose_.position.z);
        }catch(tf2::TransformException  &ex){ 
            RCLCPP_ERROR(this->get_logger(),"Failure %s",ex.what());
        }
        // fdb_msg_.yaw = 1;
        fdb_msg_.pitch = 2;
        map_yaw_ = atan2((target_y_ - robot_pose_.position.y),(target_x_ - robot_pose_.position.x)) * 180 /3.1415926;
        //基地
        target_dis_ = sqrt((target_y_ - robot_pose_.position.y) * (target_y_ - robot_pose_.position.y)+(target_x_ - robot_pose_.position.x) * (target_x_ - robot_pose_.position.x));
        if(lidar_mode == 1 || aiming_mode ==2)//low
        { 
            target_z_ = 0.3455;
            shoot_speed = 15.5;
            result_pit_ = (pitchTrajectoryCompensation(target_dis_,target_z_,shoot_speed) + pit_bias);

        }
        else//tower
        {
            target_z_ = 0.8855;
            shoot_speed = 15.5;
            result_pit_ = (pitchTrajectoryCompensation(target_dis_,target_z_,shoot_speed) + pit_bias);
        }
        if(robot_pose_.position.x >=-0.660237 && robot_pose_.position.x <= 4.069704 && robot_pose_.position.y >= 5.882196 && robot_pose_.position.y <= 8.184546)
        {
            target_z_ = 0.6855;
            shoot_speed = 15.5;
            result_pit_ = (pitchTrajectoryCompensation(target_dis_,target_z_,shoot_speed) + pit_bias);
        }
        else
        {
            target_z_ = 0.8855;
            shoot_speed = 15.5;
            result_pit_ = (pitchTrajectoryCompensation(target_dis_,target_z_,shoot_speed) + pit_bias);
        }
        robot_yaw_ = robot_yaw_ * 180 /3.1415926;
        result_yaw_ = map_yaw_ - robot_yaw_; //+往左偏，-往右偏'
        fdb_msg_.yaw = result_yaw_;
        fdb_msg_.pitch = result_pit_;
        //INFO
        RCLCPP_INFO(this->get_logger(),"map_yaw_ :%f,",map_yaw_);
        RCLCPP_INFO(this->get_logger(),"robot_yaw_ :%f,",robot_yaw_);
        RCLCPP_INFO(this->get_logger(),"result_yaw_ :%f,",result_yaw_);
        RCLCPP_INFO(this->get_logger(),"result_pit_ :%f,",result_pit_);
        RCLCPP_INFO(this->get_logger(),"result_pit_pid:%f,",result_pit_ * PI / 180);
        RCLCPP_INFO(this->get_logger(),"target_dis_ :%f,",target_dis_);
        RCLCPP_INFO(this->get_logger(),"pit_bias :%f,",pit_bias);  
        RCLCPP_INFO(this->get_logger(),"lidar_mode  :%i,",lidar_mode);  
        RCLCPP_INFO(this->get_logger(),"shoot_speed  :%f,",shoot_speed);  
        RCLCPP_INFO(this->get_logger(),"target_z_ :%f,",target_z_);  

        fdb_pub_->publish(fdb_msg_);
    }
}// namespace taurus2weee3
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(taurus::Navi_cal);
