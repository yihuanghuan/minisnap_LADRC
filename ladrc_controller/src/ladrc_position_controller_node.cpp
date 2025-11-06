#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp> // <-- 新增
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include "ladrc_controller/ladrc_core.hpp"
#include <cmath>
#include <chrono>
#include <atomic>

using namespace std::chrono_literals;

// [新增] 自动起飞状态机
enum class FlightState
{
  INIT,
  ARMING,
  SETTING_OFFBOARD,
  RUNNING_TRAJECTORY
};

class LADRCPositionControllerNode : public rclcpp::Node
{
public:
  LADRCPositionControllerNode()
      : Node("ladrc_position_controller")
  {
    // 声明参数 (与 circle_LADRC 保持一致)
    this->declare_parameter("control_frequency", 50.0);
    this->declare_parameter("omega_o_x", 15.0);
    this->declare_parameter("omega_o_y", 15.0);
    this->declare_parameter("omega_o_z", 15.0);
    this->declare_parameter("omega_c_x", 8.0);
    this->declare_parameter("omega_c_y", 8.0);
    this->declare_parameter("omega_c_z", 8.0);
    this->declare_parameter("b0_x", 1.0);
    this->declare_parameter("b0_y", 1.0);
    this->declare_parameter("b0_z", 1.0);
    this->declare_parameter("max_velocity", 5.0); // 不再使用
    this->declare_parameter("max_acceleration_x", 3.0);
    this->declare_parameter("max_acceleration_y", 3.0);
    this->declare_parameter("max_acceleration_z", 3.0);

    // 获取参数
    double control_freq = this->get_parameter("control_frequency").as_double();
    dt_ = 1.0 / control_freq;

    // 初始化 LADRC 控制器
    initializeControllers();

    // --- [修改] 订阅 minisnap 规划器的话题 ---
    reference_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/target_pose", rclcpp::QoS(10),
        std::bind(&LADRCPositionControllerNode::poseCallback, this, std::placeholders::_1));

    reference_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/target_twist", rclcpp::QoS(10),
        std::bind(&LADRCPositionControllerNode::twistCallback, this, std::placeholders::_1));

    reference_accel_sub_ = this->create_subscription<geometry_msgs::msg::AccelStamped>(
        "/target_accel", rclcpp::QoS(10),
        std::bind(&LADRCPositionControllerNode::accelCallback, this, std::placeholders::_1));
    // --- [修改结束] ---

    odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry",
        rclcpp::SensorDataQoS(),
        std::bind(&LADRCPositionControllerNode::odomCallback, this, std::placeholders::_1));

    // Publishers
    offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);

    trajectory_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);

    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", 10);

    // [修改] 分离状态机定时器和控制循环定时器
    auto control_timer_period = std::chrono::duration<double>(dt_);
    control_timer_ = this->create_wall_timer(
        control_timer_period,
        std::bind(&LADRCPositionControllerNode::controlLoop, this));

    auto command_timer_period = std::chrono::milliseconds(100); // 10 Hz
    command_timer_ = this->create_wall_timer(
        command_timer_period,
        std::bind(&LADRCPositionControllerNode::stateMachine, this));

    // [新增] 初始化状态
    flight_state_ = FlightState::INIT;
    offboard_setpoint_counter_ = 0;

    RCLCPP_INFO(this->get_logger(), "LADRC 轨迹跟踪控制器已初始化 (LADRC Position Controller Node initialized)");
    RCLCPP_INFO(this->get_logger(), "等待系统稳定... 将在10秒后开始执行自主起飞序列。");
    RCLCPP_INFO(this->get_logger(), "等待 /target_pose, /target_twist, /target_accel 和 /fmu/out/vehicle_odometry 消息...");
  }

private:
  void initializeControllers()
  {
    ladrc_controller::LADRCParams params_x, params_y, params_z;

    double max_acc_x = this->get_parameter("max_acceleration_x").as_double();
    double max_acc_y = this->get_parameter("max_acceleration_y").as_double();
    double max_acc_z = this->get_parameter("max_acceleration_z").as_double();

    // X-axis controller
    params_x.omega_o = this->get_parameter("omega_o_x").as_double();
    params_x.omega_c = this->get_parameter("omega_c_x").as_double();
    params_x.kp = params_x.omega_c * params_x.omega_c;
    params_x.kd = 2.0 * params_x.omega_c;
    params_x.b0 = this->get_parameter("b0_x").as_double();
    params_x.dt = dt_;
    params_x.max_output = max_acc_x;
    params_x.min_output = -max_acc_x;

    // Y-axis controller
    params_y.omega_o = this->get_parameter("omega_o_y").as_double();
    params_y.omega_c = this->get_parameter("omega_c_y").as_double();
    params_y.kp = params_y.omega_c * params_y.omega_c;
    params_y.kd = 2.0 * params_y.omega_c;
    params_y.b0 = this->get_parameter("b0_y").as_double();
    params_y.dt = dt_;
    params_y.max_output = max_acc_y;
    params_y.min_output = -max_acc_y;

    // Z-axis controller
    params_z.omega_o = this->get_parameter("omega_o_z").as_double();
    params_z.omega_c = this->get_parameter("omega_c_z").as_double();
    params_z.kp = params_z.omega_c * params_z.omega_c;
    params_z.kd = 2.0 * params_z.omega_c;
    params_z.b0 = this->get_parameter("b0_z").as_double();
    params_z.dt = dt_;
    params_z.max_output = max_acc_z;
    params_z.min_output = -max_acc_z;

    ladrc_x_ = std::make_unique<ladrc_controller::LADRCController>(params_x);
    ladrc_y_ = std::make_unique<ladrc_controller::LADRCController>(params_y);
    ladrc_z_ = std::make_unique<ladrc_controller::LADRCController>(params_z);
  }

  // --- [修改] 新的回调函数 ---
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO_ONCE(this->get_logger(), "DEBUG: === 成功接收到 /target_pose 消息! ===");
    reference_pose_ = *msg;
    has_pose_ = true;
  }

  void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    RCLCPP_INFO_ONCE(this->get_logger(), "DEBUG: === 成功接收到 /target_twist 消息! ===");
    reference_twist_ = *msg;
    has_twist_ = true;
  }

  void accelCallback(const geometry_msgs::msg::AccelStamped::SharedPtr msg)
  {
    RCLCPP_INFO_ONCE(this->get_logger(), "DEBUG: === 成功接收到 /target_accel 消息! ===");
    reference_accel_ = *msg;
    has_accel_ = true;
  }
  // --- [修改结束] ---

  void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    RCLCPP_INFO_ONCE(this->get_logger(), "DEBUG: === 成功接收到 /fmu/out/vehicle_odometry 消息! ===");
    current_odom_ = *msg;
    has_odom_ = true;
  }

  // --- [新增] 状态机逻辑 (来自 circle_LADRC 规划器) ---
  void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param7 = 0.0)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param7 = param7;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_pub_->publish(msg);
  }

  void stateMachine()
  {
    switch (flight_state_.load())
    {
    case FlightState::INIT:
      // 等待10秒稳定
      if (++offboard_setpoint_counter_ * 100 > 10000)
      {
        RCLCPP_INFO(this->get_logger(), "系统稳定，开始解锁 (Arming)...");
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        flight_state_ = FlightState::ARMING;
        offboard_setpoint_counter_ = 0;
      }
      break;

    case FlightState::ARMING:
      // 等待2秒
      if (++offboard_setpoint_counter_ * 100 > 2000)
      {
        RCLCPP_INFO(this->get_logger(), "解锁成功。切换到 Offboard 模式...");
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0); // 6.0 is Offboard
        flight_state_ = FlightState::SETTING_OFFBOARD;
        offboard_setpoint_counter_ = 0;
      }
      break;

    case FlightState::SETTING_OFFBOARD:
      // 等待1秒
      if (++offboard_setpoint_counter_ * 100 > 1000)
      {
        RCLCPP_INFO(this->get_logger(), "Offboard 模式已激活。LADRC 控制器接管。");
        flight_state_ = FlightState::RUNNING_TRAJECTORY;
        command_timer_->cancel(); // 状态机任务完成，停止
      }
      break;

    case FlightState::RUNNING_TRAJECTORY:
      command_timer_->cancel();
      break;
    }
  }
  // --- [新增结束] ---

  void controlLoop()
  {
    // 持续发布 offboard 模式
    publishOffboardControlMode();

    // [修改] 只有在收到所有参考和 odom，并且状态机完成后才运行控制
    if (flight_state_.load() != FlightState::RUNNING_TRAJECTORY ||
        !has_pose_ || !has_twist_ || !has_accel_ || !has_odom_)
    {
      if (flight_state_.load() != FlightState::RUNNING_TRAJECTORY) {
           RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
             "DEBUG: controlLoop 等待状态机进入 RUNNING_TRAJECTORY... (当前: %d)", (int)flight_state_.load());
      } else {
           RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
             "DEBUG: controlLoop 等待所有参考话题... (Pose: %d, Twist: %d, Accel: %d, Odom: %d)",
             has_pose_, has_twist_, has_accel_, has_odom_);
      }
      return;
    }

    // --- [修改] 使用 minisnap 规划器的完整状态 ---

    // 1. 获取测量值 (Odom) 并转换为 ENU
    // ENU.x (East)  = NED.y (position[1])
    // ENU.y (North) = NED.x (position[0])
    // ENU.z (Up)    = -NED.z (position[2])
    double x_meas = current_odom_.position[1];
    double y_meas = current_odom_.position[0];
    double z_meas = -current_odom_.position[2];

    // 2. 获取参考值 (Target) (已经是 ENU)
    double x_ref = reference_pose_.pose.position.x;
    double y_ref = reference_pose_.pose.position.y;
    double z_ref = -reference_pose_.pose.position.z;

    double vx_ref = reference_twist_.twist.linear.x;
    double vy_ref = reference_twist_.twist.linear.y;
    double vz_ref = -reference_twist_.twist.linear.z;

    double ax_ref = reference_accel_.accel.linear.x;
    double ay_ref = reference_accel_.accel.linear.y;
    double az_ref = -reference_accel_.accel.linear.z;

    // 3. 计算 LADRC 控制输出 (ENU 加速度指令)
    double ax_cmd = ladrc_x_->update(x_ref, vx_ref, ax_ref, x_meas);
    double ay_cmd = ladrc_y_->update(y_ref, vy_ref, ay_ref, y_meas);
    // Z 轴：PX4 需要的是推力，但LADRC输出加速度。
    // 我们需要添加重力补偿。
    // a_z_cmd 是期望的总加速度（包括重力）。
    // PX4 的加速度控制器通常期望 *不* 包含重力的加速度（即期望的推力加速度）。
    // 但是，ladrc_position_controller_node.cpp 原始代码没有 G 补偿，
    // 假设 b0 吸收了它，或者 PX4 内部处理了。
    // 我们保持原样，但要注意Z轴的 ref_acc 可能需要额外处理。
    // 为简单起见，我们假设 minisnap 规划器输出的 az_ref 也是 *总* 期望加速度。
    // 如果Z轴跟踪不好，可能需要在这里加上 G (az_ref + 9.81)
    double az_cmd = ladrc_z_->update(z_ref, vz_ref, az_ref, z_meas);

    // 4. Yaw 计算 (ENU)
    double qx = reference_pose_.pose.orientation.x;
    double qy = reference_pose_.pose.orientation.y;
    double qz = reference_pose_.pose.orientation.z;
    double qw = reference_pose_.pose.orientation.w;
    
    // 从四元数计算 ENU 偏航角
    double yaw_enu = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    // 5. 转换为 NED 偏航角
    double yaw_ned = -yaw_enu + M_PI / 2.0;

    // 6. 发布轨迹设定点 (NED)
    publishTrajectorySetpoint(ax_cmd, ay_cmd, az_cmd, yaw_ned);

    // 日志
    if (++log_counter_ >= 50)
    { // Log every second at 50Hz (50 iterations)
      RCLCPP_INFO(this->get_logger(),
                  "Ref: [%.2f, %.2f, %.2f] | Pos: [%.2f, %.2f, %.2f] | Cmd(A): [%.2f, %.2f, %.2f]",
                  x_ref, y_ref, z_ref,      // 指令位置 (Ref)
                  x_meas, y_meas, z_meas, // 实际位置 (Pos)
                  ax_cmd, ay_cmd, az_cmd  // 控制器输出 (Cmd)
      );
      log_counter_ = 0;
    }
  }

  void publishOffboardControlMode()
  {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.position = false;
    msg.velocity = true;      // <-- 我们需要这个来让 PX4 接受加速度
    msg.acceleration = true;  // <-- LADRC 的输出
    msg.attitude = false;
    msg.body_rate = false;

    offboard_mode_pub_->publish(msg);
  }

  // (此函数保持不变，它正确地将 ENU 加速度转换为 NED 加速度)
  void publishTrajectorySetpoint(double ax_enu, double ay_enu, double az_enu, double yaw_ref)
  {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    msg.position = {NAN, NAN, NAN};
    msg.velocity = {NAN, NAN, NAN}; 

    // [正确] 将 ENU 加速度指令 转换为 PX4(NED) 加速度指令
    // NED.acc.x (North) = ENU.acc.y (ay_enu)
    // NED.acc.y (East)  = ENU.acc.x (ax_enu)
    // NED.acc.z (Down)  = -ENU.acc.z (-az_enu)
    msg.acceleration = {
        static_cast<float>(ay_enu),
        static_cast<float>(ax_enu),
        static_cast<float>(-az_enu)};

    msg.yaw = static_cast<float>(yaw_ref);

    trajectory_pub_->publish(msg);
  }

  // Member variables
  std::unique_ptr<ladrc_controller::LADRCController> ladrc_x_;
  std::unique_ptr<ladrc_controller::LADRCController> ladrc_y_;
  std::unique_ptr<ladrc_controller::LADRCController> ladrc_z_;

  // [修改] 订阅者
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr reference_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr reference_twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr reference_accel_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_; // [新增]

  // [修改] 参考存储
  geometry_msgs::msg::PoseStamped reference_pose_;
  geometry_msgs::msg::TwistStamped reference_twist_;
  geometry_msgs::msg::AccelStamped reference_accel_;
  px4_msgs::msg::VehicleOdometry current_odom_;

  // [修改] 状态标志
  bool has_pose_ = false;
  bool has_twist_ = false;
  bool has_accel_ = false;
  bool has_odom_ = false;

  // [新增] 状态机
  std::atomic<FlightState> flight_state_;
  std::atomic<uint64_t> offboard_setpoint_counter_;

  double dt_;
  int log_counter_ = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LADRCPositionControllerNode>());
  rclcpp::shutdown();
  return 0;
}