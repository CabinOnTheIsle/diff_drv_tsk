#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>

class DiffDriveController : public rclcpp::Node
{
public:
  DiffDriveController() : Node("diff_drv_tsk")
  {
    // Declare parameters with robot values
    this->declare_parameter<double>("wheelbase", 0.35);
    this->declare_parameter<double>("wheel_radius", 0.05);
    this->declare_parameter<double>("max_rpm", 150.0);

    // Get initial parameter values
    wheelbase_   = this->get_parameter("wheelbase").as_double();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    max_rpm_      = this->get_parameter("max_rpm").as_double();

    // Set up subscriber and publishers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&DiffDriveController::cmdVelCallback, this, std::placeholders::_1));

    left_rpm_pub_ = this->create_publisher<std_msgs::msg::Float64>("/left_wheel_rpm", 10);
    right_rpm_pub_ = this->create_publisher<std_msgs::msg::Float64>("/right_wheel_rpm", 10);

    RCLCPP_INFO(this->get_logger(), "DiffDriveController node has started.");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double linear_x = msg->linear.x;     // m/s
    double angular_z = msg->angular.z;   // rad/s

    // Compute wheel speeds in rad/s using differential drive kinematics
    // v_left  = ( linear_x - (angular_z * wheelbase_ / 2) ) / wheel_radius_
    // v_right = ( linear_x + (angular_z * wheelbase_ / 2) ) / wheel_radius_

    double v_left_rad  = (linear_x - (angular_z * wheelbase_ / 2.0)) / wheel_radius_;
    double v_right_rad = (linear_x + (angular_z * wheelbase_ / 2.0)) / wheel_radius_;

    // Convert rad/s to RPM:  rpm = rad/s * (60 / 2π)
    double left_rpm  = v_left_rad  * (60.0 / (2.0 * M_PI));
    double right_rpm = v_right_rad * (60.0 / (2.0 * M_PI));

    // Clamp RPM to max_rpm_
    if (left_rpm > max_rpm_)  left_rpm = max_rpm_;
    if (left_rpm < -max_rpm_) left_rpm = -max_rpm_;

    if (right_rpm > max_rpm_)   right_rpm = max_rpm_;
    if (right_rpm < -max_rpm_)  right_rpm = -max_rpm_;

    // Publish
    auto left_msg = std_msgs::msg::Float64();
    left_msg.data = left_rpm;
    left_rpm_pub_->publish(left_msg);

    auto right_msg = std_msgs::msg::Float64();
    right_msg.data = right_rpm;
    right_rpm_pub_->publish(right_msg);
  }

  // Parameter variables
  double wheelbase_;
  double wheel_radius_;
  double max_rpm_;

  // ROS 2 interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_rpm_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_rpm_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffDriveController>());
  rclcpp::shutdown();
  return 0;
}
