#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <Eigen/Dense>
#include <vector>
#include <iomanip>

using namespace std::chrono_literals;
using namespace Eigen;

class IK_AnglePublisher : public rclcpp::Node
{
  public:
    IK_AnglePublisher()
    : Node("ik_angle_publisher")
    {
      publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("new_angles", 10);
      
      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&IK_AnglePublisher::joint_states_callback, this, std::placeholders::_1));
      
      timer_ = this->create_wall_timer(
      1000ms, std::bind(&IK_AnglePublisher::publish_angles, this));
    }

  private:
    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      const std::vector<double>& joint_positions = msg->position;

      if (joint_positions.size() == 7) {
          // Copy the received joint positions to the double_angles vector
          current_angles = joint_positions;
      } 
      else {
          // If the received joint positions size doesn't match, print an error message
          RCLCPP_ERROR(this->get_logger(), "Received joint states size does not match the expected number of joints.");
      }
    }
    void publish_angles()
    {
      auto message = std_msgs::msg::Float32MultiArray();

      std::cout << "Enter desired position&orientation : ";
      for (size_t i = 0; i < 6; ++i)
      {
          double point;
          std::cin >> point;
          double_point.push_back(point);
      }
        


      //Push back calculated angles to publish
      for (const auto& angle : double_angles) {
          message.data.push_back(static_cast<float>(angle));
      }
      // Set the precision for output(6digits)
      std::cout << std::fixed << std::setprecision(6);

      for(size_t i = 0; i < 6; ++i)
      {
        std::cout << double_point[i] << " ";
      }
      std::cout << std::endl;

      Eigen::MatrixXd J = jacobian(current_angles);

      std::cout << "Jacobian:\n" << J << std::endl;

      publisher_->publish(message);
    }

    Eigen::MatrixXd jacobian(const std::vector<double>& joint_angles)
    {
      const double alpha[7] = {-M_PI / 2, M_PI / 2, M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2, 0};
      const double a[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      const double d[7] = {0.34, 0.0, 0.4, 0.0, 0.4, 0.0, 0.126};

      Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
      std::vector<Eigen::Matrix4d> transforms(7, Eigen::Matrix4d::Identity());

      // Compute the forward kinematics
      for (size_t i = 0; i < 7; ++i)
      {
        Eigen::Matrix4d Ti;
        Ti = (Eigen::AngleAxisd(joint_angles[i], Eigen::Vector3d(0, 0, 1)) * 
              Eigen::Translation3d(0, 0, d[i]) * 
              Eigen::Translation3d(a[i], 0, 0) * 
              Eigen::AngleAxisd(alpha[i], Eigen::Vector3d(1, 0, 0))).matrix();
        T *= Ti;
        // Saves cumulative transformation matrix (used for calculating J)
        transforms[i] = T;
      }

      Eigen::Vector3d p0(0, 0, 0);

      Eigen::MatrixXd J(6, 7); // 6x7 Jacobian matrix

      for (size_t i = 0; i < 7; ++i)
      {
        Eigen::Matrix3d Ri = transforms[i].block<3, 3>(0, 0);
        Eigen::Vector3d pi = transforms[i].block<3, 1>(0, 3);

        Eigen::Vector3d zi = Ri * Eigen::Vector3d(0, 0, 1);
        Eigen::Vector3d pi_p0 = pi - p0;

        // Linear velocity component
        J.block<3, 1>(0, i) = zi.cross(pi_p0);

        // Angular velocity component
        J.block<3, 1>(3, i) = zi;
      }

      return J;
    }
    
    std::vector<double> calculate_ik()
    {
      
    }

    std::vector<double> double_angles;
    std::vector<double> current_angles;
    std::vector<double> double_point;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<IK_AnglePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}