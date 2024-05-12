#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <Eigen/Dense>

using namespace std::chrono_literals;
using namespace Eigen;

class IK_AnglePublisher : public rclcpp::Node
{
  public:
    IK_AnglePublisher()
    : Node("ik_angle_publisher")
    {
      publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("ik_angles", 10);
      timer_ = this->create_wall_timer(
      1000ms, std::bind(&IK_AnglePublisher::publish_angles, this));
    }

  private:
    void publish_angles()
    {
        auto message = std_msgs::msg::Float32MultiArray();
        std::vector<double> double_angles;

        std::cout << "Enter desired position&orientation : ";
        for (size_t i = 0; i < 6; ++i)
        {
            double angle;
            std::cin >> angle;
            double_angles.push_back(angle);
        }

        for (const auto& angle : double_angles) {
            message.data.push_back(static_cast<float>(angle));
        }
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
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
