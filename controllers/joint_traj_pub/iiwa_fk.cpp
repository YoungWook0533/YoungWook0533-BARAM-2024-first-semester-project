#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <Eigen/Dense>
#include <iomanip> //for formatting calculated result

using namespace std::chrono_literals;
using namespace Eigen;

class FK_AnglePublisher : public rclcpp::Node
{
public:
    FK_AnglePublisher()
    : Node("fk_angle_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("new_angles", 10);
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&FK_AnglePublisher::publish_angles, this));
    }

private:
    void publish_angles()
    {
        auto message = std_msgs::msg::Float32MultiArray();
        std::vector<double> double_angles;

        std::cout << "Enter joint angles : ";
        for (size_t i = 0; i < 7; ++i)
        {
            double angle;
            std::cin >> angle;
            double_angles.push_back(angle);
        }

        for (const auto &angle : double_angles)
        {
            message.data.push_back(static_cast<float>(angle));
        }

        // Calculate forward kinematics

        Matrix4d T0_7 = computeForwardKinematics(double_angles);

        // Set the precision for output
        std::cout << std::fixed << std::setprecision(6);

        std::cout << "T0_7 : " << std::endl<< T0_7 << std::endl<< "Position(x,y,z) : " << T0_7(0,3) << " " << T0_7(1,3) << " " << T0_7(2,3) << std::endl;;

        publisher_->publish(message);
    }

    Matrix4d computeForwardKinematics(const std::vector<double> &joint_angles)
    {
        // Define DH parameters
        const double d[7] = {0.34, 0.0, 0.4, 0.0, 0.4, 0.0, 0.126};
        const double a[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        const double alpha[7] = {-M_PI / 2, M_PI / 2, M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2, 0};

        Matrix4d T = Matrix4d::Identity();

        for (size_t i = 0; i < 7; ++i)
        {
            Matrix4d Ti;
            Ti = (AngleAxisd(joint_angles[i], Vector3d(0, 0, 1)) * Translation3d(0, 0, d[i])        //screw_z(d_i,theta_i)*screw_x(a_i,alpha_i)
                * Translation3d(a[i], 0, 0) * AngleAxisd(alpha[i], Vector3d(1, 0, 0))).matrix();

            T *= Ti;
        }

        return T;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FK_AnglePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
