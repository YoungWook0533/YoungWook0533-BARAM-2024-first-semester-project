#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <Eigen/Dense>
#include <vector>
#include <iomanip>

using namespace std::chrono_literals;
using namespace Eigen;

class FK_AnglePublisher : public rclcpp::Node
{
public:
    FK_AnglePublisher()
    : Node("fk_angle_publisher")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&FK_AnglePublisher::joint_states_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            100ms, std::bind(&FK_AnglePublisher::publish_angles, this));
    }

private:
    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        const std::vector<double> &joint_positions = msg->position;

        if (joint_positions.size() == 7)
        {
            current_angles = joint_positions;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Received joint states size does not match the expected number of joints.");
        }
    }

    void publish_angles()
    {
        // Calculate forward kinematics
        Matrix4d T0_7 = computeForwardKinematics(current_angles);

        // Set the precision for output (6 digits)
        std::cout << std::fixed << std::setprecision(6);

        // Extract RPY from T0_7
        Eigen::Vector3d rpy = rotationMatrixToRPY(T0_7.block<3, 3>(0, 0));

        std::cout << "T0_7 : " << std::endl
                  << T0_7 << std::endl
                  << "Position (x, y, z) : " << T0_7(0, 3) << " " << T0_7(1, 3) << " " << T0_7(2, 3) << std::endl
                  << "Orientation (roll, pitch, yaw) : " << rpy(0) << " " << rpy(1) << " " << rpy(2) << std::endl;
    }

    Matrix4d computeForwardKinematics(const std::vector<double> &joint_angles)
{
    // Define DH parameters
    const double d[7] = {0.36, 0.0, 0.42, 0.0, 0.4, 0.0, 0.126};
    const double a[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const double alpha[7] = {-M_PI / 2, M_PI / 2, M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2, 0};

    Matrix4d T = Matrix4d::Identity();

    for (size_t i = 0; i < 7; ++i)
    {
        Matrix4d Ti = Matrix4d::Identity();
        Ti.block<3, 3>(0, 0) = (AngleAxisd(joint_angles[i], Vector3d(0, 0, 1)) *
                                AngleAxisd(alpha[i], Vector3d(1, 0, 0))).matrix();
        Ti(0, 3) = a[i];
        Ti(2, 3) = d[i];

        T *= Ti;
    }

    return T;
}


    Eigen::Vector3d rotationMatrixToRPY(const Eigen::Matrix3d &R)
    {
        Eigen::Vector3d rpy;

        // Roll (phi)
        rpy(0) = atan2(R(2, 1), R(2, 2));

        // Pitch (theta)
        rpy(1) = atan2(-R(2, 0), sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));

        // Yaw (psi)
        rpy(2) = atan2(R(1, 0), R(0, 0));

        return rpy;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    std::vector<double> current_angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FK_AnglePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
