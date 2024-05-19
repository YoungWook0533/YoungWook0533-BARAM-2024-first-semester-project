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
            100ms, std::bind(&IK_AnglePublisher::publish_angles, this)); // Default state pub rate set to 0.1s
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
        auto message = std_msgs::msg::Float32MultiArray();

        if (first_time)
        {
            first_time = false;
            double_point.resize(6);
            std::cout << "Enter desired position (x, y, z) and orientation (roll, pitch, yaw): ";
            for (size_t i = 0; i < 6; ++i)
            {
                std::cin >> double_point[i];
            }
        }

        if (current_angles.size() != 7)
        {
            std::cout << "Waiting for initial joint states..." << std::endl;
            return;
        }

        std::vector<double> new_angles = calculate_ik(double_point);

        for (const auto &angle : new_angles)
        {
            message.data.push_back(static_cast<float>(angle));
        }
        // Print messages for debug
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "Desired Position: [" << double_point[0] << ", " << double_point[1] << ", " << double_point[2] << "]" << std::endl;
        std::cout << "Desired Orientation: [" << double_point[3] << ", " << double_point[4] << ", " << double_point[5] << "]" << std::endl;
        std::cout << "Current Angles: ";
        for (const auto &angle : current_angles)
        {
            std::cout << angle << " ";
        }
        std::cout << std::endl;
        std::cout << "New Angles: ";
        for (const auto &angle : new_angles)
        {
            std::cout << angle << " ";
        }
        std::cout << std::endl;

        publisher_->publish(message);
    }

    Eigen::MatrixXd jacobian(const std::vector<double> &joint_angles)
    {
        const double d[7] = {0.36, 0.0, 0.42, 0.0, 0.4, 0.0, 0.126};
        const double a[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        const double alpha[7] = {-M_PI / 2, M_PI / 2, M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2, 0};

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        std::vector<Eigen::Matrix4d> transforms(7, Eigen::Matrix4d::Identity());
        // Compute the forward kinematics
        for (size_t i = 0; i < 7; ++i)
        {
            Eigen::Matrix4d Ti;
            // screw_z(d_i,theta_i)*screw_x(a_i,alpha_i)
            Ti = (Eigen::AngleAxisd(joint_angles[i], Eigen::Vector3d(0, 0, 1)) * 
                Eigen::Translation3d(0, 0, d[i]) * 
                Eigen::Translation3d(a[i], 0, 0) * 
                Eigen::AngleAxisd(alpha[i], Eigen::Vector3d(1, 0, 0))).matrix();
            T *= Ti;
            // Saves cumulative transformation matrix (used for calculating J)
            transforms[i] = T;
        }

        Eigen::Vector3d p7 = transforms[6].block<3, 1>(0, 3); // EE position

        Eigen::MatrixXd J(6, 7); // 6x7 Jacobian matrix

        Eigen::Vector3d z0 = Eigen::Vector3d(0, 0, 1);

        J.block<3, 1>(0, 0) = z0.cross(p7); // p7-p0 = p7
        J.block<3, 1>(3, 0) = z0;

        for (size_t i = 0; i < 6; ++i)
        {
            Eigen::Matrix3d Ri = transforms[i].block<3, 3>(0, 0);
            Eigen::Vector3d pi = transforms[i].block<3, 1>(0, 3); // p1~p6

            Eigen::Vector3d zi = Ri * Eigen::Vector3d(0, 0, 1); // z1~z6
            Eigen::Vector3d p7_pi = p7 - pi;

            J.block<3, 1>(0, i + 1) = zi.cross(p7_pi); // Linear velocity component
            J.block<3, 1>(3, i + 1) = zi;              // Angular velocity component
        }

        return J;
    }

    Eigen::MatrixXd computeDampedPseudoInverse(const Eigen::MatrixXd &J, double lambda)
    {
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
        Eigen::MatrixXd dampedJ = J * J.transpose() + lambda * lambda * I;
        Eigen::MatrixXd invJ = J.transpose() * dampedJ.inverse();

        return invJ;
    }

    std::vector<double> calculate_ik(const std::vector<double> &target)
    {
        Eigen::Vector3d p_des(target[0], target[1], target[2]);
        Eigen::Vector3d rpy_des(target[3], target[4], target[5]);

        Eigen::MatrixXd J = jacobian(current_angles);
        Eigen::MatrixXd invJ = computeDampedPseudoInverse(J, 0.3);

        Eigen::Vector3d p_cur = forward_kinematics_position(current_angles);
        Eigen::Vector3d rpy_cur = forward_kinematics_orientation(current_angles);

        Eigen::VectorXd u(6);
        u.head<3>() = p_cur;
        u.tail<3>() = rpy_cur;

        Eigen::VectorXd u_d(6);
        u_d.head<3>() = p_des;
        u_d.tail<3>() = rpy_des;

        Eigen::VectorXd u_dot_des = Eigen::VectorXd::Zero(6); // Assuming zero desired velocity for simplicity
        Eigen::MatrixXd Kp = 7.0 * Eigen::MatrixXd::Identity(6, 6); // Increased gain for proportional control

        Eigen::VectorXd gradH(7);
        gradH = 2 * Eigen::VectorXd::Map(current_angles.data(), 7); // Optimal target function : minimum joint movements

        Eigen::VectorXd q_dot = invJ * (u_dot_des + Kp * (u_d - u)) - (Eigen::MatrixXd::Identity(7, 7) - invJ * J) * gradH;

        std::cout << "Joint Velocities (q_dot): \n" << q_dot << std::endl;

        std::vector<double> new_angles(7);
        for (size_t i = 0; i < 7; ++i)
        {
            new_angles[i] = current_angles[i] + 0.1 * q_dot[i];
        }

        return new_angles;
    }

    Eigen::Vector3d forward_kinematics_position(const std::vector<double> &joint_angles)
    {
        const double d[7] = {0.36, 0.0, 0.42, 0.0, 0.4, 0.0, 0.126};
        const double a[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        const double alpha[7] = {-M_PI / 2, M_PI / 2, M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2, 0};

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        for (size_t i = 0; i < 7; ++i)
        {
            Eigen::Matrix4d Ti = Eigen::Matrix4d::Identity();
            // screw_z(d_i,theta_i)*screw_x(a_i,alpha_i)
            Ti = (Eigen::AngleAxisd(joint_angles[i], Eigen::Vector3d(0, 0, 1)) * 
                Eigen::Translation3d(0, 0, d[i]) * 
                Eigen::Translation3d(a[i], 0, 0) * 
                Eigen::AngleAxisd(alpha[i], Eigen::Vector3d(1, 0, 0))).matrix();
            T *= Ti;
        }

        return T.block<3, 1>(0, 3);
    }

    Eigen::Vector3d forward_kinematics_orientation(const std::vector<double> &joint_angles)
    {
        const double d[7] = {0.36, 0.0, 0.42, 0.0, 0.4, 0.0, 0.126};
        const double a[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        const double alpha[7] = {-M_PI / 2, M_PI / 2, M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2, 0};

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        for (size_t i = 0; i < 7; ++i)
        {
            Eigen::Matrix4d Ti = Eigen::Matrix4d::Identity();
            // screw_z(d_i,theta_i)*screw_x(a_i,alpha_i)
            Ti = (Eigen::AngleAxisd(joint_angles[i], Eigen::Vector3d(0, 0, 1)) * 
                Eigen::Translation3d(0, 0, d[i]) * 
                Eigen::Translation3d(a[i], 0, 0) * 
                Eigen::AngleAxisd(alpha[i], Eigen::Vector3d(1, 0, 0))).matrix();
            T *= Ti;
        }

        Eigen::Matrix3d R = T.block<3, 3>(0, 0);
        Eigen::Vector3d rpy;
        rpy[0] = std::atan2(R(2, 1), R(2, 2));
        rpy[1] = std::atan2(-R(2, 0), std::sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
        rpy[2] = std::atan2(R(1, 0), R(0, 0));

        return rpy;
    }

    bool first_time = true;
    std::vector<double> current_angles;
    std::vector<double> double_point;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<IK_AnglePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
