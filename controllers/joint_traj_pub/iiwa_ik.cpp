#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <Eigen/Dense>
#include <vector>
#include <iomanip>
#include <chrono>

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
        if (first_time)
        {
            first_time = false;
            double_point.resize(7); // Resize to hold position (x, y, z) and quaternion (x, y, z, w)
            std::cout << "Enter desired position (x, y, z) and orientation (roll, pitch, yaw): ";
            for (size_t i = 0; i < 6; ++i)
            {
                std::cin >> double_point[i];
            }

            // Convert roll, pitch, yaw to quaternion
            AngleAxisd rollAngle(double_point[3], Vector3d::UnitX());
            AngleAxisd pitchAngle(double_point[4], Vector3d::UnitY());
            AngleAxisd yawAngle(double_point[5], Vector3d::UnitZ());
            Quaterniond q = yawAngle * pitchAngle * rollAngle;

            double_point[3] = q.x();
            double_point[4] = q.y();
            double_point[5] = q.z();
            double_point[6] = q.w();
        }

        if (current_angles.size() != 7)
        {
            std::cout << "Waiting for initial joint states..." << std::endl;
            return;
        }

        std::vector<double> initial_angles = current_angles;
        std::vector<double> new_angles = initial_angles;

        int i = 0;
        Eigen::Vector3d desired_pos(double_point[0], double_point[1], double_point[2]);
        Quaterniond desired_quat(double_point[6], double_point[3], double_point[4], double_point[5]);

        auto start_time = std::chrono::high_resolution_clock::now();

        while (true)
        {
            new_angles = calculate_ik(double_point, initial_angles);
            initial_angles = new_angles;

            Eigen::Matrix4d temp_transform = forward_kinematics(initial_angles);
            Eigen::Vector3d temp_pos = temp_transform.block<3, 1>(0, 3);
            Eigen::Matrix3d temp_rot = temp_transform.block<3, 3>(0, 0);
            Quaterniond temp_quat(temp_rot);

            double pos_error = (temp_pos - desired_pos).norm();
            double ori_error = temp_quat.angularDistance(desired_quat);
            double total_error = pos_error + ori_error;

            i++;

            if (i > 550 && total_error < 0.15)
            {
                break;
            }

            auto current_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed_time = current_time - start_time;

            if (elapsed_time.count() > 5.0)
            {
                std::cout << "Failed to calculate solution" << std::endl;
                rclcpp::shutdown();
                return;
            }
        }

        auto message = std_msgs::msg::Float32MultiArray();

        for (const auto &angle : new_angles)
        {
            message.data.push_back(static_cast<float>(angle));
        }

        // Print messages for debug
        Eigen::Matrix4d final_transform = forward_kinematics(new_angles);
        Eigen::Vector3d final_pos = final_transform.block<3, 1>(0, 3);
        Eigen::Matrix3d final_rot = final_transform.block<3, 3>(0, 0);
        Quaterniond final_quat(final_rot);
        Eigen::Vector3d rpy_des = quaternionToRPY(desired_quat);
        Eigen::Vector3d rpy_final = quaternionToRPY(final_quat);

        std::cout << std::fixed << std::setprecision(6);
        std::cout << "Desired Position: [" << double_point[0] << ", " << double_point[1] << ", " << double_point[2] << "]" << std::endl;
        std::cout << "Desired Orientation: [" << rpy_des[0] << ", " << rpy_des[1] << ", " << rpy_des[2] << "]" << std::endl;
        std::cout << "Final Position: [" << final_pos[0] << ", " << final_pos[1] << ", " << final_pos[2] << "]" << std::endl;
        std::cout << "Final Orientation: [" << rpy_final[0] << ", " << rpy_final[1] << ", " << rpy_final[2] << "]" << std::endl;
        std::cout << "Final Angles: ";
        for (const auto &angle : new_angles)
        {
            std::cout << angle << " ";
        }
        std::cout << std::endl;

        publisher_->publish(message);
        
        // Shutdown the node after publishing
        rclcpp::shutdown();
    }

    Eigen::Vector3d quaternionToRPY(const Quaterniond &q)
    {
        Eigen::Matrix3d R = q.toRotationMatrix();
        Eigen::Vector3d rpy;
        rpy[0] = std::atan2(R(2, 1), R(2, 2)); // Roll
        rpy[1] = std::atan2(-R(2, 0), std::sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2))); // Pitch
        rpy[2] = std::atan2(R(1, 0), R(0, 0)); // Yaw
        return rpy;
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

    std::vector<double> calculate_ik(const std::vector<double> &target, const std::vector<double> &initial_angles)
    {
        Eigen::Vector3d p_des(target[0], target[1], target[2]);
        Quaterniond q_des(target[6], target[3], target[4], target[5]);

        Eigen::MatrixXd J = jacobian(initial_angles);
        Eigen::MatrixXd invJ = computeDampedPseudoInverse(J, 0.3);

        Eigen::Matrix4d T_cur = forward_kinematics(initial_angles);
        Eigen::Vector3d p_cur = T_cur.block<3, 1>(0, 3);
        Eigen::Matrix3d R_cur = T_cur.block<3, 3>(0, 0);
        Quaterniond q_cur(R_cur);

        Eigen::VectorXd u(6);
        u.head<3>() = p_cur;
        AngleAxisd angle_axis_cur(q_cur);
        u.tail<3>() = angle_axis_cur.axis() * angle_axis_cur.angle();

        Eigen::VectorXd u_d(6);
        u_d.head<3>() = p_des;
        AngleAxisd angle_axis_des(q_des);
        u_d.tail<3>() = angle_axis_des.axis() * angle_axis_des.angle();

        Eigen::VectorXd u_dot_des = Eigen::VectorXd::Zero(6); // Zero desired velocity
        Eigen::MatrixXd Kp = 7.0 * Eigen::MatrixXd::Identity(6, 6); // Proportional gain matrix

        // Joint limits
        const double joint_max_limits[7] = {2.96, 2.09, 2.96, 2.09, 2.96, 2.09, 3.05};
        const double joint_min_limits[7] = {-2.96, -2.09, -2.96, -2.09, -2.96, -2.09, -3.05};

        // Define the gradient for the optimal target function
        Eigen::VectorXd gradH(7);
        gradH.setZero();

        gradH(1) = 2 * initial_angles[1];
        gradH(5) = 2 * initial_angles[5];

        Eigen::VectorXd q_dot = invJ * (u_dot_des + Kp * (u_d - u)) - (Eigen::MatrixXd::Identity(7, 7) - invJ * J) * gradH;

        std::vector<double> new_angles(7);
        for (size_t i = 0; i < 7; ++i)
        {
            new_angles[i] = initial_angles[i] + 0.1 * q_dot[i];
            new_angles[i] = std::min(std::max(new_angles[i], joint_min_limits[i]), joint_max_limits[i]); // Ensure result is within joint limits
        }

        return new_angles;
    }

    Eigen::Matrix4d forward_kinematics(const std::vector<double> &joint_angles)
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

        return T;
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
