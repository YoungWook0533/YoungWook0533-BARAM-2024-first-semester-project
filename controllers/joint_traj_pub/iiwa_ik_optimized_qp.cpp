#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <Eigen/Dense>
#include <vector>
#include <iomanip>
#include <chrono>
#include <qpOASES.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;
using namespace Eigen;
using namespace qpOASES;

class IK_AnglePublisher : public rclcpp::Node
{
public:
    IK_AnglePublisher()
        : Node("ik_angle_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("new_angles", 10);
        error_iteration_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("error_iteration_1", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&IK_AnglePublisher::joint_states_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            10ms, std::bind(&IK_AnglePublisher::publish_angles, this)); // Default rate set to 0.01s
    }

private:
    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        const std::vector<double> &joint_positions = msg->position;

        if (joint_positions.size() == 7)
        {
            std::lock_guard<std::mutex> lock(mutex_);
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

            Eigen::Vector3d desired_pos(double_point[0], double_point[1], double_point[2]);
            publish_marker(desired_pos); // Publish the marker
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (current_angles.size() != 7)
            {
                std::cout << "Waiting for initial joint states..." << std::endl;
                return;
            }

            std::vector<double> new_angles = calculate_ik(double_point, current_angles);

            for (const auto &angle : new_angles)
            {
                message.data.push_back(static_cast<float>(angle));
            }
            // Print messages for debug
            std::cout << std::fixed << std::setprecision(6);
            std::cout << "Desired Position: [" << double_point[0] << ", " << double_point[1] << ", " << double_point[2] << "]" << std::endl;
            std::cout << "Desired Orientation (Quaternion): [" << double_point[3] << ", "<<double_point[4] << ", " << double_point[5] << ", " << double_point[6] << "]" << std::endl;
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

            double ee_error = calculate_ee_error(new_angles, double_point);
            auto error_iteration_msg = std_msgs::msg::Float32MultiArray();
            error_iteration_msg.data.push_back(static_cast<float>(ee_error));
            error_iteration_msg.data.push_back(static_cast<float>(qp_iterations));

            error_iteration_publisher_->publish(error_iteration_msg);
        }
    }

    void publish_marker(const Eigen::Vector3d &desired_pos)
    {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "iiwa_base";
        marker.header.stamp = this->now();
        marker.ns = "desired_point";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = desired_pos.x();
        marker.pose.position.y = desired_pos.y();
        marker.pose.position.z = desired_pos.z() + 0.5;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.07;
        marker.scale.y = 0.07;
        marker.scale.z = 0.07;

        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        marker_publisher_->publish(marker);
    }

    double calculate_ee_error(const std::vector<double> &joint_angles, const std::vector<double> &target)
    {
        Eigen::Matrix4d T_cur = forward_kinematics(joint_angles);
        Eigen::Matrix4d T_des = Eigen::Matrix4d::Identity();
        T_des.block<3, 1>(0, 3) = Eigen::Vector3d(target[0], target[1], target[2]);
        Quaterniond q_des(target[6], target[3], target[4], target[5]);
        T_des.block<3, 3>(0, 0) = q_des.toRotationMatrix();

        Eigen::Matrix4d T_diff = T_des * T_cur.inverse();
        Eigen::AngleAxisd angle_axis(T_diff.block<3, 3>(0, 0));

        double pos_error = T_diff.block<3, 1>(0, 3).norm();
        double ori_error = angle_axis.angle();

        return pos_error + ori_error;
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

    std::vector<double> calculate_ik(const std::vector<double> &target, const std::vector<double> &initial_angles)
    {
        Eigen::Vector3d p_des(target[0], target[1], target[2]);
        Quaterniond q_des(target[6], target[3], target[4], target[5]);

        Eigen::MatrixXd J = jacobian(initial_angles);
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

        // Optimize using QP

        int num_variables = 7;

        Eigen::MatrixXd H = 2 * (J.transpose() * J + Eigen::MatrixXd::Identity(num_variables, num_variables)); // Hessian
        Eigen::VectorXd g = -2 * J.transpose() * (u_d - u); // Gradient

        // Joint limits
        Eigen::VectorXd q_max(7);
        q_max << 2.96, 2.09, 2.96, 2.09, 2.96, 2.09, 3.05;
        Eigen::VectorXd q_min(7);
        q_min << -2.96, -2.09, -2.96, -2.09, -2.96, -2.09, -3.05;

        Eigen::VectorXd lb = q_min - Eigen::VectorXd::Map(initial_angles.data(), initial_angles.size());
        Eigen::VectorXd ub = q_max - Eigen::VectorXd::Map(initial_angles.data(), initial_angles.size());

        // Define QP
        QProblem qp(num_variables, 0);
        Options options;
        qp.setOptions(options);

        int nWSR = 15;
        real_t H_qpoases[7 * 7];
        real_t g_qpoases[7];
        real_t lb_qpoases[7];
        real_t ub_qpoases[7];

        for (int i = 0; i < num_variables; ++i)
        {
            for (int j = 0; j < num_variables; ++j)
            {
                H_qpoases[i * num_variables + j] = H(i, j);
            }
            g_qpoases[i] = g(i);
            lb_qpoases[i] = lb(i);
            ub_qpoases[i] = ub(i);
        }

        // Solve QP problem
        returnValue status = qp.init(H_qpoases, g_qpoases, nullptr, lb_qpoases, ub_qpoases, nullptr, nullptr, nWSR);
        if (status != SUCCESSFUL_RETURN) {
            std::cerr << "QP problem initialization failed with status: " << status << std::endl;
            return initial_angles; // Return initial angles if QP fails
        }

        real_t xOpt[7];
        qp.getPrimalSolution(xOpt);

        // Gradient-based dynamic step size
        double gradient_norm = g.norm() + 1e-8;   // Add small value for stability
        double alpha = std::min(1.0, 1.0 / gradient_norm);  // Adjust the step size dynamically

        std::vector<double> new_angles(7);

        const double max_velocities[7] = {1.4835, 1.4835, 1.7453, 1.309, 2.2689, 2.3562, 2.3562};

        // Calculate scaling factor to respect joint velocity limits
        double scaling_factor = 1.0;
        for (size_t i = 0; i < 7; ++i)
        {
            double velocity = std::abs(alpha * xOpt[i] / 0.01); // Velocity for time step of 0.01s
            if (velocity > max_velocities[i])
            {
                double factor = max_velocities[i] / velocity;
                if (factor < scaling_factor)
                {
                    scaling_factor = factor;
                }
            }
        }

        // Apply the scaling factor to respect the velocity limits
        // Starts to oscillate if alpha > 2.0
        for (size_t i = 0; i < 7; ++i)
        {
            new_angles[i] = initial_angles[i] + scaling_factor * alpha * xOpt[i];
        }

        qp_iterations++;

        double ee_error = calculate_ee_error(new_angles, double_point);
        auto error_iteration_msg = std_msgs::msg::Float32MultiArray();
        error_iteration_msg.data.push_back(static_cast<float>(ee_error));
        error_iteration_msg.data.push_back(static_cast<float>(qp_iterations));
            
        error_iteration_publisher_->publish(error_iteration_msg);

        Eigen::Matrix4d final_transform = forward_kinematics(new_angles);
        Eigen::Vector3d final_pos = final_transform.block<3, 1>(0, 3);
        Eigen::Matrix3d final_rot = final_transform.block<3, 3>(0, 0);
        Quaterniond final_quat(final_rot);

        double pos_error = (final_pos - p_des).norm();
        double ori_error = final_quat.angularDistance(q_des);
        double total_error = pos_error + ori_error;

        if (total_error < 0.001)
        {
            std::cout << "Iterations: " << qp_iterations << std::endl;
            rclcpp::shutdown();
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

    int qp_iterations = 0;
    bool first_time = true;
    std::vector<double> current_angles;
    std::vector<double> double_point;
    std::mutex mutex_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr error_iteration_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<IK_AnglePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
