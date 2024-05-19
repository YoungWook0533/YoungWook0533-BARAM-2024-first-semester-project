#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class ManipulatorControllerNode : public rclcpp::Node {
public:
    ManipulatorControllerNode() : Node("manipulator_controller_node") {
        // Subscribe to the topic publishing joint angles
        joint_angles_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "new_angles", 10, std::bind(&ManipulatorControllerNode::jointAnglesCallback, this, std::placeholders::_1));

        // Publisher for joint trajectory
        joint_trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "iiwa_arm_controller/joint_trajectory", 10);

        // Define joint names
        joint_names_ = {"joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6", "joint_a7"};

        // Set the time from start
        point_.time_from_start = rclcpp::Duration(0, 50);   // Default state pub rate set to 200Hz (50ms)
    }

private:
    void jointAnglesCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        // Create joint trajectory message
        auto joint_trajectory_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();

        // Populate joint names
        joint_trajectory_msg->joint_names = joint_names_;

        // Convert float to double
        std::vector<double> joint_positions;
        joint_positions.reserve(msg->data.size());
        for (const auto& angle : msg->data) {
            joint_positions.push_back(static_cast<double>(angle));
        }

        // Populate joint positions
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = joint_positions;
        point.time_from_start = point_.time_from_start;
        joint_trajectory_msg->points.push_back(point);

        // Publish joint trajectory
        joint_trajectory_publisher_->publish(*joint_trajectory_msg);

    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_angles_subscriber_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_publisher_;
    std::vector<std::string> joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint point_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::cout << "* Default state pub rate set to 0.05s" << std::endl;
    auto node = std::make_shared<ManipulatorControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}