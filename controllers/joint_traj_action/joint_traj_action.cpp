#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class JointTrajActionNode : public rclcpp::Node {
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    JointTrajActionNode() : Node("joint_traj_action_node") {
        // Subscribe to the topic publishing joint angles
        joint_angles_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "new_angles", 10, std::bind(&JointTrajActionNode::jointAnglesCallback, this, std::placeholders::_1));

        // Create action client for FollowJointTrajectory
        action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, "iiwa_arm_controller/follow_joint_trajectory");

        // Define joint names
        joint_names_ = {"joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6", "joint_a7"};

        // Set the time from start
        point_.time_from_start = rclcpp::Duration(3, 0);
    }

private:
    void jointAnglesCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        // Create joint trajectory message
        auto goal_msg = FollowJointTrajectory::Goal();
        
        // Populate joint names
        goal_msg.trajectory.joint_names = joint_names_;

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
        goal_msg.trajectory.points.push_back(point);

        // Create SendGoalOptions and set callbacks
        rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions send_goal_options;
        
        send_goal_options.goal_response_callback = std::bind(&JointTrajActionNode::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.result_callback = std::bind(&JointTrajActionNode::result_callback, this, std::placeholders::_1);

        // Send goal to the action server
        action_client_->async_send_goal(goal_msg, send_goal_options);

        std::cout << "Published joint_trajectory : ";
        for (const auto& position : point.positions) {
            std::cout << position << " ";
        }
        std::cout << std::endl;
    }

    void goal_response_callback(const GoalHandleFollowJointTrajectory::SharedPtr & goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the action server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by the action server, waiting for result");
        }
    }

    void result_callback(const GoalHandleFollowJointTrajectory::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_angles_subscriber_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
    std::vector<std::string> joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint point_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::cout << "*time_from_start fixed to 3s" << std::endl;
    auto node = std::make_shared<JointTrajActionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
