#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class JointStatePublisher : public rclcpp::Node
{
public:
    JointStatePublisher()
    : Node("joint_state_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // Initialize joint states
        joint_names_ = {"joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6", "joint_a7"};
        joint_angles_.resize(joint_names_.size(), 0.0);

        // Subscribe to new_angles topic
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "new_angles", 10, std::bind(&JointStatePublisher::new_angles_callback, this, std::placeholders::_1));

        // Get state publish rate parameter
        this->declare_parameter("state_publish_rate", 200.0);
        this->get_parameter("state_publish_rate", state_publish_rate_);
        publish_period_ = std::chrono::milliseconds(static_cast<int>(1000.0 / state_publish_rate_));
    }

    double get_state_publish_rate() const
    {
        return state_publish_rate_;
    }

    void publish_joint_states()
    {
        auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
        joint_state_msg->header.stamp = this->now();
        joint_state_msg->name = joint_names_;
        joint_state_msg->position = joint_angles_;
        publisher_->publish(*joint_state_msg);
    }

private:
    void new_angles_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // Update joint angles from received data
        if (msg->data.size() == joint_angles_.size())
        {
            std::cout << "Recieved joint angles : ";
            for (size_t i = 0; i < joint_angles_.size(); ++i)
            {
                joint_angles_[i] = static_cast<double>(msg->data[i]);
                std::cout << joint_angles_[i]<< " ";
            }
            
            std::cout << std::endl;
            
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    std::vector<std::string> joint_names_;
    std::vector<double> joint_angles_;
    double state_publish_rate_;
    std::chrono::milliseconds publish_period_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStatePublisher>();

    double state_publish_rate = node->get_state_publish_rate();
    rclcpp::WallRate loop_rate(std::chrono::milliseconds(static_cast<long int>(1000.0 / state_publish_rate)));
    while (rclcpp::ok())
    {
        node->publish_joint_states();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
