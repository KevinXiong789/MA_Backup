// every joint position              **************************************************************************can run
#include <memory>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <Eigen/Dense>

class JointPoseListener : public rclcpp::Node
{
public:
    JointPoseListener()
    : Node("joint_pose_listener")
    {
        joint_positions_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/robot/joint_positions", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&JointPoseListener::timer_callback, this));
    }

    void init()
    {
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), "robot_description");
        robot_model_ = robot_model_loader_->getModel();
    }

private:
    void timer_callback()
    {
        auto kinematic_state = move_group_interface_->getCurrentState();
        const moveit::core::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup("ur_manipulator");
        const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

        std_msgs::msg::Float32MultiArray joint_positions_msg;

        // Assuming the order is elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint
        for (const std::string& joint_name : joint_names)
        {
            const moveit::core::JointModel* joint_model = robot_model_->getJointModel(joint_name);
            if (joint_model)
            {
                const std::string& child_link_name = joint_model->getChildLinkModel()->getName();
                Eigen::Isometry3d link_state = kinematic_state->getGlobalLinkTransform(child_link_name);
                Eigen::Vector3d link_position = link_state.translation();

                if(joint_name == "elbow_joint" || joint_name == "wrist_1_joint" || joint_name == "wrist_2_joint" || joint_name == "wrist_3_joint") {
                    joint_positions_msg.data.push_back(link_position.x());
                    joint_positions_msg.data.push_back(link_position.y());
                    joint_positions_msg.data.push_back(link_position.z());
                }

                RCLCPP_INFO(this->get_logger(), "Joint %s position: x: %f, y: %f, z: %f", joint_name.c_str(), link_position.x(), link_position.y(), link_position.z());
            }
        }

        joint_positions_publisher_->publish(joint_positions_msg);
    }


    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_positions_publisher_;
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    moveit::core::RobotModelPtr robot_model_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointPoseListener>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}