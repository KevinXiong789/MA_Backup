/*
// Min_distanc between Robot (6 Joints and their connections) and Point cloud           **********************************************can run
#include <memory>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/float64.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <Eigen/Dense>

class JointPoseListener : public rclcpp::Node
{
public:
    JointPoseListener()
    : Node("joint_pose_listener")
    {
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/processed_point_cloud", 10, std::bind(&JointPoseListener::point_cloud_callback, this, std::placeholders::_1));
        distance_pub_ = this->create_publisher<std_msgs::msg::Float64>("/pointcloud/minimum_distance", 10);
    }

    void init()
    {
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), "robot_description");
        robot_model_ = robot_model_loader_->getModel();
    }

private:
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Check if the point cloud is empty
        if(cloud->points.empty()) {
            std_msgs::msg::Float64 distance_msg;
            distance_msg.data = 10.0; // Setting distance to 10 when cloud is empty
            RCLCPP_INFO(this->get_logger(), "Point cloud is empty. Min_distance is set to %f", distance_msg.data);
            distance_pub_->publish(distance_msg);
            return;  // Exit the callback
        }

        // Transform the point cloud to the robot's coordinate system
        for (auto& point : cloud->points)
        {
            float x = point.x;
            float y = point.y;
            float z = point.z;
            point.x = z;
            point.y = -x;
            point.z = -y;
        }

        auto kinematic_state = move_group_interface_->getCurrentState();
        const moveit::core::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup("ur_manipulator");
        const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

        double min_distance = std::numeric_limits<double>::max();
        for (size_t i = 0; i < joint_names.size() - 1; ++i)
        {
            const moveit::core::JointModel* joint_model1 = robot_model_->getJointModel(joint_names[i]);
            const moveit::core::JointModel* joint_model2 = robot_model_->getJointModel(joint_names[i+1]);
            if (joint_model1 && joint_model2)
            {
                const std::string& child_link_name1 = joint_model1->getChildLinkModel()->getName();
                const std::string& child_link_name2 = joint_model2->getChildLinkModel()->getName();
                Eigen::Isometry3d link_state1 = kinematic_state->getGlobalLinkTransform(child_link_name1);
                Eigen::Isometry3d link_state2 = kinematic_state->getGlobalLinkTransform(child_link_name2);
                Eigen::Vector3d link_position1 = link_state1.translation();
                Eigen::Vector3d link_position2 = link_state2.translation();

                for (const auto& point : cloud->points)
                {
                    Eigen::Vector3d point_position(point.x, point.y, point.z);
                    double distance = distancePointSegment(point_position, link_position1, link_position2);
                    if (distance < min_distance)
                    {
                        min_distance = distance;
                    }
                }
            }
        }

        std_msgs::msg::Float64 distance_msg;
        distance_msg.data = min_distance;
        RCLCPP_INFO(this->get_logger(), "Min_distance is %f", min_distance);
        distance_pub_->publish(distance_msg);
    }

    double distancePointSegment(const Eigen::Vector3d& point, const Eigen::Vector3d& segStart, const Eigen::Vector3d& segEnd)
    {
        Eigen::Vector3d diff = point - segStart;
        Eigen::Vector3d dir = segEnd - segStart;
        double t = diff.dot(dir);
        if (t <= 0.0)
        {
            // point is nearest to segStart
            t = 0.0;
        }
        else
        {
            double sqlen = dir.squaredNorm();  // segment length squared
            if (t >= sqlen)
            {
                // point is nearest to segEnd
                t = 1.0;
                diff -= dir;
            }
            else
            {
                // point is nearest to middle of segment
                t /= sqlen;
                diff -= dir * t;
            }
        }
        return diff.norm();
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
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
*/




// Min_distanc between Robot (6 Joints and their connections) and Human Skeleton Joints Points           **********************************************can run
#include <memory>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/float64.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <Eigen/Dense>
#include <std_msgs/msg/float32_multi_array.hpp>

class JointPoseListener : public rclcpp::Node
{
public:
    JointPoseListener()
    : Node("joint_pose_listener")
    {
        joint_position_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/Openpose/joints_position", 10, std::bind(&JointPoseListener::joint_position_callback, this, std::placeholders::_1));
            //"/Nuitrack/joints_position", 10, std::bind(&JointPoseListener::joint_position_callback, this, std::placeholders::_1));
        distance_pub_ = this->create_publisher<std_msgs::msg::Float64>("/Jointpoints/minimum_distance", 10);

    }

    void init()
    {
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), "robot_description");
        robot_model_ = robot_model_loader_->getModel();
    }

private:
    void joint_position_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        std::vector<Eigen::Vector3d> points;
        for (size_t i = 0; i < msg->data.size(); i += 3) {
            
            // Transform the point cloud to the robot's coordinate system
            // just for Openpose coordinate system
            float x = msg->data[i+2];
            float y = -msg->data[i];
            float z = -msg->data[i+1];
            
            /*
            // for Nuitrack not
            float x = msg->data[i];
            float y = msg->data[i+1];
            float z = msg->data[i+2];
            */
            if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z)) {
                points.push_back(Eigen::Vector3d(x, y, z));
                
            }
        }

        if (msg->data[0] == 100) {
            std_msgs::msg::Float64 distance_msg;
            distance_msg.data = 10.0;  // Setting distance to 10 when all points are nan
            RCLCPP_INFO(this->get_logger(), "All joint points are NaN. Min_distance is set to %f", distance_msg.data);
            distance_pub_->publish(distance_msg);
            return;  // Exit the callback
        }

        auto kinematic_state = move_group_interface_->getCurrentState();
        const moveit::core::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup("ur_manipulator");
        const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

        double min_distance = std::numeric_limits<double>::max();
        min_distance = 10;
        for (size_t i = 0; i < joint_names.size() - 1; ++i)
        {
            const moveit::core::JointModel* joint_model1 = robot_model_->getJointModel(joint_names[i]);
            const moveit::core::JointModel* joint_model2 = robot_model_->getJointModel(joint_names[i+1]);
            if (joint_model1 && joint_model2)
            {
                const std::string& child_link_name1 = joint_model1->getChildLinkModel()->getName();
                const std::string& child_link_name2 = joint_model2->getChildLinkModel()->getName();
                Eigen::Isometry3d link_state1 = kinematic_state->getGlobalLinkTransform(child_link_name1);
                Eigen::Isometry3d link_state2 = kinematic_state->getGlobalLinkTransform(child_link_name2);
                Eigen::Vector3d link_position1 = link_state1.translation();
                Eigen::Vector3d link_position2 = link_state2.translation();

                for (const auto& point : points)
                {
                    double distance = distancePointSegment(point, link_position1, link_position2);
                    if (distance < min_distance)
                    {
                        min_distance = distance;
                    }
                }
            }
        }

        std_msgs::msg::Float64 distance_msg;
        distance_msg.data = min_distance;
        RCLCPP_INFO(this->get_logger(), "Min_distance is %f", min_distance);
        distance_pub_->publish(distance_msg);
    }

    double distancePointSegment(const Eigen::Vector3d& point, const Eigen::Vector3d& segStart, const Eigen::Vector3d& segEnd)
    {
        Eigen::Vector3d diff = point - segStart;
        Eigen::Vector3d dir = segEnd - segStart;
        double t = diff.dot(dir);
        if (t <= 0.0)
        {
            // point is nearest to segStart
            t = 0.0;
        }
        else
        {
            double sqlen = dir.squaredNorm();  // segment length squared
            if (t >= sqlen)
            {
                // point is nearest to segEnd
                t = 1.0;
                diff -= dir;
            }
            else
            {
                // point is nearest to middle of segment
                t /= sqlen;
                diff -= dir * t;
            }
        }
        return diff.norm();
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_position_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
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
