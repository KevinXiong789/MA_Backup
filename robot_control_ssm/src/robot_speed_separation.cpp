
// robot move between two points, according to the min distance, slow or stop         *******************testing
#include <memory>
#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <ur_msgs/srv/set_io.hpp>

class UR10EMoveit : public rclcpp::Node {
public:
  UR10EMoveit(const std::shared_ptr<rclcpp::Node>& node, const std::string& move_group_name) 
  : Node("ur10e_moveit"), move_group_interface_(node, move_group_name) {

    // Convert Euler angles to quaternion
    tf2::Quaternion q;
    double roll = 0, pitch = M_PI, yaw = 0;  // All in radians
    q.setRPY(roll, pitch, yaw);

    point1_pose_.orientation.w = q.getW();
    point1_pose_.orientation.x = q.getX();
    point1_pose_.orientation.y = q.getY();
    point1_pose_.orientation.z = q.getZ();

    point1_pose_.position.x = 0.6;
    point1_pose_.position.y = 0.4;
    point1_pose_.position.z = 0.4;

    // point2 for nominal task
    point2_pose_ = point1_pose_;
    
    point2_pose_.position.y = -0.4;

    // Subscribe to the joint_states topic
    //joint_states_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
    //  "joint_states", 10, std::bind(&UR10EMoveit::jointStatesCallback, this, std::placeholders::_1));


    // Begin state machine
    //timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&UR10EMoveit::stateMachine, this));
    
    // Initialize the service client for setting IO
    set_io_client_ = create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");

    while (rclcpp::ok()) {
        stateMachine();
        rclcpp::spin_some(this->get_node_base_interface());
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      }
    

  }

private:

  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Loop over the joint names and their corresponding positions
    for (size_t i = 0; i < msg->name.size(); i++) {
      std::cout << "Joint " << msg->name[i] << " is at position " << msg->position[i] << std::endl;
    }

  }


  void stateMachine() {
		addWallsAndBase();
    switcher_ = !switcher_;
		moveBetweenFixedPoints((switcher_) ? point1_pose_ : point2_pose_);
  }



  void moveBetweenFixedPoints(const geometry_msgs::msg::Pose& point_pose) {

    std::cout << "Doing nominal Task" << std::endl;

    //open_gripper(set_io_client_);

    // Move to the fixed point
    move_group_interface_.setPoseTarget(point_pose);

    /*
    // Set speed and acceleration
    double velocity_factor = 0.5;  // 50% of the maximum velocity
    double acceleration_factor = 0.5;  // 50% of the maximum acceleration
    move_group_interface_.setMaxVelocityScalingFactor(velocity_factor);
    move_group_interface_.setMaxAccelerationScalingFactor(acceleration_factor);
    */

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    /*
    if (move_group_interface_.plan(plan)) {
      move_group_interface_.execute(plan);
      std::cout << "Robot moved to fixed point successfully" << std::endl;
      //sleepSafeFor(1.0);
      //close_gripper(set_io_client_);
    } else {
      std::cerr << "Failed to plan trajectory to fixed point" << std::endl;
    }
    
    // Simulating a sleep of 3 seconds
    sleepSafeFor(3.0);
    */
    
    int tries = 0;

    while (tries < 3) {
      if (move_group_interface_.plan(plan)) {
        move_group_interface_.execute(plan);
        std::cout << "Robot moved to fixed point successfully" << std::endl;
        sleepSafeFor(3.0);
        return;
      } else {
        std::cerr << "Failed to plan trajectory to fixed point, try " << tries+1 << " of 3" << std::endl;
        tries++;
      }
    }

    std::cerr << "Failed to plan trajectory to fixed point after 3 tries" << std::endl;


  }

  bool open_gripper(rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr set_io_client_)
  {
    ur_msgs::srv::SetIO::Request set_req1;
    set_req1.fun = (float)ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
    set_req1.pin = (float)ur_msgs::srv::SetIO::Request::PIN_DOUT0;
    set_req1.state = (float)ur_msgs::srv::SetIO::Request::STATE_OFF;
    RCLCPP_INFO(rclcpp::get_logger("set_io"), "%s", ur_msgs::srv::to_yaml(set_req1).c_str());
    auto fut =
        set_io_client_->async_send_request(std::make_shared<ur_msgs::srv::SetIO::Request>(set_req1));
    auto fut_res = fut.wait_for(std::chrono::seconds(2));
    if (fut_res == std::future_status::timeout)
    {
      RCLCPP_INFO(rclcpp::get_logger("set_io"), "Set IO failed with timeout.");
      return false;
    }

    if (!fut.get()->success)
    {
      RCLCPP_INFO(rclcpp::get_logger("set_io"), "Set IO was not successful.");
      return false;
    }

    ur_msgs::srv::SetIO::Request reset_req2;
    reset_req2.fun = (float)ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
    reset_req2.pin = (float)ur_msgs::srv::SetIO::Request::PIN_DOUT1;
    reset_req2.state = (float)ur_msgs::srv::SetIO::Request::STATE_ON;

    fut =
        set_io_client_->async_send_request(std::make_shared<ur_msgs::srv::SetIO::Request>(reset_req2));
    fut_res = fut.wait_for(std::chrono::seconds(2));
    if (fut_res == std::future_status::timeout)
    {
      RCLCPP_INFO(rclcpp::get_logger("set_io"), "Set IO failed with timeout.");
      return false;
    }

    if (!fut.get()->success)
    {
      RCLCPP_INFO(rclcpp::get_logger("set_io"), "Set IO was not successful.");
      return false;
    }

    return true;
  }

  bool close_gripper(rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr set_io_client_)
  {
    ur_msgs::srv::SetIO::Request set_req2;
    set_req2.fun = (float)ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
    set_req2.pin = (float)ur_msgs::srv::SetIO::Request::PIN_DOUT1;
    set_req2.state = (float)ur_msgs::srv::SetIO::Request::STATE_OFF;

    RCLCPP_INFO(rclcpp::get_logger("set_io"), "%s", ur_msgs::srv::to_yaml(set_req2).c_str());
    auto fut =
        set_io_client_->async_send_request(std::make_shared<ur_msgs::srv::SetIO::Request>(set_req2));
    auto fut_res = fut.wait_for(std::chrono::seconds(2));
    if (fut_res == std::future_status::timeout)
    {
      RCLCPP_INFO(rclcpp::get_logger("set_io"), "Set IO failed with timeout.");
      return false;
    }

    if (!fut.get()->success)
    {
      RCLCPP_INFO(rclcpp::get_logger("set_io"), "Set IO was not successful.");
      return false;
    }

    ur_msgs::srv::SetIO::Request reset_req1;
    reset_req1.fun = (float)ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
    reset_req1.pin = (float)ur_msgs::srv::SetIO::Request::PIN_DOUT0;
    reset_req1.state = (float)ur_msgs::srv::SetIO::Request::STATE_ON;

    fut =
        set_io_client_->async_send_request(std::make_shared<ur_msgs::srv::SetIO::Request>(reset_req1));
    fut_res = fut.wait_for(std::chrono::seconds(2));
    if (fut_res == std::future_status::timeout)
    {
      RCLCPP_INFO(rclcpp::get_logger("set_io"), "Set IO failed with timeout.");
      return false;
    }

    if (!fut.get()->success)
    {
      RCLCPP_INFO(rclcpp::get_logger("set_io"), "Set IO was not successful.");
      return false;
    }

    return true;
  }



  void addWallsAndBase() {
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.resize(5);

    // Size for the walls
    double wall_height = 2.0;  // 2 meter high
    double wall_thickness = 0.01;  // 1 cm thick
    double wall_width = 1.5;
    double wall_distance = 0.75;  // 0.8 meter away from base

    // Size for the base
    double base_height = 0.01;  // 1 cm thick
    double base_side = 1.5;  // 1 meter radius, should be larger than reach of robot

    // Size for the tool box
    double box_height = 0.75;
    double box_thickness = 0.08;
    double box_width = 0.12;

    for (int i = 0; i < 3; i++) {
      collision_objects[i].header.frame_id = move_group_interface_.getPlanningFrame();
      collision_objects[i].id = "wall" + std::to_string(i+1);

      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[primitive.BOX_X] = (i < 2) ? wall_width : wall_thickness;
      primitive.dimensions[primitive.BOX_Y] = (i < 2) ? wall_thickness : wall_width;
      primitive.dimensions[primitive.BOX_Z] = wall_height;

      collision_objects[i].primitives.push_back(primitive);
      collision_objects[i].primitive_poses.resize(1);
      collision_objects[i].primitive_poses[0].orientation.w = 1.0;
      collision_objects[i].primitive_poses[0].position.x = (i < 2) ? 0.25 : -0.5;
      collision_objects[i].primitive_poses[0].position.y = (i < 2) ? ((i % 2 == 0) ? wall_distance : -wall_distance) : 0;
      collision_objects[i].primitive_poses[0].position.z = wall_height / 2;
      collision_objects[i].operation = collision_objects[i].ADD;


    }

    // Add the base
    collision_objects[3].header.frame_id = move_group_interface_.getPlanningFrame();
    collision_objects[3].id = "base";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = base_side;
    primitive.dimensions[primitive.BOX_Y] = base_side;
	  primitive.dimensions[primitive.BOX_Z] = base_height;

    collision_objects[3].primitives.push_back(primitive);
    collision_objects[3].primitive_poses.resize(1);
    collision_objects[3].primitive_poses[0].orientation.w = 1.0;
	  collision_objects[3].primitive_poses[0].position.x = 0.25;
    collision_objects[3].primitive_poses[0].position.z = -base_height / 2;
    collision_objects[3].operation = collision_objects[3].ADD;

    // Add tool box
    collision_objects[4].header.frame_id = move_group_interface_.getPlanningFrame();
    collision_objects[4].id = "tool box";

    shape_msgs::msg::SolidPrimitive primitive2;
    primitive2.type = primitive2.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[primitive2.BOX_X] = box_thickness;
    primitive2.dimensions[primitive2.BOX_Y] = box_width;
	  primitive2.dimensions[primitive2.BOX_Z] = box_height;

    collision_objects[4].primitives.push_back(primitive2);
    collision_objects[4].primitive_poses.resize(1);
    collision_objects[4].primitive_poses[0].orientation.w = 1.0;
	  collision_objects[4].primitive_poses[0].position.x = 0.7;
    collision_objects[4].primitive_poses[0].position.y = -0.53;
    collision_objects[4].primitive_poses[0].position.z = box_height / 2;
    collision_objects[4].operation = collision_objects[3].ADD;


    // Now, let's add the collision object into the world
    planning_scene_interface.addCollisionObjects(collision_objects);

  }



  void sleepSafeFor(double duration) {
    auto start = std::chrono::steady_clock::now();
    rclcpp::Rate rate(10);  // Adjust the rate as per your requirements

    while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() < duration) {
      rate.sleep();
    }
  }

  geometry_msgs::msg::Pose point1_pose_;
  geometry_msgs::msg::Pose point2_pose_;
  bool switcher_ = false;
  
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  //rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;

  rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr set_io_client_;



};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto move_group_name = "ur_manipulator";
  

  auto moveit_node = std::make_shared<UR10EMoveit>(node, move_group_name);
  rclcpp::spin(moveit_node);
  rclcpp::shutdown();
  return 0;
}




/*
// get robot joint state                *******************************************************************can run
#include <memory>
#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>
#include <Eigen/Dense>


class UR10EMoveit : public rclcpp::Node {
public:
  UR10EMoveit(const std::shared_ptr<rclcpp::Node>& node, const std::string& move_group_name) 
  : Node("ur10e_moveit"), move_group_interface_(node, move_group_name) {

    // Subscribe to the joint_states topic
    joint_states_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&UR10EMoveit::jointStatesCallback, this, std::placeholders::_1));


  }

private:

  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Loop over the joint names and their corresponding positions
    for (size_t i = 0; i < msg->name.size(); i++) {
      std::cout << "Joint " << msg->name[i] << " is at position " << msg->position[i] << std::endl;
    }
  }

  
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;


};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto move_group_name = "ur_manipulator";
  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, move_group_name);

  auto moveit_node = std::make_shared<UR10EMoveit>(node, move_group_name);
  rclcpp::spin(moveit_node);
  rclcpp::shutdown();
  return 0;
}
*/




/*
// min_distance between Robot TCP and point cloud             ********************************************can run
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

class EndEffectorPoseListener : public rclcpp::Node
{
public:
    EndEffectorPoseListener()
    : Node("end_effector_pose_listener")
    {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&EndEffectorPoseListener::timer_callback, this));
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points", 10, std::bind(&EndEffectorPoseListener::point_cloud_callback, this, std::placeholders::_1));
        distance_pub_ = this->create_publisher<std_msgs::msg::Float64>("minimum_distance_topic", 10);
    }

    void init()
    {
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::PoseStamped current_pose = move_group_interface_->getCurrentPose();
        //RCLCPP_INFO(this->get_logger(), "Current pose: x: %f, y: %f, z: %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
    }

    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

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

        geometry_msgs::msg::PoseStamped current_pose = move_group_interface_->getCurrentPose();
        double min_distance = std::numeric_limits<double>::max();

        for (const auto& point : cloud->points)
        {
            double distance = std::sqrt(std::pow(point.x - current_pose.pose.position.x, 2) +
                                        std::pow(point.y - current_pose.pose.position.y, 2) +
                                        std::pow(point.z - current_pose.pose.position.z, 2));
            if (distance < min_distance)
            {
                min_distance = distance;
            }
        }

        std_msgs::msg::Float64 distance_msg;
        distance_msg.data = min_distance;
        RCLCPP_INFO(this->get_logger(), "Min_distance is %f", min_distance);
        distance_pub_->publish(distance_msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EndEffectorPoseListener>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
*/


/*
// every joint position              **************************************************************************can run
#include <memory>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
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
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
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

        for (const std::string& joint_name : joint_names)
        {
            const moveit::core::JointModel* joint_model = robot_model_->getJointModel(joint_name);
            if (joint_model)
            {
                const std::string& child_link_name = joint_model->getChildLinkModel()->getName();
                Eigen::Isometry3d link_state = kinematic_state->getGlobalLinkTransform(child_link_name);
                Eigen::Vector3d link_position = link_state.translation();
                RCLCPP_INFO(this->get_logger(), "Joint %s position: x: %f, y: %f, z: %f", joint_name.c_str(), link_position.x(), link_position.y(), link_position.z());
            }
        }
    }


    rclcpp::TimerBase::SharedPtr timer_;
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


/*
// min_distance between 6 joints and point cloud, Methode 1             *******************************************can run but too slow
#include <memory>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <Eigen/Dense>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float64.hpp>

class JointPoseListener : public rclcpp::Node
{
public:
    JointPoseListener()
    : Node("joint_pose_listener")
    {
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points", 10, std::bind(&JointPoseListener::point_cloud_callback, this, std::placeholders::_1));
        min_distance_pub_ = this->create_publisher<std_msgs::msg::Float64>("min_distance", 10);
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
        auto kinematic_state = move_group_interface_->getCurrentState();
        const moveit::core::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup("ur_manipulator");
        const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

        double min_distance = std::numeric_limits<double>::max();

        for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y"), iter_z(*msg, "z");
             iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            // Transform the point cloud to the robot's coordinate system
            Eigen::Vector3d point(*iter_z, -*iter_x, -*iter_y);

            for (const std::string& joint_name : joint_names)
            {
                const moveit::core::JointModel* joint_model = robot_model_->getJointModel(joint_name);
                if (joint_model)
                {
                    const std::string& child_link_name = joint_model->getChildLinkModel()->getName();
                    Eigen::Isometry3d link_state = kinematic_state->getGlobalLinkTransform(child_link_name);
                    Eigen::Vector3d link_position = link_state.translation();

                    double distance = (link_position - point).norm();
                    if (distance < min_distance)
                    {
                        min_distance = distance;
                    }
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "Min_distance is %f", min_distance);
        std_msgs::msg::Float64 min_distance_msg;
        min_distance_msg.data = min_distance;
        min_distance_pub_->publish(min_distance_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr min_distance_pub_;
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



/*
// min_distance between 6 joints and point cloud, Methode 2 ************************************************************** can run
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
        distance_pub_ = this->create_publisher<std_msgs::msg::Float64>("minimum_distance_topic", 10);
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
        for (const std::string& joint_name : joint_names)
        {
            const moveit::core::JointModel* joint_model = robot_model_->getJointModel(joint_name);
            if (joint_model)
            {
                const std::string& child_link_name = joint_model->getChildLinkModel()->getName();
                Eigen::Isometry3d link_state = kinematic_state->getGlobalLinkTransform(child_link_name);
                Eigen::Vector3d link_position = link_state.translation();

                for (const auto& point : cloud->points)
                {
                    double distance = std::sqrt(std::pow(point.x - link_position.x(), 2) +
                                                std::pow(point.y - link_position.y(), 2) +
                                                std::pow(point.z - link_position.z(), 2));
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
        distance_pub_ = this->create_publisher<std_msgs::msg::Float64>("/minimum_distance_topic", 10);
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



/*
// speed control in real time                        ***********************************testing
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

#include <ur_msgs/srv/set_io.h>
#include <ur_msgs/srv/set_speed_slider_fraction.h>
//#include <ur_robot_driver/hardware_interface.hpp>

class JointPoseListener : public rclcpp::Node
{
public:
    JointPoseListener()
    : Node("joint_pose_listener")
    {
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/processed_point_cloud", 10, std::bind(&JointPoseListener::point_cloud_callback, this, std::placeholders::_1));
        distance_pub_ = this->create_publisher<std_msgs::msg::Float64>("/minimum_distance_topic", 10);

        velocity_client_ = this->create_client<ur_msgs::srv::SetSpeedSliderFraction>(
            "/ur_hardware_interface/speed_slider_fraction");
    }

    void init()
    {
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), "robot_description");
        robot_model_ = robot_model_loader_->getModel();
    }

private:
    
    void setRobotVelocity(double min_distance)
    {
        double A = 1.0;  // set your own threshold
        double B = 0.5;  // set your own threshold
        double speed_fraction = 0.0;

        if (min_distance > A)
        {
            speed_fraction = 0.5;
        }
        else if (min_distance > B)
        {
            speed_fraction = 0.1;
        }

        auto request = std::make_shared<ur_msgs::srv::SetSpeedSliderFraction::Request>();
        request->speed_slider_fraction = speed_fraction;

        while (!velocity_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service to appear...");
        }

        auto result_future = velocity_client_->async_send_request(request);
    }
    
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

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

        // set robot velocity based on min_distance
        setRobotVelocity(min_distance);
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

    rclcpp::Client<ur_msgs::srv::SetSpeedSliderFraction>::SharedPtr velocity_client_;

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













