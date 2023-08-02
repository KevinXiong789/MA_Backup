

/*
// a better structure, robot state machine, nominal takeaway give     ***************************************************can run
#include <memory>
#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>

enum RobotState {
  NOMINAL,
  TAKEAWAY,
  GIVE
};

class UR10EMoveit : public rclcpp::Node {
public:
  UR10EMoveit(const std::shared_ptr<rclcpp::Node>& node, const std::string& move_group_name) 
  : Node("ur10e_moveit"), move_group_interface_(node, move_group_name) {
	robot_state = NOMINAL;

	// point1 for nominal task
	point1_pose_.orientation.w = 0.0;
	point1_pose_.orientation.x = 0.707;
	point1_pose_.orientation.y = 0.0;
	point1_pose_.orientation.z = 0.707;
	point1_pose_.position.x = 0.6;
	point1_pose_.position.y = 0.6;
	point1_pose_.position.z = 0.4;

	// point2 for nominal task
	point2_pose_ = point1_pose_;
	point2_pose_.orientation.z = -0.707;
	point2_pose_.position.x = -0.6;

	// Declare a subscriber for the hand position topic
	hand_position_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
	  "/handover/tool_grasping_position",
	  1,
	  [&](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
		switch (robot_state) {
		  case NOMINAL:
			switcher_ = !switcher_;
			moveBetweenFixedPoints((switcher_) ? point1_pose_ : point2_pose_);
			break;

		  case TAKEAWAY:
			moveToGraspPosition(msg);
			//moveToToolPlace();
			robot_state = NOMINAL; // Reset the robot state to NOMINAL after TAKEAWAY
			break;

		  case GIVE:
			//moveToToolPlace();
			moveToGivePosition(msg);
			robot_state = NOMINAL; // Reset the robot state to NOMINAL after GIVE
			break;
		}
	  });

	// Declare a subscriber for the handover_trigger bool topic
	handover_trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
	  "/handover/handover_trigger",
	  1,
	  [&](const std_msgs::msg::Bool::SharedPtr msg) {
		if (msg->data) {
		  if (ToolInHandFlag_) {
			robot_state = TAKEAWAY;
		  } else {
			robot_state = GIVE;
		  }
		} else {
		  robot_state = NOMINAL;
		}
	  });

	// Declare a subscriber for the ToolInHandFlag bool topic
	ToolInHandFlag_sub_ = this->create_subscription<std_msgs::msg::Bool>(
	  "/handover/ToolInHandFlag",
	  1,
	  [&](const std_msgs::msg::Bool::SharedPtr msg) {
		ToolInHandFlag_ = msg->data;
	  });
  }

private:
  void moveBetweenFixedPoints(const geometry_msgs::msg::Pose& point_pose) {
	if (robot_state != NOMINAL) {
	  std::cout << "Invalid robot state for moveBetweenFixedPoints" << std::endl;
	  return;
	}

	std::cout << "Doing nominal Task" << std::endl;

	// Move to the fixed point
	move_group_interface_.setPoseTarget(point_pose);
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	if (move_group_interface_.plan(plan)) {
	  move_group_interface_.execute(plan);
	  std::cout << "Robot moved to fixed point successfully" << std::endl;
	} else {
	  std::cerr << "Failed to plan trajectory to fixed point" << std::endl;
	}

	// Simulating a sleep of 5 seconds
	sleepSafeFor(5.0);

  }

  void moveToToolPlace() {
	// Code to move the robot to the tool place position
	std::cout << "Move to tool place" << std::endl;
	// Set the tool place
	geometry_msgs::msg::Pose tool_pose;
	tool_pose.orientation.w = 0.0;
	tool_pose.orientation.x = 0.707;
	tool_pose.orientation.y = 0.0;
	tool_pose.orientation.z = -0.707;
	tool_pose.position.x = -0.6;
	tool_pose.position.y = -0.3;
	tool_pose.position.z = 0.05;

	// Set the tool pose in the MoveGroup interface
	move_group_interface_.setPoseTarget(tool_pose);
	// Create a plan to the target pose
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	if (move_group_interface_.plan(plan)) {
		move_group_interface_.execute(plan);
		std::cout << "Robot moved to tool place pose successfully" << std::endl;
	} else {
		std::cerr << "Failed to plan trajectory to target pose" << std::endl;
	}
  }


  void sleepSafeFor(double duration) {
	auto start = std::chrono::steady_clock::now();
	rclcpp::Rate rate(10);  // Adjust the rate as per your requirements

	while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() < duration) {
		rate.sleep();
	}
  }


  void moveToGraspPosition(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
	// Code to move the robot to the grasp position based on the received message
	if (robot_state != TAKEAWAY) {
		std::cout << "Invalid robot state for moveToGraspPosition" << std::endl;
		return;
	}

	std::cout << "Grasping tool from human hand" << std::endl;
	// Check if the message contains at least three values
	if (msg->data.size() >= 3) {
		// Extract the x, y, z coordinates from the message
		double x = msg->data[0];
		double y = msg->data[1];
		double z = msg->data[2];

		// Set the target pose with the received coordinates
		geometry_msgs::msg::Pose target_pose;
		target_pose.orientation.w = 0.0;
		target_pose.orientation.x = 0.707;
		target_pose.orientation.y = 0.0;
		target_pose.orientation.z = 0.707;
		target_pose.position.x = z;
		target_pose.position.y = -x;
		target_pose.position.z = -y;


		// Set the target pose in the MoveGroup interface
		move_group_interface_.setPoseTarget(target_pose);
		// Create a plan to the target pose
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		if (move_group_interface_.plan(plan)) {
			move_group_interface_.execute(plan);
			std::cout << "Robot moved to grasping point successfully" << std::endl;
		} else {
			std::cerr << "Failed to plan trajectory to target pose" << std::endl;
		}

	}

	moveToToolPlace();
  }

  void moveToGivePosition(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
	// Code to move the robot to the give position based on the received message
	if (robot_state != GIVE) {
		std::cout << "Invalid robot state for moveToGivePosition" << std::endl;
		return;
	}

	std::cout << "Give tool to human hand" << std::endl;
	// Check if the message contains at least three values
	if (msg->data.size() >= 3) {
		// Extract the x, y, z coordinates from the message
		double x = msg->data[0];
		double y = msg->data[1];
		double z = msg->data[2];

		// Set the target pose with the received coordinates
		geometry_msgs::msg::Pose target_pose;
		target_pose.orientation.w = 0.0;
		target_pose.orientation.x = 0.707;
		target_pose.orientation.y = 0.0;
		target_pose.orientation.z = 0.707;
		target_pose.position.x = z;
		target_pose.position.y = -x;
		target_pose.position.z = -y;

		// Set the target pose in the MoveGroup interface
		move_group_interface_.setPoseTarget(target_pose);
		// Create a plan to the target pose
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		if (move_group_interface_.plan(plan)) {
			move_group_interface_.execute(plan);
			std::cout << "Robot moved to giving point successfully" << std::endl;
		} else {
			std::cerr << "Failed to plan trajectory to target pose" << std::endl;
		}

	}
	moveToToolPlace();
  }

  geometry_msgs::msg::Pose point1_pose_;
  geometry_msgs::msg::Pose point2_pose_;
  RobotState robot_state;
  bool switcher_ = false;
  bool ToolInHandFlag_ = false;
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr hand_position_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr handover_trigger_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ToolInHandFlag_sub_;

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
// a better structure, robot state machine, nominal takeaway give, state machine not in subscriber loop     ************************************can run
// but still no solution for hand position updates
#include <memory>
#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>

enum RobotState {
  NOMINAL,
  TAKEAWAY,
  GIVE
};

class UR10EMoveit : public rclcpp::Node {
public:
  UR10EMoveit(const std::shared_ptr<rclcpp::Node>& node, const std::string& move_group_name) 
  : Node("ur10e_moveit"), move_group_interface_(node, move_group_name) {
	
	robot_state = NOMINAL;
	updateStatus();

	// point1 for nominal task
	point1_pose_.orientation.w = 0.0;
	point1_pose_.orientation.x = 0.707;
	point1_pose_.orientation.y = 0.0;
	point1_pose_.orientation.z = 0.707;
	point1_pose_.position.x = 0.6;
	point1_pose_.position.y = 0.6;
	point1_pose_.position.z = 0.4;

	// point2 for nominal task
	point2_pose_ = point1_pose_;
	point2_pose_.orientation.z = -0.707;
	point2_pose_.position.x = -0.6;

	// Declare a subscriber for the hand position topic
	hand_position_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
	  "/handover/tool_grasping_position", 1, std::bind(&UR10EMoveit::handPositionCallback, this, std::placeholders::_1));

	// Declare a subscriber for the handover_trigger bool topic
	handover_trigger_sub_ = create_subscription<std_msgs::msg::Bool>(
	  "/handover/handover_trigger", 1, std::bind(&UR10EMoveit::handoverTriggerCallback, this, std::placeholders::_1));

	// Declare a subscriber for the ToolInHandFlag bool topic
	ToolInHandFlag_sub_ = create_subscription<std_msgs::msg::Bool>(
	  "/handover/ToolInHandFlag", 1, std::bind(&UR10EMoveit::toolInHandCallback, this, std::placeholders::_1));

	// Begin state machine
	//timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&UR10EMoveit::stateMachine, this));
	while (rclcpp::ok()) {
	  stateMachine();
	  rclcpp::spin_some(this->get_node_base_interface());
	  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}


  }

private:
  void handoverTriggerCallback(const std_msgs::msg::Bool::SharedPtr msg) {
	handoverTriggerFlag_ = msg->data;
  }

  void toolInHandCallback(const std_msgs::msg::Bool::SharedPtr msg) {
	ToolInHandFlag_ = msg->data;
  }

  void handPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
	handPosition_ = msg;
  }

  void updateStatus() {
	if (handoverTriggerFlag_) {
		  if (ToolInHandFlag_) {
			robot_state = TAKEAWAY;
		  } else {
			robot_state = GIVE;
		  }
		} else {
		  robot_state = NOMINAL;
		}
  }

  void stateMachine() {
	updateStatus();
	switch (robot_state) {
		case NOMINAL:
		switcher_ = !switcher_;
		moveBetweenFixedPoints((switcher_) ? point1_pose_ : point2_pose_);
		//updateStatus();
		break;

		case TAKEAWAY:
		moveToGraspPosition(handPosition_);
		moveToToolPlace();
		sleepSafeFor(3.0);
		robot_state = NOMINAL; // Reset the robot state to NOMINAL after TAKEAWAY
		break;

		case GIVE:
		moveToToolPlace();
		moveToGivePosition(handPosition_);
		sleepSafeFor(3.0);
		robot_state = NOMINAL; // Reset the robot state to NOMINAL after GIVE
		break;
	}
	
  }



  void moveBetweenFixedPoints(const geometry_msgs::msg::Pose& point_pose) {
	if (robot_state != NOMINAL) {
	  std::cout << "Invalid robot state for moveBetweenFixedPoints" << std::endl;
	  return;
	}

	std::cout << "Doing nominal Task" << std::endl;

	double velocity_factor = 0.5;  
	double acceleration_factor = 0.5;  
	move_group_interface_.setMaxVelocityScalingFactor(velocity_factor);
	move_group_interface_.setMaxAccelerationScalingFactor(acceleration_factor);
	// Move to the fixed point
	move_group_interface_.setPoseTarget(point_pose);
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	if (move_group_interface_.plan(plan)) {
	  move_group_interface_.execute(plan);
	  std::cout << "Robot moved to fixed point successfully" << std::endl;
	} else {
	  std::cerr << "Failed to plan trajectory to fixed point" << std::endl;
	}

	// Simulating a sleep of 5 seconds
	sleepSafeFor(5.0);

  }

  void moveToToolPlace() {
	// Code to move the robot to the tool place position
	std::cout << "Move to tool place" << std::endl;
	// Set the tool place
	geometry_msgs::msg::Pose tool_pose;
	tool_pose.orientation.w = 0.0;
	tool_pose.orientation.x = 0.707;
	tool_pose.orientation.y = 0.0;
	tool_pose.orientation.z = -0.707;
	tool_pose.position.x = -0.6;
	tool_pose.position.y = -0.3;
	tool_pose.position.z = 0.05;

	// Set velocity of robot
	double velocity_factor = 0.5;  
	double acceleration_factor = 0.5;  
	move_group_interface_.setMaxVelocityScalingFactor(velocity_factor);
	move_group_interface_.setMaxAccelerationScalingFactor(acceleration_factor);
	// Set the tool pose in the MoveGroup interface
	move_group_interface_.setPoseTarget(tool_pose);
	// Create a plan to the target pose
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	if (move_group_interface_.plan(plan)) {
		move_group_interface_.execute(plan);
		std::cout << "Robot moved to tool place pose successfully" << std::endl;
	} else {
		std::cerr << "Failed to plan trajectory to target pose" << std::endl;
	}
  }


  void sleepSafeFor(double duration) {
	auto start = std::chrono::steady_clock::now();
	rclcpp::Rate rate(10);  // Adjust the rate as per your requirements

	while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() < duration) {
		rate.sleep();
	}
  }


  void moveToGraspPosition(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
	// Code to move the robot to the grasp position based on the received message
	if (robot_state != TAKEAWAY) {
		std::cout << "Invalid robot state for moveToGraspPosition" << std::endl;
		return;
	}

	std::cout << "Grasping tool from human hand" << std::endl;
	// Check if the message contains at least three values
	if (msg->data.size() >= 3) {
		// Extract the x, y, z coordinates from the message
		double x = msg->data[0];
		double y = msg->data[1];
		double z = msg->data[2];

		// Set the target pose with the received coordinates
		geometry_msgs::msg::Pose target_pose;
		target_pose.orientation.w = 0.0;
		target_pose.orientation.x = 0.707;
		target_pose.orientation.y = 0.0;
		target_pose.orientation.z = 0.707;
		target_pose.position.x = z;
		target_pose.position.y = -x;
		target_pose.position.z = -y;


		double velocity_factor = 0.5;  
		double acceleration_factor = 0.5;  
		move_group_interface_.setMaxVelocityScalingFactor(velocity_factor);
		move_group_interface_.setMaxAccelerationScalingFactor(acceleration_factor);

		// Set the target pose in the MoveGroup interface
		move_group_interface_.setPoseTarget(target_pose);
		// Create a plan to the target pose
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		if (move_group_interface_.plan(plan)) {
			move_group_interface_.execute(plan);
			std::cout << "Robot moved to grasping point successfully" << std::endl;
		} else {
			std::cerr << "Failed to plan trajectory to target pose" << std::endl;
		}

	}
  }



  void moveToGivePosition(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
	// Code to move the robot to the give position based on the received message
	if (robot_state != GIVE) {
		std::cout << "Invalid robot state for moveToGivePosition" << std::endl;
		return;
	}

	std::cout << "Give tool to human hand" << std::endl;


	// Check if the message contains at least three values
	//if (msg->data.size() >= 3) {
		// Extract the x, y, z coordinates from the message
		double x = msg->data[0];
		double y = msg->data[1];
		double z = msg->data[2];

		// Set the target pose with the received coordinates
		geometry_msgs::msg::Pose target_pose;
		target_pose.orientation.w = 0.0;
		target_pose.orientation.x = 0.707;
		target_pose.orientation.y = 0.0;
		target_pose.orientation.z = 0.707;
		target_pose.position.x = z;
		target_pose.position.y = -x;
		target_pose.position.z = -y;

		double velocity_factor = 0.5;  
		double acceleration_factor = 0.5;  
		move_group_interface_.setMaxVelocityScalingFactor(velocity_factor);
		move_group_interface_.setMaxAccelerationScalingFactor(acceleration_factor);

		// Set the target pose in the MoveGroup interface
		move_group_interface_.setPoseTarget(target_pose);
		// Create a plan to the target pose
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		if (move_group_interface_.plan(plan)) {
			move_group_interface_.execute(plan);
			std::cout << "Robot moved to giving point successfully" << std::endl;
		} else {
			std::cerr << "Failed to plan trajectory to target pose" << std::endl;
		}

	//}
	
  }

  geometry_msgs::msg::Pose point1_pose_;
  geometry_msgs::msg::Pose point2_pose_;
  RobotState robot_state;
  bool switcher_ = false;
  
  bool ToolInHandFlag_;
  bool handoverTriggerFlag_;
  std_msgs::msg::Float32MultiArray::SharedPtr handPosition_;
  
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr hand_position_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr handover_trigger_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ToolInHandFlag_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

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





// robot state machine, nominal takeaway give, state machine not in subscriber loop
// update hand position                                                               ******************************************************************can run
#include <memory>
#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>

enum RobotState {
  NOMINAL,
  TAKEAWAY,
  PICKTOOL,
  GIVE
};

class UR10EMoveit : public rclcpp::Node {
public:
  UR10EMoveit(const std::shared_ptr<rclcpp::Node>& node, const std::string& move_group_name) 
  : Node("ur10e_moveit"), move_group_interface_(node, move_group_name) {
	
	robot_state = NOMINAL;
	updateStatus();

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

	toolPoint_pose_ = point1_pose_;
	toolPoint_pose_.position.z = 0.3;
	toolPoint_pose_.position.y = 0.5;

	// Declare a subscriber for the hand position topic
	hand_position_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
	  "/handover/tool_grasping_position", 1, std::bind(&UR10EMoveit::handPositionCallback, this, std::placeholders::_1));

	// Declare a subscriber for the handover_trigger bool topic
	handover_trigger_sub_ = create_subscription<std_msgs::msg::Bool>(
	  "/handover/handover_trigger", 1, std::bind(&UR10EMoveit::handoverTriggerCallback, this, std::placeholders::_1));

	// Declare a subscriber for the ToolInHandFlag bool topic
	ToolInHandFlag_sub_ = create_subscription<std_msgs::msg::Bool>(
	  "/handover/ToolInHandFlag", 1, std::bind(&UR10EMoveit::toolInHandCallback, this, std::placeholders::_1));

	// Begin state machine
	//timer_ = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&UR10EMoveit::stateMachine, this));
	while (rclcpp::ok()) {
	  stateMachine();
	  rclcpp::spin_some(this->get_node_base_interface());
	  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
  }

private:
  void handoverTriggerCallback(const std_msgs::msg::Bool::SharedPtr msg) {
	if (!handoverTriggerFlag_ && msg->data) {
		triggered_handover_ = true;
	}
	handoverTriggerFlag_ = msg->data;
  }

  void toolInHandCallback(const std_msgs::msg::Bool::SharedPtr msg) {
	ToolInHandFlag_ = msg->data;
  }

  void handPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
	std::cout << "New hand position received" << std::endl;
	handPosition_ = msg;
  }

  /*
  void updateStatus() {
	if (triggered_handover_) {
		  if (ToolInHandFlag_) {
			robot_state = TAKEAWAY;
		  } else {
			if (ToolInGripper) {
				robot_state = GIVE;
			} else {
				robot_state = PICKTOOL;
			}
			
		  }
		} else {
		  robot_state = NOMINAL;
		}
  }
  */

  void updateStatus() {
	if (triggered_handover_ && robot_state == NOMINAL) {
		  if (ToolInHandFlag_) {
			robot_state = TAKEAWAY;
		  } else {
			robot_state = PICKTOOL;
			}
	} else if (robot_state == PICKTOOL) {
		robot_state = GIVE;
	} else {
		robot_state = NOMINAL;
	}
  }


  void stateMachine() {
	updateStatus();
	addWallsAndBase();
	switch (robot_state) {
		case NOMINAL:
		std::cout << "Doing nominal Task" << std::endl;
		switcher_ = !switcher_;
		moveBetweenFixedPoints((switcher_) ? point1_pose_ : point2_pose_);
		sleepSafeFor(3.0);
		break;

		case TAKEAWAY:
		std::cout << "Grasping tool from human hand" << std::endl;
		pose = getGraspGivePosition(handPosition_);
		printf("hand position xyz: %f,%f,%f\n",pose.position.x,pose.position.y,pose.position.z);
		if (isPoseWithinRange(pose)) {
			moveToGraspPosition(pose);
			PlaceTool(toolPoint_pose_);
			sleepSafeFor(3.0);
		} else {
			std::cout << "Pose is out of range!" << std::endl;
			break;
		}
		triggered_handover_ = false;
		break;

		case PICKTOOL:
		std::cout << "Move to tool cell to pick tool" << std::endl;
		PickTool(toolPoint_pose_);
		//ToolInGripper = true;
		sleepSafeFor(5.0);
		break;

		case GIVE:
		std::cout << "Give tool to human hand" << std::endl;
		pose = getGraspGivePosition(handPosition_);
		printf("hand position xyz: %f,%f,%f\n",pose.position.x,pose.position.y,pose.position.z);
		if (isPoseWithinRange(pose)) {
			moveToGivePosition(pose);
			sleepSafeFor(3.0);
		} else {
			std::cout << "Pose is out of range!" << std::endl;
			break;
		}
		//ToolInGripper = false;
		triggered_handover_ = false;
		break;
	}
	
  }


/*
  void moveBetweenFixedPoints(const geometry_msgs::msg::Pose& point_pose) {
	if (robot_state != NOMINAL) {
	  std::cout << "Invalid robot state for moveBetweenFixedPoints" << std::endl;
	  return;
	}

	std::cout << "Doing nominal Task" << std::endl;
	
	double velocity_factor = 0.5;  
	double acceleration_factor = 0.5;  
	move_group_interface_.setMaxVelocityScalingFactor(velocity_factor);
	move_group_interface_.setMaxAccelerationScalingFactor(acceleration_factor);
	// Move to the fixed point
	move_group_interface_.setPoseTarget(point_pose);
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	if (move_group_interface_.plan(plan)) {
	  move_group_interface_.execute(plan);
	  std::cout << "Robot moved to fixed point successfully" << std::endl;
	} else {
	  std::cerr << "Failed to plan trajectory to fixed point" << std::endl;
	}
  }
*/


  void moveBetweenFixedPoints(const geometry_msgs::msg::Pose& point_pose) {
	int trial = 0;
	while(trial < 20) {
	  moveit::planning_interface::MoveGroupInterface::Plan plan = getCartesianPathPlanToPose(point_pose);
	  if (moveGroupExecutePlan(plan)) {
		std::cout << "Robot moved to fixed point successfully" << std::endl;
		return;
	  }
	  std::cerr << "Execution to point trial " << trial++ << " failed. Reattempting" << std::endl;
	}
	std::cerr << "Max execution attempts reached, error" << std::endl;
	sleepSafeFor(1.0);
  }

  moveit::planning_interface::MoveGroupInterface::Plan getCartesianPathPlanToPose(const geometry_msgs::msg::Pose& point_pose) {
	std::vector<geometry_msgs::msg::Pose> waypoints;
	//waypoints.push_back(move_group_interface_.getCurrentPose().pose);
	waypoints.push_back(point_pose);

	moveit_msgs::msg::RobotTrajectory trajectory;
	const double eef_step = 0.01;
	const double jump_threshold = 0.0;
	double fraction = 0.0;

	int trial = 0;
	while(fraction < 0.5 && trial++ < 5) {
	  fraction = move_group_interface_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
	}
	
	if(trial == 5 && fraction < 0.5) {
	  std::cerr << "Could not compute cartesian path for given waypoints, aborting!!" << std::endl;
	  exit(-1);
	}

	moveit::planning_interface::MoveGroupInterface::Plan plan;
	plan.trajectory_ = trajectory;
	return plan;
  }

  bool moveGroupExecutePlan(const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
	return move_group_interface_.execute(plan) == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }



/*
  void PlaceTool() {
	// Code to move the robot to the tool place position
	std::cout << "Move to tool cell to place Tool" << std::endl;
	// Set the tool place
	geometry_msgs::msg::Pose tool_pose;
	tf2::Quaternion q;
	double roll = 0, pitch = M_PI, yaw = 0;  // All in radians
	q.setRPY(roll, pitch, yaw);

	tool_pose.orientation.w = q.getW();
	tool_pose.orientation.x = q.getX();
	tool_pose.orientation.y = q.getY();
	tool_pose.orientation.z = q.getZ();
	tool_pose.position.x = 0.0;
	tool_pose.position.y = 0.4;
	tool_pose.position.z = 0.1;

	// Set velocity of robot
	double velocity_factor = 0.5;  
	double acceleration_factor = 0.5;  
	move_group_interface_.setMaxVelocityScalingFactor(velocity_factor);
	move_group_interface_.setMaxAccelerationScalingFactor(acceleration_factor);
	// Set the tool pose in the MoveGroup interface
	move_group_interface_.setPoseTarget(tool_pose);
	// Create a plan to the target pose
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	if (move_group_interface_.plan(plan)) {
		move_group_interface_.execute(plan);
		std::cout << "Robot place Tool successfully" << std::endl;
	} else {
		std::cerr << "Failed to plan trajectory to target pose" << std::endl;
	}
  }
*/

/*
  void PickTool() {
	// Code to move the robot to the tool place position
	std::cout << "Move to tool cell to pick tool" << std::endl;
	// Set the tool place
	geometry_msgs::msg::Pose tool_pose;
	tf2::Quaternion q;
	double roll = 0, pitch = M_PI, yaw = 0;  // All in radians
	q.setRPY(roll, pitch, yaw);

	tool_pose.orientation.w = q.getW();
	tool_pose.orientation.x = q.getX();
	tool_pose.orientation.y = q.getY();
	tool_pose.orientation.z = q.getZ();
	tool_pose.position.x = 0.0;
	tool_pose.position.y = 0.4;
	tool_pose.position.z = 0.1;

	// Set velocity of robot
	double velocity_factor = 0.5;  
	double acceleration_factor = 0.5;  
	move_group_interface_.setMaxVelocityScalingFactor(velocity_factor);
	move_group_interface_.setMaxAccelerationScalingFactor(acceleration_factor);
	// Set the tool pose in the MoveGroup interface
	move_group_interface_.setPoseTarget(tool_pose);
	// Create a plan to the target pose
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	if (move_group_interface_.plan(plan)) {
		move_group_interface_.execute(plan);
		std::cout << "Robot pick tool successfully" << std::endl;
	} else {
		std::cerr << "Failed to plan trajectory to target pose" << std::endl;
	}
  }
*/

  bool isPoseWithinRange(const geometry_msgs::msg::Pose& pose) {
	return pose.position.x >= -0.5 && pose.position.x <= 1.5 &&
		pose.position.y >= -0.7 && pose.position.y <= 0.7 &&
		pose.position.z >= 0.1 && pose.position.z <= 1.0;
  }


  void PlaceTool(const geometry_msgs::msg::Pose& point_pose) {
	int trial = 0;
	while(trial < 20) {
	  moveit::planning_interface::MoveGroupInterface::Plan plan = getCartesianPathPlanToPose(point_pose);
	  if (moveGroupExecutePlan(plan)) {
		std::cout << "Robot moved to tool cell to place successfully" << std::endl;
		return;
	  }
	  std::cerr << "Execution to point trial " << trial++ << " failed. Reattempting" << std::endl;
	}
	std::cerr << "Max execution attempts reached, error" << std::endl;
	sleepSafeFor(1.0);
  }


  void PickTool(const geometry_msgs::msg::Pose& point_pose) {
	int trial = 0;
	while(trial < 20) {
	  moveit::planning_interface::MoveGroupInterface::Plan plan = getCartesianPathPlanToPose(point_pose);
	  if (moveGroupExecutePlan(plan)) {
		std::cout << "Robot moved to tool cell to pick successfully" << std::endl;
		return;
	  }
	  std::cerr << "Execution to point trial " << trial++ << " failed. Reattempting" << std::endl;
	}
	std::cerr << "Max execution attempts reached, error" << std::endl;
	sleepSafeFor(1.0);
  }


  void sleepSafeFor(double duration) {
	auto start = std::chrono::steady_clock::now();
	rclcpp::Rate rate(10);  // Adjust the rate as per your requirements

	while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() < duration) {
		rate.sleep();
	}
  }

  

  geometry_msgs::msg::Pose getGraspGivePosition(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
	// convert the coordinate system, because the xyz coordinate of Point cloud and ur10e robot are different
	geometry_msgs::msg::Pose pose;
	pose.position.x = msg->data[2];
	pose.position.y = -msg->data[0];
	pose.position.z = -msg->data[1];
	tf2::Quaternion q;
	double roll = 0, pitch = M_PI/2, yaw = 0;  // All in radians
	q.setRPY(roll, pitch, yaw);

	pose.orientation.w = q.getW();
	pose.orientation.x = q.getX();
	pose.orientation.y = q.getY();
	pose.orientation.z = q.getZ();
	
	return pose;
  }

/*
  void moveToGraspPosition(geometry_msgs::msg::Pose pose) {
	// Code to move the robot to the grasp position based on the received message
	if (robot_state != TAKEAWAY) {
		std::cout << "Invalid robot state for moveToGraspPosition" << std::endl;
		return;
	}

	std::cout << "Grasping tool from human hand" << std::endl;
	
	geometry_msgs::msg::Pose target_pose = pose;
	target_pose.position.x = pose.position.x - 0.1;

	double velocity_factor = 0.5;  
	double acceleration_factor = 0.5;  
	move_group_interface_.setMaxVelocityScalingFactor(velocity_factor);
	move_group_interface_.setMaxAccelerationScalingFactor(acceleration_factor);

	// Set the target pose in the MoveGroup interface
	move_group_interface_.setPoseTarget(target_pose);
	// Create a plan to the target pose
	moveit::planning_interface::MoveGroupInterface::Plan plan1;
	if (move_group_interface_.plan(plan1)) {
		move_group_interface_.execute(plan1);
		std::cout << "Robot moved to approximate point successfully" << std::endl;
	} else {
		std::cerr << "Failed to plan trajectory to target pose" << std::endl;
	}

	// Create waypoints for straight line motion
	std::vector<geometry_msgs::msg::Pose> waypoints;
	target_pose.position.x += 0.1; // move 0.1 along x axis
	waypoints.push_back(target_pose);

	// Create a plan to the final target pose with straight line motion
	moveit::planning_interface::MoveGroupInterface::Plan plan2;
	moveit_msgs::msg::RobotTrajectory trajectory;

	// Get the Cartesian path for straight line motion
	const double eef_step = 0.01;
	const double jump_threshold = 0.0;
	double fraction = move_group_interface_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

	if (fraction >= 1.0) {
		plan2.trajectory_ = trajectory;
		move_group_interface_.execute(plan2);
		std::cout << "Robot moved to grasping point successfully" << std::endl;
	} else {
		std::cerr << "Failed to compute Cartesian path to giving point" << std::endl;
	}
  }
*/

/*
  void moveToGivePosition(geometry_msgs::msg::Pose pose) {
	// Code to move the robot to the given position based on the received message
	if (robot_state != GIVE) {
		std::cout << "Invalid robot state for moveToGivePosition" << std::endl;
		return;
	}

	std::cout << "Give tool to human hand" << std::endl;

	geometry_msgs::msg::Pose target_pose = pose;
	target_pose.position.x = pose.position.x - 0.1;
	target_pose.position.z = pose.position.z + 0.1;

	double velocity_factor = 0.5;  
	double acceleration_factor = 0.5;  
	move_group_interface_.setMaxVelocityScalingFactor(velocity_factor);
	move_group_interface_.setMaxAccelerationScalingFactor(acceleration_factor);

	// Set the target pose in the MoveGroup interface
	move_group_interface_.setPoseTarget(target_pose);
	// Create a plan to the target pose
	moveit::planning_interface::MoveGroupInterface::Plan plan1;
	if (move_group_interface_.plan(plan1)) {
		move_group_interface_.execute(plan1);
		std::cout << "Robot moved to approximate point successfully" << std::endl;
	} else {
		std::cerr << "Failed to plan trajectory to target pose" << std::endl;
		return;
	}

	// Create waypoints for straight line motion
	std::vector<geometry_msgs::msg::Pose> waypoints;
	target_pose.position.x += 0.1; // move 0.1 along x axis
	waypoints.push_back(target_pose);

	// Create a plan to the final target pose with straight line motion
	moveit::planning_interface::MoveGroupInterface::Plan plan2;
	moveit_msgs::msg::RobotTrajectory trajectory;

	// Get the Cartesian path for straight line motion
	const double eef_step = 0.01;
	const double jump_threshold = 0.0;
	double fraction = move_group_interface_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

	if (fraction >= 1.0) {
		plan2.trajectory_ = trajectory;
		move_group_interface_.execute(plan2);
		std::cout << "Robot moved to giving point successfully" << std::endl;
	} else {
		std::cerr << "Failed to compute Cartesian path to giving point" << std::endl;
	}
  }
*/

  void moveToGraspPosition(const geometry_msgs::msg::Pose& point_pose) {
	int trial = 0;
	while(trial < 20) {
	  moveit::planning_interface::MoveGroupInterface::Plan plan = getCartesianPathPlanToPose(point_pose);
	  if (moveGroupExecutePlan(plan)) {
		std::cout << "Robot moved to grasping point successfully" << std::endl;
		return;
	  }
	  std::cerr << "Execution to point trial " << trial++ << " failed. Reattempting" << std::endl;
	}
	std::cerr << "Max execution attempts reached, error" << std::endl;
	sleepSafeFor(1.0);
  }

  void moveToGivePosition(const geometry_msgs::msg::Pose& point_pose) {
	int trial = 0;
	while(trial < 20) {
	  moveit::planning_interface::MoveGroupInterface::Plan plan = getCartesianPathPlanToPose(point_pose);
	  if (moveGroupExecutePlan(plan)) {
		std::cout << "Robot moved to give point successfully" << std::endl;
		return;
	  }
	  std::cerr << "Execution to point trial " << trial++ << " failed. Reattempting" << std::endl;
	}
	std::cerr << "Max execution attempts reached, error" << std::endl;
	sleepSafeFor(1.0);
  }



  bool checkPositionStability(geometry_msgs::msg::Pose old_pose, geometry_msgs::msg::Pose new_pose) {
	double threshold = 0.0001; // Threshold for position stability check
	double distance = std::sqrt(std::pow(old_pose.position.x - new_pose.position.x, 2) +
								std::pow(old_pose.position.y - new_pose.position.y, 2) +
								std::pow(old_pose.position.z - new_pose.position.z, 2));
	return (distance <= threshold);
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
	collision_objects[3].primitive_poses[0].position.z = -base_height;
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


  geometry_msgs::msg::Pose point1_pose_;
  geometry_msgs::msg::Pose point2_pose_;
  geometry_msgs::msg::Pose toolPoint_pose_;
  RobotState robot_state;
  bool switcher_ = false;
  bool ToolInGripper = false;
  geometry_msgs::msg::Pose pose;
  
  bool ToolInHandFlag_;
  bool handoverTriggerFlag_ = false;
  bool triggered_handover_ = false;
  std_msgs::msg::Float32MultiArray::SharedPtr handPosition_;
  
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr hand_position_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr handover_trigger_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ToolInHandFlag_sub_;
  //rclcpp::TimerBase::SharedPtr timer_;

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





/*
// handover state machine, other structure          ******************can run, not good
#include <memory>
#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>

class UR10EMoveit : public rclcpp::Node
{
public:
	UR10EMoveit()
	: Node("ur10e_moveit")
	{
		robot_state = NOMINAL;
		updateStatus();

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

		// Declare a subscriber for the hand position topic
		hand_position_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
		"/handover/tool_grasping_position", 1, std::bind(&UR10EMoveit::handPositionCallback, this, std::placeholders::_1));

		// Declare a subscriber for the handover_trigger bool topic
		handover_trigger_sub_ = create_subscription<std_msgs::msg::Bool>(
		"/handover/handover_trigger", 1, std::bind(&UR10EMoveit::handoverTriggerCallback, this, std::placeholders::_1));

		// Declare a subscriber for the ToolInHandFlag bool topic
		ToolInHandFlag_sub_ = create_subscription<std_msgs::msg::Bool>(
		"/handover/ToolInHandFlag", 1, std::bind(&UR10EMoveit::toolInHandCallback, this, std::placeholders::_1));

		// Begin state machine
		//timer_ = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&UR10EMoveit::stateMachine, this));
		
		while (rclcpp::ok()) {
			stateMachine();
			rclcpp::spin_some(this->get_node_base_interface());
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
		
	}

	void init()
	{
		move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
	}

private:

	enum RobotState {
		NOMINAL,
		TAKEAWAY,
		PICKTOOL,
		GIVE
	};

	void handoverTriggerCallback(const std_msgs::msg::Bool::SharedPtr msg) {
		if (!handoverTriggerFlag_ && msg->data) {
			triggered_handover_ = true;
		}
		handoverTriggerFlag_ = msg->data;
	}

	void toolInHandCallback(const std_msgs::msg::Bool::SharedPtr msg) {
		ToolInHandFlag_ = msg->data;
	}

	void handPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
		std::cout << "New hand position received" << std::endl;
		handPosition_ = msg;
	}


	void updateStatus() {
		if (triggered_handover_) {
			if (ToolInHandFlag_) {
				robot_state = TAKEAWAY;
			} else {
				if (ToolInGripper) {
					robot_state = GIVE;
				} else {
					robot_state = PICKTOOL;
				}
				
			}
			} else {
			robot_state = NOMINAL;
			}
	}


	void stateMachine() {
		updateStatus();
		addWallsAndBase();
		switch (robot_state) {
			case NOMINAL:
			switcher_ = !switcher_;
			moveBetweenFixedPoints((switcher_) ? point1_pose_ : point2_pose_);
			sleepSafeFor(3.0);
			break;

			case TAKEAWAY:
			pose = getGraspGivePosition(handPosition_);
			moveToGraspPosition(pose);
			PlaceTool();
			sleepSafeFor(3.0);
			robot_state = NOMINAL; // Reset the robot state to NOMINAL after TAKEAWAY
			triggered_handover_ = false;
			break;

			case PICKTOOL:
			PickTool();
			ToolInGripper = true;
			sleepSafeFor(5.0);
			break;

			case GIVE:
			pose = getGraspGivePosition(handPosition_);
			moveToGivePosition(pose);
			sleepSafeFor(3.0);
			ToolInGripper = false;
			robot_state = NOMINAL; // Reset the robot state to NOMINAL after GIVE
			triggered_handover_ = false;
			break;
		}
		
	}



	void moveBetweenFixedPoints(const geometry_msgs::msg::Pose& point_pose) {
		if (robot_state != NOMINAL) {
			std::cout << "Invalid robot state for moveBetweenFixedPoints" << std::endl;
			return;
		}

		std::cout << "Doing nominal Task" << std::endl;
		
		double velocity_factor = 0.5;  
		double acceleration_factor = 0.5;  
		move_group_interface_->setMaxVelocityScalingFactor(velocity_factor);
		move_group_interface_->setMaxAccelerationScalingFactor(acceleration_factor);
		// Move to the fixed point
		move_group_interface_->setPoseTarget(point_pose);
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		if (move_group_interface_->plan(plan)) {
			move_group_interface_->execute(plan);
			std::cout << "Robot moved to fixed point successfully" << std::endl;
		} else {
			std::cerr << "Failed to plan trajectory to fixed point" << std::endl;
		}


	}

	void PlaceTool() {
		// Code to move the robot to the tool place position
		std::cout << "Move to tool cell to place Tool" << std::endl;
		// Set the tool place
		geometry_msgs::msg::Pose tool_pose;
		tf2::Quaternion q;
		double roll = 0, pitch = M_PI, yaw = 0;  // All in radians
		q.setRPY(roll, pitch, yaw);

		tool_pose.orientation.w = q.getW();
		tool_pose.orientation.x = q.getX();
		tool_pose.orientation.y = q.getY();
		tool_pose.orientation.z = q.getZ();
		tool_pose.position.x = 0.0;
		tool_pose.position.y = 0.4;
		tool_pose.position.z = 0.1;

		// Set velocity of robot
		double velocity_factor = 0.5;  
		double acceleration_factor = 0.5;  
		move_group_interface_->setMaxVelocityScalingFactor(velocity_factor);
		move_group_interface_->setMaxAccelerationScalingFactor(acceleration_factor);
		// Set the tool pose in the MoveGroup interface
		move_group_interface_->setPoseTarget(tool_pose);
		// Create a plan to the target pose
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		if (move_group_interface_->plan(plan)) {
			move_group_interface_->execute(plan);
			std::cout << "Robot place Tool successfully" << std::endl;
		} else {
			std::cerr << "Failed to plan trajectory to target pose" << std::endl;
		}
	}

	void PickTool() {
		// Code to move the robot to the tool place position
		std::cout << "Move to tool cell to pick tool" << std::endl;
		// Set the tool place
		geometry_msgs::msg::Pose tool_pose;
		tf2::Quaternion q;
		double roll = 0, pitch = M_PI, yaw = 0;  // All in radians
		q.setRPY(roll, pitch, yaw);

		tool_pose.orientation.w = q.getW();
		tool_pose.orientation.x = q.getX();
		tool_pose.orientation.y = q.getY();
		tool_pose.orientation.z = q.getZ();
		tool_pose.position.x = 0.0;
		tool_pose.position.y = 0.4;
		tool_pose.position.z = 0.1;

		// Set velocity of robot
		double velocity_factor = 0.5;  
		double acceleration_factor = 0.5;  
		move_group_interface_->setMaxVelocityScalingFactor(velocity_factor);
		move_group_interface_->setMaxAccelerationScalingFactor(acceleration_factor);
		// Set the tool pose in the MoveGroup interface
		move_group_interface_->setPoseTarget(tool_pose);
		// Create a plan to the target pose
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		if (move_group_interface_->plan(plan)) {
			move_group_interface_->execute(plan);
			std::cout << "Robot pick tool successfully" << std::endl;
		} else {
			std::cerr << "Failed to plan trajectory to target pose" << std::endl;
		}

	}


	void sleepSafeFor(double duration) {
		auto start = std::chrono::steady_clock::now();
		rclcpp::Rate rate(10);  // Adjust the rate as per your requirements

		while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() < duration) {
			rate.sleep();
		}
	}

	void addWallsAndBase() {
		std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
		collision_objects.resize(4);

		// Size for the walls
		double wall_height = 2.0;  // 2 meter high
		double wall_thickness = 0.01;  // 1 cm thick
		double wall_width = 1.5;
		double wall_distance = 0.75;  // 0.8 meter away from base

		// Size for the base
		double base_height = 0.01;  // 1 cm thick
		double base_side = 1.5;  // 1 meter radius, should be larger than reach of robot

		for (int i = 0; i < 3; i++) {
		collision_objects[i].header.frame_id = move_group_interface_->getPlanningFrame();
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
		collision_objects[3].header.frame_id = move_group_interface_->getPlanningFrame();
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

		// Now, let's add the collision object into the world
		planning_scene_interface.addCollisionObjects(collision_objects);

	}

	geometry_msgs::msg::Pose getGraspGivePosition(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
		// convert the coordinate system, because the xyz coordinate of Point cloud and ur10e robot are different
		geometry_msgs::msg::Pose pose;
		pose.position.x = msg->data[2];
		pose.position.y = -msg->data[0];
		pose.position.z = -msg->data[1];
		tf2::Quaternion q;
		double roll = 0, pitch = M_PI/2, yaw = 0;  // All in radians
		q.setRPY(roll, pitch, yaw);

		pose.orientation.w = q.getW();
		pose.orientation.x = q.getX();
		pose.orientation.y = q.getY();
		pose.orientation.z = q.getZ();
		
		return pose;
	}

	void moveToGraspPosition(geometry_msgs::msg::Pose pose) {
		// Code to move the robot to the grasp position based on the received message
		if (robot_state != TAKEAWAY) {
			std::cout << "Invalid robot state for moveToGraspPosition" << std::endl;
			return;
		}

		std::cout << "Grasping tool from human hand" << std::endl;
		
		geometry_msgs::msg::Pose target_pose = pose;

		double velocity_factor = 0.5;  
		double acceleration_factor = 0.5;  
		move_group_interface_->setMaxVelocityScalingFactor(velocity_factor);
		move_group_interface_->setMaxAccelerationScalingFactor(acceleration_factor);

		// Set the target pose in the MoveGroup interface
		move_group_interface_->setPoseTarget(target_pose);
		// Create a plan to the target pose
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		if (move_group_interface_->plan(plan)) {
			move_group_interface_->execute(plan);
			std::cout << "Robot moved to grasping point successfully" << std::endl;
		} else {
			std::cerr << "Failed to plan trajectory to target pose" << std::endl;
		}
	}


	void moveToGivePosition(geometry_msgs::msg::Pose pose) {
		// Code to move the robot to the give position based on the received message
		if (robot_state != GIVE) {
			std::cout << "Invalid robot state for moveToGivePosition" << std::endl;
			return;
		}

		std::cout << "Give tool to human hand" << std::endl;

		geometry_msgs::msg::Pose target_pose = pose;

		double velocity_factor = 0.5;  
		double acceleration_factor = 0.5;  
		move_group_interface_->setMaxVelocityScalingFactor(velocity_factor);
		move_group_interface_->setMaxAccelerationScalingFactor(acceleration_factor);

		// Set the target pose in the MoveGroup interface
		move_group_interface_->setPoseTarget(target_pose);
		// Create a plan to the target pose
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		if (move_group_interface_->plan(plan)) {
			move_group_interface_->execute(plan);
			std::cout << "Robot moved to giving point successfully" << std::endl;
		} else {
			std::cerr << "Failed to plan trajectory to target pose" << std::endl;
		}
	}


	
	std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
	geometry_msgs::msg::Pose point1_pose_;
	geometry_msgs::msg::Pose point2_pose_;
	RobotState robot_state;
	bool switcher_ = false;
	bool ToolInGripper = false;
	geometry_msgs::msg::Pose pose;

	bool ToolInHandFlag_;
	bool handoverTriggerFlag_ = false;
	bool triggered_handover_ = false;
	std_msgs::msg::Float32MultiArray::SharedPtr handPosition_;

	//moveit::planning_interface::MoveGroupInterface move_group_interface_;
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr hand_position_sub_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr handover_trigger_sub_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ToolInHandFlag_sub_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<UR10EMoveit>();
	node->init();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
*/

