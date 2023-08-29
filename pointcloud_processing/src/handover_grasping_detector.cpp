#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include <cmath>

class PointProcessor : public rclcpp::Node
{
public:
  PointProcessor()
    : Node("point_processor")
  {
    point1_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/robot/joint_positions", 10,
      std::bind(&PointProcessor::point1Callback, this, std::placeholders::_1));
    point2_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/Openpose/hand_position", 10,
      std::bind(&PointProcessor::point2Callback, this, std::placeholders::_1));
    bool_pub_ = this->create_publisher<std_msgs::msg::Bool>("/handover/grasp_flag", 1);

    // 设置一个周期性的定时器来检查条件
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
      std::bind(&PointProcessor::checkDistance, this));

    prev_time_ = this->now();
  }

private:
    void point1Callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // get robot tcp point
        if(msg->data.size() >= 3) {
        point1_[0] = msg->data[9];
        point1_[1] = msg->data[10];
        point1_[2] = msg->data[11];
        }
        //printf("robot TCP Position xyz: %.4f, %.4f, %.4f \n", point1_[0], point1_[1], point1_[2]);
    }

    void point2Callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // get right hand position
        if(msg->data.size() >= 3) {
        point2_[0] = msg->data[2];
        point2_[1] = -msg->data[0];
        point2_[2] = -msg->data[1];
        }
        //printf("right hand Position xyz: %.4f, %.4f, %.4f \n", point2_[0], point2_[1], point2_[2]);
    }

    void checkDistance()
    {
        double distance = std::sqrt(
            std::pow(point1_[0] - point2_[0], 2) +
            std::pow(point1_[1] - point2_[1], 2) +
            std::pow(point1_[2] - point2_[2], 2)
        );

        printf("distance: %.4f \n", distance);

        if (distance > distance_threshold_) {
            distance_under_threshold_time_ = 0;
        } else {
            distance_under_threshold_time_ += (this->now() - prev_time_).seconds();
        }

        if (distance_under_threshold_time_ >= 1.0) {
            publishBoolMessage(true);
        } else {
            publishBoolMessage(false);
        }

        prev_time_ = this->now();
    }


    void publishBoolMessage(bool status)
    {
        auto msg = std_msgs::msg::Bool();
        msg.data = status;
        bool_pub_->publish(msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr point1_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr point2_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bool_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time prev_time_;

    double point1_[3];
    double point2_[3];
    double distance_threshold_ = 0.08;
    double distance_under_threshold_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointProcessor>());
  rclcpp::shutdown();
  return 0;
}
