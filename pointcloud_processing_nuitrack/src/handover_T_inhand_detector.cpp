
// *******************Nuitrack Handover triggered, whether tool in hand not yet***********************************can run
//********************Hand in cell for 3s -> Handover triggered*****************************
/*
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

class PositionSubscriber : public rclcpp::Node {
public:
  PositionSubscriber() : Node("right_hand_position_subscriber") {
    // Subscribe to the "right_hand_position" topic with a callback function
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/Nuitrack/right_hand_position", 1, std::bind(&PositionSubscriber::callback, this, std::placeholders::_1));
    prev_x_ = 0.0;
    prev_y_ = 0.0;
    prev_d_ = 0.0;
    prev_time_ = this->now();
    //timer_duration_ = 0;

    // Create publisher for bool topic
    bool_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/handover_trigger", 10);
  }

private:
  void callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    // Receive the x, y, and d coordinates from the Float32MultiArray message
    std::vector<float> sub_data = msg->data;
    float right_x = sub_data[0];
    float right_y = sub_data[1];
    float right_d = sub_data[2];

    auto handover_msg = std_msgs::msg::Bool();
    
    // Process the received coordinates as needed
    //RCLCPP_INFO(this->get_logger(), "Received right hand x: %.4f, y: %.4f, d: %.4f", right_x, right_y, right_d);
    if (right_x >= -0.1 && right_x <= 0.1 &&
        right_y >= -0.1 && right_y <= 0.1 &&
        right_d >= 0.4 && right_d <= 0.6) {
      // Check if right hand is within the fixed 3D space
      RCLCPP_INFO(this->get_logger(), "Right hand is in workcell");
    
      // Calculate the Euclidean distance between current and previous positions
      float distance = std::sqrt(std::pow(right_x - prev_x_, 2) +
                                 std::pow(right_y - prev_y_, 2) +
                                 std::pow(right_d - prev_d_, 2));
      if (distance > 0.01) {
        // If hand position remains approximately unchanged, reset the timer
        timer_duration_ = 0;
      } else {
        // If hand position changes, increment the timer duration
        timer_duration_ += (this->now() - prev_time_).seconds();
      }

      if (timer_duration_ > static_hand_duration_) {
        // If timer duration exceeds static hand duration, trigger handover
        RCLCPP_INFO(this->get_logger(), "Handover triggered!!!!!!!!!!!!!!!!!!");
        
        handover_msg.data = true;
        bool_publisher_->publish(handover_msg);

      } else {
        RCLCPP_INFO(this->get_logger(), "Handover not triggered");
      }
      
      // Update previous position and time
      prev_x_ = right_x;
      prev_y_ = right_y;
      prev_d_ = right_d;
      prev_time_ = this->now();
      handover_msg.data = false;

    } else {
      RCLCPP_INFO(this->get_logger(), "Right hand is not in workcell");
      // Update previous position and time
      prev_x_ = right_x;
      prev_y_ = right_y;
      prev_d_ = right_d;
      prev_time_ = this->now();
      handover_msg.data = false;

    }
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bool_publisher_;
  float prev_x_;
  float prev_y_;
  float prev_d_;
  rclcpp::Time prev_time_;
  float timer_duration_;
  float static_hand_duration_ = 3.0; // Static hand duration in seconds
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PositionSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
*/



//**********subscribe point cloud from cam1, get right hand position from cam2 + transform matrix between cam1 and cam2*****************can run
//*****************red point on right hand***********************************
/*
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode() : Node("subscriber_node")
  {
    // Subscribe to the point cloud topic
    pcl_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/depth/color/points", 10, std::bind(&SubscriberNode::processPointCloud, this, std::placeholders::_1));

    // Subscribe to the Float32MultiArray topic
    float_array_subscriber_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/Nuitrack/right_hand_position", 10, std::bind(&SubscriberNode::processFloatArray, this, std::placeholders::_1));

    // Create a publisher to publish the processed point cloud
    pcl_publisher = create_publisher<sensor_msgs::msg::PointCloud2>("/processed_point_cloud_topic", 10);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr float_array_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher;

  void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convert PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    // Print the processed Float32MultiArray data
    RCLCPP_INFO(get_logger(), "right hand x: %.4f, y: %.4f, z: %.4f", right_hand_x, right_hand_y, right_hand_z);

    // Process point cloud data here
    pcl::CropBox<pcl::PointXYZRGB> crop_filter;
    crop_filter.setInputCloud(pcl_cloud);
    Eigen::Vector4f min_point(-1, -1, -1, 1);
    Eigen::Vector4f max_point(1, 1, 1, 1);
    crop_filter.setMin(min_point);
    crop_filter.setMax(max_point);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    crop_filter.filter(*processed_cloud);


    // Create a new point with xyz coordinates from Float32MultiArray
    pcl::PointXYZRGB new_point;
    new_point.x = right_hand_x;
    new_point.y = right_hand_y;
    new_point.z = right_hand_z;
    new_point.r = 255; // set the color to red
    new_point.g = 0;
    new_point.b = 0;

    // Push the new point into the processed point cloud
    processed_cloud->points.push_back(new_point);

    // Preserve the color of the point cloud
    processed_cloud->width = 1;
    processed_cloud->height = processed_cloud->points.size();
    sensor_msgs::msg::PointCloud2 processed_cloud_msg;
    pcl::toROSMsg(*processed_cloud, processed_cloud_msg);
    processed_cloud_msg.header = msg->header;

    // Publish the processed point cloud
    pcl_publisher->publish(processed_cloud_msg);
  }

  void processFloatArray(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    // Process float array data here
    float_array_data_ = msg->data[0];
    float right_hand_x_cam2 = msg->data[0];
    float right_hand_y_cam2 = msg->data[1];
    float right_hand_z_cam2 = msg->data[2];

    // Define the transformation matrix from cam2 to cam1
    Eigen::Matrix4f cam2_to_cam1_transform;
    cam2_to_cam1_transform << 1, 0, 0, -0.1,
                              0, 1, 0, 0,
                              0, 0, 1, 0,
                              0, 0, 0, 1;

    // Transform the right hand position from cam2 to cam1
    Eigen::Vector4f right_hand_pos_cam2(right_hand_x_cam2, right_hand_y_cam2, right_hand_z_cam2, 1);
    Eigen::Vector4f right_hand_pos_cam1 = cam2_to_cam1_transform * right_hand_pos_cam2;
    float right_hand_x_cam1 = right_hand_pos_cam1(0);
    float right_hand_y_cam1 = right_hand_pos_cam1(1);
    float right_hand_z_cam1 = right_hand_pos_cam1(2);

    // Use the processed data to process the point cloud data in processPointCloud() function
    right_hand_x = right_hand_x_cam1;
    right_hand_y = right_hand_y_cam1;
    right_hand_z = right_hand_z_cam1;
  }

  // Processed point cloud data
  float float_array_data_;
  float right_hand_x;
  float right_hand_y;
  float right_hand_z;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubscriberNode>());
  rclcpp::shutdown();
  return 0;
}
*/




//**********subscribe point cloud from cam1, get right hand position (Nuitrack) from cam2 + transform matrix between cam1 and cam2*****************can run
//*********crop, vg sor filters, then get point cloud, get tool grasping position (xyz and angle), then pub*****************************
/*
#include <iostream>
#include <stdio.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <math.h>
#include <angles/angles.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/kdtree/kdtree_flann.h>


class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode() : Node("subscriber_node")
  {
    // Subscribe to the point cloud topic
    pcl_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/depth/color/points", 10, std::bind(&SubscriberNode::processPointCloud, this, std::placeholders::_1));

    // Subscribe to the Float32MultiArray topic
    float_array_subscriber_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/Nuitrack/right_hand_position", 10, std::bind(&SubscriberNode::processFloatArray, this, std::placeholders::_1));

    // Create a publisher to publish the processed point cloud
    pcl_publisher = create_publisher<sensor_msgs::msg::PointCloud2>("/processed_point_cloud", 1);

    // Create a publisher to publish the tool grasp position and angle
    tool_grasping_pub = create_publisher<std_msgs::msg::Float32MultiArray>("/handover/tool_grasping_position", 1);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr float_array_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr tool_grasping_pub;

  float distanceComputing (Eigen::Vector4f point, Eigen::Vector4f point2){
    //compute the distance between 2 points
    float distance;
    distance= sqrt(pow(point[0]-point2(0,0),2)+pow(point[1]-point2(1,0),2)+pow(point[2]-point2(2,0),2));
    return distance;
  }

  void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convert PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    // Crop point cloud around right hand
    pcl::CropBox<pcl::PointXYZ> crop_filter;
    // Set the center of the crop box to the right hand
    crop_filter.setTranslation(Eigen::Vector3f(right_hand_x, right_hand_y, right_hand_z));
    crop_filter.setInputCloud(pcl_cloud);
    // Set the size of the crop box to 0.2 x 0.2 x 0.2
    Eigen::Vector4f min_point(-0.15, -0.15, -0.15, 1);
    Eigen::Vector4f max_point(0.15, 0.15, 0.15, 1);
    crop_filter.setMin(min_point);
    crop_filter.setMax(max_point);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    crop_filter.filter(*cropped_cloud);


    // Check if the cropped_cloud is empty before applying the VoxelGrid filter
    // with this VoxelGrid filter, FPS>29
    pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (!cropped_cloud->empty()){
      // VoxelGrid Filter point cloud around right hand
      pcl::PointCloud<pcl::PointXYZ>::Ptr filter_1_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::VoxelGrid <pcl::PointXYZ> vg_filter;
      vg_filter.setInputCloud(cropped_cloud);
      vg_filter.setLeafSize(0.005f,0.005f,0.005f);
      vg_filter.filter(*filter_1_cloud);

      // add SOR filter, after vg filter,sor filter works good, FPS>29
      pcl::StatisticalOutlierRemoval <pcl::PointXYZ> SOR_filter;
      SOR_filter.setInputCloud(filter_1_cloud);
      SOR_filter.setMeanK(30);
      SOR_filter.setStddevMulThresh(1);
      SOR_filter.filter(*processed_cloud);

    } else {
      *processed_cloud = *cropped_cloud;
    }


    
    pcl::PointCloud<pcl::PointXYZ> processed_cloud_const = *processed_cloud;
    Eigen::Vector4f elbow_point = Eigen::Vector4f(right_elbow_x, right_elbow_y, right_elbow_z,1);
    Eigen::Vector4f tool_max_point;
    // get the max distance in processed cloud from elbow
    pcl::getMaxDistance(processed_cloud_const, elbow_point, tool_max_point);
    //RCLCPP_INFO(get_logger(), "tool_max_point xyz: (%.4f, %.4f, %.4f)", tool_max_point[0], tool_max_point[1], tool_max_point[2]);
    // Compute the center of mass of the cropped point cloud
    Eigen::Vector4f hand_centroid;
    pcl::compute3DCentroid(*processed_cloud, hand_centroid);
    //RCLCPP_INFO(get_logger(), "Center of mass: (%.4f, %.4f, %.4f)", centroid[0], centroid[1], centroid[2]);

    float distance_threshold = 0.16;

    if (!isnan(tool_max_point[0]) && distanceComputing(tool_max_point, hand_centroid)> distance_threshold){
      //RCLCPP_INFO(get_logger(), "Tool in hand!!!!!!!!!!!!!");

      // search for the nearest points from max point to compute the centroid
      // and so to get the tool grasping position
      
      pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
      kdtree.setInputCloud (processed_cloud);
      pcl::PointXYZ searchPoint;
      searchPoint.x=tool_max_point[0];
      searchPoint.y=tool_max_point[1];
      searchPoint.z=tool_max_point[2];
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      float radius = 0.03;
      pcl::PointCloud<pcl::PointXYZ>::Ptr max_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
        {
          
          max_cloud->width = pointIdxRadiusSearch.size ();
          max_cloud->height = 1;
          max_cloud->points.resize (max_cloud->width * max_cloud->height);
          for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i) {
            pcl::PointXYZ point;
            max_cloud->points[i].x=processed_cloud->points[ pointIdxRadiusSearch[i] ].x;
            max_cloud->points[i].y=processed_cloud->points[ pointIdxRadiusSearch[i] ].y;
            max_cloud->points[i].z=processed_cloud->points[ pointIdxRadiusSearch[i] ].z;
            
          }
        }
      const pcl::PointCloud<pcl::PointXYZ> max_cloud_const=*max_cloud;
      Eigen::Vector4f toolgrasp_centroid;
      // tool grasping position is toolgrasp_centroid
      pcl::compute3DCentroid(max_cloud_const, toolgrasp_centroid);
      printf("toolgrasp_position: %f, %f, %f \n",toolgrasp_centroid(0),toolgrasp_centroid(1),toolgrasp_centroid(2));


      // get the angle to grasp the tool
      // compute the tool vector
      const Eigen::Vector3f tool_vector = Eigen::Vector3f(toolgrasp_centroid(0)-right_hand_x, toolgrasp_centroid(1)-right_hand_y, toolgrasp_centroid(2)-right_hand_z);
      //unit vectors
      const Eigen::Vector3f x_vector = Eigen::Vector3f(1,0,0);
      const Eigen::Vector3f y_vector = Eigen::Vector3f(0,1,0);
      const Eigen::Vector3f z_vector = Eigen::Vector3f(0,0,1);
      //for roll
      double R_angle = std::acos((tool_vector-tool_vector.dot(x_vector)*x_vector).normalized().dot(y_vector));
      //for pitch
      double P_angle = std::acos((tool_vector-tool_vector.dot(y_vector)*y_vector).normalized().dot(x_vector));

      float roll_command_deg = pcl::rad2deg(R_angle+angles::from_degrees(-180));
      float pitch_command_deg = pcl::rad2deg(-P_angle+angles::from_degrees(90));
      printf("roll: %f, pitch: %f \n", roll_command_deg, pitch_command_deg);

      // publish tool grasping position
      std_msgs::msg::Float32MultiArray tool_grasping_position_msg;
      tool_grasping_position_msg.data = {toolgrasp_centroid(0),toolgrasp_centroid(1),toolgrasp_centroid(2),roll_command_deg,pitch_command_deg};
      tool_grasping_pub->publish(tool_grasping_position_msg);


    } else {
      //RCLCPP_INFO(get_logger(), "Tool not in hand");
    }

  
   
    


    // convert pcl to Pointcloud2
    //processed_cloud->width = 1;
    //processed_cloud->height = processed_cloud->points.size();
    sensor_msgs::msg::PointCloud2 processed_cloud_msg;
    pcl::toROSMsg(*processed_cloud, processed_cloud_msg);
    processed_cloud_msg.header = msg->header;

    // Publish the processed point cloud
    pcl_publisher->publish(processed_cloud_msg);
  }

  void processFloatArray(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    // Process float array data here
    float_array_data_ = msg->data[0];
    float right_hand_x_cam2 = msg->data[0];
    float right_hand_y_cam2 = msg->data[1];
    float right_hand_z_cam2 = msg->data[2];

    float right_elbow_x_cam2 = msg->data[3];
    float right_elbow_y_cam2 = msg->data[4];
    float right_elbow_z_cam2 = msg->data[5];

    // Define the transformation matrix from cam2 to cam1
    Eigen::Matrix4f cam2_to_cam1_transform;
    cam2_to_cam1_transform << 1, 0, 0, -0.1,
                              0, 1, 0, 0,
                              0, 0, 1, 0,
                              0, 0, 0, 1;

    // Transform the right hand position from cam2 to cam1
    Eigen::Vector4f right_hand_pos_cam2(right_hand_x_cam2, right_hand_y_cam2, right_hand_z_cam2, 1);
    Eigen::Vector4f right_hand_pos_cam1 = cam2_to_cam1_transform * right_hand_pos_cam2;
    float right_hand_x_cam1 = right_hand_pos_cam1(0);
    float right_hand_y_cam1 = right_hand_pos_cam1(1);
    float right_hand_z_cam1 = right_hand_pos_cam1(2);

    // Transform the right elbow position from cam2 to cam1
    Eigen::Vector4f right_elbow_pos_cam2(right_elbow_x_cam2, right_elbow_y_cam2, right_elbow_z_cam2, 1);
    Eigen::Vector4f right_elbow_pos_cam1 = cam2_to_cam1_transform * right_elbow_pos_cam2;
    float right_elbow_x_cam1 = right_elbow_pos_cam1(0);
    float right_elbow_y_cam1 = right_elbow_pos_cam1(1);
    float right_elbow_z_cam1 = right_elbow_pos_cam1(2);

    // Use the processed data to process the point cloud data in processPointCloud() function
    right_hand_x = right_hand_x_cam1;
    right_hand_y = right_hand_y_cam1;
    right_hand_z = right_hand_z_cam1;

    right_elbow_x = right_elbow_x_cam1;
    right_elbow_y = right_elbow_y_cam1;
    right_elbow_z = right_elbow_z_cam1;
  }

  // Processed point cloud data
  float float_array_data_;
  float right_hand_x;
  float right_hand_y;
  float right_hand_z;

  float right_elbow_x;
  float right_elbow_y;
  float right_elbow_z;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubscriberNode>());
  rclcpp::shutdown();
  return 0;
}
*/


//*********subscribe point cloud and right wrist and elbow position from cam *******************************************testing
//*********crop, vg sor filters, then get point cloud, get tool grasping position (xyz and angle), then pub*****************************
//*********also Hand in cell for 3s -> Handover triggered**************************************

#include <iostream>
#include <stdio.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <math.h>
#include <angles/angles.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/kdtree/kdtree_flann.h>

class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode() : Node("Handover_detector")
  {
    // Subscribe to the point cloud topic
    pcl_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/Nuitrack/depth_cloud", 10, std::bind(&SubscriberNode::processPointCloud, this, std::placeholders::_1));

    // Subscribe to the Float32MultiArray topic
    float_array_subscriber_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/Nuitrack/right_hand", 10, std::bind(&SubscriberNode::processFloatArray, this, std::placeholders::_1));

    // Create a publisher to publish the processed point cloud
    pcl_publisher = create_publisher<sensor_msgs::msg::PointCloud2>("/processed_point_cloud", 1);

    // Create a publisher to publish the tool grasp position and angle
    tool_grasping_pub = create_publisher<std_msgs::msg::Float32MultiArray>("/handover/tool_grasping_position", 1);

    // Create a publisher to publish flag indicating whether the tool in in hand
    tool_inhand_pub = create_publisher<std_msgs::msg::Bool>("/handover/ToolInHandFlag", 1);

    // Create publisher for handover trigged Bool
    handover_bool_pub = create_publisher<std_msgs::msg::Bool>("/handover/handover_trigger", 1);

    prev_time = this->now();
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr float_array_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr tool_grasping_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tool_inhand_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr handover_bool_pub;
  rclcpp::Time prev_time;

  // Position date will be used in all function
  float right_hand_x;
  float right_hand_y;
  float right_hand_z;
  float right_elbow_x;
  float right_elbow_y;
  float right_elbow_z;

  // Value for handover trigged
  float prev_x = 0;
  float prev_y = 0;
  float prev_z = 0;
  
  float timer_duration;
  float static_hand_duration = 3.0;
  std_msgs::msg::Bool handover_trigged_msg;
  std_msgs::msg::Bool ToolInHand_msg;


  float distanceComputing (Eigen::Vector4f point, Eigen::Vector4f point2){
    //compute the distance between 2 points
    float distance;
    distance= sqrt(pow(point[0]-point2[0],2)+pow(point[1]-point2[1],2)+pow(point[2]-point2[2],2));
    return distance;
  }

  bool is_in_the_cell(){
    if (right_hand_x >= -1.5 && right_hand_x <= 1.5 &&
        right_hand_y >= -1.5 && right_hand_y <= 1.5 &&
        right_hand_z >= -0.2 && right_hand_z <= 2.0){
          return true;
        }
    return false;
  }


  void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convert PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    // Crop point cloud around right hand
    pcl::CropBox<pcl::PointXYZ> crop_filter;
    // Set the center of the crop box to the right hand
    crop_filter.setTranslation(Eigen::Vector3f(right_hand_x, right_hand_y, right_hand_z));
    crop_filter.setInputCloud(pcl_cloud);
    // Set the size of the crop box to 0.2 x 0.2 x 0.2
    Eigen::Vector4f min_point(-0.15, -0.15, -0.15, 1);
    Eigen::Vector4f max_point(0.15, 0.15, 0.15, 1);
    crop_filter.setMin(min_point);
    crop_filter.setMax(max_point);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    crop_filter.filter(*cropped_cloud);


    // Check if the cropped_cloud is empty before applying the VoxelGrid filter
    // with this VoxelGrid filter, FPS>29
    pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (!cropped_cloud->empty()){
      // VoxelGrid Filter point cloud around right hand
      pcl::PointCloud<pcl::PointXYZ>::Ptr filter_1_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::VoxelGrid <pcl::PointXYZ> vg_filter;
      vg_filter.setInputCloud(cropped_cloud);
      vg_filter.setLeafSize(0.01f,0.01f,0.01f);
      vg_filter.filter(*filter_1_cloud);

      // add SOR filter, after vg filter,sor filter works good, FPS>29
      pcl::StatisticalOutlierRemoval <pcl::PointXYZ> SOR_filter;
      SOR_filter.setInputCloud(filter_1_cloud);
      SOR_filter.setMeanK(30);
      SOR_filter.setStddevMulThresh(1);
      SOR_filter.filter(*processed_cloud);

    } else {
      *processed_cloud = *cropped_cloud;
    }


    
    pcl::PointCloud<pcl::PointXYZ> processed_cloud_const = *processed_cloud;
    Eigen::Vector4f elbow_point = Eigen::Vector4f(right_elbow_x, right_elbow_y, right_elbow_z,1);
    Eigen::Vector4f tool_max_point;
    // get the max distance in processed cloud from elbow
    pcl::getMaxDistance(processed_cloud_const, elbow_point, tool_max_point);
    //RCLCPP_INFO(get_logger(), "tool_max_point xyz: (%.4f, %.4f, %.4f)", tool_max_point[0], tool_max_point[1], tool_max_point[2]);
    // Compute the center of mass of the cropped point cloud
    //Eigen::Vector4f hand_centroid;
    //pcl::compute3DCentroid(*processed_cloud, hand_centroid);
    //RCLCPP_INFO(get_logger(), "Center of mass: (%.4f, %.4f, %.4f)", hand_centroid[0], hand_centroid[1], hand_centroid[2]);
    Eigen::Vector4f wrist_point = Eigen::Vector4f(right_hand_x, right_hand_y, right_hand_z,1);
    //RCLCPP_INFO(get_logger(), "distance: (%.4f)", distanceComputing(tool_max_point, wrist_point));

    float distance_threshold = 0.12;
    
    if (!isnan(tool_max_point[0]) && distanceComputing(tool_max_point, wrist_point)> distance_threshold){
      RCLCPP_INFO(get_logger(), "Tool in hand!!!!!!!!!!!!!");
      ToolInHand_msg.data = true;
      tool_inhand_pub->publish(ToolInHand_msg);

      // search for the nearest points from max point to compute the centroid
      // and so to get the tool grasping position
      
      pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
      kdtree.setInputCloud (processed_cloud);
      pcl::PointXYZ searchPoint;
      searchPoint.x=tool_max_point[0];
      searchPoint.y=tool_max_point[1];
      searchPoint.z=tool_max_point[2];
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      float radius = 0.03;
      pcl::PointCloud<pcl::PointXYZ>::Ptr max_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
        {
          
          max_cloud->width = pointIdxRadiusSearch.size ();
          max_cloud->height = 1;
          max_cloud->points.resize (max_cloud->width * max_cloud->height);
          for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i) {
            pcl::PointXYZ point;
            max_cloud->points[i].x=processed_cloud->points[ pointIdxRadiusSearch[i] ].x;
            max_cloud->points[i].y=processed_cloud->points[ pointIdxRadiusSearch[i] ].y;
            max_cloud->points[i].z=processed_cloud->points[ pointIdxRadiusSearch[i] ].z;
            
          }
        }
      const pcl::PointCloud<pcl::PointXYZ> max_cloud_const=*max_cloud;
      Eigen::Vector4f toolgrasp_centroid;
      // tool grasping position is toolgrasp_centroid
      pcl::compute3DCentroid(max_cloud_const, toolgrasp_centroid);
      //printf("toolgrasp_position: %.4f, %.4f, %.4f \n",toolgrasp_centroid(0),toolgrasp_centroid(1),toolgrasp_centroid(2));


      // get the angle to grasp the tool
      // compute the tool vector
      const Eigen::Vector3f tool_vector = Eigen::Vector3f(toolgrasp_centroid(0)-right_hand_x, toolgrasp_centroid(1)-right_hand_y, toolgrasp_centroid(2)-right_hand_z);
      //unit vectors
      const Eigen::Vector3f x_vector = Eigen::Vector3f(1,0,0);
      const Eigen::Vector3f y_vector = Eigen::Vector3f(0,1,0);
      //const Eigen::Vector3f z_vector = Eigen::Vector3f(0,0,1);
      //for roll
      double R_angle = std::acos((tool_vector-tool_vector.dot(x_vector)*x_vector).normalized().dot(y_vector));
      //for pitch
      double P_angle = std::acos((tool_vector-tool_vector.dot(y_vector)*y_vector).normalized().dot(x_vector));

      float roll_command_deg = pcl::rad2deg(R_angle+angles::from_degrees(-180));
      float pitch_command_deg = pcl::rad2deg(-P_angle+angles::from_degrees(90));
      //printf("roll: %.4f, pitch: %.4f \n", roll_command_deg, pitch_command_deg);

      // publish tool grasping position
      std_msgs::msg::Float32MultiArray tool_grasping_position_msg;
      tool_grasping_position_msg.data = {toolgrasp_centroid(0),toolgrasp_centroid(1),toolgrasp_centroid(2),roll_command_deg,pitch_command_deg};
      tool_grasping_pub->publish(tool_grasping_position_msg);
      handover_trigged_msg.data = false;

    } else {
      RCLCPP_INFO(get_logger(), "Tool not in hand");
      std_msgs::msg::Float32MultiArray tool_grasping_position_msg;
      tool_grasping_position_msg.data = {wrist_point(0),wrist_point(1),wrist_point(2)};
      tool_grasping_pub->publish(tool_grasping_position_msg);
      ToolInHand_msg.data = false;
      tool_inhand_pub->publish(ToolInHand_msg);
    }
    


    // convert pcl to Pointcloud2
    //processed_cloud->width = 1;
    //processed_cloud->height = processed_cloud->points.size();
    sensor_msgs::msg::PointCloud2 processed_cloud_msg;
    pcl::toROSMsg(*processed_cloud, processed_cloud_msg);
    processed_cloud_msg.header = msg->header;

    // Publish the processed point cloud
    pcl_publisher->publish(processed_cloud_msg);
  }

  void processFloatArray(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    // Process float array data here
    
    right_hand_x = msg->data[0];
    right_hand_y = msg->data[1];
    right_hand_z = msg->data[2];
    right_elbow_x = msg->data[3];
    right_elbow_y = msg->data[4];
    right_elbow_z = msg->data[5];
    //printf("%f,%f,%f,%f,%f,%f \n",right_hand_x,right_hand_y,right_hand_z,right_elbow_x,right_elbow_y,right_elbow_z);
    //RCLCPP_INFO(get_logger(), "wrist:%.4f,%.4f,%.4f,elbow:%.4f,%.4f,%.4f", right_hand_x,right_hand_y,right_hand_z,right_elbow_x,right_elbow_y,right_elbow_z);

    // check right hand is in workcell
    //printf("xyz: %.4f, %.4f, %.4f \n", right_hand_x, right_hand_y, right_hand_z);
    if (is_in_the_cell()){
      float distance = std::sqrt(std::pow(right_elbow_x - prev_x, 2) +
                                 std::pow(right_elbow_y - prev_y, 2) +
                                 std::pow(right_elbow_z - prev_z, 2));
      

      if (distance > 0.01){
        timer_duration = 0;
      } else {
        timer_duration += (this->now() - prev_time).seconds();
      }

      if (timer_duration > static_hand_duration){
        RCLCPP_INFO(this->get_logger(), "Handover triggered");
        
        handover_trigged_msg.data = true;
        handover_bool_pub->publish(handover_trigged_msg);
      } else {
        handover_trigged_msg.data = false;
        handover_bool_pub->publish(handover_trigged_msg);
      }

      // Update previous position and time
      prev_x = right_elbow_x;
      prev_y = right_elbow_y;
      prev_z = right_elbow_z;
      prev_time = this->now();
      handover_trigged_msg.data = false;
    }
  }


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubscriberNode>());
  rclcpp::shutdown();
  return 0;
}



