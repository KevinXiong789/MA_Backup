/*this node subscribe the raw pointcloud, then do some cropping, voxelation and filter*/
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

class PointCloudProcessor : public rclcpp::Node
{
public:
	PointCloudProcessor() : Node("pointcloud_processor")
	{
	// Subscribe to the PointCloud message
	subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
				"/camera/depth/color/points", rclcpp::QoS(10),
				std::bind(&PointCloudProcessor::pointCloudCallback, this, std::placeholders::_1));

	// Create a publisher for the processed point cloud
	publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
				"processed_pointcloud", rclcpp::QoS(10));
	}

private:
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  
	void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
	{
	// Convert sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*msg, *cloud);

	// Crop the point cloud in a box
	float crop_center_x_ = 0.0;
	float crop_center_y_ = 0.0;
	float crop_center_z_ = 0.5;

	pcl::CropBox<pcl::PointXYZ> crop_filter;
	crop_filter.setTranslation(Eigen::Vector3f(crop_center_x_, crop_center_y_, crop_center_z_));
	crop_filter.setInputCloud(cloud);
	crop_filter.setMin(Eigen::Vector4f(-0.5, -0.5, -0.5, 1.0));
	crop_filter.setMax(Eigen::Vector4f(0.5, 0.5, 0.5, 1.0));
	crop_filter.filter(*cloud);

	// Apply VoxelGrid filtering
	pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
	voxel_filter.setInputCloud(cloud);
	voxel_filter.setLeafSize(0.01, 0.01, 0.01); // Adjust the leaf size according to your requirements
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	voxel_filter.filter(*filtered_cloud);

	// Convert pcl::PointCloud<pcl::PointXYZ> back to sensor_msgs::PointCloud2
	sensor_msgs::msg::PointCloud2 filtered_msg;
	pcl::toROSMsg(*filtered_cloud, filtered_msg);
	filtered_msg.header = msg->header;

	// Publish the processed point cloud
	publisher_->publish(filtered_msg);
	}



};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<PointCloudProcessor>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

