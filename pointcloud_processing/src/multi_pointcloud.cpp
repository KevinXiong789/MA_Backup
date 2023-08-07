/*
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>

class PointCloudTransformation : public rclcpp::Node {
private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

  Eigen::Matrix4f getTransformMatrix() {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    
    // 首先进行旋转
    Eigen::AngleAxisf rotate_y(M_PI / 6, Eigen::Vector3f::UnitY());  // 逆时针旋转30度
    Eigen::Matrix3f rotation_matrix = rotate_y.toRotationMatrix();
    
    // 根据所描述的坐标轴关系，定义一个旋转矩阵
    Eigen::Matrix3f axis_transform;
    axis_transform << 0, 0, 1,
                      -1, 0, 0,
                      0, -1, 0;
    
    transform.block<3,3>(0,0) = rotation_matrix * axis_transform;
    
    // 平移
    transform(2, 3) = 1.0;  // 在世界坐标系z轴方向偏移1m
    return transform;
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::fromROSMsg(*msg, *cloud);

    // 执行变换
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, getTransformMatrix());

    sensor_msgs::msg::PointCloud2::SharedPtr output(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*transformed_cloud, *output);

    // 发布转换后的点云
    pub->publish(*output);
  }

public:
  PointCloudTransformation() : Node("point_cloud_transformation_node"), cloud(new pcl::PointCloud<pcl::PointXYZ>) {
    pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_cloud", 1);
    sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/camera/depth/color/points", 1, std::bind(&PointCloudTransformation::cloud_callback, this, std::placeholders::_1));
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudTransformation>());
  rclcpp::shutdown();
  return 0;
}
*/



#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

class PointCloudProcessor : public rclcpp::Node
{
public:
  PointCloudProcessor() : Node("point_cloud_processor")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      //"/camera/depth/color/points", 10, std::bind(&PointCloudProcessor::pointcloud_callback, this, std::placeholders::_1));
      "/Nuitrack/depth_cloud", 10, std::bind(&PointCloudProcessor::pointcloud_callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filter/new_pointcloud", 10);
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // 转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // 去除RGB信息
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_color(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *cloud_without_color);

    // 在z方向上进行裁剪，1.5米之内
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_without_color);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 1.5);
    pass.filter(*cloud_without_color);

    // 使用VoxelGrid过滤
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud_without_color);
    voxel_grid.setLeafSize(0.02f, 0.02f, 0.02f);
    voxel_grid.filter(*cloud_without_color);

    // 使用SOR过滤
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_without_color);
    sor.setMeanK(30);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_without_color);

    // 转换回sensor_msgs::msg::PointCloud2并发布
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_without_color, output);
    output.header = msg->header;
    pub_->publish(output);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudProcessor>());
  rclcpp::shutdown();
  return 0;
}

