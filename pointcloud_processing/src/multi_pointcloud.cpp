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


/*
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
      "/camera/depth/color/points", 10, std::bind(&PointCloudProcessor::pointcloud_callback, this, std::placeholders::_1));
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
    sor.setMeanK(50);
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
*/




#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>


class PointCloudTransformer : public rclcpp::Node
{
public:
    PointCloudTransformer() : Node("pointcloud_transformer")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/transformed_points", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/color/points", 10, std::bind(&PointCloudTransformer::pointCloudCallback, this, std::placeholders::_1));
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    }

private:
    
    // use lookupTransform
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Step 1: Convert the incoming ROS message to a PCL point cloud
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        // Step 2: Perform the coordinate transformation
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = buffer_->lookupTransform("camera_depth_optical_frame", "world", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            return;
        }

        // Extract the rotation and translation from the transform message
        Eigen::Matrix4f eigen_transform = Eigen::Matrix4f::Identity();
        eigen_transform.block<3, 3>(0, 0) = Eigen::Quaternionf(
            transform.transform.rotation.w,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z
        ).toRotationMatrix();
        eigen_transform(0, 3) = transform.transform.translation.x;
        eigen_transform(1, 3) = transform.transform.translation.y;
        eigen_transform(2, 3) = transform.transform.translation.z;

        // Apply the transform to the point cloud
        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        pcl::transformPointCloud(pcl_cloud, transformed_cloud, eigen_transform);
/*
        // Apply filters
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(transformed_cloud.makeShared());
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-1.5, 1.5);
        pass.filter(transformed_cloud);

        pass.setInputCloud(transformed_cloud.makeShared());
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-1.0, 1.0);
        pass.filter(transformed_cloud);

        pass.setInputCloud(transformed_cloud.makeShared());
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 1.0);
        pass.filter(transformed_cloud);
*/
        // Step 3: Convert the transformed PCL point cloud back to a ROS message
        sensor_msgs::msg::PointCloud2 transformed_msg;
        pcl::toROSMsg(transformed_cloud, transformed_msg);

        // Copy the header from the input cloud for the output message
        transformed_msg.header = msg->header;

        // Publish the transformed point cloud
        publisher_->publish(transformed_msg);
    }
    
    /*
    // manual set Transformation Matrix
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Constructing the transformation matrix manually
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

        // Translation
        transform(0, 3) = 0.0; // x translation
        transform(1, 3) = 0.0; // y translation
        transform(2, 3) = 0.0; // z translation

        // Rotation (using roll, pitch, yaw)
        double yaw = 0.0;
        double pitch = 0.0;
        double roll = 0.0;
        Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
        Eigen::Quaternion<float> q = yawAngle * pitchAngle * rollAngle;
        transform.block<3, 3>(0, 0) = q.matrix();

        // Convert to PCL
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        // Apply the transformation
        pcl::transformPointCloud(pcl_cloud, pcl_cloud, transform);

        
        // Apply filters
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(pcl_cloud.makeShared());
        pass.setFilterFieldName("y");
        pass.setFilterLimits(0.0, 2.0);
        pass.filter(pcl_cloud);

        pass.setInputCloud(pcl_cloud.makeShared());
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-1.0, 1.0);
        pass.filter(pcl_cloud);

        pass.setInputCloud(pcl_cloud.makeShared());
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 1.0);
        pass.filter(pcl_cloud);
        

        // Convert back to ROS msg
        sensor_msgs::msg::PointCloud2 transformed;
        pcl::toROSMsg(pcl_cloud, transformed);
        transformed.header.frame_id = "world"; // Set the transformed frame ID

        publisher_->publish(transformed);
    }
    */


  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudTransformer>());
  rclcpp::shutdown();
  return 0;
}







