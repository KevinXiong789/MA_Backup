/*
// get robot joint positions and filter the point cloud of robot
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>


class PointCloudFilterNode : public rclcpp::Node
{
public:
    PointCloudFilterNode()
    : Node("point_cloud_filter_node")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 10);
        cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points", 10,
            std::bind(&PointCloudFilterNode::callback, this, std::placeholders::_1));
        
        points_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/robot/joint_positions", 10,
            std::bind(&PointCloudFilterNode::pointsCallback, this, std::placeholders::_1));

        p1_ = Eigen::Vector3f(0, 0, 0);
        p2_ = Eigen::Vector3f(0, 0, 0);

        box_width_ = 0.5;
        box_depth_ = 0.5;
    }

private:
    void pointsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if(msg->data.size() >= 6)  // 确保有足够的数据来提取两个点
        {
            p1_ = Eigen::Vector3f(msg->data[0], msg->data[1], msg->data[2]);
            p2_ = Eigen::Vector3f(msg->data[3], msg->data[4], msg->data[5]);
        }
    }

    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(*msg, *cloud);

        Eigen::Vector3f direction = (p2_ - p1_).normalized();
        float half_height = (p2_ - p1_).norm() / 2.0;
        Eigen::Vector3f center = p1_ + half_height * direction;

        Eigen::Vector4f min_pt(center[0] - box_width_/2, center[1] - box_depth_/2, center[2] - half_height, 1.0);
        Eigen::Vector4f max_pt(center[0] + box_width_/2, center[1] + box_depth_/2, center[2] + half_height, 1.0);


        pcl::CropBox<pcl::PointXYZ> box_filter;
        box_filter.setMin(min_pt);
        box_filter.setMax(max_pt);
        box_filter.setNegative(true);  // 保留box外的点
        box_filter.setInputCloud(cloud);
        box_filter.filter(*filtered_cloud);

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*filtered_cloud, output);
        output.header.frame_id = msg->header.frame_id;

        publisher_->publish(output);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr points_subscription_;

    Eigen::Vector3f p1_, p2_;
    float box_width_, box_depth_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudFilterNode>());
    rclcpp::shutdown();
    return 0;
}
*/



// get robot joint positions and filter the point cloud of robot (robot filter is cuboid)       ******************** can run
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>


class PointCloudFilterNode : public rclcpp::Node
{
public:
    PointCloudFilterNode()
    : Node("point_cloud_filter_node")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 10);
        cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points", 10,
            std::bind(&PointCloudFilterNode::callback, this, std::placeholders::_1));
        
        points_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/robot/joint_positions", 10,
            std::bind(&PointCloudFilterNode::pointsCallback, this, std::placeholders::_1));

        p1_ = Eigen::Vector3f(0, 0, 0);
        p2_ = Eigen::Vector3f(0, 0, 0);
        p3_ = Eigen::Vector3f(0, 0, 0);
        p4_ = Eigen::Vector3f(0, 0, 0);

        box_width_ = 0.1;
        box_depth_ = 0.1;
    }

private:
    void pointsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if(msg->data.size() >= 12)  // 确保有足够的数据来提取4个点
        {
            p1_ = Eigen::Vector3f(msg->data[0], msg->data[1], msg->data[2]);
            p2_ = Eigen::Vector3f(msg->data[3], msg->data[4], msg->data[5]);
            p3_ = Eigen::Vector3f(msg->data[6], msg->data[7], msg->data[8]);
            p4_ = Eigen::Vector3f(msg->data[9], msg->data[10], msg->data[11]);
        }
    }

    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr accumulator_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        
        pcl::fromROSMsg(*msg, *cloud);
        *accumulator_cloud = *cloud;

        std::vector<Eigen::Vector3f> points = {p1_, p2_, p3_, p4_};

        for (size_t i = 0; i < points.size() - 1; i++)
        {
            Eigen::Vector3f start_point = points[i];
            Eigen::Vector3f end_point = points[i + 1];

            Eigen::Vector3f direction = (end_point - start_point).normalized();
            float half_height = (end_point - start_point).norm() / 2.0;
            Eigen::Vector3f center = start_point + half_height * direction;

            if (i == points.size() - 2) // 最后一个点
            {
                half_height += 0.2;
                center += direction * 0.2;
            }

            Eigen::Vector4f min_pt(center[0] - box_width_/2, center[1] - box_depth_/2, center[2] - half_height, 1.0);
            Eigen::Vector4f max_pt(center[0] + box_width_/2, center[1] + box_depth_/2, center[2] + half_height, 1.0);

            pcl::CropBox<pcl::PointXYZ> box_filter;
            box_filter.setMin(min_pt);
            box_filter.setMax(max_pt);
            box_filter.setNegative(true);  // 保留box外的点
            box_filter.setInputCloud(accumulator_cloud);
            box_filter.filter(*temp_cloud);

            accumulator_cloud->swap(*temp_cloud);
        }

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*accumulator_cloud, output);
        output.header.frame_id = msg->header.frame_id;

        publisher_->publish(output);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr points_subscription_;

    Eigen::Vector3f p1_, p2_, p3_, p4_;
    float box_width_, box_depth_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudFilterNode>());
    rclcpp::shutdown();
    return 0;
}





/*
// when Filter is cylinder, problem
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudFilterNode : public rclcpp::Node
{
public:
    PointCloudFilterNode()
    : Node("point_cloud_filter_node")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 10);
        cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points", 10,
            std::bind(&PointCloudFilterNode::callback, this, std::placeholders::_1));
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        pcl::fromROSMsg(*msg, *cloud);

        // Estimate normals
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        ne.setRadiusSearch(0.03);
        ne.compute(*cloud_normals);

        // Segment the largest cylindrical component from the point cloud
        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
        pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CYLINDER);
        seg.setNormalDistanceWeight(0.1);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.05);
        seg.setInputCloud(cloud);
        seg.setInputNormals(cloud_normals);
        seg.segment(*inliers_cylinder, *coefficients_cylinder);

        if(inliers_cylinder->indices.size() == 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not estimate a cylindrical model.");
            return;
        }

        // Extract the cylindrical part of the point cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers_cylinder);
        extract.setNegative(false);
        extract.filter(*cylinder_cloud);

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cylinder_cloud, output);
        output.header.frame_id = msg->header.frame_id;
        publisher_->publish(output);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudFilterNode>());
    rclcpp::shutdown();
    return 0;
}
*/
