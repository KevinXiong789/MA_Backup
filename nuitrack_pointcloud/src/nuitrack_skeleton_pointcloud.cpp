#include <iostream>
#include <nuitrack/Nuitrack.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using namespace tdv::nuitrack;

class NuitrackApp : public rclcpp::Node
{
public:
    NuitrackApp() : Node("nuitrack_app")
    {
        // Initialize ROS 2 publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("right_hand_coordinates", 10);
        // Initialize publishers for depth image and point cloud
        depth_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("depth_image", 10);
        depth_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Nuitrack/depth_cloud", 10);


        try {
            Nuitrack::init("");
        } catch (const Exception& e) {
            std::cerr << "Cannot initialize Nuitrack (ExceptionType: " << e.type() << ")" << std::endl;
            exit(EXIT_FAILURE);
        }

        // Create Skeleton Tracker
        skeletonTracker_ = SkeletonTracker::create();
        skeletonTracker_->connectOnUpdate(std::bind(&NuitrackApp::onSkeletonUpdate, this, std::placeholders::_1));

        // Create Depth Sensor
        depthSensor_ = DepthSensor::create();
        depthSensor_->connectOnNewFrame(std::bind(&NuitrackApp::onNewDepthFrame, this, std::placeholders::_1));

        // Start Nuitrack
        try {
            Nuitrack::run();
        } catch (const Exception& e) {
            std::cerr << "Cannot start Nuitrack (ExceptionType: " << e.type() << ")" << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    ~NuitrackApp() {
        try {
            std::cerr << "Releasing Nuitrack..." << std::endl;
            Nuitrack::release();
        } catch (const Exception& e) {
            std::cerr << "Nuitrack release failed (ExceptionType: " << e.type() << ")" << std::endl;
        }
    }

    void run()
    {
        while (rclcpp::ok()) {
            try {
                Nuitrack::waitUpdate(skeletonTracker_);
            } catch (const Exception& e) {
                std::cerr << "Nuitrack update failed (ExceptionType: " << e.type() << ")" << std::endl;
                break;
            }
        }
    }

private:
    void onSkeletonUpdate(SkeletonData::Ptr skeletonData)
    {
        const std::vector<Skeleton> skeletons = skeletonData->getSkeletons();
        for (const Skeleton& skeleton : skeletons) {
            const Joint& rightHand = skeleton.joints[JOINT_RIGHT_HAND];
            
            // Publish right hand 3D coordinates
            geometry_msgs::msg::Point rightHandPos;
            rightHandPos.x = rightHand.real.x;
            rightHandPos.y = rightHand.real.y;
            rightHandPos.z = rightHand.real.z;

            publisher_->publish(rightHandPos);

            std::cout << "Detected skeleton with ID: " << skeleton.id
                      << ". Right Hand Position (X, Y, Z): " << rightHand.real.x
                      << ", " << rightHand.real.y << ", " << rightHand.real.z << std::endl;
        }
    }

    void onNewDepthFrame(DepthFrame::Ptr frame)
    {
        int _width = frame->getCols(); 
        int _height = frame->getRows();
        const uint16_t* depthPtr = frame->getData();

        sensor_msgs::msg::Image depth_msg;
        depth_msg.header.stamp = this->now();
        depth_msg.header.frame_id = "map";
        depth_msg.height = _height; 
        depth_msg.width = _width; 
        depth_msg.encoding = "rgb8";
        depth_msg.is_bigendian = false;
        depth_msg.step = 3 * _width;

        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = "map";
        cloud_msg.width = _width;
        cloud_msg.height = _height;
        cloud_msg.is_bigendian = false;
        cloud_msg.is_dense = false;
        cloud_msg.point_step = sizeof(float) * 3;  // XYZ
        cloud_msg.row_step = cloud_msg.point_step * _width;

        // Define the fields in the PointCloud2 message
        cloud_msg.fields.resize(3);
        cloud_msg.fields[0].name = "x"; cloud_msg.fields[0].offset = 0; cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud_msg.fields[0].count = 1;
        cloud_msg.fields[1].name = "y"; cloud_msg.fields[1].offset = 4; cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud_msg.fields[1].count = 1;
        cloud_msg.fields[2].name = "z"; cloud_msg.fields[2].offset = 8; cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud_msg.fields[2].count = 1;
        cloud_msg.data.resize(cloud_msg.row_step * _height);

        sensor_msgs::PointCloud2Modifier cloud_mod(cloud_msg);
        cloud_mod.resize(_width * _height);
        
        sensor_msgs::PointCloud2Iterator<float> out_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(cloud_msg, "z");

        for (size_t row = 0; row < _height; ++row)
        {
            for (size_t col = 0; col < _width; ++col)
            {
                uint16_t fulldepthValue = *(depthPtr+ col);
                uint16_t depthValue = *(depthPtr+ col) >> 5;
                
                // RGB are all the same for depth (monochrome)
                depth_msg.data.push_back(depthValue); 
                depth_msg.data.push_back(depthValue);
                depth_msg.data.push_back(depthValue);
                
                Vector3 cloud_point = depthSensor_->convertProjToRealCoords(col, row, fulldepthValue );
                float X_World = cloud_point.x / 1000.0; 
                float Y_World = cloud_point.y / 1000.0;
                float Z_World = cloud_point.z / 1000.0; 
                
                *out_x = Z_World;
                *out_y = -X_World;
                *out_z = Y_World; 
                
                ++out_x;
                ++out_y;
                ++out_z;
            }
            depthPtr += _width; 
        }

        depth_image_pub_->publish(depth_msg);
        depth_cloud_pub_->publish(cloud_msg);
    }

private:
    SkeletonTracker::Ptr skeletonTracker_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_cloud_pub_;
    // Assuming depthSensor_ is also a member of this class
    DepthSensor::Ptr depthSensor_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    NuitrackApp app;
    app.run();
    rclcpp::shutdown();
    return 0;
}
