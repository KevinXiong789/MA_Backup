/*
#include <iostream>
#include <nuitrack/Nuitrack.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


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
        marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("joint_markers", 10);


        try {
            
            Nuitrack::init("");
            auto devices = Nuitrack::getDeviceList();
            auto selectedDevice = devices[1];
            std::cout << "Selected device: " 
                  << selectedDevice->getInfo(tdv::nuitrack::device::DeviceInfoType::DEVICE_NAME) 
                  << " with serial number: " 
                  << selectedDevice->getInfo(tdv::nuitrack::device::DeviceInfoType::SERIAL_NUMBER) 
                  << std::endl;
            Nuitrack::setDevice(selectedDevice);
            
            
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
            rightHandPos.x = rightHand.real.x/1000.0;
            rightHandPos.y = rightHand.real.y/1000.0;
            rightHandPos.z = rightHand.real.z/1000.0;

            publisher_->publish(rightHandPos);

            std::cout << "Detected skeleton with ID: " << skeleton.id
                      << ". Right Hand Position (X, Y, Z): " << rightHandPos.x
                      << ", " << rightHandPos.y << ", " << rightHandPos.z << std::endl;
        }
        publishJointMarkers(skeletonData->getSkeletons());
    }

    void publishJointMarkers(const std::vector<Skeleton>& skeletons) {
        visualization_msgs::msg::MarkerArray marker_array;

        int marker_id = 0; // Unique ID for each marker

        for (const Skeleton& skeleton : skeletons) {
            for (const Joint& joint : skeleton.joints) {
                visualization_msgs::msg::Marker marker;
                
                marker.header.frame_id = "map"; // Assuming you're using this frame
                marker.header.stamp = this->now();
                marker.ns = "joints";
                marker.id = marker_id++;
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;

                // Set the marker position to the joint position
                marker.pose.position.x = joint.real.x/1000.0;
                marker.pose.position.y = joint.real.y/1000.0;
                marker.pose.position.z = joint.real.z/1000.0;
                marker.pose.orientation.w = 1.0;  // No rotation

                marker.scale.x = 0.05;  // Set the size of the marker
                marker.scale.y = 0.05;
                marker.scale.z = 0.05;

                marker.color.r = 1.0; // Set the color
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0; // Don't forget to set the alpha!

                marker.lifetime = rclcpp::Duration::from_seconds(0.1); // Duration the marker will be shown

                marker_array.markers.push_back(marker);
            }
        }

        marker_array_publisher_->publish(marker_array);
    }


    void onNewDepthFrame(DepthFrame::Ptr frame)
    {
        int _width = frame->getCols(); 
        int _height = frame->getRows();
        const uint16_t* depthPtr = frame->getData();
        
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
                
                Vector3 cloud_point = depthSensor_->convertProjToRealCoords(col, row, fulldepthValue );
                float X_World = cloud_point.x / 1000.0; 
                float Y_World = cloud_point.y / 1000.0;
                float Z_World = cloud_point.z / 1000.0; 
                
                
                *out_x = Z_World;
                *out_y = -X_World;
                *out_z = Y_World; 
                
                *out_x = X_World;
                *out_y = Y_World;
                *out_z = Z_World; 

                ++out_x;
                ++out_y;
                ++out_z;
            }
            depthPtr += _width; 
        }

        //depth_image_pub_->publish(depth_msg);
        depth_cloud_pub_->publish(cloud_msg);
    }

private:
    SkeletonTracker::Ptr skeletonTracker_;
    DepthSensor::Ptr depthSensor_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    NuitrackApp app;
    app.run();
    rclcpp::shutdown();
    return 0;
}
*/



// Use Nuitrack to get skeleton key point and convert depth frame to pointcloud2, publish right hand position and pointcloud
#include <iostream>
#include <nuitrack/Nuitrack.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


using namespace tdv::nuitrack;

class NuitrackApp : public rclcpp::Node
{
public:
    NuitrackApp() : Node("nuitrack_app")
    {
        // Initialize ROS 2 publisher
        joints_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/Nuitrack/right_hand", 10);
        // Initialize publishers for depth image and point cloud
        depth_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Nuitrack/depth_cloud", 10);
        marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/Nuitrack/joint_markers", 10);
 
        Nuitrack::init("");
        auto devices = Nuitrack::getDeviceList();
        auto selectedDevice = devices[1];
        std::cout << "Selected device: " 
                << selectedDevice->getInfo(tdv::nuitrack::device::DeviceInfoType::DEVICE_NAME) 
                << " with serial number: " 
                << selectedDevice->getInfo(tdv::nuitrack::device::DeviceInfoType::SERIAL_NUMBER) 
                << std::endl;
        Nuitrack::setDevice(selectedDevice);

        // Create Skeleton Tracker
        skeletonTracker_ = SkeletonTracker::create();
        skeletonTracker_->connectOnUpdate(std::bind(&NuitrackApp::onSkeletonUpdate, this, std::placeholders::_1));

        // Create Depth Sensor
        depthSensor_ = DepthSensor::create();
        depthSensor_->connectOnNewFrame(std::bind(&NuitrackApp::onNewDepthFrame, this, std::placeholders::_1));
  
        // Start Nuitrack
        Nuitrack::run();
        while (rclcpp::ok()) { 
            Nuitrack::waitUpdate(skeletonTracker_);
            //Nuitrack::waitUpdate(depthSensor_);
        }
        Nuitrack::release();
    }

private:
    void onSkeletonUpdate(SkeletonData::Ptr skeletonData)
    {
        const std::vector<Skeleton> skeletons = skeletonData->getSkeletons();
        for (const Skeleton& skeleton : skeletons) {
            // Get Right Hand joint
            const Joint& rightHand = skeleton.joints[JOINT_RIGHT_HAND];
            const Joint& rightElbow = skeleton.joints[JOINT_RIGHT_ELBOW];
            
            std_msgs::msg::Float32MultiArray joint_coordinates;
            
            float rightHand_x = rightHand.real.z / 1000.0;
            float rightHand_y = -rightHand.real.x / 1000.0;
            float rightHand_z = rightHand.real.y / 1000.0;
            float rightElbow_x = rightElbow.real.z / 1000.0;
            float rightElbow_y = -rightElbow.real.x / 1000.0;
            float rightElbow_z = rightElbow.real.y / 1000.0;

            joint_coordinates.data = {
                rightHand_x, rightHand_y, rightHand_z,
                rightElbow_x, rightElbow_y, rightElbow_z
            };
            
            joints_publisher_->publish(joint_coordinates);
            /*
            std::cout << "Detected skeleton with ID: " << skeleton.id
                    << ". Right Hand Position (X, Y, Z): " 
                    << rightHand_x << ", " << rightHand_y << ", " << rightHand_z
                    << ". Right Elbow Position (X, Y, Z): " 
                    << rightElbow_x << ", " << rightElbow_y << ", " << rightElbow_z
                    << std::endl;
            */
        }
        publishJointMarkers(skeletonData->getSkeletons());
    }


    void publishJointMarkers(const std::vector<Skeleton>& skeletons) {
        visualization_msgs::msg::MarkerArray marker_array;

        int marker_id = 0; // Unique ID for each marker

        for (const Skeleton& skeleton : skeletons) {
            for (const Joint& joint : skeleton.joints) {
                visualization_msgs::msg::Marker marker;
                
                marker.header.frame_id = "camera_link"; 
                marker.header.stamp = this->now();
                marker.ns = "joints";
                marker.id = marker_id++;
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;

                // Set the marker position to the joint position
                marker.pose.position.x = joint.real.z / 1000.0;
                marker.pose.position.y = -joint.real.x / 1000.0;
                marker.pose.position.z = joint.real.y / 1000.0;
                marker.pose.orientation.w = 1.0;  // No rotation

                marker.scale.x = 0.05;  // Set the size of the marker
                marker.scale.y = 0.05;
                marker.scale.z = 0.05;

                marker.color.r = 1.0; // Set the color
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0; // Don't forget to set the alpha!

                marker.lifetime = rclcpp::Duration::from_seconds(0.1); // Duration the marker will be shown

                marker_array.markers.push_back(marker);
            }
        }

        marker_array_publisher_->publish(marker_array);
    }

    void onNewDepthFrame(DepthFrame::Ptr frame)
    {
        int _width = frame->getCols(); 
        int _height = frame->getRows();
        const uint16_t* depthPtr = frame->getData();
        
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = "camera_link";
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

        depth_cloud_pub_->publish(cloud_msg);
    }

private:
    SkeletonTracker::Ptr skeletonTracker_;
    DepthSensor::Ptr depthSensor_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joints_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto nuitrack_app = std::make_shared<NuitrackApp>();
    rclcpp::spin(nuitrack_app);
    rclcpp::shutdown();
    return 0;
}



