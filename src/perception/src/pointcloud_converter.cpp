#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/filters/voxel_grid.h"

class PointCloudConverter : public rclcpp::Node {
    public: 
        PointCloudConverter() : Node("pointcloud_converter") {
            subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/sim/realsense1/depth/points", 
                rclcpp::SensorDataQoS(),
                std::bind(&PointCloudConverter::pointcloud_callback, this, std::placeholders::_1)
            );
            publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/sim/realsense1/depth/points_filtered",
                rclcpp::SensorDataQoS()
            ); 
        }
    private:
        void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); 
            pcl::fromROSMsg(*msg, *cloud); 

            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>); 
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter; 
            voxel_filter.setInputCloud(cloud);
            voxel_filter.setLeafSize(0.01f, 0.01f, 0.01f); //voxel size
            voxel_filter.filter(*filtered_cloud);

            sensor_msgs::msg::PointCloud2 output_msg;
            pcl::toROSMsg(*filtered_cloud, output_msg);
            output_msg.header = msg->header; 
            
            publisher_->publish(output_msg);
        }
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    }; 

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudConverter>());
    rclcpp::shutdown();
    return 0;
}