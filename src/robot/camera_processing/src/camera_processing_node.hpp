#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "camera_processing_core.hpp"

class CameraProcessingNode : public rclcpp::Node
{
    public: 
        CameraProcessingNode();
    private: 
        void cloudCallback(
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &c1,
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &c2);
        
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_cam1_;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_cam2_;

        using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud1_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud2_;
        robot::CameraProcessingCore camera_processing_;
};
        
        