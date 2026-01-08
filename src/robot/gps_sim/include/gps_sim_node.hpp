#ifndef GPS_SIM_NODE_HPP_
#define GPS_SIM_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <random>

class GpsSimNode : public rclcpp::Node {
  public:
    GpsSimNode();

  private:
    void timerCallback();
    void processParameters();
    double generateGaussianNoise(double mean, double stddev);

    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;

    // Transform Utilities
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Timer to periodically publish GPS data
    rclcpp::TimerBase::SharedPtr timer_;

    // Random number generator for noise
    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<double> gaussian_noise_;

    // Configuration parameters
    std::string source_frame_;
    std::string target_frame_;
    double gps_noise_std_;                // Standard deviation for GPS position noise (meters)
    double gps_update_rate_;               // GPS update rate (Hz)
    double gps_dropout_probability_;       // Probability of GPS dropout (0.0-1.0)
    bool simulate_dropouts_;               // Whether to simulate GPS dropouts
};

#endif

