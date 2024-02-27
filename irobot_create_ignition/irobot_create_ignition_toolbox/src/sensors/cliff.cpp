/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#include <memory>
#include <string>
#include <vector>

#include "irobot_create_ignition_toolbox/sensors/cliff.hpp"
#include "irobot_create_toolbox/math.hpp"

using irobot_create_ignition_toolbox::Cliff;

Cliff::Cliff(std::shared_ptr<rclcpp::Node> & nh)
: nh_(nh),
  cliff_sensors_{
    "front_left",
    "front_right",
    "side_left",
    "side_right"
  }
{
  auto cliff_sub_topics =
    nh_->declare_parameter("cliff_subscription_topics", std::vector<std::string>());

  auto cliff_pub_topics =
    nh_->declare_parameter("cliff_publish_topics", std::vector<std::string>());

  for (std::string topic : cliff_sub_topics) {
    cliff_sub_.push_back(
      nh_->create_subscription<sensor_msgs::msg::LaserScan>(
        topic,
        rclcpp::SensorDataQoS(),
        std::bind(&Cliff::cliff_callback, this, std::placeholders::_1)));
        //std::bind(&Cliff::cliff_intensities_callback, this, std::placeholders::_1)));
  }

  for (const std::string & topic : cliff_pub_topics) {
    for (const std::string & sensor : cliff_sensors_) {
      if (topic.find(sensor) != std::string::npos) {
        hazard_pub_[sensor] = nh_->create_publisher<
          irobot_create_msgs::msg::HazardDetection>(
          topic,
          rclcpp::SensorDataQoS());

      }
    }
  }

  // publisher added by Paolo
  intensites_pub_ = nh_->create_publisher<std_msgs::msg::Float32>("cliff_dummy_topic", 10);

}

void Cliff::cliff_callback(const sensor_msgs::msg::LaserScan::SharedPtr cliff_msg)
{
  constexpr double detection_threshold = 0.03;

  const double range_detection = irobot_create_toolbox::FindMinimumRange(cliff_msg->ranges);

  if (range_detection > detection_threshold) {
    auto hazard_msg = irobot_create_msgs::msg::HazardDetection();
    hazard_msg.type = irobot_create_msgs::msg::HazardDetection::CLIFF;
    hazard_msg.header.frame_id = "base_link";

    // Publish to appropriate topic
    for (const std::string & sensor : cliff_sensors_) {
      if (cliff_msg->header.frame_id.find(sensor) != std::string::npos) {
        hazard_pub_[sensor]->publish(hazard_msg);
      }
    }
  }

  for (const auto& cliff_intensity : cliff_msg->intensities){
    auto dummy_msg = std_msgs::msg::Float32();
    dummy_msg.data = cliff_intensity;  
    intensites_pub_->publish(dummy_msg);
  }
  

}

//void Cliff::cliff_intensities_callback(const std_msgs::msg::Float32 cliff_msg)
//{
//  // Publish to appropriate topic
//  auto dummy_msg = std_msgs::msg::Float32();
//  dummy_msg.data = 1.0;  
//  intensites_pub_->publish(dummy_msg);
//    
//  
//}
