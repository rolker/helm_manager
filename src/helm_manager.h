#ifndef HELM_MANAGER_HELM_MANAGER_H
#define HELM_MANAGER_HELM_MANAGER_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "project11_msgs/msg/helm.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "project11_msgs/msg/heartbeat.hpp"

namespace helm_manager
{

class PilotingMode;

class HelmManager: public rclcpp::Node
{
public:
  HelmManager();

  void update(const std::string& mode, const geometry_msgs::msg::TwistStamped& msg);
  void update(const std::string& mode, const project11_msgs::msg::Helm& msg);

private: 
  void addPilotingMode(const std::string& mode, bool enable_output=true);

  bool canPublish(const std::string& mode);

  void pilotingModeCallback(const std_msgs::msg::String& msg);
  
  void helmStatusCallback(const project11_msgs::msg::Heartbeat& msg);

  void updateParameters(const std::vector<rclcpp::Parameter> & parameters);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr piloting_mode_subscription_;
  std::string piloting_mode_;

  rclcpp::Publisher<project11_msgs::msg::Heartbeat>::SharedPtr heartbeat_publisher_;
  rclcpp::Subscription<project11_msgs::msg::Heartbeat>::SharedPtr helm_status_subscription_;
  
  std::vector<std::shared_ptr<PilotingMode> > piloting_modes_;
 
  std::string mode_prefix_;

  std::string output_type_;

  rclcpp::Publisher<project11_msgs::msg::Helm>::SharedPtr helm_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;

  double max_speed_;
  double max_yaw_speed_;

  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr update_parameters_callback_;
};


} // namespace helm_manager

#endif