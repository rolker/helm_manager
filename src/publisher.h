#ifndef HELM_MANAGER_PUBLISHER_H
#define HELM_MANAGER_PUBLISHER_H

#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "project11_msgs/Helm.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "project11_msgs/Heartbeat.h"
#include "project11/pid.h"
#include "nav_msgs/Odometry.h"

namespace helm_manager
{

class PilotingMode;

class Publisher
{
public:
  Publisher(const std::string & output_type);
  
  void addPilotingMode(const std::string& mode, bool enable_output=true);
  void update(const std::string& mode, const geometry_msgs::TwistStamped::ConstPtr& msg, const std::string& publisher);
  void update(const std::string& mode, const project11_msgs::Helm::ConstPtr& msg, const std::string& publisher);

private:
  bool canPublish(const std::string& mode, const std::string& publisher);

  void pilotingModeCallback(const std_msgs::String::ConstPtr& inmsg);
  
  void helmStatusCallback(const project11_msgs::Heartbeat::ConstPtr& msg);

  void collisionDetecterCallback(const ros::TimerEvent event);
  
  // help detect if multiple publishers are publishing simultaneously
  void trackPublisherChange(const std::string& publisher);

  void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg);

  ros::Subscriber m_piloting_mode_sub;
  std::string m_piloting_mode;

  ros::Publisher m_heartbeat_pub;
  ros::Subscriber m_helm_status_sub;
  
  std::vector<std::shared_ptr<PilotingMode> > m_piloting_modes;
  
  ros::Timer m_collision_detecter_timer;
  
  std::map<std::string, int> m_publisher_change_counts;
  std::string m_last_publisher;
  int m_collision_detection_threshold = 2;
  
  std::string m_mode_prefix;

  std::string output_type_;

  ros::Publisher m_helm_pub;
  ros::Subscriber odom_sub_;
  ros::Publisher m_twist_pub;

  double m_max_speed;
  double m_max_yaw_speed;
  //project11::PID pid_;
  nav_msgs::Odometry latest_odometry_;
};


} // namespace helm_manager

#endif