#include "piloting_mode.h"

#include "publisher.h"

namespace helm_manager
{

PilotingMode::PilotingMode(std::string mode, std::string mode_prefix, Publisher &publisher, bool enable):
    m_piloting_mode(mode),
    m_active(false),
    m_publisher(publisher)
{
  ros::NodeHandle nh;
  m_active_pub = nh.advertise<std_msgs::Bool>(mode_prefix+mode+"/active", 1, true);
  if(enable)
  {
    m_helm_sub = nh.subscribe(mode_prefix+mode+"/helm", 10, &PilotingMode::callback<project11_msgs::Helm const>, this);
    m_twist_sub = nh.subscribe(mode_prefix+mode+"/cmd_vel", 10, &PilotingMode::callback<geometry_msgs::TwistStamped const>, this);
  }
}
  
void PilotingMode::activeMode(std::string const &mode)
{
  m_active = (mode == m_piloting_mode);
  std_msgs::Bool active;
  active.data = m_active;
  m_active_pub.publish(active);
}

} // namespace helm_manager
