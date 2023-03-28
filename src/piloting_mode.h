#ifndef HELM_MANAGER_PILOTING_MODE_H
#define HELM_MANAGER_PILOTING_MODE_H

#include <ros/ros.h>
#include "publisher.h"

namespace helm_manager
{

/// Listens to command topics for a given piloting mode
class PilotingMode
{
public:
  PilotingMode(std::string mode, std::string mode_prefix, Publisher &publisher, bool enable=true);
  
  void activeMode(std::string const &mode);
  
private:
  template<typename T> void callback(const ros::MessageEvent<T>& event)
  {
    if(m_active)
      m_publisher.update(m_piloting_mode, event.getMessage(), event.getPublisherName());
  }

  std::string m_piloting_mode;
  bool m_active;
  ros::Publisher m_active_pub;
  ros::Subscriber m_helm_sub;
  ros::Subscriber m_twist_sub;
  Publisher& m_publisher;
};

}

#endif
