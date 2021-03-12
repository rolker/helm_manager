// Roland Arsenault
// Center for Coastal and Ocean Mapping
// University of New Hampshire
// Copyright 2021, All rights reserved.

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/TwistStamped.h"
#include "marine_msgs/Heartbeat.h"
#include "marine_msgs/Helm.h"

class PilotingMode;

class BasePublisher
{
public:
  BasePublisher():m_collision_detection_threshold(2)
  {
    
    ros::param::param<std::string>("~piloting_mode_prefix", m_mode_prefix, "");
    
    ros::NodeHandle nh;
    m_piloting_mode_sub = nh.subscribe("piloting_mode", 10, &BasePublisher::pilotingModeCallback, this );
    m_heartbeat_pub = nh.advertise<marine_msgs::Heartbeat>("heartbeat", 10);
    m_helm_status_sub = nh.subscribe("status/helm", 10, &BasePublisher::helmStatusCallback, this );
    m_collision_detecter_timer = nh.createTimer(ros::Duration(1.0), std::bind(&BasePublisher::collisionDetecterCallback, this, std::placeholders::_1));
  }
  
  void addPilotingMode(const std::string& mode, bool enable_output=true);
  virtual void update(const std::string& mode, const geometry_msgs::TwistStamped::ConstPtr& msg, const std::string& publisher) = 0;
  virtual void update(const std::string& mode, const marine_msgs::Helm::ConstPtr& msg, const std::string& publisher) = 0;

protected:  
  bool canPublish(const std::string& mode, const std::string& publisher)
  {
    if(mode == m_piloting_mode)
    {
      trackPublisherChange(publisher);
      return true;
    }
    return false;
  }

private:
  void pilotingModeCallback(const std_msgs::String::ConstPtr& inmsg);
  
  void helmStatusCallback(const marine_msgs::Heartbeat::ConstPtr& msg)
  {
    marine_msgs::Heartbeat hb;
    hb.header = msg->header;

    marine_msgs::KeyValue kv;

    kv.key = "piloting_mode";
    kv.value = m_piloting_mode;
    hb.values.push_back(kv);
    
    for(const auto& kv: msg->values)
      hb.values.push_back(kv);
    
    m_heartbeat_pub.publish(hb);
  }

  void collisionDetecterCallback(const ros::TimerEvent event)
  {
    for(auto pub: m_publisher_change_counts)
      if(pub.second > m_collision_detection_threshold)
        ROS_WARN_STREAM("helm_manager: Potential publisher collision: " << pub.first);
    m_publisher_change_counts.clear();
  }
  
  // help detect if multiple publishers are publishing simultaneously
  void trackPublisherChange(const std::string& publisher)
  {
    if(!m_last_publisher.empty() && publisher != m_last_publisher)
      m_publisher_change_counts[publisher] += 1;
    m_last_publisher = publisher;
  }

  ros::Subscriber m_piloting_mode_sub;
  std::string m_piloting_mode;

  ros::Publisher m_heartbeat_pub;
  ros::Subscriber m_helm_status_sub;
  
  std::vector<std::shared_ptr<PilotingMode> > m_piloting_modes;
  
  ros::Timer m_collision_detecter_timer;
  
  std::map<std::string, int> m_publisher_change_counts;
  std::string m_last_publisher;
  int m_collision_detection_threshold;
  
  std::string m_mode_prefix;
};

class HelmPublisher: public BasePublisher
{
public:
  HelmPublisher()
  {
    ros::NodeHandle nh;
    m_helm_pub = nh.advertise<marine_msgs::Helm>("out/helm", 10);
    
    ros::NodeHandle nh_private("~");
    
    nh_private.param<double>("helm/max_speed", m_max_speed, 1.0);
    nh_private.param<double>("helm/max_yaw_speed", m_max_yaw_speed, 1.0);
    
  }
  
  virtual void update(const std::string & mode, const marine_msgs::Helm::ConstPtr & msg, const std::string & publisher) override
  {
    if(canPublish(mode, publisher))
      m_helm_pub.publish(msg);
  }
  
  void update(const std::string & mode, const geometry_msgs::TwistStamped::ConstPtr & msg, const std::string & publisher) override
  {
    if(canPublish(mode, publisher))
    {
      marine_msgs::Helm helm;
      helm.header = msg->header;
      helm.throttle = msg->twist.linear.x/m_max_speed;
      helm.throttle = std::max(-1.0, std::min(1.0, double(helm.throttle)));
      helm.rudder = -msg->twist.angular.z/m_max_yaw_speed;
      helm.rudder = std::max(-1.0, std::min(1.0, double(helm.rudder)));
      m_helm_pub.publish(helm);
    }
  }
  
  
private:
  ros::Publisher m_helm_pub;
  
  double m_max_speed;
  double m_max_yaw_speed;
};

class TwistPublisher: public BasePublisher
{
public:
  TwistPublisher()
  {
    ros::NodeHandle nh;
    m_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("out/cmd_vel", 10);
    
    ros::NodeHandle nh_private("~");
    
    nh_private.param<double>("twist/max_speed", m_max_speed, 1.0);
    nh_private.param<double>("twist/max_yaw_speed", m_max_yaw_speed, 1.0);
  }
  
  void update(const std::string & mode, const geometry_msgs::TwistStamped::ConstPtr & msg, const std::string & publisher) override
  {
    if(canPublish(mode, publisher))
    {
      geometry_msgs::TwistStamped twist_clamped = *msg;
      twist_clamped.twist.linear.x = std::max(-m_max_speed, std::min(m_max_speed, twist_clamped.twist.linear.x));
      twist_clamped.twist.angular.z = std::max(-m_max_yaw_speed, std::min(m_max_yaw_speed, twist_clamped.twist.angular.z));
      m_twist_pub.publish(twist_clamped);
    }
  }
  
  void update(const std::string & mode, const marine_msgs::Helm::ConstPtr & msg, const std::string & publisher) override
  {
    if(canPublish(mode, publisher))
    {
      geometry_msgs::TwistStamped twist;
      twist.header = msg->header;
      twist.twist.linear.x = msg->throttle*m_max_speed;
      twist.twist.linear.x = std::max(-m_max_speed, std::min(m_max_speed, twist.twist.linear.x));
      twist.twist.angular.z = -msg->rudder*m_max_yaw_speed;
      twist.twist.angular.z = std::max(-m_max_yaw_speed, std::min(m_max_yaw_speed, twist.twist.angular.z));
      m_twist_pub.publish(twist);
    }
  }
  
private:
  ros::Publisher m_twist_pub;

  double m_max_speed;
  double m_max_yaw_speed;
};


/// Listens to command topics for a given piloting mode
class PilotingMode
{
public:
  PilotingMode(std::string mode, std::string mode_prefix, BasePublisher &publisher, bool enable=true):m_piloting_mode(mode),m_active(false),m_publisher(publisher)
  {
    ros::NodeHandle nh;
    m_active_pub = nh.advertise<std_msgs::Bool>(mode_prefix+mode+"/active", 1, true);
    if(enable)
    {
      m_helm_sub = nh.subscribe(mode_prefix+mode+"/helm", 10, &PilotingMode::callback<marine_msgs::Helm const>, this);
      m_twist_sub = nh.subscribe(mode_prefix+mode+"/cmd_vel", 10, &PilotingMode::callback<geometry_msgs::TwistStamped const>, this);
    }
  }
  
  void activeMode(std::string const &mode)
  {
    m_active = (mode == m_piloting_mode);
    std_msgs::Bool active;
    active.data = m_active;
    m_active_pub.publish(active);
  }
  
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
  BasePublisher& m_publisher;
};


void BasePublisher::addPilotingMode(const std::string& mode, bool enable_output)
{
  m_piloting_modes.push_back(std::shared_ptr<PilotingMode>(new PilotingMode(mode, m_mode_prefix, *this, enable_output)));
}

void BasePublisher::pilotingModeCallback(const std_msgs::String::ConstPtr& inmsg)
{
  m_piloting_mode = inmsg->data;
  m_publisher_change_counts.clear();
  for(auto m: m_piloting_modes)
    m->activeMode(m_piloting_mode);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "helm_manager");
  ros::NodeHandle nh_private("~");
  
  std::string output_type = "helm";
  nh_private.getParam("output_type", output_type);

  std::shared_ptr<BasePublisher> publisher;
  if(output_type == "helm")
    publisher = std::shared_ptr<HelmPublisher>(new HelmPublisher);
  if(output_type == "twist")
    publisher = std::shared_ptr<TwistPublisher>(new TwistPublisher);

  if(publisher)
  {
    XmlRpc::XmlRpcValue piloting_modes;
    if(nh_private.getParam("piloting_modes/active", piloting_modes) && piloting_modes.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for(int i = 0; i < piloting_modes.size(); i++)
        publisher->addPilotingMode(piloting_modes[i]);
      if(nh_private.getParam("piloting_modes/standby", piloting_modes) && piloting_modes.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        for(int i = 0; i < piloting_modes.size(); i++)
          publisher->addPilotingMode(piloting_modes[i],false);
      }
    }
    else
    {
      publisher->addPilotingMode("standby",false);
      publisher->addPilotingMode("manual");
      publisher->addPilotingMode("autonomous");
    }
    ros::spin();
  }
  else
  {
    ROS_ERROR_STREAM("Could not create publisher of type " << output_type);
    return 1;
  }
    
  return 0;
}
