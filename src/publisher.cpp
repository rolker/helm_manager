#include "publisher.h"
#include "piloting_mode.h"

namespace helm_manager
{

Publisher::Publisher(const std::string & output_type):output_type_(output_type)
{
  ros::param::param<std::string>("~piloting_mode_prefix", m_mode_prefix, "");
  
  ros::NodeHandle nh;
  m_piloting_mode_sub = nh.subscribe("piloting_mode", 10, &Publisher::pilotingModeCallback, this );
  m_heartbeat_pub = nh.advertise<project11_msgs::Heartbeat>("heartbeat", 10);
  m_helm_status_sub = nh.subscribe("status/helm", 10, &Publisher::helmStatusCallback, this );
  m_collision_detecter_timer = nh.createTimer(ros::Duration(1.0), std::bind(&Publisher::collisionDetecterCallback, this, std::placeholders::_1));

  ros::NodeHandle nh_private("~");
  nh_private.param<double>("max_speed", m_max_speed, 1.0);
  nh_private.param<double>("max_yaw_speed", m_max_yaw_speed, 1.0);

  if(output_type == "helm" || output_type == "dual")
  {
    m_helm_pub = nh.advertise<project11_msgs::Helm>("out/helm", 10);

    if(output_type != "dual")
    {
      odom_sub_ = nh.subscribe("odom", 5, &Publisher::odometryCallback, this);
      //pid_.configure(ros::NodeHandle("~/helm/pid"));
    }
  }

  if(output_type == "twist" || output_type == "dual")
    m_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("out/cmd_vel", 10);
}

bool Publisher::canPublish(const std::string& mode, const std::string& publisher)
{
  if(mode == m_piloting_mode)
  {
    trackPublisherChange(publisher);
    return true;
  }
  return false;
}

void Publisher::helmStatusCallback(const project11_msgs::Heartbeat::ConstPtr& msg)
{
  project11_msgs::Heartbeat hb;
  hb.header = msg->header;

  project11_msgs::KeyValue kv;

  kv.key = "piloting_mode";
  kv.value = m_piloting_mode;
  hb.values.push_back(kv);
  
  for(const auto& kv: msg->values)
    hb.values.push_back(kv);
  
  m_heartbeat_pub.publish(hb);
}

void Publisher::collisionDetecterCallback(const ros::TimerEvent event)
{
  for(auto pub: m_publisher_change_counts)
    if(pub.second > m_collision_detection_threshold)
      ROS_WARN_STREAM("helm_manager: Potential publisher collision: " << pub.first);
  m_publisher_change_counts.clear();
}

void Publisher::trackPublisherChange(const std::string& publisher)
{
  if(!m_last_publisher.empty() && publisher != m_last_publisher)
    m_publisher_change_counts[publisher] += 1;
  m_last_publisher = publisher;
}

void Publisher::odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  latest_odometry_ = *msg;
}

void Publisher::update(const std::string & mode, const project11_msgs::Helm::ConstPtr & msg, const std::string & publisher)
{
  if(canPublish(mode, publisher))
    if(output_type_ == "helm" || output_type_ == "dual")
      m_helm_pub.publish(msg);
    else
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
  
void Publisher::update(const std::string & mode, const geometry_msgs::TwistStamped::ConstPtr & msg, const std::string & publisher)
{
  if(canPublish(mode, publisher))
    if(output_type_ == "twist" || output_type_ == "dual")
    {
      geometry_msgs::TwistStamped twist_clamped = *msg;
      twist_clamped.twist.linear.x = std::max(-m_max_speed, std::min(m_max_speed, twist_clamped.twist.linear.x));
      twist_clamped.twist.angular.z = std::max(-m_max_yaw_speed, std::min(m_max_yaw_speed, twist_clamped.twist.angular.z));
      m_twist_pub.publish(twist_clamped);
    }
    else
    {
      project11_msgs::Helm helm;
      helm.header = msg->header;
      helm.throttle = msg->twist.linear.x/m_max_speed;
      // if(msg->header.stamp - latest_odometry_.header.stamp < ros::Duration(1.0))
      // {
      //   //pid_.setPoint(msg->twist.linear.x);
      //   //helm.throttle = pid_.update(latest_odometry_.twist.twist.linear.x, latest_odometry_.header.stamp);
      // }
      // else
      //   ROS_WARN_STREAM_THROTTLE(2.0,"No recent odometry for use with throttle PID");
      helm.throttle = std::max(-1.0, std::min(1.0, double(helm.throttle)));
      helm.rudder = -msg->twist.angular.z/m_max_yaw_speed;
      helm.rudder = std::max(-1.0, std::min(1.0, double(helm.rudder)));
      m_helm_pub.publish(helm);
    }
}

void Publisher::addPilotingMode(const std::string& mode, bool enable_output)
{
  m_piloting_modes.push_back(std::shared_ptr<PilotingMode>(new PilotingMode(mode, m_mode_prefix, *this, enable_output)));
}

void Publisher::pilotingModeCallback(const std_msgs::String::ConstPtr& inmsg)
{
  m_piloting_mode = inmsg->data;
  m_publisher_change_counts.clear();
  for(auto m: m_piloting_modes)
    m->activeMode(m_piloting_mode);
}


} // namespace helm_manager
