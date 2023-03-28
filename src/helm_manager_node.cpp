// Roland Arsenault
// Center for Coastal and Ocean Mapping
// University of New Hampshire
// Copyright 2021, All rights reserved.

#include "ros/ros.h"
#include "publisher.h"
#include "piloting_mode.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "helm_manager");
  ros::NodeHandle nh_private("~");
  
  std::string output_type = "helm";
  nh_private.getParam("output_type", output_type);

  auto publisher = std::make_shared<helm_manager::Publisher>(output_type);

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
