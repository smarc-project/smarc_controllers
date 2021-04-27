//Node to forward propeller PID control action to UAVCAN by publishing to sam_msgs.
//Sriharsha Bhat, 13.6.2019

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sam_msgs/ThrusterAngles.h>
#include <std_msgs/Bool.h>

sam_msgs::ThrusterAngles control_action;
//std_msgs::Float64 control_action;
double prev_control_msg1,prev_control_msg2,limit,freq;
std::string topic_from_controller_1_,topic_from_controller_2_, topic_to_actuator_, pid_enable_topic_1, pid_enable_topic_2 ;
bool message_received, enable_state_dheading, enable_state_ddepth;

void PIDCallback_Elevator(const std_msgs::Float64& control_msg)
{
  if(abs(prev_control_msg1-control_msg.data) > limit) {
    	  message_received=true;	
	  control_action.thruster_vertical_radians = control_msg.data;
    }
    ROS_INFO_THROTTLE(1.0, "[ pid_actuator ]  Control action heard: %f", control_msg.data);
}

void PIDCallback_Rudder(const std_msgs::Float64& control_msg)
{
   if(abs(prev_control_msg2-control_msg.data) > limit) {
    	   message_received= true;
	   control_action.thruster_horizontal_radians = control_msg.data;
    }
ROS_INFO_THROTTLE(1.0, "[ pid_actuator ]  Control action heard: %f", control_msg.data);
}

void enableCB_dheading(const std_msgs::Bool enable_msg){
	if(enable_msg.data == false)
  {
    enable_state_dheading = false;
  }
  else enable_state_dheading = true;
}

void enableCB_ddepth(const std_msgs::Bool enable_msg){
	if(enable_msg.data == false)
  {
    enable_state_ddepth = false;
  }
  else enable_state_ddepth = true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "pid_actuator_tv");

  ros::NodeHandle node;
  message_received=false;

  ros::NodeHandle node_priv("~");
  node_priv.param<std::string>("topic_from_controller_1", topic_from_controller_1_, "control_action_elevator");
  node_priv.param<std::string>("topic_from_controller_2", topic_from_controller_2_, "control_action_rudder");
  node_priv.param<std::string>("topic_to_actuator", topic_to_actuator_, "uavcan_tv_angle");
  node_priv.param<std::string>("pid_enable_topic_dheading", pid_enable_topic_1, "pid_enable");
  node_priv.param<std::string>("pid_enable_topic_ddepth", pid_enable_topic_2, "pid_enable");
  node_priv.param<double>("limit_between_setpoints", limit, 0.01);
  node_priv.param<double>("loop_freq", freq, 50);
  //initiate subscribers
  ros::Subscriber pid_action_sub_elevator = node.subscribe(topic_from_controller_1_, 1, PIDCallback_Elevator);
  ros::Subscriber pid_action_sub_rudder = node.subscribe(topic_from_controller_2_, 1, PIDCallback_Rudder);
  ros::Subscriber enable_sub_dheading = node.subscribe(pid_enable_topic_1, 10, enableCB_dheading);
  ros::Subscriber enable_sub_ddepth = node.subscribe(pid_enable_topic_2, 10, enableCB_ddepth);


  //initiate publishers
  ros::Publisher control_action_pub = node.advertise<sam_msgs::ThrusterAngles>(topic_to_actuator_, freq);
  //ros::Publisher control_action_pub = node.advertise<std_msgs::Float64>(topic_to_actuator_, 10);
  enable_state_dheading = true;
  enable_state_ddepth = true;


  ros::Rate rate(freq);

  while (node.ok()){

    if (message_received && (enable_state_ddepth || enable_state_dheading)) {
	    	control_action_pub.publish(control_action);
 	    	prev_control_msg1 = control_action.thruster_vertical_radians;
    		prev_control_msg2 = control_action.thruster_horizontal_radians;
    		//prev_control_msg = control_action.data;
    		ROS_INFO_THROTTLE(1.0, "[ pid_actuator ]  Control forwarded: Elevator:%f Rudder:%f", control_action.thruster_vertical_radians,control_action.thruster_horizontal_radians);
  	//  ROS_INFO_THROTTLE(1.0, "[ pid_actuator ]  Control forwarded: %f", control_action.data); //Gazebo
	}
    rate.sleep();
    ros::spinOnce();

  }
  return 0;
};
