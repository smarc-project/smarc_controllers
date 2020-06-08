//Node to forward PID control action to UAVCAN by publishing to sam_msgs.
//Sriharsha Bhat, 05.12.2018

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sam_msgs/PercentStamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

sam_msgs::PercentStamped control_action;
double prev_control_msg,limit, freq;
std::string topic_from_controller_, topic_to_actuator_,pid_enable_topic_,abort_topic_;
bool message_received;
bool emergency_state, enable_state;

void PIDCallback(const std_msgs::Float64& control_msg)
{
  if(abs(prev_control_msg-control_msg.data) > limit) {
	message_received = true;
	control_action.value = control_msg.data + 50.;//transforms.transform.rotation.x;//data;
    }
ROS_INFO_THROTTLE(1.0, "[ pid_actuator ]  Control action heard: %f", control_msg.data);
}

void abortCB(const std_msgs::Empty& abort_msg){
	emergency_state = true;
}

void enableCB(const std_msgs::Bool enable_msg){
	if(enable_msg.data == false)
  {
    enable_state = false;
  }
  else enable_state = true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "pid_actuator");

  ros::NodeHandle node;
  message_received = false;

  ros::NodeHandle node_priv("~");
  node_priv.param<std::string>("topic_from_controller", topic_from_controller_, "control_action");
  node_priv.param<std::string>("topic_to_actuator", topic_to_actuator_, "uavcan_lcg_command");
  node_priv.param<std::string>("pid_enable_topic", pid_enable_topic_, "pid_enable");
  node_priv.param<std::string>("abort_topic", abort_topic_, "pid_enable");
  node_priv.param<double>("limit_between_setpoints", limit, 5);
  node_priv.param<double>("loop_freq", freq, 50);

  //initiate subscribers
  ros::Subscriber pid_action_sub = node.subscribe(topic_from_controller_, 1, PIDCallback);
  ros::Subscriber abort_sub = node.subscribe(abort_topic_, 10, abortCB);
  ros::Subscriber enable_sub = node.subscribe(pid_enable_topic_, 10, enableCB);

  //initiate publishers
  ros::Publisher control_action_pub = node.advertise<sam_msgs::PercentStamped>(topic_to_actuator_, freq);
  emergency_state = false;
  enable_state = true;

  ros::Rate rate(freq);

  while (node.ok()){

      if (message_received && !emergency_state && enable_state)
      {  
    	control_action_pub.publish(control_action);
    	prev_control_msg = control_action.value;
    	ROS_INFO_THROTTLE(1.0, "[ pid_actuator ]  Control forwarded: %f", control_action.value);
      }

    rate.sleep();
    ros::spinOnce();

  }
  return 0;
};
