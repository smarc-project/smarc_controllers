//Node to forward PID control action to UAVCAN by publishing to sam_msgs.
//Sriharsha Bhat, 05.12.2018

#include <ros/ros.h>
#include <std_msgs/Float64.h>
//#include <sam_msgs/PercentStamped.h>
#include <sam_msgs/BallastAngles.h>
#include <std_msgs/Bool.h>

sam_msgs::BallastAngles control_action;
//std_msgs::Float64 control_action;
double prev_control_msg1,prev_control_msg2,limit, freq;
std::string topic_from_controller_1_,topic_from_controller_2_, topic_to_actuator_, pid_enable_topic_;
bool message_received, enable_state;

void PIDCallback1(const std_msgs::Float64& control_msg)
{
  if(fabs(prev_control_msg1-control_msg.data) > limit) {
	control_action.weight_1_offset_radians = control_msg.data;//transforms.transform.rotation.x;//data; // We will use +-2Pi radians, so the +-50offset is not necessary
    	message_received = true;
	  //control_action.data = control_msg.data + 50.;//transforms.transform.rotation.x;//data; //Gazebo
    }
    ROS_INFO_THROTTLE(1.0, "[ pid_actuator ]  Control action heard: %f", control_msg.data);
}

void PIDCallback2(const std_msgs::Float64& control_msg)
{
   if(fabs(prev_control_msg2-control_msg.data) > limit) {
    	control_action.weight_2_offset_radians = control_msg.data;//transforms.transform.rotation.x;//data;
      	message_received = true;
	//control_action.data = control_msg.data + 50.;//transforms.transform.rotation.x;//data; //Gazebo
    }
ROS_INFO_THROTTLE(1.0, "[ pid_actuator ]  Control action heard: %f", control_msg.data);
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
  node_priv.param<std::string>("topic_from_controller_1", topic_from_controller_1_, "control_action_weight1");
  node_priv.param<std::string>("topic_from_controller_2", topic_from_controller_2_, "control_action_weight2");
  node_priv.param<std::string>("topic_to_actuator", topic_to_actuator_, "uavcan_tcg_command");
  node_priv.param<std::string>("pid_enable_topic", pid_enable_topic_, "pid_enable");
  node_priv.param<double>("limit_between_setpoints", limit, 5);
  node_priv.param<double>("loop_freq", freq, 10);

  //initiate subscribers
  ros::Subscriber pid_action_sub_w1 = node.subscribe(topic_from_controller_1_, 1, PIDCallback1);
  ros::Subscriber pid_action_sub_w2 = node.subscribe(topic_from_controller_2_, 1, PIDCallback2);
  ros::Subscriber enable_sub = node.subscribe(pid_enable_topic_, 10, enableCB);

  //initiate publishers
  ros::Publisher control_action_pub = node.advertise<sam_msgs::BallastAngles>(topic_to_actuator_, freq);
  //ros::Publisher control_action_pub = node.advertise<std_msgs::Float64>(topic_to_actuator_, 10);
  enable_state = true;

  ros::Rate rate(freq);

  while (node.ok()){

	if (message_received && enable_state) {  
    		control_action_pub.publish(control_action);
    		prev_control_msg1 = control_action.weight_1_offset_radians;
    		prev_control_msg2 = control_action.weight_2_offset_radians;
    		//prev_control_msg = control_action.data;
    		ROS_INFO_THROTTLE(1.0, "[ pid_actuator ]  Control forwarded: W1:%f W2:%f", control_action.weight_1_offset_radians,control_action.weight_2_offset_radians);
  		//  ROS_INFO_THROTTLE(1.0, "[ pid_actuator ]  Control forwarded: %f", control_action.data); //Gazebo
 	}
    rate.sleep();
    ros::spinOnce();

  }
  return 0;
};
