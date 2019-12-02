//Node to forward propeller PID control action to UAVCAN by publishing to sam_msgs.
//Sriharsha Bhat, 13.6.2019

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sam_msgs/ThrusterRPMs.h>

sam_msgs::ThrusterRPMs control_action;
//std_msgs::Float64 control_action;
double prev_control_msg1,prev_control_msg2,limit,freq;
std::string topic_from_controller_1_,topic_from_controller_2_, topic_to_actuator_;


void PIDCallback1(const std_msgs::Float64& control_msg)
{
  if(abs(prev_control_msg1-control_msg.data) > limit) {
    control_action.thruster_1_rpm = control_msg.data;
    }
    ROS_INFO_THROTTLE(1.0, "[ pid_actuator ]  Control action heard: %f", control_msg.data);
}

void PIDCallback2(const std_msgs::Float64& control_msg)
{
   if(abs(prev_control_msg2-control_msg.data) > limit) {
    control_action.thruster_2_rpm = control_msg.data;
    }
ROS_INFO_THROTTLE(1.0, "[ pid_actuator ]  Control action heard: %f", control_msg.data);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "pid_actuator_prop");

  ros::NodeHandle node;

  ros::NodeHandle node_priv("~");
  node_priv.param<std::string>("topic_from_controller_1", topic_from_controller_1_, "control_action_prop1");
  node_priv.param<std::string>("topic_from_controller_2", topic_from_controller_2_, "control_action_prop2");

  node_priv.param<std::string>("topic_to_actuator", topic_to_actuator_, "uavcan_prop_command");
  node_priv.param<double>("limit_between_setpoints", limit, 5);
  node_priv.param<double>("loop_freq", freq, 50);
  //initiate subscribers
  ros::Subscriber pid_action_sub_prop1 = node.subscribe(topic_from_controller_1_, 1, PIDCallback1);
  ros::Subscriber pid_action_sub_prop2 = node.subscribe(topic_from_controller_2_, 1, PIDCallback2);

  //initiate publishers
  ros::Publisher control_action_pub = node.advertise<sam_msgs::ThrusterRPMs>(topic_to_actuator_, freq);
  //ros::Publisher control_action_pub = node.advertise<std_msgs::Float64>(topic_to_actuator_, 10);

  ros::Rate rate(freq);

  while (node.ok()){

    control_action_pub.publish(control_action);
    prev_control_msg1 = control_action.thruster_1_rpm;
    prev_control_msg2 = control_action.thruster_2_rpm;
    //prev_control_msg = control_action.data;
    ROS_INFO_THROTTLE(1.0, "[ pid_actuator ]  Control forwarded: Prop1:%f Prop2:%f", control_action.thruster_1_rpm,control_action.thruster_2_rpm);
  //  ROS_INFO_THROTTLE(1.0, "[ pid_actuator ]  Control forwarded: %f", control_action.data); //Gazebo

    rate.sleep();
    ros::spinOnce();

  }
  return 0;
};
