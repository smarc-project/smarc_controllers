//Node to forward PID control action to Gazebo by publishing thrust vector values to the joint position controller.
//Sriharsha Bhat, 01.02.2019

#include <ros/ros.h>
#include <std_msgs/Float64.h>

std_msgs::Float64 control_action; // Gazebo
double prev_control_msg,limit, freq;
std::string topic_from_controller_, topic_to_actuator_;


void PIDCallback(const std_msgs::Float64& control_msg)
{
  if(abs(prev_control_msg-control_msg.data) > limit) {
    control_action.data = control_msg.data;// + 50.;//transforms.transform.rotation.x;//data; //Gazebo
  }
ROS_INFO_THROTTLE(1.0, "[ pid_gazebo ]  Control action heard: %f", control_msg.data);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "pid_gazebo");

  ros::NodeHandle node;

  ros::NodeHandle node_priv("~");
  node_priv.param<std::string>("topic_from_controller", topic_from_controller_, "control_action");
  node_priv.param<std::string>("topic_to_actuator", topic_to_actuator_, "actuator_command");
  node_priv.param<double>("limit_between_setpoints", limit, 1);
  node_priv.param<double>("loop_freq", freq, 10);
  //initiate subscribers
  ros::Subscriber pid_action_sub = node.subscribe(topic_from_controller_, 1, PIDCallback);

  //initiate publishers
  ros::Publisher control_action_pub = node.advertise<std_msgs::Float64>(topic_to_actuator_, 10);

  ros::Rate rate(10.0);

  while (node.ok()){

    control_action_pub.publish(control_action);
    prev_control_msg = control_action.data;
    ROS_INFO_THROTTLE(1.0, "[ pid_gazebo ]  Control forwarded: %f", control_action.data); //Gazebo

    rate.sleep();
    ros::spinOnce();

  }
  return 0;
};
