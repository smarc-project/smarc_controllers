//Node to forward PID control action to UAVCAN by publishing to sam_msgs.
//Sriharsha Bhat, 05.12.2018

#include <ros/ros.h>
#include <std_msgs/Float64.h>
//#include <sam_msgs/PercentStamped.h>
#include <sam_msgs/BallastAngles.h>

sam_msgs::BallastAngles control_action;
//std_msgs::Float64 control_action;
double prev_control_msg1,prev_control_msg2,limit;
std::string topic_from_controller_1_,topic_from_controller_2_, topic_to_actuator_;


void PIDCallback1(const std_msgs::Float64& control_msg)
{
  if(abs(prev_control_msg1-control_msg.data) > limit) {
    control_action.weight_1_offset_radians = control_msg.data;//transforms.transform.rotation.x;//data; // We will use +-2Pi radians, so the +-50offset is not necessary
    //control_action.data = control_msg.data + 50.;//transforms.transform.rotation.x;//data; //Gazebo
    }
    ROS_INFO_THROTTLE(1.0, "[ pid_actuator ]  Control action heard: %f", control_msg.data);
}

void PIDCallback2(const std_msgs::Float64& control_msg)
{
   if(abs(prev_control_msg2-control_msg.data) > limit) {
    control_action.weight_2_offset_radians = control_msg.data;//transforms.transform.rotation.x;//data;
      //control_action.data = control_msg.data + 50.;//transforms.transform.rotation.x;//data; //Gazebo
    }
ROS_INFO_THROTTLE(1.0, "[ pid_actuator ]  Control action heard: %f", control_msg.data);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "pid_actuator");

  ros::NodeHandle node;

  ros::NodeHandle node_priv("~");
  node_priv.param<std::string>("topic_from_controller_1", topic_from_controller_1_, "control_action_weight1");
  node_priv.param<std::string>("topic_from_controller_2", topic_from_controller_2_, "control_action_weight2");

  node_priv.param<std::string>("topic_to_actuator", topic_to_actuator_, "uavcan_tcg_command");
  node_priv.param<double>("limit_between_setpoints", limit, 5);

  //initiate subscribers
  ros::Subscriber pid_action_sub_w1 = node.subscribe(topic_from_controller_1_, 1, PIDCallback1);
  ros::Subscriber pid_action_sub_w2 = node.subscribe(topic_from_controller_2_, 1, PIDCallback2);

  //initiate publishers
  ros::Publisher control_action_pub = node.advertise<sam_msgs::BallastAngles>(topic_to_actuator_, 10);
  //ros::Publisher control_action_pub = node.advertise<std_msgs::Float64>(topic_to_actuator_, 10);

  ros::Rate rate(10.0);

  while (node.ok()){

    control_action_pub.publish(control_action);
    prev_control_msg1 = control_action.weight_1_offset_radians;
    prev_control_msg2 = control_action.weight_2_offset_radians;
    //prev_control_msg = control_action.data;
    ROS_INFO_THROTTLE(1.0, "[ pid_actuator ]  Control forwarded: W1:%f W2:%f", control_action.weight_1_offset_radians,control_action.weight_2_offset_radians);
  //  ROS_INFO_THROTTLE(1.0, "[ pid_actuator ]  Control forwarded: %f", control_action.data); //Gazebo

    rate.sleep();
    ros::spinOnce();

  }
  return 0;
};
