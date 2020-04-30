//Node to wrap the angular error so that the smallest error value is provided to the controller.
//Sriharsha Bhat, 19.1.2019

#include <ros/ros.h>
#include <std_msgs/Float64.h>


double setpoint,feedback,error,freq;
std::string setpoint_topic_,feedback_topic_, angle_error_topic_, zero_setpoint_topic_;
bool setpoint_received, feedback_received;

void FeedbackCallback(const std_msgs::Float64& angle_msg)
{
	feedback = angle_msg.data;
	feedback_received = true;
	ROS_INFO_THROTTLE(1.0, "[angle_wrapper]  Feedback: %f", angle_msg.data);
}

void SetpointCallback(const std_msgs::Float64& angle_msg)
{
	setpoint = angle_msg.data;
	setpoint_received = true;
    	ROS_INFO_THROTTLE(1.0, "[angle_wrapper]  Setpoint: %f", angle_msg.data);
}


int main(int argc, char** argv){
  
  ros::init(argc, argv, "angle_wrapper");

  ros::NodeHandle node;
  feedback_received = false;
  setpoint_received = false;

  ros::NodeHandle node_priv("~");
  node_priv.param<std::string>("setpoint_topic", setpoint_topic_, "/sam/ctrl/dynamic_heading/setpoint");
  node_priv.param<std::string>("feedback_topic", feedback_topic_, "/sam/ctrl/yaw_feedback");
  node_priv.param<std::string>("angle_error_topic", angle_error_topic_, "/sam/ctrl/dynamic_heading/angle_error");
  node_priv.param<std::string>("zero_topic", zero_setpoint_topic_, "/sam/ctrl/dynamic_heading/zero_error");
  node_priv.param<double>("loop_freq", freq, 50);
  //initiate subscribers

  ros::Subscriber feedback_sub = node.subscribe(feedback_topic_, 1, FeedbackCallback);
  ros::Subscriber setpoint_sub = node.subscribe(setpoint_topic_, 1, SetpointCallback);

  //initiate publishers
  ros::Publisher angle_error_pub = node.advertise<std_msgs::Float64>(angle_error_topic_, freq);
  ros::Publisher zero_setpoint_pub = node.advertise<std_msgs::Float64>(zero_setpoint_topic_, freq);

  ros::Rate rate(freq);

  while (node.ok()){

        error= -(setpoint-feedback);

        if(error>3.141516)
            	error= -(2*3.141516 - error);

        if(feedback_received){
		angle_error_pub.publish(error);
		ROS_INFO_THROTTLE(1.0, "[angle_wrapper] Angle Error:%f", error);
		}
	if(setpoint_received)
		zero_setpoint_pub.publish(0.0);
						
    rate.sleep();
    ros::spinOnce();

  }
  return 0;
};