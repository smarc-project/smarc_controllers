//PID Transform Listener Node for SAM, Sriharsha Bhat,  7 Dec 2018
#include <ros/ros.h>
#include <tf/transform_listener.h>
//#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Float64.h>
//#include <tf2_msgs/TFMessage.h>
//#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>



int main(int argc, char** argv){
  
  std::string node_name = "pid_tf_listener";
  ros::init(argc, argv, node_name);

  ros::NodeHandle node;

  std::string base_frame;
  std::string odom_frame;
  std::string world_frame;

  node.param<std::string>(node_name + "/base_frame", base_frame, "base_link");
  node.param<std::string>(node_name + "/world_frame", world_frame, "world");
  node.param<std::string>(node_name + "/odom_frame", odom_frame, "odom");

//initiate publishers
  ros::Publisher feedback_pitch = node.advertise<std_msgs::Float64>("pitch_feedback", 10);
  ros::Publisher feedback_roll = node.advertise<std_msgs::Float64>("roll_feedback", 10);
  ros::Publisher feedback_yaw = node.advertise<std_msgs::Float64>("yaw_feedback", 10);
  ros::Publisher feedback_depth = node.advertise<std_msgs::Float64>("depth_feedback", 10);

//Variable initialization
  tf::TransformListener listener;
  std_msgs::Float64 current_roll,current_pitch,current_yaw,current_depth;
  double r,p,y;
  tf::Quaternion tfq;

  //Define the transforms to make this modular : can be done once you have a working system
  //node_priv.param<std::string>("topic_from_controller", topic_from_controller_, "control_effort");


  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{//transform between world and required frame
      listener.lookupTransform(world_frame, base_frame,
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    //get orientation with quaternions
    tfq = transform.getRotation();
    tf::Matrix3x3(tfq).getEulerYPR(y,p,r);

    current_pitch.data= p;
    current_roll.data= r;
    current_yaw.data= y;
    current_depth.data= -transform.getOrigin().z();

    feedback_pitch.publish(current_pitch);
    feedback_roll.publish(current_roll);
    feedback_yaw.publish(current_yaw);
    feedback_depth.publish(current_depth);

    ROS_INFO_THROTTLE(1.0, "[ pid_tf_listener ] roll: %f, pitch: %f, yaw: %f,, depth: %f ", current_roll.data,current_pitch.data,current_yaw.data,current_depth.data);

    rate.sleep();
  }
  return 0;
};
