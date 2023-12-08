// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"
// #include <std_msgs/Float64.h>
#include "std_msgs/msg/float64.hpp"
// #include "sam_msgs/PercentStamped.h"
#include "sam_msgs/msg/percent_stamped.hpp"
#include "std_msgs/msg/bool.hpp"

class PIDTrim{

  public:
    std::string topic_from_controller_, topic_to_actuator_, topic_from_plant_, setpoint_req_, setpoint_res_, setpoint_reached_;
    rclcpp::Node *nh_;
    ros::Subscriber pid_action_sub, plant_sub, sp_sub, state_sub;
    ros::Publisher control_action_pub, setpoint_pub, sp_reached_pub_;

    double setpoint_, error_t_, setpoint_tolerance_, neutral_setpoint_, ff_term_;
    bool setpoint_rcv_;
    std_msgs::msg::Bool sp_reached_;

    PIDTrim(rclcpp::Node &nh) : nh_(&nh)
    {
      nh_ ->param<std::string>("topic_from_controller", topic_from_controller_, "control_action");
      nh_ ->param<std::string>("topic_to_actuator", topic_to_actuator_, "uavcan_lcg_command");
      nh_ ->param<std::string>("topic_from_plant", topic_from_plant_, "uavcan_lcg_command");
      nh_ ->param<std::string>("setpoint_req", setpoint_req_, "uavcan_lcg_command");
      nh_ ->param<std::string>("setpoint_res", setpoint_res_, "uavcan_lcg_command");
      nh_ ->param<std::string>("setpoint_reached", setpoint_reached_, "uavcan_lcg_command");
      nh_ ->param<double>("setpoint_tolerance", setpoint_tolerance_, 0.1);
      nh_ ->param<double>("neutral_point", neutral_setpoint_, 0.1);
      nh_ ->param<double>("ff_term", ff_term_, 0.1);

      setpoint_rcv_ = false;

      // initiate subscribers
      pid_action_sub = nh_->subscribe(topic_from_controller_, 1, &PIDTrim::PIDCallback, this);
      sp_sub = nh_->subscribe(setpoint_req_, 1, &PIDTrim::SetpointCallback, this);
      state_sub = nh_->subscribe(topic_from_plant_, 1, &PIDTrim::PlantCallback, this);

      // initiate publishers
      control_action_pub = nh_->advertise<sam_msgs::PercentStamped>(topic_to_actuator_, 10);
      setpoint_pub = nh_->advertise<std_msgs::Float64>(setpoint_res_, 10);
      sp_reached_pub_ = nh_->advertise<std_msgs::Bool>(setpoint_reached_, 10, true);
    }

    // Gets setpoint from GUI
    void SetpointCallback(const std_msgs::Float64 & setpoint)
    {
      setpoint_ = setpoint.data;
      setpoint_rcv_ = true;
      sp_reached_.data = false;
      sp_reached_pub_.publish(sp_reached_);
    }
    
    // Checks current error in the controller: if over tolerance, publish setpoint to PID. Else, stop PID 
    // by not publishing setpoints anymore
    void PlantCallback(const std_msgs::Float64 &plant_state)
    {
      if (setpoint_rcv_){
        if (fabs(setpoint_ - plant_state.data) > setpoint_tolerance_){
          std_msgs::Float64 msg;
          msg.data = setpoint_;
          setpoint_pub.publish(msg);

        }
        else{
          ROS_INFO_NAMED(ros::this_node::getName(), "Setpoint reached %f", plant_state.data);
          setpoint_rcv_ = false;

          // When the setpoint has been reached, set actuator to neutral to maintain position
          sam_msgs::PercentStamped control_action;
          control_action.value = neutral_setpoint_; 
          control_action_pub.publish(control_action);
          
          sp_reached_.data = true;
          sp_reached_pub_.publish(sp_reached_);
        }
      } 
    }

    // This one just republishes adding the feedforward term
    void PIDCallback(const std_msgs::Float64 &control_msg)
    {
      sam_msgs::PercentStamped control_action;
      control_action.value = control_msg.data + ff_term_;
      control_action_pub.publish(control_action);
    }
};


int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pid_actuator");

  PIDTrim* pid_obj = new PIDTrim(*node);

  rclcpp::Rate loop_rate(10);
  while(rclcpp::ok())
  {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  delete pid_obj;
  RCLCPP_INFO(node->get_logger(), "PID trim finished");

  return 0;
}
