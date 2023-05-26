#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sam_msgs/PercentStamped.h>
//#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

class PIDTrim{

  public:
    
    std::string topic_from_controller_, topic_to_actuator_, topic_from_plant_, setpoint_req_, setpoint_res_;
    ros::NodeHandle *nh_priv_;
    ros::NodeHandle *nh_;
    ros::Subscriber pid_action_sub, plant_sub, sp_sub, state_sub;
    ros::Publisher control_action_pub, setpoint_pub;

    double setpoint_, error_t_, setpoint_tolerance_, neutral_setpoint_;
    bool setpoint_rcv_;

    PIDTrim(ros::NodeHandle &nh_priv, ros::NodeHandle &nh) : nh_priv_(&nh_priv), nh_(&nh)
    {
      nh_priv_->param<std::string>("topic_from_controller", topic_from_controller_, "control_action");
      nh_priv_->param<std::string>("topic_to_actuator", topic_to_actuator_, "uavcan_lcg_command");
      nh_priv_->param<std::string>("topic_from_plant", topic_from_plant_, "uavcan_lcg_command");
      nh_priv_->param<std::string>("setpoint_req", setpoint_req_, "uavcan_lcg_command");
      nh_priv_->param<std::string>("setpoint_res", setpoint_res_, "uavcan_lcg_command");
      nh_priv_->param<double>("setpoint_tolerance", setpoint_tolerance_, 0.1);
      nh_priv_->param<double>("neutral_point", neutral_setpoint_, 0.1);

      setpoint_rcv_ = false;

      // initiate subscribers
      pid_action_sub = nh_->subscribe(topic_from_controller_, 1, &PIDTrim::PIDCallback, this);
      plant_sub = nh_->subscribe(topic_from_plant_, 1, &PIDTrim::PIDCallback, this);
      sp_sub = nh_->subscribe(setpoint_req_, 1, &PIDTrim::SetpointCallback, this);
      state_sub = nh_->subscribe(topic_from_plant_, 1, &PIDTrim::PlantCallback, this);

      // initiate publishers
      control_action_pub = nh_->advertise<sam_msgs::PercentStamped>(topic_to_actuator_, 10);
      setpoint_pub = nh_->advertise<std_msgs::Float64>(setpoint_res_, 10);
    }

    // Gets setpoint from GUI
    void SetpointCallback(const std_msgs::Float64 & setpoint)
    {
      setpoint_ = setpoint.data;
      setpoint_rcv_ = true;
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
        }
      } 
    }

    // This one just republishes adding the feedforward term
    void PIDCallback(const std_msgs::Float64 &control_msg)
    {
      sam_msgs::PercentStamped control_action;
      control_action.value = control_msg.data + 50.;
      control_action_pub.publish(control_action);
    }
};


int main(int argc, char** argv){
  ros::init(argc, argv, "pid_actuator");

  ros::NodeHandle node_priv("~");
  ros::NodeHandle node;

  PIDTrim* pid_obj = new PIDTrim(node_priv, node);

  ros::spin();

  ros::waitForShutdown();

  if (!ros::ok())
  {
      delete pid_obj;
  }
  ROS_INFO("PID trim finished");

  return 0;
}
