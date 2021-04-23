// Initialization for a controller manager, enabling and disabling controllers based on services.
// Sriharsha Bhat 23-4-2021


#ifndef PID_MANAGER_H
#define PID_MANAGER_H

// ROS includes.
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <smarc_msgs/ControllerStatus.h>
#include <std_srvs/SetBool.h>

// TODO: this seems quite excessive to me, 10 sounds more reasonable
// in the end, this should definitely be a parameter
// #define MASK_SIZE 100

namespace pid_manager_cpp
{
    class PIDManager
    {
    public:
        //! Constructor.
        PIDManager(ros::NodeHandle& nodehandle);

    private:
        ros::NodeHandle nh_;

        //Subscribers
        // none so far consider redistributing topics based on Nils new nomenclature!
 
        // Publishers to enable controllers and print status
        ros::Publisher vbs_enable_pub_;
        ros::Publisher vbs_status_pub_;

        ros::Publisher lcg_enable_pub_;
        ros::Publisher lcg_status_pub_;

        ros::Publisher tcg_enable_pub_;
        ros::Publisher tcg_status_pub_;

        ros::Publisher vbs_alt_enable_pub_;
        ros::Publisher vbs_alt_status_pub_;
        
        ros::Publisher dheading_enable_pub_;
        ros::Publisher dheading_status_pub_;
        
        ros::Publisher ddepth_enable_pub_;
        ros::Publisher ddepth_status_pub_;
        
        ros::Publisher dalt_enable_pub_;
        ros::Publisher dalt_status_pub_;
        
        ros::Publisher dvel_enable_pub_;
        ros::Publisher dvel_status_pub_;

        ros::Publisher droll_enable_pub_;
        ros::Publisher droll_status_pub_;

        ros::Publisher yaw_setpoint_pub_;
        ros::Publisher depth_setpoint_pub_;
        ros::Publisher altitude_setpoint_pub_;
        ros::Publisher speed_setpoint_pub_;
        ros::Publisher pitch_setpoint_pub_; 
        ros::Publisher roll_setpoint_pub_;  

        //Subscribers
        ros::Subscriber yaw_setpoint_sub_;      
        ros::Subscriber depth_setpoint_sub_;
        ros::Subscriber altitude_setpoint_sub_;
        ros::Subscriber speed_setpoint_sub_;
        ros::Subscriber pitch_setpoint_sub_;
        ros::Subscriber roll_setpoint_sub_;

        // Services to toggle controllers
        ros::ServiceServer vbs_ctrl_srv_;
        ros::ServiceServer lcg_ctrl_srv_;
        ros::ServiceServer tcg_ctrl_srv_;
        ros::ServiceServer vbs_alt_ctrl_srv_;        
        ros::ServiceServer dheading_ctrl_srv_;
        ros::ServiceServer ddepth_ctrl_srv_;
        ros::ServiceServer dalt_ctrl_srv_;
        ros::ServiceServer dvel_ctrl_srv_;
        ros::ServiceServer droll_ctrl_srv_;
        
        void initialize_subscribers();
        void initialize_publishers();
        void initialize_services();

        // Callback functions for services.
        bool vbs_ctrl_srv_cb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);
        bool lcg_ctrl_srv_cb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);
        bool tcg_ctrl_srv_cb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);
        bool vbs_alt_ctrl_srv_cb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);
        bool dheading_ctrl_srv_cb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);
        bool ddepth_ctrl_srv_cb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);
        bool dalt_ctrl_srv_cb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);
        bool dvel_ctrl_srv_cb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);
        bool droll_ctrl_srv_cb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);

        // Callback functions for subscribers.
        void yaw_setpoint_cb(const std_msgs::Float64& setpoint);
        void depth_setpoint_cb(const std_msgs::Float64& setpoint);
        void altitude_setpoint_cb(const std_msgs::Float64& setpoint);
        void speed_setpoint_cb(const std_msgs::Float64& setpoint);
        void pitch_setpoint_cb(const std_msgs::Float64& setpoint);
        void roll_setpoint_cb(const std_msgs::Float64& setpoint);

        std_msgs::Bool vbs_enable_msg, lcg_enable_msg, tcg_enable_msg, vbs_alt_enable_msg, dheading_enable_msg, ddepth_enable_msg, dalt_enable_msg, dvel_enable_msg, droll_enable_msg;
        smarc_msgs::ControllerStatus vbs_status_msg, lcg_status_msg, tcg_status_msg, vbs_alt_status_msg, dheading_status_msg, ddepth_status_msg, dalt_status_msg, dvel_status_msg, droll_status_msg;
        std_msgs::Float64 yaw_setpoint, depth_setpoint, altitude_setpoint, speed_setpoint, pitch_setpoint, roll_setpoint;
        
        // Parameters
        std::string vbs_enable_topic_;
        std::string vbs_status_topic_;
        std::string vbs_ctrl_srv_name_;

        std::string lcg_enable_topic_;
        std::string lcg_status_topic_;
        std::string lcg_ctrl_srv_name_;

        std::string tcg_enable_topic_;
        std::string tcg_status_topic_;
        std::string tcg_ctrl_srv_name_;

        std::string vbs_alt_enable_topic_;
        std::string vbs_alt_status_topic_;
        std::string vbs_alt_ctrl_srv_name_;

        std::string dheading_enable_topic_;
        std::string dheading_status_topic_;
        std::string dheading_ctrl_srv_name_;

        std::string ddepth_enable_topic_;
        std::string ddepth_status_topic_;
        std::string ddepth_ctrl_srv_name_;

        std::string dalt_enable_topic_;
        std::string dalt_status_topic_;
        std::string dalt_ctrl_srv_name_;

        std::string dvel_enable_topic_;
        std::string dvel_status_topic_;
        std::string dvel_ctrl_srv_name_;

        std::string droll_enable_topic_;
        std::string droll_status_topic_;
        std::string droll_ctrl_srv_name_;

        bool republish_setpoint_;

        std::string yaw_setpoint_topic_;
        std::string depth_setpoint_topic_;
        std::string altitude_setpoint_topic_;
        std::string speed_setpoint_topic_;
        std::string pitch_setpoint_topic_;
        std::string roll_setpoint_topic_;

        std::string yaw_setpoint_topic_repub_;
        std::string depth_setpoint_topic_repub_;
        std::string altitude_setpoint_topic_repub_;
        std::string speed_setpoint_topic_repub_;
        std::string pitch_setpoint_topic_repub_;
        std::string roll_setpoint_topic_repub_;

    };

} // namespace pid_manager_cpp

#endif // NODE_EXAMPLE_LISTENER_H
