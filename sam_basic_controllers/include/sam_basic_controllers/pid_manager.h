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
    
    class PIDToggleService
    {
    public: 
        // Constructor
        PIDToggleService(ros::NodeHandle& nodehandle, std::string service_name, std::string enable_topic, std::string status_topic);

    //private:
        ros::NodeHandle nh_;
        std::string enable_topic_;
        std::string status_topic_;
        std::string ctrl_srv_name_;
        
        //publishers
        ros::Publisher enable_pub_;
        ros::Publisher status_pub_;

        //service
        ros::ServiceServer ctrl_srv_;

        // callbacks
        bool ctrl_srv_cb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);


        std_msgs::Bool enable_msg;
        smarc_msgs::ControllerStatus status_msg;


    };

    class PIDSetpointRepub
    {
    public: 
        //constructor
        PIDSetpointRepub(ros::NodeHandle& nodehandle,std::string setpoint_topic, std::string setpoint_repub_topic);

        //variables
        ros::NodeHandle nh_;
        std_msgs::Float64 setpoint;
	bool setpoint_available;
        //std::string setpoint_topic_;
        //std::string setpoint_topic_repub_;

        //subscriber
        ros::Subscriber setpoint_sub_;      

        //callback
        void setpoint_cb(const std_msgs::Float64& setpoint);

        //publisher
        ros::Publisher setpoint_pub_;

    };
    
    
    
    class PIDManager
    {
    public:
        //! Constructor.
        PIDManager(ros::NodeHandle& nodehandle);

    private:
        ros::NodeHandle nh_;

        
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

        std::string dpitch_enable_topic_;
        std::string dpitch_status_topic_;
        std::string dpitch_ctrl_srv_name_;

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
        //Additional setpoints to vbs,tcg for overactuation in depth, altitude and roll
        std::string vbs_depth_setpoint_topic_repub_;
        std::string vbs_altitude_setpoint_topic_repub_;
        std::string tcg_roll_setpoint_topic_repub_;
        std::string dpitch_setpoint_topic_repub_;
    };

} // namespace pid_manager_cpp

#endif // NODE_EXAMPLE_LISTENER_H
