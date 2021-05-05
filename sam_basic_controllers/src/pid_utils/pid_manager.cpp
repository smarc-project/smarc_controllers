// PID controller manager, enabling and disabling controllers based on services.
// Sriharsha Bhat 23-4-2021

// TODO: Consider topic republishing. Basic version implemented now. 
// TODO: COntroller throttle service to republish setpoints at specific frequencies. Needed?


#include <sam_basic_controllers/pid_manager.h>
//#include <ros/ros.h>
//#include <std_msgs/Bool.h>
//#include <smarc_msgs/ControllerStatus.h>
//#include <std_srvs/SetBool.h>

namespace pid_manager_cpp
{
    //Generic controller toggle service constructor
    PIDToggleService::PIDToggleService(ros::NodeHandle& nodehandle, std::string service_name, std::string enable_topic, std::string status_topic) : nh_(nodehandle)
    {
        //initialize variables
        ctrl_srv_name_ = service_name;
        enable_topic_ = enable_topic;
        status_topic_ = status_topic;
        
        enable_msg.data = false;
        status_msg.control_status = 0;
        status_msg.service_name = ctrl_srv_name_;     

        //initialize service

        ctrl_srv_ = nh_.advertiseService(ctrl_srv_name_,
                                         &PIDToggleService::ctrl_srv_cb,
                                         this); // testing modular service
        
        //initialize publishers
        enable_pub_ = nh_.advertise<std_msgs::Bool>(enable_topic_, 10);
        status_pub_ = nh_.advertise<smarc_msgs::ControllerStatus>(status_topic_, 1);
    
      } 

    //Generic controller toggle service callback
    bool PIDToggleService::ctrl_srv_cb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response)
    {
        ROS_INFO("Toggle PID service activated");
        if (request.data)
        {
            try
            {
                //enable  controller
                enable_msg.data= true;
                status_msg.control_status = 1;
                response.success = true;
                response.message = std::string("Connected");
            }
            catch (std::exception &e)
            {
                ROS_INFO("Caught exception: %s", e.what());
                response.success = false;
                response.message = std::string("Couldn't connect");
            }
        }
        else
        {
            try
            {
                enable_msg.data= false;
                status_msg.control_status = 0;
                response.success = true;
                response.message = std::string("Disconnected");
            }
            catch (std::exception &e)
            {
                ROS_INFO("Caught exception: %s", e.what());
                response.success = false;
                response.message = std::string("Couldn't disconnect");
            }
        }

        return true;
    }

    //Setpoint republisher constructor
    PIDSetpointRepub::PIDSetpointRepub(ros::NodeHandle& nodehandle,std::string setpoint_topic_, std::string setpoint_topic_repub_): nh_(nodehandle)
    {
        //Initialize variables
       // setpoint_topic_= setpoint_topic;
       // setpoint_topic_repub_ = setpoint_repub_topic;

        //Subscriber
        setpoint_sub_= nh_.subscribe(setpoint_topic_, 1, &PIDSetpointRepub::setpoint_cb, this);

        //Publisher
        setpoint_pub_ = nh_.advertise<std_msgs::Float64>(setpoint_topic_repub_, 10);

    }

    //Generic setpoint republishing callback
    void PIDSetpointRepub::setpoint_cb(const std_msgs::Float64& setpoint_)
    {
        setpoint.data = setpoint_.data;
        ROS_INFO_THROTTLE(1.0, "[ pid_manager ]  Republishing setpoint");
    } 
      

    PIDManager::PIDManager(ros::NodeHandle& nodehandle) : nh_(nodehandle)
    {
        // Initialize node parameters from launch file or command line. Use a private node handle so that multiple instances
        // of the node can be run simultaneously while using different parameters.
        ros::NodeHandle pnh("~");
        pnh.param<std::string>("vbs_ctrl_srv_name", vbs_ctrl_srv_name_, "ctrl/toggle_vbs_ctrl");
        pnh.param<std::string>("vbs_enable_topic", vbs_enable_topic_, "sam/ctrl/vbs/pid_enable");
        pnh.param<std::string>("vbs_status_topic", vbs_status_topic_, "sam/ctrl/vbs_controller_status");

        pnh.param<std::string>("lcg_ctrl_srv_name", lcg_ctrl_srv_name_, "ctrl/toggle_pitch_ctrl");
        pnh.param<std::string>("lcg_enable_topic", lcg_enable_topic_, "sam/ctrl/lcg/pid_enable");
        pnh.param<std::string>("lcg_status_topic", lcg_status_topic_, "sam/ctrl/pitch_controller_status");

        pnh.param<std::string>("tcg_ctrl_srv_name", tcg_ctrl_srv_name_, "ctrl/toggle_tcg_ctrl");
        pnh.param<std::string>("tcg_enable_topic", tcg_enable_topic_, "sam/ctrl/tcg/pid_enable");
        pnh.param<std::string>("tcg_status_topic", tcg_status_topic_, "sam/ctrl/tcg_controller_status");

        pnh.param<std::string>("vbs_alt_ctrl_srv_name", vbs_alt_ctrl_srv_name_, "ctrl/toggle_vbs_alt_ctrl");
        pnh.param<std::string>("vbs_alt_enable_topic", vbs_alt_enable_topic_, "sam/ctrl/vbs_alt/pid_enable");
        pnh.param<std::string>("vbs_alt_status_topic", vbs_alt_status_topic_, "sam/ctrl/vbs_alt_controller_status");

        pnh.param<std::string>("dheading_ctrl_srv_name", dheading_ctrl_srv_name_, "ctrl/toggle_yaw_ctrl");
        pnh.param<std::string>("dheading_enable_topic", dheading_enable_topic_, "sam/ctrl/dynamic_heading/pid_enable");
        pnh.param<std::string>("dheading_status_topic", dheading_status_topic_, "sam/ctrl/yaw_controller_status");

        pnh.param<std::string>("ddepth_ctrl_srv_name", ddepth_ctrl_srv_name_, "ctrl/toggle_depth_ctrl");
        pnh.param<std::string>("ddepth_enable_topic", ddepth_enable_topic_, "sam/ctrl/dynamic_depth/pid_enable");
        pnh.param<std::string>("ddepth_status_topic", ddepth_status_topic_, "sam/ctrl/depth_controller_status");

        pnh.param<std::string>("dalt_ctrl_srv_name", dalt_ctrl_srv_name_, "ctrl/toggle_altitude_ctrl");
        pnh.param<std::string>("dalt_enable_topic", dalt_enable_topic_, "sam/ctrl/dynamic_alt/pid_enable");
        pnh.param<std::string>("dalt_status_topic", dalt_status_topic_, "sam/ctrl/altitude_controller_status");

        pnh.param<std::string>("dvel_ctrl_srv_name", dvel_ctrl_srv_name_, "ctrl/toggle_speed_ctrl");
        pnh.param<std::string>("dvel_enable_topic", dvel_enable_topic_, "sam/ctrl/dynamic_velocity/pid_enable");
        pnh.param<std::string>("dvel_status_topic", dvel_status_topic_, "sam/ctrl/speed_controller_status");

        pnh.param<std::string>("droll_ctrl_srv_name", droll_ctrl_srv_name_, "ctrl/toggle_roll_ctrl");
        pnh.param<std::string>("droll_enable_topic", droll_enable_topic_, "sam/ctrl/dynamic_roll/pid_enable");
        pnh.param<std::string>("droll_status_topic", droll_status_topic_, "sam/ctrl/roll_controller_status");
       
        pnh.param<bool>("republish_setpoint", republish_setpoint_, false);

        //Source topics
        pnh.param<std::string>("yaw_setpoint_topic", yaw_setpoint_topic_, "sam/ctrl/yaw_setpoint");
        pnh.param<std::string>("depth_setpoint_topic", depth_setpoint_topic_, "sam/ctrl/depth_setpoint");
        pnh.param<std::string>("altitude_setpoint_topic", altitude_setpoint_topic_, "sam/ctrl/altitude_setpoint");
        pnh.param<std::string>("speed_setpoint_topic", speed_setpoint_topic_, "sam/ctrl/speed_setpoint");        
        pnh.param<std::string>("pitch_setpoint_topic", pitch_setpoint_topic_, "sam/ctrl/pitch_setpoint");
        pnh.param<std::string>("roll_setpoint_topic", roll_setpoint_topic_, "sam/ctrl/roll_setpoint");  

        //Republish setpoint topics to controllers as
        pnh.param<std::string>("yaw_setpoint_topic_repub", yaw_setpoint_topic_repub_, "sam/ctrl/dynamic_heading/setpoint");
        pnh.param<std::string>("depth_setpoint_topic_repub", depth_setpoint_topic_repub_, "sam/ctrl/dynamic_depth/setpoint");
        pnh.param<std::string>("altitude_setpoint_topic_repub", altitude_setpoint_topic_repub_, "sam/ctrl/dynamic_alt/setpoint");
        pnh.param<std::string>("speed_setpoint_topic_repub", speed_setpoint_topic_repub_, "sam/ctrl/dynamic_velocity/setpoint");        
        pnh.param<std::string>("pitch_setpoint_topic_repub", pitch_setpoint_topic_repub_, "sam/ctrl/lcg/setpoint");
        pnh.param<std::string>("roll_setpoint_topic_repub", roll_setpoint_topic_repub_, "sam/ctrl/dynamic_roll/setpoint");        


        // create controller services
        PIDToggleService vbs(nh_,vbs_ctrl_srv_name_,vbs_enable_topic_,vbs_status_topic_);
        PIDToggleService lcg(nh_,lcg_ctrl_srv_name_,lcg_enable_topic_,lcg_status_topic_);
        PIDToggleService tcg(nh_,tcg_ctrl_srv_name_,tcg_enable_topic_,tcg_status_topic_);
        PIDToggleService vbs_alt(nh_,vbs_alt_ctrl_srv_name_,vbs_alt_enable_topic_,vbs_alt_status_topic_);
        PIDToggleService dheading(nh_,dheading_ctrl_srv_name_, dheading_enable_topic_,dheading_status_topic_);
        PIDToggleService ddepth(nh_,ddepth_ctrl_srv_name_, ddepth_enable_topic_,ddepth_status_topic_);
        PIDToggleService dalt(nh_,dalt_ctrl_srv_name_, dalt_enable_topic_,dalt_status_topic_);
        PIDToggleService dvel(nh_,dvel_ctrl_srv_name_, dvel_enable_topic_,dvel_status_topic_);
        PIDToggleService droll(nh_,droll_ctrl_srv_name_, droll_enable_topic_,droll_status_topic_);

        //Assign republishing of setpoints
        PIDSetpointRepub yaw(nh_,yaw_setpoint_topic_,yaw_setpoint_topic_repub_);
        PIDSetpointRepub depth(nh_,depth_setpoint_topic_,depth_setpoint_topic_repub_);
        PIDSetpointRepub altitude(nh_,altitude_setpoint_topic_,altitude_setpoint_topic_repub_);
        PIDSetpointRepub speed(nh_,speed_setpoint_topic_,speed_setpoint_topic_repub_);
        PIDSetpointRepub pitch(nh_,pitch_setpoint_topic_,pitch_setpoint_topic_repub_);
        PIDSetpointRepub roll(nh_,roll_setpoint_topic_,roll_setpoint_topic_repub_);
        

        ros::Rate idle_rate(10);
        while (ros::ok())
        {

            //Publish enable messages            
            vbs.enable_pub_.publish(vbs.enable_msg);
            lcg.enable_pub_.publish(lcg.enable_msg);
            tcg.enable_pub_.publish(tcg.enable_msg);
            vbs_alt.enable_pub_.publish(vbs_alt.enable_msg);
            dheading.enable_pub_.publish(dheading.enable_msg);
            ddepth.enable_pub_.publish(ddepth.enable_msg);
            dalt.enable_pub_.publish(dalt.enable_msg);
            dvel.enable_pub_.publish(dvel.enable_msg);
            droll.enable_pub_.publish(droll.enable_msg);


            //Publish status messages
            vbs.status_pub_.publish(vbs.status_msg);
            lcg.status_pub_.publish(lcg.status_msg);
            tcg.status_pub_.publish(tcg.status_msg);
            vbs_alt.status_pub_.publish(vbs_alt.status_msg);
            dheading.status_pub_.publish(dheading.status_msg);
            ddepth.status_pub_.publish(ddepth.status_msg);
            dalt.status_pub_.publish(dalt.status_msg);
            dvel.status_pub_.publish(dvel.status_msg);
            droll.status_pub_.publish(droll.status_msg);

            //TODO: Republish setpoints to specific controllers- currently only dynamic, later add a logic.
            if(republish_setpoint_)
            {
                //republishing controller setpoints
                yaw.setpoint_pub_.publish(yaw.setpoint);
                depth.setpoint_pub_.publish(depth.setpoint);
                altitude.setpoint_pub_.publish(altitude.setpoint);
                speed.setpoint_pub_.publish(speed.setpoint);  
                pitch.setpoint_pub_.publish(pitch.setpoint);                
                roll.setpoint_pub_.publish(roll.setpoint);           
            }
            
            idle_rate.sleep();
            
            ros::spinOnce();
            //ros::spin();
        }
    }

} // namespace pid_manager_cpp

int main(int argc, char **argv)
{
    // ROS set-ups:
    ros::init(argc, argv, "pid_manager"); //node name
    
    // create a node handle; need to pass this to the class constructor
    ros::NodeHandle nh;

    pid_manager_cpp::PIDManager PIDManager(nh);

    ros::spin();

    return 0;
}
