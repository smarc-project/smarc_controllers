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
      
        
        //Have controllers disabled by default
        vbs_enable_msg.data = false;
        vbs_status_msg.control_status = 0;
        vbs_status_msg.service_name = vbs_ctrl_srv_name_;

        lcg_enable_msg.data = false;
        lcg_status_msg.control_status = 0;
        lcg_status_msg.service_name = lcg_ctrl_srv_name_;

        tcg_enable_msg.data = false;
        tcg_status_msg.control_status = 0;
        tcg_status_msg.service_name = tcg_ctrl_srv_name_;

        vbs_alt_enable_msg.data = false;
        vbs_alt_status_msg.control_status = 0;
        vbs_alt_status_msg.service_name = vbs_alt_ctrl_srv_name_;

        dheading_enable_msg.data = false;
        dheading_status_msg.control_status = 0;
        dheading_status_msg.service_name = dheading_ctrl_srv_name_;

        ddepth_enable_msg.data = false;
        ddepth_status_msg.control_status = 0;
        ddepth_status_msg.service_name = ddepth_ctrl_srv_name_;

        dalt_enable_msg.data = false;
        dalt_status_msg.control_status = 0;
        dalt_status_msg.service_name = dalt_ctrl_srv_name_;

        dvel_enable_msg.data = false;
        dvel_status_msg.control_status = 0;
        dvel_status_msg.service_name = dvel_ctrl_srv_name_;

        droll_enable_msg.data = false;
        droll_status_msg.control_status = 0;
        droll_status_msg.service_name = droll_ctrl_srv_name_;


        // initialize ros interfaces
        initialize_subscribers();
        initialize_publishers();
        initialize_services();


        ros::Rate idle_rate(10);
        while (ros::ok())
        {
            //Check if controllers are enabled and set controller status

            //VBS
            if(vbs_enable_msg.data)
            {
                vbs_status_msg.control_status = 1;
            }
            else
            {
                vbs_status_msg.control_status = 0;
            }

            //LCG
            if(lcg_enable_msg.data)
            {
                lcg_status_msg.control_status = 1;
            }
            else
            {
                lcg_status_msg.control_status = 0;
            }

            //TCG
            if(tcg_enable_msg.data)
            {
                tcg_status_msg.control_status = 1;
            }
            else
            {
                tcg_status_msg.control_status = 0;
            }

            //VBS Alt
            if(vbs_alt_enable_msg.data)
            {
                vbs_alt_status_msg.control_status = 1;
            }
            else
            {
                vbs_alt_status_msg.control_status = 0;
            }

            //Dynamic heading
            if(dheading_enable_msg.data)
            {
                dheading_status_msg.control_status = 1;
            }
            else
            {
                dheading_status_msg.control_status = 0;
            }

            //Dynamic depth
            if(ddepth_enable_msg.data)
            {
                ddepth_status_msg.control_status = 1;
            }
            else
            {
                ddepth_status_msg.control_status = 0;
            }

            //Dynamic altitude
            if(dalt_enable_msg.data)
            {
                dalt_status_msg.control_status = 1;
            }
            else
            {
                dalt_status_msg.control_status = 0;
            }

            //Dynamic velocity
            if(dvel_enable_msg.data)
            {
                dvel_status_msg.control_status = 1;
            }
            else
            {
                dvel_status_msg.control_status = 0;
            }

            //Dynamic roll
            if(droll_enable_msg.data)
            {
                droll_status_msg.control_status = 1;
            }
            else
            {
                droll_status_msg.control_status = 0;
            }

            

            //Publish enable messages            
            vbs_enable_pub_.publish(vbs_enable_msg);
            lcg_enable_pub_.publish(lcg_enable_msg);
            tcg_enable_pub_.publish(tcg_enable_msg);
            vbs_alt_enable_pub_.publish(vbs_alt_enable_msg);
            dheading_enable_pub_.publish(dheading_enable_msg);
            ddepth_enable_pub_.publish(ddepth_enable_msg);
            dalt_enable_pub_.publish(dalt_enable_msg);
            dvel_enable_pub_.publish(dvel_enable_msg);
            droll_enable_pub_.publish(droll_enable_msg);


            //Publish status messages
            vbs_status_pub_.publish(vbs_status_msg);
            lcg_status_pub_.publish(lcg_status_msg);
            tcg_status_pub_.publish(tcg_status_msg);
            vbs_alt_status_pub_.publish(vbs_alt_status_msg);
            dheading_status_pub_.publish(dheading_status_msg);
            ddepth_status_pub_.publish(ddepth_status_msg);
            dalt_status_pub_.publish(dalt_status_msg);
            dvel_status_pub_.publish(dvel_status_msg);
            droll_status_pub_.publish(droll_status_msg);

            //TODO: Republish setpoints to specific controllers- currently only dynamic, later add a logic.
            if(republish_setpoint_)
            {
                //republishing controller setpoints
                yaw_setpoint_pub_.publish(yaw_setpoint);
                depth_setpoint_pub_.publish(depth_setpoint);
                altitude_setpoint_pub_.publish(altitude_setpoint);
                speed_setpoint_pub_.publish(speed_setpoint);  
                pitch_setpoint_pub_.publish(pitch_setpoint);                
                roll_setpoint_pub_.publish(roll_setpoint);           
            }


            
            idle_rate.sleep();
            
            ros::spinOnce();
            //ros::spin();
        }
    }

    void PIDManager::initialize_subscribers()
    {
        // initialize subscribers here - consider republishing setpoints from Nils' nomenclature
        yaw_setpoint_sub_= nh_.subscribe(yaw_setpoint_topic_, 1, &PIDManager::yaw_setpoint_cb, this);
        depth_setpoint_sub_= nh_.subscribe(depth_setpoint_topic_, 1, &PIDManager::depth_setpoint_cb, this);
        altitude_setpoint_sub_= nh_.subscribe(altitude_setpoint_topic_, 1, &PIDManager::altitude_setpoint_cb, this);
        speed_setpoint_sub_= nh_.subscribe(speed_setpoint_topic_, 1, &PIDManager::speed_setpoint_cb, this);
        pitch_setpoint_sub_= nh_.subscribe(pitch_setpoint_topic_, 1, &PIDManager::pitch_setpoint_cb, this);
        roll_setpoint_sub_= nh_.subscribe(roll_setpoint_topic_, 1, &PIDManager::roll_setpoint_cb, this);        
    }

    void PIDManager::initialize_services()
    {
       vbs_ctrl_srv_ = nh_.advertiseService(vbs_ctrl_srv_name_,
                                           &PIDManager::vbs_ctrl_srv_cb,
                                           this);
       lcg_ctrl_srv_ = nh_.advertiseService(lcg_ctrl_srv_name_,
                                           &PIDManager::lcg_ctrl_srv_cb,
                                           this);
       tcg_ctrl_srv_ = nh_.advertiseService(tcg_ctrl_srv_name_,
                                           &PIDManager::tcg_ctrl_srv_cb,
                                           this);
       vbs_alt_ctrl_srv_ = nh_.advertiseService(vbs_alt_ctrl_srv_name_,
                                           &PIDManager::vbs_alt_ctrl_srv_cb,
                                           this);
       dheading_ctrl_srv_ = nh_.advertiseService(dheading_ctrl_srv_name_,
                                           &PIDManager::dheading_ctrl_srv_cb,
                                           this);
       ddepth_ctrl_srv_ = nh_.advertiseService(ddepth_ctrl_srv_name_,
                                           &PIDManager::ddepth_ctrl_srv_cb,
                                           this);
       dalt_ctrl_srv_ = nh_.advertiseService(dalt_ctrl_srv_name_,
                                           &PIDManager::dalt_ctrl_srv_cb,
                                           this);
       dvel_ctrl_srv_ = nh_.advertiseService(dvel_ctrl_srv_name_,
                                           &PIDManager::dvel_ctrl_srv_cb,
                                           this);
       droll_ctrl_srv_ = nh_.advertiseService(droll_ctrl_srv_name_,
                                           &PIDManager::droll_ctrl_srv_cb,
                                           this);

    }

    void PIDManager::initialize_publishers()
    {
        // Publish enable flag and status
        vbs_enable_pub_ = nh_.advertise<std_msgs::Bool>(vbs_enable_topic_, 10);
        vbs_status_pub_ = nh_.advertise<smarc_msgs::ControllerStatus>(vbs_status_topic_, 1);

        lcg_enable_pub_ = nh_.advertise<std_msgs::Bool>(lcg_enable_topic_, 10);
        lcg_status_pub_ = nh_.advertise<smarc_msgs::ControllerStatus>(lcg_status_topic_, 1);

        tcg_enable_pub_ = nh_.advertise<std_msgs::Bool>(tcg_enable_topic_, 10);
        tcg_status_pub_ = nh_.advertise<smarc_msgs::ControllerStatus>(tcg_status_topic_, 1);

        vbs_alt_enable_pub_ = nh_.advertise<std_msgs::Bool>(vbs_alt_enable_topic_, 10);
        vbs_alt_status_pub_ = nh_.advertise<smarc_msgs::ControllerStatus>(vbs_alt_status_topic_, 1);

        dheading_enable_pub_ = nh_.advertise<std_msgs::Bool>(dheading_enable_topic_, 10);
        dheading_status_pub_ = nh_.advertise<smarc_msgs::ControllerStatus>(dheading_status_topic_, 1);

        ddepth_enable_pub_ = nh_.advertise<std_msgs::Bool>(ddepth_enable_topic_, 10);
        ddepth_status_pub_ = nh_.advertise<smarc_msgs::ControllerStatus>(ddepth_status_topic_, 1);

        dalt_enable_pub_ = nh_.advertise<std_msgs::Bool>(dalt_enable_topic_, 10);
        dalt_status_pub_ = nh_.advertise<smarc_msgs::ControllerStatus>(dalt_status_topic_, 1);

        dvel_enable_pub_ = nh_.advertise<std_msgs::Bool>(dvel_enable_topic_, 10);
        dvel_status_pub_ = nh_.advertise<smarc_msgs::ControllerStatus>(dvel_status_topic_, 1);

        droll_enable_pub_ = nh_.advertise<std_msgs::Bool>(droll_enable_topic_, 10);
        droll_status_pub_ = nh_.advertise<smarc_msgs::ControllerStatus>(droll_status_topic_, 1);

        //Republish setpoints if needed
        yaw_setpoint_pub_ = nh_.advertise<std_msgs::Float64>(yaw_setpoint_topic_repub_, 10);
        depth_setpoint_pub_ = nh_.advertise<std_msgs::Float64>(depth_setpoint_topic_repub_, 10);
        altitude_setpoint_pub_ = nh_.advertise<std_msgs::Float64>(altitude_setpoint_topic_repub_, 10);
        speed_setpoint_pub_ = nh_.advertise<std_msgs::Float64>(speed_setpoint_topic_repub_, 10);
        pitch_setpoint_pub_ = nh_.advertise<std_msgs::Float64>(pitch_setpoint_topic_repub_, 10);
        roll_setpoint_pub_ = nh_.advertise<std_msgs::Float64>(roll_setpoint_topic_repub_, 10);


    }

    bool PIDManager::vbs_ctrl_srv_cb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response)
    {
        ROS_INFO("VBS service callback activated");
        if (request.data)
        {
            try
            {
                //enable  controller
                vbs_enable_msg.data= true;
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
                vbs_enable_msg.data= false;
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

        //if (response.success)
        //{
        //    vbs_enable_msg.data = request.data;
        //}

        return true;
    }

    //LCG service
    bool PIDManager::lcg_ctrl_srv_cb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response)
    {
        ROS_INFO("LCG service callback activated");
        if (request.data)
        {
            try
            {
                //enable  controller
                lcg_enable_msg.data= true;
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
                lcg_enable_msg.data= false;
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

    //TCG service
    bool PIDManager::tcg_ctrl_srv_cb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response)
    {
        ROS_INFO("TCG service callback activated");
        if (request.data)
        {
            try
            {
                //enable  controller
                tcg_enable_msg.data= true;
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
                tcg_enable_msg.data= false;
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

    //VBS altitude service
    bool PIDManager::vbs_alt_ctrl_srv_cb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response)
    {
        ROS_INFO("VBS altitude service callback activated");
        if (request.data)
        {
            try
            {
                //enable  controller
                vbs_alt_enable_msg.data= true;
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
                vbs_alt_enable_msg.data= false;
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

    //Dynamic heading service
    bool PIDManager::dheading_ctrl_srv_cb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response)
    {
        ROS_INFO("Dynamic heading service callback activated");
        if (request.data)
        {
            try
            {
                //enable  controller
                dheading_enable_msg.data= true;
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
                dheading_enable_msg.data= false;
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

    //Dynamic depth service
    bool PIDManager::ddepth_ctrl_srv_cb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response)
    {
        ROS_INFO("Dynamic depth service callback activated");
        if (request.data)
        {
            try
            {
                //enable  controller
                ddepth_enable_msg.data= true;
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
                ddepth_enable_msg.data= false;
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

    //Dynamic altitude service
    bool PIDManager::dalt_ctrl_srv_cb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response)
    {
        ROS_INFO("Dynamic altitude service callback activated");
        if (request.data)
        {
            try
            {
                //enable  controller
                dalt_enable_msg.data= true;
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
                dalt_enable_msg.data= false;
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

    //Dynamic velocity service
    bool PIDManager::dvel_ctrl_srv_cb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response)
    {
        ROS_INFO("Dynamic velocity service callback activated");
        if (request.data)
        {
            try
            {
                //enable  controller
                dvel_enable_msg.data= true;
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
                dvel_enable_msg.data= false;
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

    //Dynamic roll service
    bool PIDManager::droll_ctrl_srv_cb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response)
    {
        ROS_INFO("Dynamic roll service callback activated");
        if (request.data)
        {
            try
            {
                //enable  controller
                droll_enable_msg.data= true;
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
                droll_enable_msg.data= false;
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

    //subscriber callbacks to setpoints
    void PIDManager::yaw_setpoint_cb(const std_msgs::Float64& setpoint)
    {
        yaw_setpoint.data = setpoint.data;
        ROS_INFO_THROTTLE(1.0, "[ pid_manager ]  Republishing setpoint: %f");
    }

    void PIDManager::depth_setpoint_cb(const std_msgs::Float64& setpoint)
    {
        depth_setpoint.data = setpoint.data;
        ROS_INFO_THROTTLE(1.0, "[ pid_manager ]  Republishing setpoint: %f");
    }

    void PIDManager::altitude_setpoint_cb(const std_msgs::Float64& setpoint)
    {
        altitude_setpoint.data = setpoint.data;
        ROS_INFO_THROTTLE(1.0, "[ pid_manager ]  Republishing setpoint: %f");
    }

    void PIDManager::speed_setpoint_cb(const std_msgs::Float64& setpoint)
    {
        speed_setpoint.data = setpoint.data;
        ROS_INFO_THROTTLE(1.0, "[ pid_manager ]  Republishing setpoint: %f");
    }

    void PIDManager::pitch_setpoint_cb(const std_msgs::Float64& setpoint)
    {
        pitch_setpoint.data = setpoint.data;
        ROS_INFO_THROTTLE(1.0, "[ pid_manager ]  Republishing setpoint: %f");
    }

    void PIDManager::roll_setpoint_cb(const std_msgs::Float64& setpoint)
    {
        roll_setpoint.data = setpoint.data;
        ROS_INFO_THROTTLE(1.0, "[ pid_manager ]  Republishing setpoint: %f");
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
