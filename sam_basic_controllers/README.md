# sam_basic_controllers

Package containing the basic controllers for trim, thrust vectoring and buoyancy systems for the SAM AUV using ROS-PID.

To launch pid_manager, static trim and depth controllers, launch static_controllers.launch.

To launch cascaded heading and depth controllers, launch dynamic_controllers.launch after launching static_controllers.launch.

To activate the controllers call the following services ([link](https://github.com/smarc-project/smarc_msgs#id11)) - set the value to "true" to engage and "false" to disable:

  - _VBS static depth:_ ```/sam/ctrl/toggle_vbs_ctrl```
  
  - _LCG static pitch:_ ```/sam/ctrl/toggle_pitch_ctrl```
  
  - _VBS static altitude:_ ```/sam/ctrl/toggle_vbs_alt_ctrl```
  
  - _Yaw:_ ```/sam/ctrl/toggle_yaw_ctrl```
  
  - _Depth:_ ```/sam/ctrl/toggle_depth_ctrl```
  
  - _Altitude:_ ```/sam/ctrl/toggle_altitude_ctrl```
  
  - _Speed:_ ```/sam/ctrl/toggle_speed_ctrl```
  
  - _Roll:_ ```/sam/ctrl/toggle_roll_ctrl```
  
  - (_Static Roll TCG:_ ```/sam/ctrl/toggle_tcg_ctrl```) 

e.g. usage: ```rosservice call /sam/ctrl/toggle_vbs_ctrl "data: true"```

TODO: Combine both controller launch files in one after further testing.

