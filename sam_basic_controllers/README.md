# sam_basic_controllers

Package containing the basic controllers for trim, thrust vectoring and buoyancy systems for the SAM AUV using ROS-PID.

To launch pid_manager, static trim and depth controllers, launch static_controllers.launch.

To launch cascaded heading and depth controllers, launch dynamic_controllers.launch after launching static_controllers.launch.

To activate the controllers call the following services - set the value to "true" to engage and "false" to disable:

  VBS static depth: /sam/ctrl/toggle_vbs_ctrl
  LCG static pitch: /sam/ctrl/toggle_pitch_ctrl
  VBS static altitude: /sam/ctrl/toggle_vbs_alt_ctrl
  Yaw: /sam/ctrl/toggle_yaw_ctrl
  Depth: /sam/ctrl/toggle_depth_ctrl
  Altitude: /sam/ctrl/toggle_altitude_ctrl
  Speed: /sam/ctrl/toggle_speed_ctrl
  Roll: /sam/ctrl/toggle_roll_ctrl
  (Static Roll TCG: /sam/ctrl/toggle_tcg_ctrl) 

TODO: Combine both controller launch files in one after further testing.

