# sam_basic_controllers

Package containing the basic controllers for trim, thrust vectoring and buoyancy systems for the SAM AUV using ROS-PID.

To launch static trim and depth controllers and the state feedback tf listener, launch static_controllers.launch
To launch cascaded heading and depth controllers, launch dynamic_controllers.launch after launching static_controllers.launch.
