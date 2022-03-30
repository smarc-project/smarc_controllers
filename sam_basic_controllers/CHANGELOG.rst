^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sam_basic_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2022-03-30)
------------------
* Merge pull request `#4 <https://github.com/smarc-project/smarc_controllers/issues/4>`_ from svbhat/noetic-devel
  Enabled topics republishing, added a 'dynamic_pitch' controller
* Included a dynamic pitch controller.
* Changed republish flag to true. Republished topics used for setpoints!
* bump version sam_controllers
* Merge pull request `#3 <https://github.com/smarc-project/smarc_controllers/issues/3>`_ from svbhat/noetic-devel
  updated altitude topic
* updated altitude topic
* Update package.xml
* Merge pull request `#2 <https://github.com/smarc-project/smarc_controllers/issues/2>`_ from svbhat/noetic-devel
  Updated controller signs to be consistent with real SAM, and added controller services.
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* added pid_manager to static_controllers.launch. Disabled republishing of setpoints.
* Included overactuation for depth, altitude and roll control. Publishing to the same topic republishes to all controllers.
* Updated status topics to have VBS and ddepth publish to the same topic, same for roll and altitude.
* Changed abs() to fabs()
* Updated depth controller
* Updated depth controller coefficients
* Updated controller enable services and topic republishing to make them modular as objects of a single class.
* updated controller signs to keep SIM consistent with REAL.
* Updated roll and velocity controllers and services
* Updated controller manager and enable topics
* Added controller manager with services to enable and disable controllers.
* Merge pull request `#1 <https://github.com/smarc-project/smarc_controllers/issues/1>`_ from svbhat/noetic-devel
  Updated controllers to run with the latest version of SAM in Stonefish
* updated feedback topics to fit naming convention.
* updated to latest naming convention for state feedback.
* Updated dr topics and moved odom listener launch to dead reckoning
* Added dependencies to smarc_navigation
* removed cola2_msgs, replaced with smarc_msgs/DVL
* Added odom_listener from tf_convenience_topics.
* Updated link to odom_listener for wp following
* Updated controllers to use tf_convenience_topics.
* updated install targets
* Updated controllers to make them work with the latest version of Stonefish
* add_smarc_msgs_depend
* Merge pull request `#17 <https://github.com/smarc-project/smarc_controllers/issues/17>`_ from svbhat/master
  Changed thruster topics and datatypes in sam_basic_controllers and sam_maneuvering_primitives.
* Updated thruster tipics and datatypes in Maneuvering Primitives, and updated topics in controller launch files.
* Changed thruster topics and datatypes.
* Merge pull request `#15 <https://github.com/smarc-project/smarc_controllers/issues/15>`_ from svbhat/patch-1
  Update dynamic_depth_ctrl_REAL.launch
* Merge pull request `#16 <https://github.com/smarc-project/smarc_controllers/issues/16>`_ from svbhat/patch-2
  Update dynamic_heading_ctrl_SIM.launch
* Update dynamic_heading_ctrl_SIM.launch
  Updated feedback topics
* Update dynamic_depth_ctrl_REAL.launch
  updated feedback topics
* Merge pull request `#14 <https://github.com/smarc-project/smarc_controllers/issues/14>`_ from svbhat/master
  Included a flag in the launch files to switch between SIM and REAL SAM.
* Added SIM and REAL flags in launch files.
* Cleanup to switch between SIM and REAL
* Merge pull request `#5 <https://github.com/smarc-project/smarc_controllers/issues/5>`_ from svbhat/asko_working_version
  Merge Askö changes to Turbo turn, and controller parameters on SAM to Master
* New branch with working version from Asko tests with waypoints underwater
* Merge pull request `#13 <https://github.com/smarc-project/smarc_controllers/issues/13>`_ from svbhat/master
  Clean up of old files
* Added odom_listener namespace to feedback topics
* Merge branch 'master' of https://gitr.sys.kth.se/svbhat/sam_controllers
* Updated static pitch controller at Askö.
* Updated topics from the odom listener pointing to the controllers.
* Update pid_tf_listener.cpp
* Update static_controllers.launch
* Update static_controllers.launch
* Update tf_listener.launch
* Merge pull request `#4 <https://github.com/smarc-project/smarc_controllers/issues/4>`_ from smarc-project/master
  Update latest upstream branch before merging velocity control
* Update pid_tf_listener.cpp
* Update static_controllers.launch
* Update static_controllers.launch
* Update static_controllers.launch
* Update tf_listener.launch
* Updated velocity control and added an odom listener node to /sam/dr/local/odom/filtered
* Included altitude control
* Update README.md
* Update static_controllers.launch
  Included commented out lines for altitude control.
* Update pid_tf_listener.cpp
  Added Nacho's update of utm frame instead of world frame
* Update static_controllers.launch
* Cleaned up controller nodes and launch files
* Merge pull request `#12 <https://github.com/smarc-project/smarc_controllers/issues/12>`_ from smarc-project/dual_ekf_test
  Changed frames from world to utm
* Changed frames from world to utm
* Merge pull request `#11 <https://github.com/smarc-project/smarc_controllers/issues/11>`_ from svbhat/master
  Added enable flag to all controllers and actuator forwarding.
* Flags to compile with C++11
* Update dynamic_heading_ctrl.launch
  Included angle wrapping within the PID to avoid singularities at +/- pi radians yaw.
* Update static_depth_ctrl.launch
* Update dynamic_heading_ctrl.launch
  Coefficients updated to work smoothly in Stonefish
* Create static_depth_ctrl_SAM.launch
  Backup of VBS controller on SAM
* Create dynamic_heading_ctrl_SAM.launch
  Backup on SAM heading controller
* Update dynamic_depth_ctrl.launch
  Updated controller limits and coefficients to work in Stonefish.
* Create dynamic_depth_ctrl_SAM.launch
  Backup of controllers on SAM
* Update dynamic_controllers.launch
* Update tf_listener.launch
* Update dynamic_velocity_ctrl.launch
* Update dynamic_roll_ctrl.launch
* Update dynamic_heading_ctrl.launch
* Update dynamic_controllers.launch
* Update dynamic_depth_ctrl.launch
* Update namespace to robot_name
* Update static_depth_ctrl.launch
* Update static_pitch_ctrl.launch
* Update static_pitch_ctrl.launch
* Update static_pitch_ctrl.launch
* Update dynamic_controllers.launch
* Updated enable flags to all controllers.
* Included enable flag for LCG to test
* Merge pull request `#10 <https://github.com/smarc-project/smarc_controllers/issues/10>`_ from ozero/master
  Removed the stupid DS_Store crap macs like to put everywhere, added t…
* Removed the stupid DS_Store crap macs like to put everywhere, added to gitignore.
* Merge pull request `#9 <https://github.com/smarc-project/smarc_controllers/issues/9>`_ from svbhat/master
  Updated controllers from Harnosand, abort flag for emergencies and namespacing for multiple SAMs.
* Changed dynamic controllers launch file arg from namespace to robot_name for consistency
* changed launch arg to robot_name from namespace to be consistent with the rest of the system
* Changed heading PID coefficients after tuning on Stonefish
* Changes after Harnosand tests, edited node names and added an abort flag to VBS
* Merge branch 'master' of https://gitr.sys.kth.se/svbhat/sam_controllers
* Update angle_wrapper.cpp
  Changed spacing
* added a condition to prevent contiuous publishing of setpoints.
* Added a condition to the angle wrapper to prevent continuous publishing
* Tuning coeffiecients for dynamic depth updated
* Update angle_wrapper.cpp
* Updated topics nad logic for angle wrapping
* Added angle wrapper
* Updated controllers to latest version on SAM. Changed the location of the tf_listener
* Merge pull request `#2 <https://github.com/smarc-project/smarc_controllers/issues/2>`_ from smarc-project/master
  Updated with latest developments on 20 Jan 2020
* Changed abs to fabs
* Merge pull request `#7 <https://github.com/smarc-project/smarc_controllers/issues/7>`_ from smarc-project/new_topics
  Added new topic and node names for the static pid controllers
* Fixed a couple of things for Kristineberg trials
* Merge pull request `#8 <https://github.com/smarc-project/smarc_controllers/issues/8>`_ from svbhat/new_topics
  Updated Velocity Controller and related topics
* Updated PID gains for Trim topics
* Updated launch files, added velocity controller and updated pid_actuator_prop
* Fixed the dynamic control topic routing
* Added new topic and node names for the static pid controllers
* Merge pull request `#6 <https://github.com/smarc-project/smarc_controllers/issues/6>`_ from svbhat/master
  Replace sam_loop_test with a sam_basic_controllers.
* Update dynamic_heading_ctrl.launch
* Removed conflicts from tf_listener code
* Updated CMakeLists.txt
* Merge remote-tracking branch 'upstream/master'
* Merge pull request `#1 <https://github.com/smarc-project/smarc_controllers/issues/1>`_ from svbhat/Harsha/Basic_Controllers
  Harsha/basic controllers
* Delete settings.json
* Delete .DS_Store
* Update tf_listener.launch
* Update pid_tf_listener.cpp
  changed duration to 2 seconds
* Update pid_tf_listener.cpp
  Output all states
* Delete static_depth_ctrl .launch
* Added dynamic depth and heading control. Modified Tf listener to include velocity
* Update thrust_vector_ctrl.launch
* Update thrust_vector_ctrl.launch
* Update thrust_vector_ctrl.launch
* Update thrust_vector_ctrl.launch
  Change the topic name in uavcan
* Rename static_depth_ctrl .launch to static_depth_ctrl.launch
* Update README.md
* Modifications to test branch commit
* Modifications to sam_basic_controllers
* Code cleanup to remove redundancies and confusing names. Nodes and launch files separated for clarity.
* Contributors: Carl Ljung, Ignacio Torroba Balmori, Jollerprutt, Nils Bore, Ozer, Ozer Ozkahraman, Sriharsha Bhat, Sriharsha Vishnu Bhat, Torroba, Xavier1, svbhat, xyp8023
