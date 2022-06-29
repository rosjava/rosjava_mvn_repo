# The RosJava Maven Repo

Maven artifact repository for rosjava dependencies and builds.

## Contents


**From Dependent Repositories**

Many of the artifacts here are collected from a variety of other repositories to provide a single stable source of artifacts for development of the rosjava/android core stacks. This list (hopefully relatively complete) includes:

* http://repo1.maven.org/maven2/org/apache/apache/
* http://repo1.maven.org/maven2/junit/junit/
* http://repo.maven.apache.org/maven2/org/sonatype/oss/oss-parent/
* http://repo.jfrog.org/artifactory/libs-releases/org/apache/commons/com.springsource.org.apache.commons.codec/
* http://repo.jfrog.org/artifactory/libs-releases/org/apache/commons/com.springsource.org.apache.commons.httpclient/
* http://repo.jfrog.org/artifactory/libs-releases/org/apache/commons/com.springsource.org.apache.commons.io/
* http://repo.jfrog.org/artifactory/libs-releases/org/apache/commons/com.springsource.org.apache.commons.lang/
* http://repo.jfrog.org/artifactory/libs-releases/org/apache/commons/com.springsource.org.apache.commons.logging/
* http://repo.jfrog.org/artifactory/libs-releases/org/apache/commons/com.springsource.org.apache.commons.net/
* http://central.maven.org/maven2/org/apache/commons/commons-parent/
* http://central.maven.org/maven2/ws-commons-util/ws-commons-util/
* http://central.maven.org/maven2/com/google/guava/guava/
* http://central.maven.org/maven2/com/google/guava/guava-parent/
* http://central.maven.org/maven2/com/google/code/findbugs/jsr305/
* http://central.maven.org/maven2/io/netty/netty/
* http://central.maven.org/maven2/io/netty/netty-all/
* http://central.maven.org/maven2/xml-apis/xml-apis/
* http://central.maven.org/maven2/commons-pool/commons-pool/
* http://central.maven.org/maven2/org/apache/commons/commons-pool2/
* http://central.maven.org/maven2/dnsjava/dnsjava/
* http://jcenter.bintray.com/com/android/tools/build/gradle/

**Official RosJava Artifacts**

All official rosjava artifacts that are generated have their official maven home here.

**Interesting 3rd Party Dependencies**

We also include some esoteric, but useful 3rd party dependencies here that the community may find useful. Some of these are custom built and added here. Others come from the following list of repositories:

## Other Maven Repositories

Some other maven repositories that may be of interest to ros users:

* http://central.maven.org/maven2/org/bytedeco/javacpp-presets/opencv/
* http://central.maven.org/maven2/nu/pattern/opencv/




# List of ROS Noetic Interfaces included
The following table lists all the interfaces compiled into ROS Java Interfaces.
These are distributed as jars, targeting Java 17


The table lists:


-  The package name
-  The version used for compilation
-  The package dependencies
-  A list of urls related to each package, such as links to the source, issues or wikis
-  A short description
-  The number of ROS messages compiled
-  The number of ROS services compiled
-  The number of ROS actions compiled



Package Name | Version | Dependencies | Url | Description
--- | --- | --- |---|---
actionlib | 1.13.2 |std_msgs <BR> rostest <BR> actionlib_msgs <BR> rospy <BR> message_generation <BR> roscpp <BR> libboost-dev <BR> libboost-thread-dev |http://www.ros.org/wiki/actionlib <BR> https://github.com/ros/actionlib/issues <BR> https://github.com/ros/actionlib |     The actionlib stack provides a standardized interface for     interfacing with preemptable tasks. Examples of this include moving     the base to a target location, performing a laser scan and returning     the resulting point cloud, detecting the handle of a door, etc.
actionlib_msgs | 1.13.0 |std_msgs <BR> message_generation <BR> message_runtime |http://wiki.ros.org/actionlib_msgs |      actionlib_msgs defines the common messages to interact with an      action server and an action client.  For full documentation of      the actionlib API see      the actionlib      package.
actionlib_tutorials | 0.2.0 |std_msgs <BR> actionlib_msgs <BR> roscpp <BR> message_generation <BR> message_runtime <BR> actionlib |http://www.ros.org/wiki/actionlib/Tutorials <BR> https://github.com/ros/common_tutorials <BR> https://github.com/ros/common_tutorials/issues |The actionlib_tutorials package
base_local_planner | 1.17.1 |voxel_grid <BR> tf2 <BR> std_msgs <BR> angles <BR> rospy <BR> visualization_msgs <BR> nav_msgs <BR> eigen <BR> geometry_msgs <BR> dynamic_reconfigure <BR> tf2_geometry_msgs <BR> pluginlib <BR> message_generation <BR> roscpp <BR> cmake_modules <BR> nav_core <BR> sensor_msgs <BR> tf2_ros <BR> costmap_2d <BR> rosconsole |http://wiki.ros.org/base_local_planner |          This package provides implementations of the Trajectory Rollout and Dynamic Window approaches to local robot navigation on a plane. Given a plan to follow and a costmap, the controller produces velocity commands to send to a mobile base. This package supports both holonomic and non-holonomic robots, any robot footprint that can be represented as a convex polygon or circle, and exposes its configuration as ROS parameters that can be set in a launch file. This package's ROS wrapper adheres to the BaseLocalPlanner interface specified in the nav_core package.
bond | 1.8.6 |std_msgs <BR> message_generation |http://www.ros.org/wiki/bond <BR> https://github.com/ros/bond_core/issues <BR> https://github.com/ros/bond_core |     A bond allows two processes, A and B, to know when the other has     terminated, either cleanly or by crashing.  The bond remains     connected until it is either broken explicitly or until a     heartbeat times out.
cartographer_ros_msgs | 1.0.0 |std_msgs <BR> message_generation <BR> geometry_msgs |https://github.com/cartographer-project/cartographer_ros |     ROS messages for the cartographer_ros package.
controller_manager_msgs | 0.19.4 |std_msgs <BR> message_generation |https://github.com/ros-controls/ros_control/wiki <BR> https://github.com/ros-controls/ros_control/issues <BR> https://github.com/ros-controls/ros_control |Messages and services for the controller manager.
control_msgs | 1.5.2 |std_msgs <BR> trajectory_msgs <BR> actionlib_msgs <BR> message_generation <BR> message_runtime <BR> geometry_msgs |http://ros.org/wiki/control_msgs |     control_msgs contains base messages and actions useful for     controlling robots.  It provides representations for controller     setpoints and joint and cartesian trajectories.
control_toolbox | 1.18.2 |tinyxml <BR> std_msgs <BR> control_msgs <BR> realtime_tools <BR> message_generation <BR> roscpp <BR> cmake_modules <BR> dynamic_reconfigure |http://ros.org/wiki/control_toolbox <BR> https://github.com/ros-controls/control_toolbox/issues <BR> https://github.com/ros-controls/control_toolbox/ |The control toolbox contains modules that are useful across all controllers.
costmap_2d | 1.17.1 |voxel_grid <BR> tf2 <BR> std_msgs <BR> tf2_sensor_msgs <BR> visualization_msgs <BR> message_filters <BR> nav_msgs <BR> geometry_msgs <BR> dynamic_reconfigure <BR> tf2_geometry_msgs <BR> rostest <BR> pluginlib <BR> message_generation <BR> roscpp <BR> cmake_modules <BR> laser_geometry <BR> sensor_msgs <BR> tf2_ros <BR> map_msgs |http://wiki.ros.org/costmap_2d |         This package provides an implementation of a 2D costmap that takes in sensor         data from the world, builds a 2D or 3D occupancy grid of the data (depending         on whether a voxel based implementation is used), and inflates costs in a         2D costmap based on the occupancy grid and a user specified inflation radius.         This package also provides support for map_server based initialization of a         costmap, rolling window based costmaps, and parameter based subscription to         and configuration of sensor topics.
diagnostic_msgs | 1.13.0 |std_msgs <BR> message_generation <BR> message_runtime |http://wiki.ros.org/diagnostic_msgs |     This package holds the diagnostic messages which provide the     standardized interface for the diagnostic and runtime monitoring     systems in ROS. These messages are currently used by     the diagnostics     Stack, which provides libraries for simple ways to set and access     the messages, as well as automated ways to process the diagnostic     data.      These messages are used for long term logging and will not be     changed unless there is a very important reason.
dynamic_reconfigure | 1.7.1 |std_msgs <BR> rostest <BR> message_generation <BR> roscpp_serialization <BR> roscpp <BR> libboost-dev <BR> cpp_common |http://ros.org/wiki/dynamic_reconfigure <BR> https://github.com/ros/dynamic_reconfigure/issues <BR> https://github.com/ros/dynamic_reconfigure |     The dynamic_reconfigure package provides a means to update parameters     at runtime without having to restart the node.
four_wheel_steering_msgs | 1.1.1 |std_msgs <BR> message_generation |http://ros.org/wiki/four_wheel_steering_msgs <BR> https://github.com/ros-drivers/four_wheel_steering_msgs.git <BR> https://github.com/ros-drivers/four_wheel_steering_msgs/issues |     ROS messages for robots using FourWheelSteering.
gazebo_msgs | 2.9.1 |trajectory_msgs <BR> std_msgs <BR> std_srvs <BR> message_generation <BR> sensor_msgs <BR> geometry_msgs |http://gazebosim.org/tutorials?cat=connect_ros <BR> https://github.com/ros-simulation/gazebo_ros_pkgs/issues <BR> https://github.com/ros-simulation/gazebo_ros_pkgs |     Message and service data structures for interacting with Gazebo from ROS.
geographic_msgs | 0.5.5 |std_msgs <BR> message_generation <BR> uuid_msgs <BR> message_runtime <BR> geometry_msgs |http://wiki.ros.org/geographic_msgs <BR> https://github.com/ros-geographic-info/geographic_info/issues <BR> https://github.com/ros-geographic-info/geographic_info.git |      ROS messages for Geographic Information Systems.
geometry_msgs | 1.13.0 |std_msgs <BR> message_generation <BR> message_runtime |http://wiki.ros.org/geometry_msgs |     geometry_msgs provides messages for common geometric primitives     such as points, vectors, and poses. These primitives are designed     to provide a common data type and facilitate interoperability     throughout the system.
graph_msgs | 0.1.0 |std_msgs <BR> message_generation <BR> message_runtime <BR> geometry_msgs |https://github.com/davetcoleman/graph_msgs <BR> https://github.com/davetcoleman/graph_msgs/issues <BR> https://github.com/davetcoleman/graph_msgs/ |ROS messages for publishing graphs of different data types
laser_assembler | 1.7.8 |tf <BR> rostest <BR> pluginlib <BR> message_generation <BR> roscpp <BR> sensor_msgs <BR> message_filters <BR> laser_geometry <BR> filters <BR> message_runtime |http://ros.org/wiki/laser_assembler |     Provides nodes to assemble point clouds from either LaserScan or PointCloud messages
map_msgs | 1.14.1 |std_msgs <BR> message_generation <BR> nav_msgs <BR> sensor_msgs <BR> message_runtime |http://ros.org/wiki/map_msgs <BR> https://github.com/ros-planning/navigation_msgs/issues |         This package defines messages commonly used in mapping packages.
moveit_msgs | 0.11.1 |octomap_msgs <BR> trajectory_msgs <BR> std_msgs <BR> actionlib_msgs <BR> message_generation <BR> sensor_msgs <BR> geometry_msgs <BR> object_recognition_msgs <BR> shape_msgs |http://moveit.ros.org <BR> https://github.com/ros-planning/moveit_msgs/issues <BR> https://github.com/ros-planning/moveit_msgs |Messages, services and actions used by MoveIt
move_base_msgs | 1.14.1 |actionlib_msgs <BR> message_generation <BR> message_runtime <BR> geometry_msgs |http://wiki.ros.org/move_base_msgs <BR> https://github.com/ros-planning/navigation_msgs/issues |          Holds the action description and relevant messages for the move_base package.
navfn | 1.17.1 |netpbm <BR> visualization_msgs <BR> nav_msgs <BR> geometry_msgs <BR> pluginlib <BR> message_generation <BR> roscpp <BR> cmake_modules <BR> nav_core <BR> sensor_msgs <BR> tf2_ros <BR> costmap_2d <BR> rosconsole |http://wiki.ros.org/navfn |          navfn provides a fast interpolated navigation function that can be used to create plans for         a mobile base. The planner assumes a circular robot and operates on a costmap to find a         minimum cost plan from a start point to an end point in a grid. The navigation function is         computed with Dijkstra's algorithm, but support for an A* heuristic may also be added in the         near future. navfn also provides a ROS wrapper for the navfn planner that adheres to the         nav_core::BaseGlobalPlanner interface specified in nav_core.
nav_msgs | 1.13.0 |std_msgs <BR> actionlib_msgs <BR> message_generation <BR> message_runtime <BR> geometry_msgs |http://wiki.ros.org/nav_msgs |     nav_msgs defines the common messages used to interact with the     navigation stack.
nodelet | 1.10.0 |std_msgs <BR> pluginlib <BR> message_generation <BR> roscpp <BR> cmake_modules <BR> bondcpp <BR> boost <BR> uuid <BR> rosconsole |http://ros.org/wiki/nodelet <BR> https://github.com/ros/nodelet_core/issues <BR> https://github.com/ros/nodelet_core |     The nodelet package is designed to provide a way to run multiple     algorithms in the same process with zero copy transport between     algorithms.      This package provides both the nodelet base class needed for     implementing a nodelet, as well as the NodeletLoader class used     for instantiating nodelets.
object_recognition_msgs | 0.4.2 |std_msgs <BR> actionlib_msgs <BR> message_generation <BR> sensor_msgs <BR> message_runtime <BR> geometry_msgs <BR> shape_msgs |http://www.ros.org/wiki/object_recognition |Object_recognition_msgs contains the ROS message and the actionlib definition used in object_recognition_core
octomap_msgs | 0.3.5 |std_msgs <BR> message_generation <BR> message_runtime <BR> geometry_msgs |http://ros.org/wiki/octomap_msgs <BR> https://github.com/OctoMap/octomap_msgs/issues |    This package provides messages and serializations / conversion for the OctoMap library.
pcl_msgs | 0.3.0 |std_msgs <BR> message_generation <BR> sensor_msgs <BR> message_runtime |http://wiki.ros.org/pcl_msgs <BR> https://github.com/ros-perception/pcl_msgs <BR> https://github.com/ros-perception/pcl_msgs/issues |Package containing PCL (Point Cloud Library)-related ROS messages.
polled_camera | 1.12.0 |std_msgs <BR> message_generation <BR> roscpp <BR> sensor_msgs <BR> message_runtime <BR> image_transport |http://ros.org/wiki/polled_camera <BR> https://github.com/ros-perception/image_common/issues <BR> https://github.com/ros-perception/image_common |       polled_camera contains a service and C++ helper classes for implementing a polled      camera driver node and requesting images from it. The package is currently for      internal use as the API is still under development.
robot_localization | 2.6.8 |python-catkin-pkg <BR> tf2 <BR> std_msgs <BR> geographic_msgs <BR> message_filters <BR> nav_msgs <BR> eigen <BR> geometry_msgs <BR> roslint <BR> yaml-cpp <BR> diagnostic_msgs <BR> nodelet <BR> eigen_conversions <BR> tf2_geometry_msgs <BR> xmlrpcpp <BR> message_generation <BR> roscpp <BR> std_srvs <BR> cmake_modules <BR> sensor_msgs <BR> tf2_ros <BR> diagnostic_updater <BR> python3-catkin-pkg |http://ros.org/wiki/robot_localization |Provides nonlinear state estimation through sensor fusion of an abritrary number of sensors.
rosapi | 0.11.12 |message_generation |http://ros.org/wiki/rosapi <BR> https://github.com/RobotWebTools/rosbridge_suite/issues <BR> https://github.com/RobotWebTools/rosbridge_suite |     Provides service calls for getting ros meta-information, like list of     topics, services, params, etc.
rosauth | 1.0.1 |roscpp <BR> message_generation <BR> libssl-dev |http://ros.org/wiki/rosauth <BR> https://github.com/GT-RAIL/rosauth/issues <BR> https://github.com/GT-RAIL/rosauth |Server Side tools for Authorization and Authentication of ROS Clients
rosbridge_msgs | 0.11.12 |std_msgs <BR> message_generation | |Package containing message files
roscpp | 1.15.9 |std_msgs <BR> libboost-filesystem-dev <BR> rosgraph_msgs <BR> message_runtime <BR> cpp_common <BR> pkg-config <BR> roslang <BR> xmlrpcpp <BR> libboost-system-dev <BR> message_generation <BR> roscpp_serialization <BR> rostime <BR> roscpp_traits <BR> libboost-chrono-dev <BR> rosconsole |http://ros.org/wiki/roscpp |     roscpp is a C++ implementation of ROS. It provides     a client     library that enables C++ programmers to quickly interface with     ROS Topics,     Services,     and Parameters.      roscpp is the most widely used ROS client library and is designed to     be the high-performance library for ROS.
roscpp_tutorials | 0.10.2 |std_msgs <BR> message_generation <BR> roscpp <BR> roscpp_serialization <BR> rostime <BR> message_runtime <BR> libboost-thread-dev <BR> libboost-date-time-dev <BR> rosconsole |http://www.ros.org/wiki/roscpp_tutorials <BR> https://github.com/ros/ros_tutorials/issues <BR> https://github.com/ros/ros_tutorials |     This package attempts to show the features of ROS step-by-step,     including using messages, servers, parameters, etc.
rosgraph_msgs | 1.11.3 |std_msgs <BR> message_generation <BR> message_runtime |http://ros.org/wiki/rosgraph_msgs |      Messages relating to the ROS Computation Graph. These are generally considered to be low-level messages that end users do not interact with.
rospy_tutorials | 0.10.2 |std_msgs <BR> rostest <BR> rospy <BR> message_generation <BR> message_runtime |http://www.ros.org/wiki/rospy_tutorials <BR> https://github.com/ros/ros_tutorials/issues <BR> https://github.com/ros/ros_tutorials |     This package attempts to show the features of ROS python API step-by-step,     including using messages, servers, parameters, etc. These tutorials are compatible with the nodes in roscpp_tutorial.
rosserial_arduino | 0.9.1 |rosserial_client <BR> rospy <BR> rosserial_msgs <BR> message_generation <BR> message_runtime <BR> rosserial_python <BR> arduino-core |http://ros.org/wiki/rosserial_arduino |     rosserial for Arduino/AVR platforms.
rosserial_mbed | 0.9.1 |rosserial_client <BR> rospy <BR> rosserial_msgs <BR> message_generation <BR> message_runtime |http://ros.org/wiki/rosserial_mbed |     rosserial for mbed platforms.
rosserial_msgs | 0.9.1 |message_generation <BR> message_runtime |http://ros.org/wiki/rosserial_msgs |     Messages for automatic topic configuration using rosserial.
rviz | 1.14.4 |liburdfdom-headers-dev <BR> visualization_msgs <BR> liburdfdom-dev <BR> qtbase5-dev <BR> geometry_msgs <BR> image_transport <BR> yaml-cpp <BR> assimp-dev <BR> pluginlib <BR> roscpp <BR> tf2_ros <BR> interactive_markers <BR> std_msgs <BR> libqt5-opengl-dev <BR> rospy <BR> libogre-dev <BR> rosbag <BR> message_filters <BR> nav_msgs <BR> resource_retriever <BR> urdf <BR> eigen <BR> opengl <BR> tf2_geometry_msgs <BR> roslib <BR> libogre <BR> tinyxml2 <BR> message_generation <BR> std_srvs <BR> cmake_modules <BR> laser_geometry <BR> python_qt_binding <BR> sensor_msgs <BR> map_msgs <BR> rosconsole |http://wiki.ros.org/rviz <BR> https://github.com/ros-visualization/rviz <BR> https://github.com/ros-visualization/rviz/issues |      3D visualization tool for ROS.
sensor_msgs | 1.13.0 |std_msgs <BR> message_generation <BR> geometry_msgs |http://ros.org/wiki/sensor_msgs |     This package defines messages for commonly used sensors, including     cameras and scanning laser rangefinders.
shape_msgs | 1.13.0 |std_msgs <BR> message_generation <BR> message_runtime <BR> geometry_msgs |http://wiki.ros.org/shape_msgs |     This package contains messages for defining shapes, such as simple solid     object primitives (cube, sphere, etc), planes, and meshes.
smach_msgs | 2.5.0 |std_msgs <BR> message_generation | |     this package contains a set of messages that are used by the introspection     interfaces for smach.
std_msgs | 0.5.13 |message_generation <BR> message_runtime |http://www.ros.org/wiki/std_msgs <BR> https://github.com/ros/std_msgs <BR> https://github.com/ros/std_msgs/issues |     Standard ROS Messages including common message types representing primitive data types and other basic message constructs, such as multiarrays.     For common, generic robot-specific message types, please see common_msgs.
std_srvs | 1.11.3 |message_generation <BR> message_runtime |http://ros.org/wiki/std_srvs |Common service definitions.
stereo_msgs | 1.13.0 |std_msgs <BR> message_generation <BR> sensor_msgs <BR> message_runtime |http://wiki.ros.org/stereo_msgs |     stereo_msgs contains messages specific to stereo processing, such as disparity images.
tf | 1.13.2 |std_msgs <BR> angles <BR> message_filters <BR> roswtf <BR> message_runtime <BR> geometry_msgs <BR> message_generation <BR> roscpp <BR> rostime <BR> sensor_msgs <BR> tf2_ros <BR> graphviz <BR> rosconsole |http://www.ros.org/wiki/tf |  tf is a package that lets the user keep track of multiple coordinate frames over time. tf maintains the relationship between coordinate frames in a tree structure buffered in time, and lets the user transform points, vectors, etc between any two coordinate frames at any desired point in time.      Migration: Since ROS Hydro, tf has been "deprecated" in favor of tf2. tf2 is an iteration on tf providing generally the same feature set more efficiently. As well as adding a few new features.     As tf2 is a major change the tf API has been maintained in its current form. Since tf2 has a superset of the tf features with a subset of the dependencies the tf implementation has been removed and replaced with calls to tf2 under the hood. This will mean that all users will be compatible with tf2. It is recommended for new work to use tf2 directly as it has a cleaner interface. However tf will continue to be supported for through at least J Turtle.
tf2_msgs | 0.7.5 |actionlib_msgs <BR> message_generation <BR> geometry_msgs |http://www.ros.org/wiki/tf2_msgs |     tf2_msgs
topic_tools | 1.15.9 |rosunit <BR> std_msgs <BR> rostest <BR> xmlrpcpp <BR> message_generation <BR> roscpp <BR> rostime <BR> message_runtime <BR> cpp_common <BR> rosconsole |http://ros.org/wiki/topic_tools |     Tools for directing, throttling, selecting, and otherwise messing with     ROS topics at a meta level. None of the programs in this package actually     know about the topics whose streams they are altering; instead, these     tools deal with messages as generic binary blobs. This means they can be     applied to any ROS topic.
trajectory_msgs | 1.13.0 |std_msgs <BR> message_generation <BR> rosbag_migration_rule <BR> message_runtime <BR> geometry_msgs |http://wiki.ros.org/trajectory_msgs |     This package defines messages for defining robot trajectories. These messages are     also the building blocks of most of the     control_msgs actions.
turtlesim | 0.10.2 |libqt5-core <BR> std_msgs <BR> libqt5-gui <BR> message_runtime <BR> geometry_msgs <BR> qtbase5-dev <BR> roslib <BR> message_generation <BR> roscpp <BR> roscpp_serialization <BR> std_srvs <BR> rostime <BR> qt5-qmake <BR> libboost-thread-dev <BR> rosconsole |http://www.ros.org/wiki/turtlesim <BR> https://github.com/ros/ros_tutorials/issues <BR> https://github.com/ros/ros_tutorials |     turtlesim is a tool made for teaching ROS and ROS packages.
turtle_actionlib | 0.2.0 |std_msgs <BR> angles <BR> actionlib_msgs <BR> message_generation <BR> roscpp <BR> message_runtime <BR> geometry_msgs <BR> actionlib <BR> turtlesim <BR> rosconsole |http://ros.org/wiki/turtle_actionlib <BR> https://github.com/ros/common_tutorials <BR> https://github.com/ros/common_tutorials/issues |turtle_actionlib demonstrates how to write an action server and client with the turtlesim. The shape_server provides and action interface for drawing regular polygons with the turtlesim.
uuid_msgs | 1.0.6 |std_msgs <BR> message_generation <BR> message_runtime |http://ros.org/wiki/uuid_msgs |     ROS messages for universally unique identifiers.
visualization_msgs | 1.13.0 |std_msgs <BR> message_generation <BR> message_runtime <BR> geometry_msgs |http://ros.org/wiki/visualization_msgs |     visualization_msgs is a set of messages used by higher level packages, such as rviz, that deal in visualization-specific data.      The main messages in visualization_msgs is visualization_msgs/Marker.     The marker message is used to send visualization "markers" such as boxes, spheres, arrows, lines, etc. to a visualization environment such as rviz.     See the rviz tutorial rviz tutorials for more information.   

