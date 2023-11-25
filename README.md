# Lab 6: ARIAC 2019 Part 2: Inverse Kinematics Service

## Overview

In this lab, an inverse kinematics service will be used to return the joints of the manipulator given an end effector goal.
Inverse Kinematics is the mathematical process to acquire joint angles (and/or displacements) that will yield a desired end effector pose. This is not a straightforward process as poses can be unreachable, or they may be reachable with an infinite set of joint angles.

### Lab 6 Link

Lab 6 Link: [Laboratory #6_20231027_cert.pdf](https://canvas.case.edu/courses/38747/assignments/509275)

ARIAC 2019 Environment: [Ariac 2019] (https://bitbucket.org/osrf/ariac/wiki/2019/Home)

## Creating Service & Client

Services and Clients are a method of passing information between ROS Nodes. Services and Clients are a more structured method of one to one connections/transactions. A service issues responses of a specified message to information in a request from a client sent in
a specified message. The typical method of operations is synchronous, as in, a client request will block until a response is received from a service provider. Services can respond to requests from multiple clients, but it provides a response based on the specific request.

### 1. create appropriate package

This lab package depends on several other packages, so use the following command to set up the package:

    cd ~/lab5_ws/src
    catkin_create_pkg ik_service roscpp std_msgs geometry_msgs message_generation ur_kinematics

The ik_service package will depend on roscpp, std_msgs, geometry_msgs, message_generation and ur_kinematics.

### 2. create srv and msg files

It is necessary to create a message "msg" type that will hold a set of joint angles. The request will contain a geometry_msgs/Pose, and the response will contain an integer holding the number of solutions generated and a float vector containing those solutions.
Services and clients use "srv" types that are analogous to the "msg" types used in publishers and subscribers. 

The definition of a service "srv" must be in the srv/ directory of a package. 

To create msg/ directory & file:

    mkdir ~/lab5_ws/srv/ik_service/msg
    cd lab5_ws/src/ik_service/msg
    cat > JointSolutions.msg
    touch src/ik_service/msg/JointSolutions.msg

To create srv/ directory & file:

    mkdir ~/lab5_ws/src/ik_service/srv
    cd lab5_ws/src/ik_service/srv
    cat > PoseIK.srv
    touch src/ik_service/srv/PoseIK.srv

### 3. modify JointSolutions.msg and PoseIk.srv files

For the JointSolutions.msg file, add the following:

    float64[6] joint_angles

This creates the ik_service/JointSolutions message type that will be used in the ik_service/PoseIK.srv definition. Remember to not add ANY comments in the file. 

For the PoseIK.srv file, add the following:

    
    geometry_msgs/Pose part_pose
    ---
    int8 num_sols    
    ik_service/JointSolutions[8] joint_solutions

Part pose is the request part of the srv. num_sols is the number of solutions returned. Joint_solutions is the joint angles that can reach the part. Also remember to not add any comments in this file.

The best way to understand the srv definition file and how it will be used is to think of it as defining two message types. The top defines the message type being used for Requests and the bottom is being used to define Responses. The srv definition above defines
ik_service::PoseIK::Request, ik_service::PoseIK::Response, and ik_servive::PoseIK which is composed of a request and a response field of types ik_service::PoseIK::Request and ik_service::PoseIK::Response, respectively.

### 4. service and client node examples

Use the following link for the example service and client node:

    ROS Noetic: [Simple Service and Client (C++)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29)

### 5. create the service and client nodes

Go to the src folder and create the two node:

    cd ~/lab5_ws/src/ik_service/src
    cat > ik_service.cpp
    cat > ik_client.cpp

Both the service and client source code must be updated to use the new service type. This is accomplished by including the definitions of the srv type. Add the following header file to both nodes:

    #include "ik_service/PoseIK.h"

The example service is advertised with the name of "add_two_ints". Updated the service name to "pose_ik" in both the service and client node. For ik_service.cpp, change to the following:

    ros::init(argc, argv, "ik_service");

and for the ik_cliend.cpp, change to the following:

    ros::init(argc, argv, "ik_client");

The client statement needs to use the new srv type to register with the client. In the ik_service.cpp file, change to the following:

    ros::ServiceServer service = n.advertiseService("pose_ik", pose_ik);

and in the ik_cliend.cpp file, change to the following:

    ros::ServiceClient client = n.serviceClient<ik_service::PoseIK>("pose_ik");

Then, in the ik_service.cpp file, change to the following example for the start:

    bool pose_ik(ik_service::PoseIK::Request &req, ik_service::PoseIK::Response &res)
    {
    ROS_INFO("Pose Information...");
    return true;
    }

This is a good point to set the CMakeLists.txt and package.xml files to include the two new nodes.

### 6. update CMakeLists.txt and package.xml files

The CMakeLists.txt file must be configured to generate the message and service types. All packages containing the message msg types used in this package, as well as message_generation, should have been in the dependencies when creating the package as this ensures all message types are included wherever they may be needed in CMakelists.txt.

First, for the package.xml file, update to the following:

    ...
  <buildtool_depend>catkin</buildtool_depend>
  
  <build_depend>geometry_msgs</build_depend>
  <build_depend>message_generation</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>ur_kinematics</build_depend>
  
  <build_export_depend>geometry_msgs</build_export_depend>
  <build_export_depend>message_generation</build_export_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>message_generation</exec_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>message_runtime</exec_depend>
  <exec_depend>ur_kinematics</exec_depend>

With this update, all required packages should be included.

Second, for CMakeList.txt file, update to the following:

    find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        message_generation
        roscpp
        std_msgs
        ur_kinematics
    )
    ...

    add_message_files(
        FILES
    JointSolutions.msg
    )
    ...

    add_service_files(
        FILES
    PoseIK.srv
    )
    ...

    generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
    )
    ...

    catkin_package(
        #INCLUDE_DIRS include
        #LIBRARIES ik_service
        CATKIN_DEPENDS geometry_msgs message_generation roscpp std_msgs ur_kinematics
        #DEPENDS system_lib
    )
    ...

    add_executable(ik_service src/ik_service.cpp)
    add_executable(ik_client src/ik_client.cpp)
    add_executable(test_forward src/test_forward.cpp)
    add_executable(test_inverse src/test_inverse.cpp)
    ...

    add_dependencies(ik_service ${ik_service_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    add_dependencies(ik_client ${ik_client_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    add_dependencies(test_forward ${ik_client_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    add_dependencies(test_inverse ${ik_client_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    ...

    target_link_libraries(ik_service ${catkin_LIBRARIES})
    target_link_libraries(ik_client ${catkin_LIBRARIES})
    target_link_libraries(test_forward ${catkin_LIBRARIES})
    target_link_libraries(test_inverse ${catkin_LIBRARIES})

test_forward and test_inverse can be deleted if testing was not needed. 
catkin_make to resolve any errors and warnings. 