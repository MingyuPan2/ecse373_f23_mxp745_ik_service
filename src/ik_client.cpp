#include "ur_kinematics/ur_kin.h"
#include "ros/ros.h"
#include <cstdlib>
#include "ik_service/PoseIK.h"                //definition of srv type part_pose -- num_sols joint_solutions

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ik_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<ik_service::PoseIK>("pose_ik");

    ik_service::PoseIK ik_pose;               //declare ik_service::PoseIK variable as ik_pose
    geometry_msgs::Pose part_pose;            //declare geometry_msgs::Pose variable as part_pose

    part_pose.position.x = 0.5;               //ini x
    ik_pose.request.part_pose = part_pose;    //assign part_pose variable to part_pose in the ik_pose object (object of type ik_service::PoseIK)

    if (client.call(ik_pose))                 //call to send request to service, true if a response is received
    {
        ROS_INFO("Calling ik_service gave %d solutions.", ik_pose.response.num_sols); //access num_sol after receiving response

        for (int i = 0; i < ik_pose.response.num_sols; ++i) {   //for all sets of joint solutions
            ROS_INFO("For solution set number %d :", i + 1);         //for each set of joint solutions
            for (int j = 0; j < 6; ++j) {                       //for each joints of each set of joint solutions
                ROS_INFO("Joint %d = %f", j + 1, ik_pose.response.joint_solutions[i].joint_angles[j]); //ith solution's jth angle
            }
        }

    }

    return 0;
}
