#include "ur_kinematics/ur_kin.h"
#include "ros/ros.h"
#include <cstdlib>
#include "ik_service/PoseIK.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ik_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<ik_service::PoseIK>("pose_ik");

    ik_service::PoseIK ik_pose;
    geometry_msgs::Pose part_pose;

    part_pose.position.x = 0.5;
    ik_pose.request.part_pose = part_pose;

    if (client.call(ik_pose))
    {
        ROS_INFO("Call to ik_service returned [%d] solutions", ik_pose.response.num_sols);

        for (int i = 0; i < ik_pose.response.num_sols; ++i) {
            ROS_INFO("Set of joint angles %d:", i + 1);
            for (int j = 0; j < 6; ++j) {
                ROS_INFO("Joint %d: %f", j + 1, ik_pose.response.joint_solutions[i].joint_angles[j]);
            }
        }

    }
    else
    {
        ROS_ERROR("Failed to call ik_service");
        return 1;
    }

    return 0;
}
