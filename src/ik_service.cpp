#include "ur_kinematics/ur_kin.h"
#include "ros/ros.h"
#include "ik_service/PoseIK.h"            //definition of srv type part_pose -- num_sols joint_solutions

bool pose_ik(ik_service::PoseIK::Request &req, ik_service::PoseIK::Response &res) //the service function
{
    double X_POS = req.part_pose.position.x;  //position is a member of partpose is a member of req, part_pose is of type geometry_msgs::Pose
    double Y_POS = req.part_pose.position.y;
    double Z_POS = req.part_pose.position.z;

    double T[4][4] = {{0.0, -1.0, 0.0, X_POS},  // gripper facing down
                      {0.0, 0.0, 1.0, Y_POS},
                      {-1.0, 0.0, 0.0, Z_POS},
                      {0.0, 0.0, 0.0, 1.0}};

    double q_sols[8][6];                        //allocate space for up to eight solutions of six joint angles
    int num_sols;                               //variable to receive the number of solutions returned

    num_sols = ur_kinematics::inverse(&T[0][0], &q_sols[0][0], 0.0); //calling ur_kinematics::inverse function, interpreted a pointer to type double, 
                                                                    //T=transformation matrix, q to store sol array
    res.num_sols = num_sols;                    //in response message, assigns value of num_sols to num_sols field

     for (int i = 0; i < num_sols; ++i) {       //for all calculated solutions
        ik_service::JointSolutions joint_solution;  //based on poseik to store values in a loop of joint solutions
        for (int j = 0; j < 6; ++j) {           //for each of 6 joints
            //joint_solution.joint_angles[j] = q_sols[i][j];  //from each element of q_sols, assign each joint angle of each solution to joint angles in joint_solutions
            res.joint_solutions[i].joint_angles[j] = q_sols[i][j]; //in the ik_service::PoseIK::Request message req, assign joint values to the joint angles to joint solutions
        }
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ik_service");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("pose_ik", pose_ik);
    ROS_INFO("Ready for requests");
    ros::spin();

    return 0;
}
