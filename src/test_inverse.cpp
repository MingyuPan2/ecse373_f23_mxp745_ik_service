#include <iostream>
#include "ur_kinematics/ur_kin.h"

int main() {                            
    double X_POS = 0.1;                 //given x, y, z position with respect to the arm1_base_link
    double Y_POS = 0.2;                 //change to desired value
    double Z_POS = 0.1;

    double T[4][4] = {{0.0, -1.0, 0.0, X_POS},    //2d definition,gripper facing down
                      {0.0, 0.0, 1.0, Y_POS},
                      {-1.0, 0.0, 0.0 , Z_POS},
                      {0.0, 0.0, 0.0, 1.0}};

    int num_sol;                                    // num_sol = # of solutions variable

    double q_sols[8][6];                            //allocate space for up to eight solutions of six joint angles

    num_sol = ur_kinematics::inverse(&T[0][0], &q_sols[0][0], 0.0); //IK solutionss

    std::cout << "# of solutiuons: " << num_sol << std::endl;

    for (int i = 0; i < num_sol; ++i) {                                 //for a given number of solutions
        std::cout << "Sol" << i + 1 << ":" << std::endl;                //for each solution (up to 8)
        for (int j = 0; j < 6; ++j) {                                   //for each joint, from joint 1 to joint 6
            std::cout << "Joint " << j + 1 << ": " << q_sols[i][j] << std::endl;    //q=8*6 of doubles of solutions. Find the i-th solution's j-th member.
        }
        std::cout << std::endl;
    }

    return 0;
}
