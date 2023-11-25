#include <iostream>
#include "ur_kinematics/ur_kin.h"

int main() {
    double q[] = {3.14, -1.13, 1.51, 3.77, -1.51, 0}; //for fk was given these joint angles

    double T[4][4];                                   //variable to receive forward kinematic solution

    ur_kinematics::forward((double *)&q[0], (double *)&T[0][0]);    //fk solution

    std::cout << "FK:" << std::endl;
    for (int i = 0; i < 4; ++i) {                       //for T is the 4x4 end effector pose in row-major ordering, using a for loop
        for (int j = 0; j < 4; ++j) {
            std::cout << T[i][j] << " ";                // 0->0 1 2 3, 1->0 1 2 3, 2->0 1 2 3, 3->0 1 2 3
        }
        std::cout << std::endl;                         //new line after 4
    }

    return 0;
}
