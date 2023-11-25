#include <iostream>
#include "ur_kinematics/ur_kin.h"

int main() {
    double q[] = {3.14, -1.13, 1.51, 3.77, -1.51, 0};

    double T[4][4];

    ur_kinematics::forward((double *)&q[0], (double *)&T[0][0]);

    std::cout << "FK:" << std::endl;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            std::cout << T[i][j] << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
