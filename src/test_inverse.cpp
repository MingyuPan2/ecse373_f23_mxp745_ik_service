#include <iostream>
#include "ur_kinematics/ur_kin.h"

int main() {
    double X_POS = 0.1;
    double Y_POS = 0.1;
    double Z_POS = 0.1;

    double T[4][4] = {{0.0, -1.0, 0.0, X_POS}, \
                      {0.0, 0.0, 1.0, Y_POS}, \
                      {-1.0, 0.0, 0.0 , Z_POS}, \
                      {0.0, 0.0, 0.0, 1.0}};

    int num_sol;

    double q_sols[8][6];

    num_sol = ur_kinematics::inverse(&T[0][0], &q_sols[0][0], 0.0);

    std::cout << "# of solutiuons: " << num_sol << std::endl;

    for (int i = 0; i < num_sol; ++i) {
        std::cout << "Sol" << i + 1 << ":" << std::endl;
        for (int j = 0; j < 6; ++j) {
            std::cout << "Joint " << j + 1 << ": " << q_sols[i][j] << std::endl;
        }
        std::cout << std::endl;
    }

    return 0;
}
