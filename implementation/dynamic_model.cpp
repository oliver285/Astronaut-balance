/*
 * Title: FollowDigitalTorque
 *
 * Objective:
 *    This example demonstrates control of the ClearPath-MC operational mode
 *    Follow Digital Torque Command, Unipolar PWM Command.
 *
 * Description:
 *    This example enables and then commands a ClearPath motor to output various
 *    torques. in both the clockwise and counterclockwise directions.
 *    During operation, various move statuses are written to the USB serial
 *    port.
 *
 * Requirements:
 * 1. A ClearPath motor must be connected to Connector M-0.
 * 2. The connected ClearPath motor must be configured through the MSP software
 *    for Follow Digital Torque Command, Unipolar PWM Command mode  (In MSP
 *    select Mode>>Torque>>Follow Digital Torque Command, then with
 *    "Unipolar PWM Command" selected hit the OK button).
 * 3. The ClearPath motor must be set to use the HLFB mode "ASG-Torque"
 *    through the MSP software (select Advanced>>High Level Feedback [Mode]...
 *    then choose "All Systems Go (ASG) - Torque" from the dropdown and hit
 *    the OK button).
 * 4. The ClearPath must have a defined Max Torque Command configured through
 *    the MSP software. This value must match the "maxTorque" variable defined
 *    below.
 * 5. Ensure the Input A filter in MSP is set to 20ms (In MSP select
 *    Advanced>>Input A, B Filtering... then in the Settings box fill in the
 *    textbox labeled "Input A Filter Time Constant (msec)" then hit the OK
 *    button).
 * 6. Set your Max Speed and Over-Speed Timeout in MSP according to your
 *    mechanical system. Note you may notice any of the following if this max
 *    speed is reached: motor shutdown, speed limit cycling at the max speed,
 *    HLFB not signaling ASG/move done.
 * 7. Ensure the "Invert PWM Input" box is unchecked in MSP.
 *
 * Links:
 * ** ClearCore Documentation: https://teknic-inc.github.io/ClearCore-library/
 * ** ClearCore Manual: https://www.teknic.com/files/downloads/clearcore_user_manual.pdf
 * ** ClearPath Manual (DC Power): https://www.teknic.com/files/downloads/clearpath_user_manual.pdf
 * ** ClearPath Manual (AC Power): https://www.teknic.com/files/downloads/ac_clearpath-mc-sd_manual.pdf
 *
 *
 * Copyright (c) 2020 Teknic Inc. This work is free to use, copy and distribute under the terms of
 * the standard MIT permissive software license which can be found at https://opensource.org/licenses/MIT
 */

#include "ClearCore.h"
#include"/BasicLinearAlgebra/BasicLinearAlgebra.h"
#include <cmath>
//#include <Eigen/Dense>
// The INPUT_A_FILTER must match the Input A filter setting in
// MSP (Advanced >> Input A, B Filtering...)
#define INPUT_A_FILTER 20
#include <array> // For std::array
// Defines the motor's connector as ConnectorM0
#define motor ConnectorM0

// Select the baud rate to match the target device.
#define baudRate 9600

using namespace std;

// Defines the limit of the torque command, as a percent of the motor's peak
// torque rating (must match the value used in MSP).
double maxTorque = 100;

/*def equations(p, a, b, c, r):
    x, y, z = p
    return (
        x**2 + (y-(2-r))**2 + z**2 - a**2,
        (x-(2-r)*np.cos(210*np.pi/180))**2 + (y-(2-r)*np.sin(210*np.pi/180))**2 + z**2 - b**2,
        (x-(2-r)*np.cos(330*np.pi/180))**2 + (y-(2-r)*np.sin(330*np.pi/180))**2 + z**2 - c**2
    )
*/



array<double, 3> equations(const double p[], double a, double b, double c, double r) {
    const double pi = M_PI; // Use M_PI from <cmath>, or define it if unavailable
    double eq1 = pow(p[0], 2) + pow(p[1] - (2 - r), 2) + pow(p[2], 2) - pow(a, 2);
    double eq2 = pow(p[0] - (2 - r) * cos(210 * pi / 180), 2) +
                 pow(p[1] - (2 - r) * sin(210 * pi / 180), 2) +
                 pow(p[2], 2) - pow(b, 2);
    double eq3 = pow(p[0] - (2 - r) * cos(330 * pi / 180), 2) +
                 pow(p[1] - (2 - r) * sin(330 * pi / 180), 2) +
                 pow(p[2], 2) - pow(c, 2);

    return {eq1, eq2, eq3}; // Return as a std::array
}
double calculate_apex(const double p[], double a, double b, double c, double r){
    int initial_guess[2] = {2, 2, 4};//  # Start with a point above the base
    double apex = fsolve(equations, initial_guess, args=(a, b, c, r)); // left out andrew will need to define how he wants this to work
    return apex;
}


// Norm of a 3D vector
float norm(const float vec[3]) {
    return sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
}

// Vector subtraction
void subtract(const float a[3], const float b[3], float result[3]) {
    result[0] = a[0] - b[0];
    result[1] = a[1] - b[1];
    result[2] = a[2] - b[2];
}

// Inverse of a 3x3 matrix
bool inverse(const float mat[3][3], float result[3][3]) {
    float det = mat[0][0] * (mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1]) -
                mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0]) +
                mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);

    if (fabs(det) < 1e-6) {
        return false; // Matrix is singular
    }

    float invDet = 1.0 / det;

    result[0][0] = invDet * (mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1]);
    result[0][1] = invDet * (mat[0][2] * mat[2][1] - mat[0][1] * mat[2][2]);
    result[0][2] = invDet * (mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1]);

    result[1][0] = invDet * (mat[1][2] * mat[2][0] - mat[1][0] * mat[2][2]);
    result[1][1] = invDet * (mat[0][0] * mat[2][2] - mat[0][2] * mat[2][0]);
    result[1][2] = invDet * (mat[0][2] * mat[1][0] - mat[0][0] * mat[1][2]);

    result[2][0] = invDet * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
    result[2][1] = invDet * (mat[0][1] * mat[2][0] - mat[0][0] * mat[2][1]);
    result[2][2] = invDet * (mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0]);

    return true;
}

// Calculate tether forces
bool calculate_tether_forces(const float apex[3], float mass, float r, float forces[3]) {
    const float pi = 3.14159;

    // Tether vectors
    float teth1_vec[3], teth2_vec[3], teth3_vec[3];

 //   subtract(apex, (float[]){0, 2 - r, 0}, teth1_vec);
teth1_vec[0]=apex[0];
teth1_vec[1]=apex[1]-(2-r);
teth1_vec[2]=apex[2];

 //   subtract(apex, (float[]){(2 - r) * cos(210 * pi / 180), (2 - r) * sin(210 * pi / 180), 0}, teth2_vec);
    teth2_vec[0]=apex[0]- (2 - r) * cos(210 * pi / 180); 
teth2_vec[1]=apex[1]-(2-r)-(2 - r) * sin(210 * pi / 180);
teth2_vec[2]=apex[2];
   // subtract(apex, (float[]){(2 - r) * cos(330 * pi / 180), (2 - r) * sin(330 * pi / 180), 0}, teth3_vec);
  teth3_vec[0]=apex[0]- (2 - r) * cos(330 * pi / 180); 
teth3_vec[1]=apex[1]-(2-r)-(2 - r) * sin(330 * pi / 180);
teth3_vec[2]=apex[2];
    // Normalized vectors
    float teth1_hat[3] = {teth1_vec[0] / norm(teth1_vec), teth1_vec[1] / norm(teth1_vec), teth1_vec[2] / norm(teth1_vec)};
    float teth2_hat[3] = {teth2_vec[0] / norm(teth2_vec), teth2_vec[1] / norm(teth2_vec), teth2_vec[2] / norm(teth2_vec)};
    float teth3_hat[3] = {teth3_vec[0] / norm(teth3_vec), teth3_vec[1] / norm(teth3_vec), teth3_vec[2] / norm(teth3_vec)};

    // Matrix of normalized vectors
    float M1[3][3] = {
        {teth1_hat[0], teth2_hat[0], teth3_hat[0]},
        {teth1_hat[1], teth2_hat[1], teth3_hat[1]},
        {teth1_hat[2], teth2_hat[2], teth3_hat[2]}
    };

    // Force vector
    float M2[3] = {0, 0, mass};

    // Invert the matrix
    float M1_inv[3][3];
    if (!inverse(M1, M1_inv)) {
        return false; // Matrix inversion failed
    }

    // Multiply M1_inv by M2 to calculate forces
    for (int i = 0; i < 3; i++) {
        forces[i] = 0;
        for (int j = 0; j < 3; j++) {
            forces[i] += M1_inv[i][j] * M2[j];
        }
    }

    return true;
}

// Main loop


float dot_product(const float a[3], const float b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

// Calculate tether error
void calculate_tether_error(const float COM[3], const float f[3], float mass, float r,
                            float &f_err, float &angle_err, float teth1_vec[3], float teth2_vec[3], float teth3_vec[3]) {
    const float pi = 3.14159;

    // Compute tether attachment locations
    float teth1_attach_loc[3] = {COM[0], COM[1] + r, COM[2]};
    float teth2_attach_loc[3] = {COM[0] + r * cos(210 * pi / 180), COM[1] + r * sin(210 * pi / 180), COM[2]};
    float teth3_attach_loc[3] = {COM[0] + r * cos(330 * pi / 180), COM[1] + r * sin(330 * pi / 180), COM[2]};

    // Compute tether initial vectors
    float teth1_init_vec[3] = {0 - teth1_attach_loc[0], 2 - teth1_attach_loc[1], 0 - teth1_attach_loc[2]};
    float teth2_init_vec[3] = {2 * cos(210 * pi / 180) - teth2_attach_loc[0], 
                               2 * sin(210 * pi / 180) - teth2_attach_loc[1], 
                               0 - teth2_attach_loc[2]};
    float teth3_init_vec[3] = {2 * cos(330 * pi / 180) - teth3_attach_loc[0], 
                               2 * sin(330 * pi / 180) - teth3_attach_loc[1], 
                               0 - teth3_attach_loc[2]};

    // Normalize tether initial vectors
    float teth1_norm = norm(teth1_init_vec);
    float teth2_norm = norm(teth2_init_vec);
    float teth3_norm = norm(teth3_init_vec);

    // Compute tether force vectors
    for (int i = 0; i < 3; i++) {
        teth1_vec[i] = f[0] * (teth1_init_vec[i] / teth1_norm);
        teth2_vec[i] = f[1] * (teth2_init_vec[i] / teth2_norm);
        teth3_vec[i] = f[2] * (teth3_init_vec[i] / teth3_norm);
    }

    // Compute resulting force vector
    float f_vec[3] = {
        teth1_vec[0] + teth2_vec[0] + teth3_vec[0],
        teth1_vec[1] + teth2_vec[1] + teth3_vec[1],
        teth1_vec[2] + teth2_vec[2] + teth3_vec[2]
    };

    // Expected force vector
    float expected_f_vec[3] = {0, 0, -mass};

    // Compute errors
    float f_vec_norm = norm(f_vec);
    float expected_f_vec_norm = norm(expected_f_vec);

    if (f_vec_norm > 0 && expected_f_vec_norm > 0) {
        angle_err = acos(dot_product(expected_f_vec, f_vec) / (expected_f_vec_norm * f_vec_norm)) * (180 / pi);
    } else {
        angle_err = 0;
    }

    f_err = fabs(expected_f_vec_norm - f_vec_norm);
}