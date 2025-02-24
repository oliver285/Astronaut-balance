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
//#include <cmath>
#include <BasicLinearAlgebra.h>
#include <math.h>

using namespace BLA;
//using namespace Eigen;
// The INPUT_A_FILTER must match the Input A filter setting in
// MSP (Advanced >> Input A, B Filtering...)
#define INPUT_A_FILTER 20
#define DEG_TO_RAD(angle) ((angle) * (M_PI / 180.0))  // Convert degrees to radians
//#include <array> // For std::array
// Defines the motor's connector as ConnectorM0
#define motor ConnectorM0

// Select the baud rate to match the target device.
#define baudRate 9600

//using namespace std;

// Defines the limit of the torque command, as a percent of the motor's peak
// torque rating (must match the value used in MSP).
double maxTorque = 100;

/*def calculate_tether_vecs(COM, teth_anchor, offset):
    # determine the tether unit vector
    teth1_vec = np.array(teth_anchor[0][:]) - (np.array(COM) - np.array(offset[0][:]))
    teth2_vec = np.array(teth_anchor[1][:]) - (np.array(COM) - np.array(offset[1][:]))
    teth3_vec = np.array(teth_anchor[2][:]) - (np.array(COM) - np.array(offset[2][:]))
    teth1_hat = teth1_vec/np.linalg.norm(teth1_vec)
    teth2_hat = teth2_vec/np.linalg.norm(teth2_vec)
    teth3_hat = teth3_vec/np.linalg.norm(teth3_vec)
    lengths = np.array([np.linalg.norm(teth1_vec),np.linalg.norm(teth2_vec),np.linalg.norm(teth3_vec)])

    return (teth1_hat, teth2_hat, teth3_hat, lengths)

*/

BLA::Matrix<3, 4, float> calculate_tether_vecs(BLA::Matrix<3,1, float> COM, BLA::Matrix<3, 3, float> teth_anchor, BLA::Matrix<3, 3, float> offset) {
    // Define tether vectors
    BLA::Matrix<3, 1, float> tethvec1, tethvec2, tethvec3;
    BLA::Matrix<3, 1, float> teth1_hat, teth2_hat, teth3_hat;

    // Compute tether vectors
    for (int i = 0; i < 3; i++) {
        tethvec1(i, 0) = teth_anchor(i, 0) - (COM(i, 0) - offset(i, 0));
        tethvec2(i, 0) = teth_anchor(i, 1) - (COM(i, 0) - offset(i, 1));
        tethvec3(i, 0) = teth_anchor(i, 2) - (COM(i, 0) - offset(i, 2));
    }

    // Normalize tether vectors to get unit vectors
    teth1_hat = tethvec1 / sqrt(tethvec1(0, 0) * tethvec1(0, 0) + tethvec1(1, 0) * tethvec1(1, 0) + tethvec1(2, 0) * tethvec1(2, 0));
    teth2_hat = tethvec2 / sqrt(tethvec2(0, 0) * tethvec2(0, 0) + tethvec2(1, 0) * tethvec2(1, 0) + tethvec2(2, 0) * tethvec2(2, 0));
    teth3_hat = tethvec3 / sqrt(tethvec3(0, 0) * tethvec3(0, 0) + tethvec3(1, 0) * tethvec3(1, 0) + tethvec3(2, 0) * tethvec3(2, 0));

    // Define result matrix (3x4)
    BLA::Matrix<3, 4, float> ANS;
    for (int i = 0; i < 3; i++) {
ANS(0,i) = teth1_hat(i);
ANS(1,i) = teth1_hat(i);
ANS(2,i) = teth1_hat(i);

    }

    ANS(0, 3) = sqrt(tethvec1(0, 0) * tethvec1(0, 0) + tethvec1(1, 0) * tethvec1(1, 0) + tethvec1(2, 0) * tethvec1(2, 0));
    ANS(1, 3) = sqrt(tethvec2(0, 0) * tethvec2(0, 0) + tethvec2(1, 0) * tethvec2(1, 0) + tethvec2(2, 0) * tethvec2(2, 0));
    ANS(2, 3) = sqrt(tethvec3(0, 0) * tethvec3(0, 0) + tethvec3(1, 0) * tethvec3(1, 0) + tethvec3(2, 0) * tethvec3(2, 0));

    return ANS;
}


/*
def calculate_tether_forces(apex, mass, teth_anchor, offset):
    # solve the system of eqns of the unit vectors to find the equations
    teth1_hat, teth2_hat, teth3_hat, _ = calculate_tether_vecs(apex, teth_anchor, offset)

    M1 = np.array([[teth1_hat[0], teth2_hat[0], teth3_hat[0]],
                   [teth1_hat[1], teth2_hat[1], teth3_hat[1]],
                   [teth1_hat[2], teth2_hat[2], teth3_hat[2]]])
    M2 = np.array([[0],[0],[mass]])
    f = np.dot(np.linalg.inv(M1), M2)
    return f

*/

BLA::Matrix<3, 1, float> calculate_tether_forces(BLA::Matrix<3,1, float> apex, int mass, BLA::Matrix<3, 3, float> teth_anchor, BLA::Matrix<3, 3, float> offset) {
    // Compute unit vectors
    BLA::Matrix<3, 4, float> Initial_Matrix = calculate_tether_vecs(apex, teth_anchor, offset);

    // Extract the first 3 columns as a 3x3 matrix
    BLA::Matrix<3, 3, float> M1;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            M1(i, j) = Initial_Matrix(i, j);
        }
    }

    // Define force matrix M2
    BLA::Matrix<3, 1, float> M2;
    M2(0, 0) = 0;
    M2(1, 0) = 0;
    M2(2, 0) = mass;

    // Solve for forces using matrix inversion
    BLA::Matrix<3, 1, float> F = Inverse(M1) * M2;  // 

    return F;  // Return the calculated tether forces
}

void testMatrixFunctions() {
    Serial.println("Running Matrix Function Tests...");

    // Define test input matrices
    BLA::Matrix<3, 1, float> COM = { 1.0, 2.0, 3.0,
                                   };

    BLA::Matrix<3, 3, float> teth_anchor = { 10.0, 11.0, 12.0,
                                             13.0, 14.0, 15.0,
                                             16.0, 17.0, 18.0 };

    BLA::Matrix<3, 3, float> offset = { 1.0, 1.0, 1.0,
                                        1.0, 1.0, 1.0,
                                        1.0, 1.0, 1.0 };

    int mass = 200; // Example mass value

    // Call calculate_tether_vecs
    Serial.println("\nTesting calculate_tether_vecs...");
    BLA::Matrix<3, 4, float> tether_vectors = calculate_tether_vecs(COM, teth_anchor, offset);

    Serial.println("Tether Vectors Result:");
    printMatrix(tether_vectors);

    // Call calculate_tether_forces
    Serial.println("\nTesting calculate_tether_forces...");
    BLA::Matrix<3, 1, float> tether_forces = calculate_tether_forces(COM, mass, teth_anchor, offset);

    Serial.println("Tether Forces Result:");
    printMatrix(tether_forces);

    Serial.println("\nMatrix Function Tests Completed.");
}

// Helper function to print matrices
template<int rows, int cols, class T>
void printMatrix(BLA::Matrix<rows, cols, T> mat) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            Serial.print(mat(i, j), 6); // Print 6 decimal places
            Serial.print("\t");
        }
        Serial.println();
    }
}


void setup() {
    // Put your setup code here, it will only run once:

    Serial.begin(9600); // Start serial communication

    while (!Serial) { } // Wait for serial connection

    testMatrixFunctions(); // Run matrix function tests


    // Sets all motor connectors to the correct mode for Follow Digital
    // Torque mode.
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,
                          Connector::CPM_MODE_A_DIRECT_B_PWM);

    // Sets up serial communication and waits up to 5 seconds for a port to open
    // Serial communication is not required for this example to run
    Serial.begin(baudRate);
    uint32_t timeout = 5000;
    uint32_t startTime = millis();
    while (!Serial && millis() - startTime < timeout) {
        continue;
    }

    // Enables the motor
    motor.EnableRequest(true);
    Serial.println("Motor Enabled");

    // Waits for HLFB to assert (waits for homing to complete if applicable)
    Serial.println("Waiting for HLFB...");
    while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
        continue;
    }
    Serial.println("Motor Ready");


}


void loop() {


float teth1_posX = analogRead(B1); // pins will be adjusted for microcontroller
float teth1_posY = analogRead(B0);
float teth1_posZ = analogRead(B1);

float teth2_posX = analogRead(B1); // pins will be adjusted for microcontroller
float teth2_posY = analogRead(B1);
float teth2_posZ = analogRead(B1);

float teth3_posX = analogRead(B1); // pins will be adjusted for microcontroller
float teth3_posY = analogRead(B1);
float teth3_posZ = analogRead(B1);

 float r = 1.0;  // Example radius value
 int mass = 200;
 BLA::Matrix<3,1,float> apex = {1,2,3}; 

    // Compute tether attachment points
    BLA::Matrix<3, 3, float> teth_anchor = { 
         2.0, 0.0, 0.0 ,  
         2.0 * cos(DEG_TO_RAD(225)), 2.0 * sin(DEG_TO_RAD(225)), 0.0 ,  
        2.0 * cos(DEG_TO_RAD(135)), 2.0 * sin(DEG_TO_RAD(135)), 0.0  
    };

    // Compute attachment offset vectors
    BLA::Matrix<3, 3, float> offset = { 
         -r, 0.0, 0.0 ,  
         -r * cos(DEG_TO_RAD(225)), -r * sin(DEG_TO_RAD(225)), 0.0 ,  
        -r * cos(DEG_TO_RAD(135)), -r * sin(DEG_TO_RAD(135)), 0.0  
    };
    // Put your main code here, it will run repeatedly:




  //BLA::Matrix<3, 4, float> tether_vectors = calculate_tether_vecs(COM, teth_anchor, offset);

  BLA::Matrix<3,1,float> forces = calculate_tether_forces( apex, mass, teth_anchor,offset);


    // Output 15% of the motor's peak torque in the positive (CCW) direction.
    CommandTorque(15);    // See below for the detailed function definition.
    // Wait 2000ms.
    delay(2000);

    CommandTorque(-75); // Output 75% peak torque in the negative (CW) direction.
    delay(2000);

    CommandTorque(5); // Output 5% peak torque in the positive (CCW) direction.
    delay(2000);

    CommandTorque(-35); // Output 35% peak torque in the negative (CW) direction.
    delay(2000);

    CommandTorque(10); // Output 10% peak torque in the positive (CCW) direction.
    delay(2000);
}

/*------------------------------------------------------------------------------
 * CommandTorque
 *
 *    Command the motor to move using a torque of commandedTorque
 *    Prints the move status to the USB serial port
 *    Returns when HLFB asserts (indicating the motor has reached the commanded
 *    torque)
 *
 * Parameters:
 *    int commandedTorque  - The torque to command
 *
 * Returns: True/False depending on whether the torque was successfully
 * commanded.
 */
 
bool CommandTorque(int commandedTorque) {
    if (abs(commandedTorque) > abs(maxTorque)) {
        Serial.println("Move rejected, invalid torque requested");
        return false;
    }
	
    Serial.print("Commanding torque: ");
    Serial.println(commandedTorque);

    // Find the scaling factor of our torque range mapped to the PWM duty cycle
    // range (255 is the max duty cycle).
    double scaleFactor = 255 / maxTorque;

    // Scale the torque command to our duty cycle range.
    int dutyRequest = abs(commandedTorque) * scaleFactor;

    // Set input A to match the direction of torque.
    if (commandedTorque < 0) {
        motor.MotorInAState(true);
    }
    else {
        motor.MotorInAState(false);
    }

    // Ensures this delay is at least 20ms longer than the Input A filter
    // setting in MSP
    delay(20 + INPUT_A_FILTER);

    // Command the move
    motor.MotorInBDuty(dutyRequest);

    // Waits for HLFB to assert (signaling the move has successfully completed)
    Serial.println("Moving... Waiting for HLFB");
    // Allow some time for HLFB to transition
    delay(1);
    while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
        continue;
    }

    Serial.println("Move Done");
    return true;
}
//------------------------------------------------------------------------------