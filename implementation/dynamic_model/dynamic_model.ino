/*
BUGS: solveSystem returning wrong deltap values (becomes NaNs)
*/









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
#include <cmath>
#include <BasicLinearAlgebra.h>
#include <math.h>
#include <Keypad.h>

// The INPUT_A_FILTER must match the Input A filter setting in
// MSP (Advanced >> Input A, B Filtering...)

// Defines the motor's connector as ConnectorM0
#define ioPort ConnectorUsb

// algorithm function headers
BLA::Matrix<3, 1, float> equations(BLA::Matrix<3,1, float> p, BLA::Matrix<3, 3, float> teth_anchor, BLA::Matrix<3, 3, float> offset,BLA::Matrix<3,1, float> tether_lengths);
BLA::Matrix<3, 3, float> jacobian(BLA::Matrix<3,1, float> p,BLA::Matrix<3,1, float> tether_lengths,BLA::Matrix<3, 3, float> teth_anchor,BLA::Matrix<3, 3, float> offset);
void swap(float& a, float& b);
BLA::Matrix<3, 1, float> solveSystem(BLA::Matrix<3, 3, float> A, BLA::Matrix<3, 1, float> b);
BLA::Matrix<3, 1, float> newtonSolve(BLA::Matrix<3,1, float> p,BLA::Matrix<3,1, float> tether_lengths,BLA::Matrix<3, 3, float> teth_anchor,BLA::Matrix<3, 3, float> offset);
BLA::Matrix<3, 4, float> calculate_tether_vecs(BLA::Matrix<3,1, float> COM, BLA::Matrix<3, 3, float> teth_anchor, BLA::Matrix<3, 3, float> offset);
BLA::Matrix<3, 1, float> calculate_tether_forces(BLA::Matrix<3,1, float> apex, int mass, BLA::Matrix<3, 3, float> teth_anchor, BLA::Matrix<3, 3, float> offset);

using namespace BLA;
//using namespace Eigen;
// The INPUT_A_FILTER must match the Input A filter setting in
// MSP (Advanced >> Input A, B Filtering...)
#define INPUT_A_FILTER 20
#define DEG_TO_RAD(angle) ((angle) * (M_PI / 180.0))  // Convert degrees to radians
//#include <array> // For std::array
// Defines the motor's connector as ConnectorM0
#define motor ConnectorM0

#define Maxiterations 50
#define TOL 1e-6 
// Select the baud rate to match the target device.
#define baudRate 9600

//using namespace std;

// Defines the limit of the torque command, as a percent of the motor's peak
// torque rating (must match the value used in MSP).
double maxTorque = 100;

// temporary may be used later
#define IN_BUFFER_LEN 32
double input[IN_BUFFER_LEN+1];
//float temp[IN_BUFFER_LEN+1];
double t_init = 0;


// helper function to print matrices
template<int rows, int cols, typename T>
void printMatrix(const BLA::Matrix<rows, cols, T>& mat) {
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

    Serial.begin(115200); // Start serial communication

    while (!Serial) { } // Wait for serial connection

    // Sets up serial communication and waits up to 5 seconds for a port to open
    // Serial communication is not required for this example to run
    Serial.begin(baudRate);
    uint32_t timeout = 5000;
    uint32_t startTime = millis();
    while (!Serial && millis() - startTime < timeout) {
        continue;
    }










    // temp
      float r = 1.0;  // Example radius value
  int mass = 200;
  //  BLA::Matrix<3,1,float> apex = {1,2,3}; 
  BLA::Matrix<3, 1, float> lengths = {
    5.0,
    5.0,
    5.0
  };
  BLA::Matrix<3, 1, float> p = {2.0,2.0,-4.0};
  BLA::Matrix<3, 1, float> apex;
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

  // int i = 0;
  // strcpy(temp, input);


  // while(i<3 && ioPort.CharPeek() != -1){
  // Serial.println("Enter tether length " + String(i) + "\n");

  //   input[i] = (float) ioPort.CharGet();
  //   i++;
  //   // valid_input_flag
  // Delay_ms(1);
  // }

  // lengths(1) = input[1];
  // lengths(2) = input[2];
  // lengths(3) = input[3];


  Serial.print("-----------------------------------------------------------------------------------------------\n");
  apex =newtonSolve( p, lengths, teth_anchor, offset);
  Serial.println("Apex : " + String(apex(0)) + ", " + String(apex(1)) + ", " + String(apex(2)) + "\n");
}


void loop() {
  // // float teth1_posX = analogRead(B1); // pins will be adjusted for microcontroller
  // // float teth1_posY = analogRead(B0);
  // // float teth1_posZ = analogRead(B1);

  // // float teth2_posX = analogRead(B1); // pins will be adjusted for microcontroller
  // // float teth2_posY = analogRead(B1);
  // // float teth2_posZ = analogRead(B1);

  // // float teth3_posX = analogRead(B1); // pins will be adjusted for microcontroller
  // // float teth3_posY = analogRead(B1);
  // // float teth3_posZ = analogRead(B1);

  // float r = 1.0;  // Example radius value
  // int mass = 200;
  // //  BLA::Matrix<3,1,float> apex = {1,2,3}; 
  // BLA::Matrix<3, 1, float> lengths = {
  //   5.0,
  //   5.0,
  //   5.0
  // };
  // BLA::Matrix<3, 1, float> p = {2.0,2.0,-4.0};
  // BLA::Matrix<3, 1, float> apex;
  // // Compute tether attachment points
  // BLA::Matrix<3, 3, float> teth_anchor = { 
  //     2.0, 0.0, 0.0 ,  
  //     2.0 * cos(DEG_TO_RAD(225)), 2.0 * sin(DEG_TO_RAD(225)), 0.0 ,  
  //     2.0 * cos(DEG_TO_RAD(135)), 2.0 * sin(DEG_TO_RAD(135)), 0.0  
  // };

  // // Compute attachment offset vectors
  // BLA::Matrix<3, 3, float> offset = { 
  //     -r, 0.0, 0.0 ,  
  //     -r * cos(DEG_TO_RAD(225)), -r * sin(DEG_TO_RAD(225)), 0.0 ,  
  //     -r * cos(DEG_TO_RAD(135)), -r * sin(DEG_TO_RAD(135)), 0.0  
  // };
  // // Put your main code here, it will run repeatedly:

  // // int i = 0;
  // // strcpy(temp, input);


  // // while(i<3 && ioPort.CharPeek() != -1){
  // // Serial.println("Enter tether length " + String(i) + "\n");

  // //   input[i] = (float) ioPort.CharGet();
  // //   i++;
  // //   // valid_input_flag
  // // Delay_ms(1);
  // // }

  // // lengths(1) = input[1];
  // // lengths(2) = input[2];
  // // lengths(3) = input[3];



  // apex =newtonSolve( p, lengths, teth_anchor, offset);

  // Serial.println("lengths : " + String(lengths(0)) + ", " + String(lengths(1)) + ", " + String(lengths(2)) + "\n");
  // Serial.println("Apex : " + String(apex(0)) + ", " + String(apex(1)) + ", " + String(apex(2)) + "\n");
  // // delay(5000);


  // //BLA::Matrix<3, 4, float> tether_vectors = calculate_tether_vecs(COM, teth_anchor, offset);

  // //BLA::Matrix<3,1,float> forces = calculate_tether_forces( apex, mass, teth_anchor,offset);


  // // // Output 15% of the motor's peak torque in the positive (CCW) direction.
  // // CommandTorque(15);    // See below for the detailed function definition.
  // // // Wait 2000ms.
  // // delay(2000);

  // // CommandTorque(-75); // Output 75% peak torque in the negative (CW) direction.
  // // delay(2000);

  // // CommandTorque(5); // Output 5% peak torque in the positive (CCW) direction.
  // // delay(2000);

  // // CommandTorque(-35); // Output 35% peak torque in the negative (CW) direction.
  // // delay(2000);

  // // CommandTorque(10); // Output 10% peak torque in the positive (CCW) direction.
  // // delay(2000);
}




// returns equations for the apex 
BLA::Matrix<3, 1, float> equations(BLA::Matrix<3,1, float> p, BLA::Matrix<3, 3, float> teth_anchor, BLA::Matrix<3, 3, float> offset, BLA::Matrix<3,1, float> tether_lengths){
  double x = p(0), y = p(1), z = p(2);

  return {
    pow(x - (teth_anchor(0,0) + offset(0,0)), 2) + 
    pow(y - (teth_anchor(0,1) + offset(0,1)), 2) + 
    pow(z - (teth_anchor(0,2) + offset(0,2)), 2) - tether_lengths(0) * tether_lengths(0),

    pow(x - (teth_anchor(1,0) + offset(1,0)), 2) + 
    pow(y - (teth_anchor(1,1) + offset(1,1)), 2) + 
    pow(z - (teth_anchor(1,2) + offset(1,2)), 2) - tether_lengths(1) * tether_lengths(1),

    pow(x - (teth_anchor(2,0) + offset(2,0)), 2) + 
    pow(y - (teth_anchor(2,1) + offset(2,1)), 2) + 
    pow(z - (teth_anchor(2,2) + offset(2,2)), 2) - tether_lengths(2) * tether_lengths(2)
  };  
}



// Function to compute the Jacobian matrix using finite differences
BLA::Matrix<3, 3, float> jacobian(BLA::Matrix<3,1, float> p,BLA::Matrix<3,1, float> tether_lengths,BLA::Matrix<3, 3, float> teth_anchor,BLA::Matrix<3, 3, float> offset){

  BLA::Matrix<3, 3, float> J;
  double h = 1e-5;
  BLA::Matrix<3,1, float> p1;
  BLA::Matrix<3,1, float> p2;
  BLA::Matrix<3,1, float> f1;
  BLA::Matrix<3,1, float> f2;
  for(int i=0;i<3;i++){
    p1(i)= p(i)+h;
    p2(i)=p(i)-h;
    f1 = equations(p1,teth_anchor, offset,tether_lengths);
    f2 = equations(p2,teth_anchor, offset,tether_lengths);

    for (int j = 0; j < 3; j++) {
      J(j,i) = (f1(j)-f2(j))/(2*h); 
    }
  }
  return J;
}

// swap rows
void swap(float& a, float& b) {
    float temp = a;
    a = b;
    b = temp;
}

// solve system by doing things
BLA::Matrix<3, 1, float> solveSystem(BLA::Matrix<3, 3, float> A, BLA::Matrix<3, 1, float> b) {
  BLA::Matrix<3, 1, float> x;
  int maxRow;
  double factor;
  for(int i=0;i<3;i++){
    maxRow= i;
    for(int k=i+1;k<3;k++){
      if (fabs(A(k,i)) > fabs(A(maxRow,i))) {
        maxRow = k;
      }
    }

    swap(A(i), A(maxRow));
    swap(b(i), b(maxRow));

    if (fabs(A(i,i)) < 1e-10) {
        Serial.println("Jacobian is singular, Newton's method failed.");
    } // very unlikely with set up but can test if something goes wrong

    // Forward elimination
    for (int k = i + 1; k < 3; k++) {
      factor = A(k,i) / A(i,i);
      for (int j = i; j < 3; j++) {
        A(k,j) -= factor * A(i,j);
      }
      b(k) -= factor * b(i);
    }
  }

  // Back substitution

  for (int i = 3 - 1; i >= 0; i--) {
    x(i) = b(i);
    for (int j = i + 1; j < 3; j++) {
      x(i) -= A(i,j) * x(j);  
    }
    x(i) /= A(i,i);
  }

  return x;

}



// solving non linear system to get apex
BLA::Matrix<3, 1, float> newtonSolve(BLA::Matrix<3,1, float> p,BLA::Matrix<3,1, float> tether_lengths,BLA::Matrix<3, 3, float> teth_anchor,BLA::Matrix<3, 3, float> offset){
  BLA::Matrix<3, 1, float> F;
  BLA::Matrix<3, 1, float> delta_p;
  BLA::Matrix<3, 3, float> J;
  double norm;
  for(int iter=0;iter<Maxiterations;iter++){
    F = equations( p,  teth_anchor, offset, tether_lengths);
    // printMatrix(F);
    J= jacobian(p,tether_lengths,teth_anchor, offset);
    // Solve J * delta_p = -F
    delta_p = solveSystem(J, -F);

    // Serial.println("p:");
    // printMatrix(p);
    Serial.println("delta_p");
    printMatrix(delta_p);

    //update solution
    for (int i = 0; i < 3; i++) {
      p(i) += delta_p(i);
    }

    // Check for convergence
    norm = sqrt(delta_p(0) * delta_p(0) + delta_p(1) * delta_p(1) + delta_p(2) * delta_p(2));
    if (norm < TOL) {
        return p;
    }
  }
  //  Serial.println("Estimate : " + String(p(0)) + ", " + String(p(1)) + ", " + String(p(2)) + "\n");

}


// find tether unit vectors
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

// calculae tether forces given apex, anchor locations, and mass
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


