// TODO: when hitting errors and returning tp base state, the chracter array input[0] has to be changed or else the jump will not be acknowledged,
//  potentially change this to allow the int input1 to be changed and not be overwritten by input array (or not, might be for the better)

// TODO: motor 1 inverted (marked with x physucally)
// TODO: input a state should not be necessary to check, since we are torquing in one direction will only need one direction and not allow torque commands in the other direction.


/*
 * Objective:
 *    Develop the data pipeline for each motor in the system, the force determination will occur on the arduino sketch itself for this, so inputs 2,3,4 will be angle/lengths 
 *    instead of force commands
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
 */

#include "ClearCore.h"
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <BasicLinearAlgebra.h>
#include <math.h>
// weird shit going on with class between preprocessor min/max macros and std min/max (undefine both of them)
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#include <vector>
#include <string>

// The INPUT_A_FILTER must match the Input A filter setting in
// MSP (Advanced >> Input A, B Filtering...)
#define INPUT_A_FILTER 20

// Defines the motor connectors
#define ioPort ConnectorUsb
#define motor1 ConnectorM0
#define motor2 ConnectorM1
#define motor3 ConnectorM2

// Select the baud rate and io port baud rate to match the target device.
#define baudRate 115200
#define ioPortBaudRate  115200

// define number of motors being used
int num_motors = 2;

// Defines the limit of the torque command, as a percent of the motor's peak
// torque rating (must match the value used in MSP).
double maxTorque = 100;

// define serial buffer length and character input and float_input arrays
#define IN_BUFFER_LEN 32
char input[IN_BUFFER_LEN+1];
std::vector<float> float_inputs(4);

// temp ensures we don't jump from state 1 to state 3, input1 is the state command
int prev_input1, input1 = 0;
double input2, input3, input4; // torque/force command

// defines start of test (entering state3)
double t_init = 0;

// flag to tell input 3 to normalize millis()
bool test_start = false;


using namespace BLA;


// algorithm function headers
BLA::Matrix<3, 1, float> equations(BLA::Matrix<3,1, float> p, BLA::Matrix<3, 3, float> teth_anchor, BLA::Matrix<3, 3, float> offset,BLA::Matrix<3,1, float> tether_lengths);
BLA::Matrix<3, 3, float> jacobian(BLA::Matrix<3,1, float> p,BLA::Matrix<3,1, float> tether_lengths,BLA::Matrix<3, 3, float> teth_anchor,BLA::Matrix<3, 3, float> offset);
void swap(float& a, float& b);
BLA::Matrix<3, 1, float> solveSystem(BLA::Matrix<3, 3, float> A, BLA::Matrix<3, 1, float> b);
BLA::Matrix<3, 1, float> newtonSolve(BLA::Matrix<3,1, float> p,BLA::Matrix<3,1, float> tether_lengths,BLA::Matrix<3, 3, float> teth_anchor,BLA::Matrix<3, 3, float> offset);
BLA::Matrix<3, 3, float> calculate_tether_vecs(BLA::Matrix<3,1, float> COM, BLA::Matrix<3, 3, float> teth_anchor, BLA::Matrix<3, 3, float> offset);
BLA::Matrix<3, 1, float> calculate_tether_forces(BLA::Matrix<3,1, float> apex, int mass, BLA::Matrix<3, 3, float> teth_anchor, BLA::Matrix<3, 3, float> offset);
#define Maxiterations 500
#define TOL 1e-3 
// end of dynamic model functions



// Declares helper functions
void getNumMotors();
void multipleMotorEnable(bool request);
void checkMotorAState(const std::vector<float>& commandedTorque);
bool CommandTorque(int commandedTorque);


void setup() {
    // Put your setup code here, it will only run once:
    // set digital and analog pins as inputs //
    

    // Sets all motor connectors to the correct mode for Follow Digital
    // Torque mode.
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,
                          Connector::CPM_MODE_A_DIRECT_B_PWM);

    // Sets up serial communication and waits up to 5 seconds for a port to open
    Serial.begin(baudRate);
    uint32_t timeout = 5000;
    uint32_t startTime = millis();
    while (!Serial && millis() - startTime < timeout) {
        continue;
    }

    // ask for number of motors
    getNumMotors();

    // set input1 to state 1
    input[0] = '1';

    // set up motors to start loop disabled
    multipleMotorEnable(false);
    Serial.println(String(num_motors) + " Motor(s) Ready");
}


void loop() {
    // Put your main code here, it will run repeatedly:
  
    // read serial port
    int i = 0;
    char rc;
    char endMarker = '\n';
    prev_input1 = input1;
    // serial port messages in are currently of the form: state_input, input1, input2, input3
    while (Serial.available() > 0) {
      rc = Serial.read();
      if (rc != endMarker) {
        input[i] = rc;
        i++;
      }
      else{
        input[i] = '\0'; // terminate the string
        i = 0;
      }
    }

    // inputs converted to floats
    i = 0;
    char input_temp[IN_BUFFER_LEN+1];
    strcpy(input_temp, input);

    char *token = strtok(input_temp, ",");
    while(token != nullptr){
      if(i > 4){
        Serial.println("to many serial inputs");
        // if command greater than 4 inputs, send idle state with zero forces
        float_inputs = {1,0,0,0};
        break;
        }
        float_inputs[i] = atof(token);
        token = strtok(nullptr, ",");
        i++;
    }

    input1 = float_inputs[0];
    input2 = float_inputs[1];
    input3 = float_inputs[2];
    input4 = float_inputs[3];


    switch(input1){
      case 1:
        {
          test_start = false;
          // Serial.print("state 1 \n");

          // disable motors
          if(motor1.EnableRequest()){
            multipleMotorEnable(false);
            Serial.print("motor(s) disabled \n");
          }
          break;
        }

      case 2:
        {
          test_start = false;
          // Serial.print("state 2 \n");
          
          // enable motors but dont print data to serial port
          if(!motor1.EnableRequest()){
            multipleMotorEnable(true);
            Serial.print("motor(s) enabled \n");
          }

          // TODO: currently inverted
          // TODO: need force to torque conversion here
          std::vector<float> currCommand(3);
          currCommand = {maxTorque - input2, input3, maxTorque - input4};;
          CommandTorque(currCommand);    // See below for the detailed function definition.
          // Wait 2000ms.

          break;
        }

      case 3:
        {
          // cannot jump directly to state 3 from 1
          if (prev_input1 == 1){
            Serial.print("Invalid jump \n");
            input[0] = '1';
            break;
          }

          // start millis at zero
          if(!test_start){
            t_init = millis();
            test_start = true;
          }

          // TODO: currently inverted
          // TODO: need force to torque conversion here
          std::vector<float> currCommand(3);
          currCommand = {maxTorque - input2, input3, maxTorque - input4};
          CommandTorque(currCommand);    // See below for the detailed function definition.
          // currently prints inputs, will prints load cell force outputs
          Serial.println(String(millis() - t_init, 4) + ", " + String(input2) + ", " + String(input3) + ", " + String(input4));
          delay(200);

          break;
        }

      default:
        // currently sets state back to one if an invalid input is received
        Serial.print("Invalid input \n");
        input[0] = '1';
        break;
    }

    // TODO: temporary delay
    delay(200);
}

void getNumMotors(){
  // continue receiving inputs until valid motor number is entered
  bool valid_input = false;
  int i = 0;
  char rc; // received character
  char rint; // received int
  char motor_num_in[IN_BUFFER_LEN+1];
  while(!valid_input){
    Serial.print("input number of motors\n");
    // expect single character to be received
    while(Serial.available() == 0){
      continue;
    }
    while (Serial.available() > 0) {
      rc = Serial.read();
      motor_num_in[i] = rc;
      i++;
    }
    
    // convert char to int and check if it's within bounds
    rint = atoi(motor_num_in);
    if(rint < 1 || rint > 3){
      Serial.print("input should be 1, 2, or 3\n");
    }
    else{
      num_motors = rint;
      valid_input = true;
    }
  }
  return;
}

// enable or disable 1-3 motors depending on what num_motors is
void multipleMotorEnable(bool request){
  switch (num_motors){
    case 1:
      motor1.EnableRequest(request);
      break;
    case 2:
      motor1.EnableRequest(request);
      motor2.EnableRequest(request);
      break;
    case 3:
      motor1.EnableRequest(request);
      motor2.EnableRequest(request);
      motor3.EnableRequest(request);
      break;
    default:
      Serial.print("incompatible number of motors\n");
      break;
  }
}

// toggle direction (A state) depending on num motors
void checkMotorAState(const std::vector<float>& commandedTorque){
  switch (num_motors){
    case 1:
      if (commandedTorque[0] < 0) {motor1.MotorInAState(true);}
      else {motor1.MotorInAState(false);}
      break;
    case 2:
      if (commandedTorque[0] < 0) {motor1.MotorInAState(true);}
      else {motor1.MotorInAState(false);}

      if (commandedTorque[1] < 0) {motor2.MotorInAState(true);}
      else {motor2.MotorInAState(false);}
      break;
    case 3:
      if (commandedTorque[0] < 0) {motor1.MotorInAState(true);}
      else {motor1.MotorInAState(false);}

      if (commandedTorque[1] < 0) {motor2.MotorInAState(true);}
      else {motor2.MotorInAState(false);}

      if (commandedTorque[2] < 0) {motor3.MotorInAState(true);}
      else {motor3.MotorInAState(false);}
      break;
    default:
      Serial.print("incompatible number of motors\n");
      break;
  }
}

/*------------------------------------------------------------------------------
 * CommandTorque commands set of up to three motors given vector of torque commands
 */
bool CommandTorque(const std::vector<float>& commandedTorque) {
    for (int i = 0; i < (commandedTorque.size()-1); i++){
      if (abs(commandedTorque[i]) > abs(maxTorque)) {
          Serial.println("Move rejected, invalid torque requested");
          return false;
      }
    }

    // Find the scaling factor of our torque range mapped to the PWM duty cycle
    // range (255 is the max duty cycle).
    double scaleFactor = 255 / maxTorque;

    // Scale the torque command to our duty cycle range.
    std::vector<float> dutyRequest(3);
    dutyRequest = {abs(commandedTorque[0])*scaleFactor, abs(commandedTorque[1])*scaleFactor, abs(commandedTorque[2])*scaleFactor};

    // Set input A to match the direction of torque.
    checkMotorAState(commandedTorque);

    // Ensures this delay is at least 20ms longer than the Input A filter
    // setting in MSP
    delay(20 + INPUT_A_FILTER);

    // switch over number of motor (there has to be a more elegant way to do this)
    switch (num_motors){
      case 1:
        // Command the move
        motor1.MotorInBDuty(dutyRequest[0]);

        // Waits for HLFB to assert (signaling the move has successfully completed)
        // Serial.println("Moving... Waiting for HLFB");
        // Allow some time for HLFB to transition
        delay(1);

        // return to state one if the signal is not asserted
        if (motor1.HlfbState() != MotorDriver::HLFB_ASSERTED) {
          Serial.println("Motor Fault or Overspeed Timeout Detected!");
          delay(200);
          multipleMotorEnable(false); // Disable motor
          // overwrite input1 
          input[0] = '1';
          return false;
        }
        break;
      case 2:
        // Command the move
        motor1.MotorInBDuty(dutyRequest[0]);
        motor2.MotorInBDuty(dutyRequest[1]);

        // Waits for HLFB to assert (signaling the move has successfully completed)
        // Serial.println("Moving... Waiting for HLFB");
        // Allow some time for HLFB to transition
        delay(1);

        // return to state one if the signal is not asserted
        if (motor1.HlfbState() != MotorDriver::HLFB_ASSERTED || motor2.HlfbState() != MotorDriver::HLFB_ASSERTED) {
          Serial.println("Motor Fault or Overspeed Timeout Detected!");
          delay(200);
          multipleMotorEnable(false); // Disable motor
          // overwrite input1 
          input[0] = '1';
          return false;
        }
        break;
      case 3:
        // Command the move
        motor1.MotorInBDuty(dutyRequest[0]);
        motor2.MotorInBDuty(dutyRequest[1]);
        motor3.MotorInBDuty(dutyRequest[2]);

        // Waits for HLFB to assert (signaling the move has successfully completed)
        // Serial.println("Moving... Waiting for HLFB");
        // Allow some time for HLFB to transition
        delay(1);

        // return to state one if the signal is not asserted
        if (motor1.HlfbState() != MotorDriver::HLFB_ASSERTED || motor2.HlfbState() != MotorDriver::HLFB_ASSERTED || motor3.HlfbState() != MotorDriver::HLFB_ASSERTED) {
          Serial.println("Motor Fault Detected! Returning to idle state");
          delay(200);
          multipleMotorEnable(false); // Disable motor
          // overwrite input1 
          input[0] = '1';
          return false;
        }
        break;
      default:
        Serial.print("incompatible number of motors\n");
        break;
    }

    // Serial.println("Move Done");
    return true;
}

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













//     // temp
//       float r = 1.0;  // Example radius value
//   int mass = 200;
//   //  BLA::Matrix<3,1,float> apex = {1,2,3}; 
//   BLA::Matrix<3, 1, float> lengths = {
//     5.0,
//     5.0,
//     5.0
//   };
//   BLA::Matrix<3, 1, float> p = {2.0,2.0,-4.0};
//   BLA::Matrix<3, 1, float> apex;
//   // Compute tether attachment points
//   BLA::Matrix<3, 3, float> teth_anchor = { 
//       2.0, 0.0, 0.0 ,  
//       2.0 * cos(DEG_TO_RAD(225)), 2.0 * sin(DEG_TO_RAD(225)), 0.0 ,  
//       2.0 * cos(DEG_TO_RAD(135)), 2.0 * sin(DEG_TO_RAD(135)), 0.0  
//   };

//   // Compute attachment offset vectors
//   BLA::Matrix<3, 3, float> offset = { 
//       -r, 0.0, 0.0 ,  
//       -r * cos(DEG_TO_RAD(225)), -r * sin(DEG_TO_RAD(225)), 0.0 ,  
//       -r * cos(DEG_TO_RAD(135)), -r * sin(DEG_TO_RAD(135)), 0.0  
//   };
//   // Put your main code here, it will run repeatedly:

//   // int i = 0;
//   // strcpy(temp, input);


//   // while(i<3 && ioPort.CharPeek() != -1){
//   // Serial.println("Enter tether length " + String(i) + "\n");

//   //   input[i] = (float) ioPort.CharGet();
//   //   i++;
//   //   // valid_input_flag
//   // Delay_ms(1);
//   // }

//   // lengths(1) = input[1];
//   // lengths(2) = input[2];
//   // lengths(3) = input[3];


//   Serial.print("-----------------------------------------------------------------------------------------------\n");
//   apex =newtonSolve( p, lengths, teth_anchor, offset);
//   Serial.println("Apex : " + String(apex(0)) + ", " + String(apex(1)) + ", " + String(apex(2)) + "\n");
// }


//--------------------------------------------------------------------------------------------------------------
// Dynamic model code



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



// // Function to compute the Jacobian matrix using finite differences
// BLA::Matrix<3, 3, float> jacobian(BLA::Matrix<3,1, float> p, 
//                                   BLA::Matrix<3,1, float> tether_lengths, 
//                                   BLA::Matrix<3, 3, float> teth_anchor, 
//                                   BLA::Matrix<3, 3, float> offset) {
//     BLA::Matrix<3, 3, float> J;
//     double h = 1e-5;
//     BLA::Matrix<3,1, float> p1, f1, f2;

//     for (int i = 0; i < 3; i++) {
//         p1 = p;
//         p1(i) += h;
//         f1 = equations(p1, teth_anchor, offset, tether_lengths);
        
//         p1(i) -= 2*h;
//         f2 = equations(p1, teth_anchor, offset, tether_lengths);

//         for (int j = 0; j < 3; j++) {
//             J(j, i) = (f1(j) - f2(j)) / (2 * h);
//         }
//     }
//     return J;
// }


BLA::Matrix<3, 3, float> jacobian(BLA::Matrix<3, 1, float> p, 
                                  BLA::Matrix<3, 1, float> tether_lengths, 
                                  BLA::Matrix<3, 3, float> teth_anchor, 
                                  BLA::Matrix<3, 3, float> offset) {
    BLA::Matrix<3, 3, float> J;

    // Initialize the Identity matrix
    BLA::Matrix<3, 3, float> Identity;
    Identity(0, 0) = 1.0f; Identity(0, 1) = 0.0f; Identity(0, 2) = 0.0f;
    Identity(1, 0) = 0.0f; Identity(1, 1) = 1.0f; Identity(1, 2) = 0.0f;
    Identity(2, 0) = 0.0f; Identity(2, 1) = 0.0f; Identity(2, 2) = 1.0f;

    // Populate the Jacobian matrix
    for (int i = 0; i < 3; i++) {
        J(i, 0) = 2 * (p(0) - (teth_anchor(i, 0) + offset(i, 0)));
        J(i, 1) = 2 * (p(1) - (teth_anchor(i, 1) + offset(i, 1)));
        J(i, 2) = 2 * (p(2) - (teth_anchor(i, 2) + offset(i, 2)));
    }

    // Check if the Jacobian is nearly singular and apply regularization
    if (abs(Determinant(J)) < 1e-6) { 
        Serial.println("Jacobian nearly singular! Adding regularization.");
        J = J + (1e-6f * Identity);  // Cast 1e-6 to float for consistency
    }

    return J;
}



// swap rows
void swap(float& a, float& b) {
    float temp = a;
    a = b;
    b = temp;
}

// // solve system by doing things
// BLA::Matrix<3, 1, float> solveSystem(BLA::Matrix<3, 3, float> A, BLA::Matrix<3, 1, float> b) {
//   BLA::Matrix<3, 1, float> x;
//   int maxRow;
//   double factor;
//   for(int i=0;i<3;i++){
//     maxRow= i;
//     for(int k=i+1;k<3;k++){
//       if (fabs(A(k,i)) > fabs(A(maxRow,i))) {
//         maxRow = k;
//       }
//     }

//     swap(A(i), A(maxRow));
//     swap(b(i), b(maxRow));

//     if (fabs(A(i,i)) < 1e-10) {
//         Serial.println("Jacobian is singular, Newton's method failed.");
//     } // very unlikely with set up but can test if something goes wrong

//     // Forward elimination
//     for (int k = i + 1; k < 3; k++) {
//       factor = A(k,i) / A(i,i);
//       for (int j = i; j < 3; j++) {
//         A(k,j) -= factor * A(i,j);
//       }
//       b(k) -= factor * b(i);
//     }
//   }

//   // Back substitution

//   for (int i = 3 - 1; i >= 0; i--) {
//     x(i) = b(i);
//     for (int j = i + 1; j < 3; j++) {
//       x(i) -= A(i,j) * x(j);  
//     }
//     x(i) /= A(i,i);
//   }

//   return x;

// }

BLA::Matrix<3, 1, float> solveSystem(BLA::Matrix<3, 3, float> A, BLA::Matrix<3, 1, float> b) {
    return Inverse(A) * b;  // Directly use matrix division in BLA
}



// Solving nonlinear system to get apex using Newton-Raphson method
BLA::Matrix<3, 1, float> newtonSolve(BLA::Matrix<3, 1, float> p, 
                                     BLA::Matrix<3, 1, float> tether_lengths, 
                                     BLA::Matrix<3, 3, float> teth_anchor, 
                                     BLA::Matrix<3, 3, float> offset) {
    BLA::Matrix<3, 1, float> F, delta_p;
    BLA::Matrix<3, 3, float> J;
    double norm;

    for (int iter = 0; iter < Maxiterations; iter++) {
        // Compute function values
        F = equations(p, teth_anchor, offset, tether_lengths);
        
     //   Serial.print("Iteration: "); Serial.println(iter);
       // Serial.println("Current p:");
    //    printMatrix(p);

       // Serial.println("F(p):");
      //  printMatrix(F);

        // Compute Jacobian
        J = jacobian(p, tether_lengths, teth_anchor, offset);
        
       // Serial.println("Jacobian:");
      //  printMatrix(J);

        // Check if the Jacobian is nearly singular
        float det_J = Determinant(J);
       // Serial.print("Determinant of Jacobian: ");
       // Serial.println(det_J);

        if (abs(det_J) < 1e-6) { 
            Serial.println("Jacobian is nearly singular! Stopping.");
            return {0, 0, 0};
        }

        // Solve J * delta_p = -F
        delta_p = solveSystem(J, -F);

    //    Serial.println("delta_p:");
     //   printMatrix(delta_p);

        // Update solution
        for (int i = 0; i < 3; i++) {
            p(i) = p(i) + delta_p(i);
        }

        // Check for convergence
        norm = sqrt(delta_p(0) * delta_p(0) + delta_p(1) * delta_p(1) + delta_p(2) * delta_p(2));
        if (norm < TOL) {
            return p;
        }
    }

    Serial.println("Maximum iterations reached. Returning last estimate.");
    return p;
}


// find tether unit vectors
BLA::Matrix<3, 3, float> calculate_tether_vecs(BLA::Matrix<3,1, float> COM, BLA::Matrix<3, 3, float> teth_anchor, BLA::Matrix<3, 3, float> offset) {
  // Define tether vectors
  BLA::Matrix<3, 1, float> tethvec1, tethvec2, tethvec3;
  BLA::Matrix<3, 1, float> teth1_hat, teth2_hat, teth3_hat;
  BLA::Matrix<3, 3, float> ANS;
  // Compute tether vectors
  for (int i = 0; i < 3; i++) {
    tethvec1(i, 0) = teth_anchor(0, i) - (COM(i, 0) - offset(0, i));
    tethvec2(i, 0) = teth_anchor(1, i) - (COM(i, 0) - offset(1, i));
    tethvec3(i, 0) = teth_anchor(2, i) - (COM(i, 0) - offset(2, i));
  }

  // Normalize tether vectors to get unit vectors
 // Compute norm of tether vectors
float norm_teth1 = sqrt(tethvec1(0,0) * tethvec1(0,0) + tethvec1(1,0) * tethvec1(1,0) + tethvec1(2,0) * tethvec1(2,0));
float norm_teth2 = sqrt(tethvec2(0,0) * tethvec2(0,0) + tethvec2(1,0) * tethvec2(1,0) + tethvec2(2,0) * tethvec2(2,0));
float norm_teth3 = sqrt(tethvec3(0,0) * tethvec3(0,0) + tethvec3(1,0) * tethvec3(1,0) + tethvec3(2,0) * tethvec3(2,0));

// Avoid division by zero

    for (int i = 0; i < 3; i++) {
        teth1_hat(i, 0) = tethvec1(i, 0) / norm_teth1;
         teth2_hat(i, 0) = tethvec2(i, 0) / norm_teth2;
          teth3_hat(i, 0) = tethvec3(i, 0) / norm_teth3;
           ANS(i, 0) = teth1_hat(i, 0);  // Assign teth1_hat to column 0
    ANS(i, 1) = teth2_hat(i, 0);  // Assign teth2_hat to column 1
    ANS(i, 2) = teth3_hat(i, 0);  // Assign teth3_hat to column 2
    }





//   // Define result matrix (3x4)
//   BLA::Matrix<3, 3, float> ANS;

// for (int i = 0; i < 3; i++) {
//     ANS(i, 0) = teth1_hat(i, 0);  // Assign teth1_hat to column 0
//     ANS(i, 1) = teth2_hat(i, 0);  // Assign teth2_hat to column 1
//     ANS(i, 2) = teth3_hat(i, 0);  // Assign teth3_hat to column 2
// }
  

  // ANS(0, 3) = sqrt(tethvec1(0, 0) * tethvec1(0, 0) + tethvec1(1, 0) * tethvec1(1, 0) + tethvec1(2, 0) * tethvec1(2, 0));
  // ANS(1, 3) = sqrt(tethvec2(0, 0) * tethvec2(0, 0) + tethvec2(1, 0) * tethvec2(1, 0) + tethvec2(2, 0) * tethvec2(2, 0));
  // ANS(2, 3) = sqrt(tethvec3(0, 0) * tethvec3(0, 0) + tethvec3(1, 0) * tethvec3(1, 0) + tethvec3(2, 0) * tethvec3(2, 0));

  return ANS;
}

// calculae tether forces given apex, anchor locations, and mass
BLA::Matrix<3, 1, float> calculate_tether_forces(BLA::Matrix<3,1, float> apex, int mass, BLA::Matrix<3, 3, float> teth_anchor, BLA::Matrix<3, 3, float> offset) {
    // Compute unit vectors
    BLA::Matrix<3, 3, float> M1 = calculate_tether_vecs(apex, teth_anchor, offset);
//printMatrix(Initial_Matrix);
    // Extract the first 3 columns as a 3x3 matrix
   // BLA::Matrix<3, 3, float> M1;
    // for (int i = 0; i < 3; i++) {
    //     for (int j = 0; j < 3; j++) {
    //         M1(i, j) = Initial_Matrix(i, j);
    //     }
    // }



    // Define force matrix M2
    BLA::Matrix<3, 1, float> M2;
    M2(0, 0) = 0;
    M2(1, 0) = 0;
    M2(2, 0) = mass;

    // Solve for forces using matrix inversion
    BLA::Matrix<3, 1, float> F = Inverse(M1) * M2;  // 


    return F;  // Return the calculated tether forces
}
//------------------------------------------------------------------------------
