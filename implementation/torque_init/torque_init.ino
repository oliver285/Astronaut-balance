// TODO: when hitting errors and returning tp base state, the chracter array input[0] has to be changed or else the jump will not be acknowledged,
//  potentially change this to allow the int input1 to be changed and not be overwritten by input array (or not, might be for the better)

// TODO: motor 1 inverted (marked with x physucally)
// TODO: automate motor number input (currently hard coded)


/*
 * Objective:
 *    Develop the data pipeline for each motor in the system
 *
 * Description:
 *    This file runs a proof of concept for the pipeline for one motor. It takes in 
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
// weird shit going on with class between preprocessor min/max macros and std min/max
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

// Defines the motor's connector as ConnectorM0
#define ioPort ConnectorUsb
#define motor1 ConnectorM0
#define motor2 ConnectorM1
#define motor3 ConnectorM2

// Select the baud rate to match the target device.
#define baudRate 115200
#define ioPortBaudRate  115200

// define number of motors being used
int num_motors = 2;

// Defines the limit of the torque command, as a percent of the motor's peak
// torque rating (must match the value used in MSP).
double maxTorque = 100;

// temporary may be used later
#define IN_BUFFER_LEN 32
char input[IN_BUFFER_LEN+1];
std::vector<float> float_inputs(4);
int temp, input1 = 0;
double input2, input3, input4; // torque/force command
double t_init = 0;

// flag to tell input 3 to normalize millis()
bool test_start = false;

// Declares our user-defined helper function, which is used to command torque.
// The definition/implementation of this function is at the bottom of the sketch.
bool CommandTorque(int commandedTorque);
void multipleMotorEnable(bool request);
void checkMotorAState(const std::vector<float>& commandedTorque);

void setup() {
    // Put your setup code here, it will only run once:

    // set digital and analog pins as inputs //


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


    // set up motor
    multipleMotorEnable(false);
    Serial.println("Motor(s) Ready");
}


void loop() {
    // Put your main code here, it will run repeatedly:
  
    // read serial port
    int i = 0;
    char rc;
    char endMarker = '\n';
    temp = input1;
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
          // Serial.print("input 1 \n");
          if(motor1.EnableRequest()){
            multipleMotorEnable(false);
            Serial.print("motor(s) disabled \n");
          }
          // motor.EnableRequest(false); // Disable motor
          break;
        }

      case 2:
        {
          test_start = false;

          // Serial.print("input 2 \n");
          // Serial.println(motor.EnableRequest());
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
          if (temp == 1){
            Serial.print("Invalid jump \n");
            input1 = temp;
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
          // Wait 2000ms.
          // double hlfbP = motor.HlfbPercent();
          // Serial.println(hlfbP);
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

    delay(200);
    // // Output 15% of the motor's peak torque in the positive (CCW) direction.
    // CommandTorque(5);    // See below for the detailed function definition.
    // // Wait 2000ms.
    // delay(2000);

    // CommandTorque(8); // Output 8% peak torque in the negative (CW) direction.
    // delay(2000);


    
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
      if (commandedTorque[0] < 0) {
          motor1.MotorInAState(true);
      }
      else {
          motor1.MotorInAState(false);
      }
      break;
    case 2:
      if (commandedTorque[0] < 0) {
          motor1.MotorInAState(true);
      }
      else {
          motor1.MotorInAState(false);
      }

      if (commandedTorque[1] < 0) {
          motor2.MotorInAState(true);
      }
      else {
          motor2.MotorInAState(false);
      }
      break;
    case 3:
      if (commandedTorque[0] < 0) {
          motor1.MotorInAState(true);
      }
      else {
          motor1.MotorInAState(false);
      }

      if (commandedTorque[1] < 0) {
          motor2.MotorInAState(true);
      }
      else {
          motor2.MotorInAState(false);
      }

      if (commandedTorque[2] < 0) {
          motor3.MotorInAState(true);
      }
      else {
          motor3.MotorInAState(false);
      }
      break;
    default:
      Serial.print("incompatible number of motors\n");
      break;

  }
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
bool CommandTorque(const std::vector<float>& commandedTorque) {
    for (int i = 0; i < (commandedTorque.size()-1); i++){
      if (abs(commandedTorque[i]) > abs(maxTorque)) {
          Serial.println("Move rejected, invalid torque requested");
          return false;
      }
    }
	
    // Serial.print("Commanding torque: ");
    // Serial.println(commandedTorque);

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
          Serial.println("Motor Fault or Overspeed Timeout Detected!");
          delay(200);
          multipleMotorEnable(false); // Disable motor
          // overwrite input1 
          input[0] = '1';
        }
        break;
      default:
        Serial.print("incompatible number of motors\n");
        break;
    }

    // Serial.println("Move Done");
    return true;
}
//------------------------------------------------------------------------------
