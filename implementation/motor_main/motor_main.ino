// TODO: this crashes (check back later)








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


using namespace std;

// The INPUT_A_FILTER must match the Input A filter setting in
// MSP (Advanced >> Input A, B Filtering...)
#define INPUT_A_FILTER 20

// Defines the motor's connector as ConnectorM0
#define ioPort ConnectorUsb
#define motor ConnectorM0

// Select the baud rate to match the target device.
#define baudRate 115200
#define ioPortBaudRate  115200
// adc resolution
#define adcResolution 12

// define max load cell force and scale factor for V to lb conversion
#define LOAD_CELL_MAX_FORCE 220.462
#define scale_factor 1.1316

// define digital pin inputs being used //

// Defines the limit of the torque command, as a percent of the motor's peak
// torque rating (must match the value used in MSP).
double maxTorque = 100;
double currCommand = 0;

// char input = '0';
// setup paramater indicating how many motors are being used
int num_motors = 1;

// temporary may be used later
#define IN_BUFFER_LEN 32
std::vector<float> input(4);
int temp, input1 = 0; // temp to remember previous state and state input
double input2, input3, input4; // torque/force command
double t_init = 0;

// flag to tell input 3 to normalize millis()
bool test_start = false;
// flag to tell input 2 whether to ask for input torque
bool torque_input_needed = true;

// Declares our user-defined helper function, which is used to command torque.
// The definition/implementation of this function is at the bottom of the sketch.
std::vector<float> get_inputs();
bool CommandTorque(int commandedTorque);

void setup() {
    // Put your setup code here, it will only run once:

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

    // Set the resolution of the ADC.
    analogReadResolution(adcResolution);

    // initializes the motor
    motor.EnableRequest(false);
    Serial.println("Motor Ready");
}


void loop() {
    // Put your main code here, it will run repeatedly:

    // analog and digital read //
    // analog adc convert (load cell) //
    // digital (encoder input) //
    temp = input1;

    // get serial input
    input = get_inputs();
    input1 = input[0];
    input2 = input[1];
  





    // Serial.println(input1);
    // Serial.println(input2);


    switch(input1){
      case 1:
        {
        torque_input_needed = false;
        // Serial.print("input 1 \n");
        if(motor.EnableRequest()){
          motor.EnableRequest(false);
          Serial.print("motor disabled \n");
        }
        // motor.EnableRequest(false); // Disable motor
        break;
        }

      case 2:
        {
        char force_input[IN_BUFFER_LEN+1];
        double force_input_double;
        double torque_input;
        bool valid_input = false;
        if(!torque_input_needed){
          test_start = false;
          torque_input_needed = true;
          // TODO: input of 1 does not return to state 1

          // // temporary loop to capture custom user torque input.
          // while(!valid_input){
          //   // wait for force input
          //   force_input_double = 0;
          //   torque_input = 0;
          //   memset(force_input, 0, sizeof(force_input));
          //   i = 0;
          //   Serial.print("input desired force \n");
          //   while(ioPort.CharPeek() == -1){
          //     continue;
          //   }
          //   while(i<IN_BUFFER_LEN && ioPort.CharPeek() != -1){
          //     force_input[i] = (char) ioPort.CharGet();
          //     i++;
          //     // valid_input_flag
          //     Delay_ms(1);  
          //   }
          //   // convert input to double
          //   force_input_double = atof(force_input);
          //   // placeholder for force to torque conversion
          //   torque_input = force_input_double;
          //   if (torque_input > maxTorque){
          //     Serial.println("torque input:" + String(torque_input) + ", is greater than max torque:" + String(maxTorque) + "\n");
          //   }
          //   else{
          //     valid_input = true;
          //     currCommand = torque_input;
          //   }
          // }


        }
        // Serial.print("input 2 \n");
        // Serial.println(motor.EnableRequest());
        if(!motor.EnableRequest()){
          motor.EnableRequest(true);
          Serial.print("motor enabled \n");
        }

        currCommand = maxTorque - abs(10*(fmod(input2,360.0))/360.0);
        CommandTorque(currCommand);    // See below for the detailed function definition.
        // Wait 2000ms.

        break;
        }

      case 3:
        {
        if (temp == 1){
          Serial.print("Invalid jump \n");
          input1 = temp;
          break;
        }

        if(!test_start){
          t_init = millis();
          test_start = true;
        }

        currCommand = maxTorque - abs(10*(fmod(input2,360.0))/360.0);
        CommandTorque(currCommand);    // See below for the detailed function definition.
        // Wait 2000ms.
        // double hlfbP = motor.HlfbPercent();
        // Serial.println(hlfbP);
        Serial.println(String(millis() - t_init, 4) + ", " +String(currCommand) + ", " + String(input2));
        delay(200);

        break;
        }

      default:
        Serial.print("Invalid input \n");
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


std::vector<float> get_inputs(){
  int i = 0;
  char rc; // current read character
  char endMarker = '\n';
  char str_input[IN_BUFFER_LEN+1];
  // serial port messages in are currently of the form: state input, encoder angle
  while (Serial.available() > 0) {
    rc = Serial.read();
    if (rc != endMarker) {
      str_input[i] = rc;
      i++;
    }
    else{
      str_input[i] = '\0'; // terminate the string
      i = 0;
    }
  }

  // Serial.println(input);
  i = 0;
  // inputs converted to floats
  std::vector<float> float_inputs(4);

  char *token = strtok(str_input, ",");
  while(token != nullptr){
    if(i > 4){
      Serial.println("to many serial inputs");
      // if command greater than 4 inputs, send idle state with zero forces
      float_inputs = {1,0,0,0};
      break;
      }
      float_inputs[i] = atof(token);
      token = strtok(nullptr, ",");
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
bool CommandTorque(int commandedTorque) {
    if (abs(commandedTorque) > abs(maxTorque)) {
        Serial.println("Move rejected, invalid torque requested");
        return false;
    }
	
    // Serial.print("Commanding torque: ");
    // Serial.println(commandedTorque);

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
    // Serial.println("Moving... Waiting for HLFB");
    // Allow some time for HLFB to transition
    delay(1);

    // TODO: careful about this, auto reenables after unasserted hlfb
    if (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
      Serial.println("Motor Fault or Overspeed Timeout Detected!");
      delay(5000);
      motor.EnableRequest(false); // Disable motor
      input1 = 1; // return tp state 1
    }

    while(motor.HlfbState() != MotorDriver::HLFB_ASSERTED){

      continue;
    }
    // Serial.println("Move Done");
    return true;
}
//------------------------------------------------------------------------------

