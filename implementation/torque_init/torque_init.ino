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
#include <Keypad.h>

// The INPUT_A_FILTER must match the Input A filter setting in
// MSP (Advanced >> Input A, B Filtering...)
#define INPUT_A_FILTER 20

// Defines the motor's connector as ConnectorM0
#define ioPort ConnectorUsb
#define motor ConnectorM0

// Select the baud rate to match the target device.
#define baudRate 115200
#define ioPortBaudRate  115200

// define digital pin inputs being used //

// Defines the limit of the torque command, as a percent of the motor's peak
// torque rating (must match the value used in MSP).
double maxTorque = 10;
double currCommand = 0;

// char input = '0';

// temporary may be used later
#define IN_BUFFER_LEN 32
char input[IN_BUFFER_LEN+1];
char temp[IN_BUFFER_LEN+1];
double t_init = 0;

// flag to tell input 3 to normalize millis()
bool test_start = false;
// flag to tell input 2 whether to ask for input torque
bool torque_input_needed = true;

// Declares our user-defined helper function, which is used to command torque.
// The definition/implementation of this function is at the bottom of the sketch.
bool CommandTorque(int commandedTorque);

void setup() {
    // Put your setup code here, it will only run once:

    // set digital and analog pins as inputs //
    ioPort.Mode(Connector::USB_CDC);  
    ioPort.Speed(ioPortBaudRate);
    ioPort.PortOpen();
    while (!ioPort) {
      continue;
    }
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

    motor.EnableRequest(false);
    Serial.println("Motor Enabled");

    // Waits for HLFB to assert (waits for homing to complete if applicable)
    // Serial.println("Waiting for HLFB...");
    // while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
    //     continue;
    // }
    Serial.println("Motor Ready");
}


void loop() {
    // Put your main code here, it will run repeatedly:

    // analog and digital read //
    // analog adc convert (load cell) //
    // digital (encoder input) //
    int i = 0;
    strcpy(temp, input);
    // read the serial port input
    while(i<IN_BUFFER_LEN && ioPort.CharPeek() != -1){
      input[i] = (char) ioPort.CharGet();
      i++;
      // valid_input_flag
	  Delay_ms(1);
    }

    // Serial.print("-----------\n");
    // Serial.println(input[0]);
    // Serial.println(temp[0]);
    // Serial.println(i);
    // Serial.print("----------\n");


    switch(input[0]){
      case '1':
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

      case '2':
        {
        char force_input[IN_BUFFER_LEN+1];
        double force_input_double;
        double torque_input;
        bool valid_input = false;
        if(!torque_input_needed){
          test_start = false;
          torque_input_needed = true;
          while(!valid_input){
            // wait for force input
            force_input_double = 0;
            torque_input = 0;
            memset(force_input, 0, sizeof(force_input));
            i = 0;
            Serial.print("input desired force \n");
            while(ioPort.CharPeek() == -1){
              continue;
            }
            while(i<IN_BUFFER_LEN && ioPort.CharPeek() != -1){
              force_input[i] = (char) ioPort.CharGet();
              i++;
              // valid_input_flag
              Delay_ms(1);  
            }
            // convert input to double
            force_input_double = atof(force_input);
            // placeholder for force to torque conversion
            torque_input = force_input_double;
            if (torque_input > maxTorque){
              Serial.println("torque input:" + String(torque_input) + ", is greater than max torque:" + String(maxTorque) + "\n");
            }
            else{
              valid_input = true;
              currCommand = torque_input;
            }
          }
        }
        // Serial.print("input 2 \n");
        // Serial.println(motor.EnableRequest());
        if(!motor.EnableRequest()){
          motor.EnableRequest(true);
          Serial.print("motor enabled \n");
        }


        CommandTorque(currCommand);    // See below for the detailed function definition.
        // Wait 2000ms.

        break;
        }

      case '3':
        {
        if (temp[0] == '1'){
          Serial.print("Invalid jump \n");
          strcpy(input, temp);
          break;
        }

        if(!test_start){
          t_init = millis();
          test_start = true;
        }
        Serial.println(currCommand);
        CommandTorque(currCommand);    // See below for the detailed function definition.
        // Wait 2000ms.
        double hlfbP = motor.HlfbPercent();
        Serial.println(hlfbP);
        Serial.println(String(millis() - t_init, 4) + ", 1, " + String(motor.HlfbPercent()) + ", meas1, meas2, meas3");
        delay(200);
        // time, meas1, meas2, meas3
        break;
        }

      default:
        Serial.print("Invalid input \n");
        break;
    }

    delay(500);
    // // Output 15% of the motor's peak torque in the positive (CCW) direction.
    // CommandTorque(5);    // See below for the detailed function definition.
    // // Wait 2000ms.
    // delay(2000);

    // CommandTorque(8); // Output 8% peak torque in the negative (CW) direction.
    // delay(2000);


    
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
      delay(500);
      motor.EnableRequest(true);  // Re-enable motor
    }

    while(motor.HlfbState() != MotorDriver::HLFB_ASSERTED){

      continue;
    }
    // Serial.println("Move Done");
    return true;
}
//------------------------------------------------------------------------------
