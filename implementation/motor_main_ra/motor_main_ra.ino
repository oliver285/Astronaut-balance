// TODO: when hitting errors and returning tp base state, the chracter array input[0] has to be changed or else the jump will not be acknowledged,
//        potentially change this to allow the int input1 to be changed and not be overwritten by input array (or not, might be for the better)
// TODO: input a state should not be necessary to check, since we are torquing in one direction will only need one direction and not allow torque commands in the other direction.
// TODO: create load cell taring routine in setup script, read load cell for x amount of time, take that average, and that is the offset


// TODO: tune gains
// TODO: add negative torque protection

/*
 * Objective:
 *    Develop the data pipeline for each motor in the system, the force determination of the system for this script will be done on the computer
 *    through python. inputs 2,3,4 will be force commands that get converted to torques on this script
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
#include <RunningAverage.h> 
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
#define INPUT_A_FILTER 2

// Defines the motor connectors
#define ioPort ConnectorUsb
#define motor1 ConnectorM0
#define motor2 ConnectorM1
#define motor3 ConnectorM2

// Select the baud rate and io port baud rate to match the target device.
#define baudRate 115200
#define ioPortBaudRate  115200

// load cell parameters
#define adcResolution 12
#define LOAD_CELL_MAX_FORCE 220.462 // lbs
double max_allowable_force = 100; // lbs, cutoff before absolute max load cell force
const int analogPins[3] = {A10, A11, A12};
// empirically determined linear scale factor of the load cell
// double scale_factor[3] = {1.1316, 1.1316, 1.1316};
double scale_factor[3] = {0.977662904235, 0.974399048854, 0.975303160436};
// this offset is from the load cell hanging vertically 
// double offset[3] = {1.4, 1.4, 1.4};
double offset[3] = {1.5, 3.3, 1.34};
double load_cell[3] = {0, 0, 0};

// control parameters
double kp[3] = {1.5, 1.5, 1.5};
double ki[3] = {0.1, 0.1, 0.1};
double kd[3] = {0.05, 0.05, 0.05};
// double kp[3] = {0.0, 0.0, 0.0};
// double ki[3] = {0.0, 0.0, 0.0};
// double kd[3] = {0.0, 0.0, 0.0};
double F_err_prev[3] = {0.0, 0.0, 0.0};
// maybe make this array?
unsigned long prev_time[3] = {0.0, 0.0, 0.0};
double integral[3] = {0.0, 0.0, 0.0};
std::vector<float> torque_command(3);

// define number of motors being used
int num_motors = 0;

// Defines the limit of the torque command, as a percent of the motor's peak
// torque rating (must match the value used in MSP).
double maxTorque = 100;
double maxTorqueMag = 14.0; // Nm
// define serial buffer length and character input and float_input arrays
#define IN_BUFFER_LEN 32
char input[IN_BUFFER_LEN+1];

// temp ensures we don't jump from state 1 to state 3, input1 is the state command
int prev_input1, input1 = 0;

// defines start of test (entering state3)
double t_init = 0;

// flag to tell input 3 to normalize millis()
bool test_start = false;
// flag to determine whether the torque has to be ramped from zero or not
bool ramp_done = false;

// Declares helper functions
void ramp_up_motor();
// void ramp_up_motor(const std::vector<float>& inputs);
double force2Torque(double force);
void readLoadCell();
void getNumMotors();
void multipleMotorEnable(bool request);
void checkMotorAState();
bool CommandTorque();

// moving average object
RunningAverage* ra[3]; 

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

    // Set the resolution of the ADC.
    analogReadResolution(adcResolution);

    // ask for number of motors
    getNumMotors();

    // set input1 to state 1
    input[0] = '1';

    // begin rolling average
    for (int i = 0; i < 3; i++) {
      ra[i] = new RunningAverage(10); // Set the sample size for each object
    }

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

    std::vector<float> float_inputs(4);

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

    // save off state to input 1
    input1 = float_inputs[0];

    // read the load cells
    readLoadCell();
    // debugging message
    // Serial.println(String(load_cell[0]) + ", " + String(load_cell[1]) + ", " + String(load_cell[2]) + "\n");
    

    // convert force inputs into motor command (% of max torque) and apply PD control
    double F_err_dot = 0;
    double Tau_m_adj = 0;
    double F_err = 0;
    double dt = 0;
    for(int i = 0; i<num_motors; i++){
      // define holding torque
      torque_command[i] = force2Torque(float_inputs[i+1]);

      // add control torque to holding torque command
      if(ramp_done){
        F_err = float_inputs[i+1]-load_cell[i];
        dt = (millis() - prev_time[i])/1000.0; // dt in seconds for derivative gain
        F_err_dot = (F_err-F_err_prev[i])/dt;
        integral[i] += F_err*dt;
        Tau_m_adj = kp[i]*F_err + ki[i]*integral[i] + kd[i]*F_err_dot;
        // Serial.println(String(force2Torque(Tau_m_adj)));
        torque_command[i] = torque_command[i] + force2Torque(Tau_m_adj);
      }

      // update F_err_prev and prev time
      F_err_prev[i] = F_err;
      prev_time[i] = millis();
    }

    // Serial.println("Torque command: " + String(torque_command[0]) + ", " + String(torque_command[1]) + ", " + String(torque_command[2]) + "\n");




    switch(input1){
      case 0:
        // reboot the system
        NVIC_SystemReset(); 
      case 1:
        {
          test_start = false;
          ramp_done = false;
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

          if(!ramp_done){
            // ramp_up_motor();
            ramp_up_motor(float_inputs);
            ramp_done = true;
          }
          else{
            CommandTorque();    // See below for the detailed function definition.
          }

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

          CommandTorque();    // See below for the detailed function definition.
          // prints load cell outputs, depends on numbers of motors connected
          switch (num_motors){
            case 1:
              // Serial.println(String(millis() - t_init, 4) + ", " + String(load_cell[0]) + "\n");
              Serial.println(String(millis() - t_init, 4) + ", " + String(load_cell[0]) + ", " + String(torque_command[0]) + "\n");
              break;
            case 2:
              Serial.println(String(millis() - t_init, 4) + ", " + String(load_cell[0]) + ", " + String(load_cell[1]) + "\n");
              break;
            case 3:
              Serial.println(String(millis() - t_init, 4) + ", " + String(load_cell[0]) + ", " + String(load_cell[1]) + ", " + String(load_cell[2]) + "\n");
              // Serial.println(String(millis() - t_init, 4) + ", " + String(torque_command[0]) + ", " + String(torque_command[1]) + ", " + String(torque_command[2]) + "\n");
              // Serial.println(String(millis() - t_init, 4) + ", " + String(load_cell[0]) + ", " + String(load_cell[1]) + ", " + String(load_cell[2]) + ", " + String(torque_command[0]) + ", " + String(torque_command[1]) + ", " + String(torque_command[2]) +"\n");
              break;
          }

          break;
        }

      default:
        // currently sets state back to one if an invalid input is received
        Serial.print("Invalid input \n");
        input[0] = '1';
        break;
    }

    // TODO: temporary delay
    delay(1);
}

// slowly ramp up the motor to the desired force whenever the state goes from 1 to 2, assumes motors are already enabled
void ramp_up_motor(const std::vector<float>& inputs){
  
  torque_command = {0.0, 0.0, 0.0};
  Serial.print("Motors ramping\n");

  bool ramped[3] = {false,false,false};
  
  while(!ramped[0] || !ramped[1] || !ramped[2]){
    readLoadCell();
    for(int i=0; i<3; i++){
      if (abs(load_cell[i]  - inputs[i+1]) < 5.0){
        ramped[i] = true;
      }
      else{
        if (load_cell[i] < inputs[i+1]){
          torque_command[i] += 1.0;
        }
        else{
          torque_command[i] -= 1.0;
        }
        CommandTorque();
      }  
    }

    if (input[0] == '1'){
      Serial.print("Motor ramp failed\n");
      return;
    }
    delay(200);
  }
  
  Serial.print("Motors ramped to commanded torque\n");
}


// input force returns torque command as percentage of max torque
double force2Torque(double force){
  // 94.48 (N/Nm)
  // 21.241 (lbf/Nm)
  // todo: 1.25 tmeporary scale factor
  double torque = 1.25*((1.0/21.241)*force);
  return (torque/maxTorqueMag*100);
}

void readLoadCell(){
  int adcResult = 0;
  double inputVoltage = 0;
  double force = 0;
  for (int i = 0; i<3; i++){
    adcResult = analogRead(analogPins[i]);
    inputVoltage = 10.0 * adcResult / ((1 << adcResolution) - 1);
    force = scale_factor[i]*((inputVoltage/10)*LOAD_CELL_MAX_FORCE) - offset[i];
    ra[i]->addValue(force);   
    load_cell[i] = ra[i]->getAverage();

    // disable motors if load cell over 175 lbs
    if (load_cell[i] > max_allowable_force){
      Serial.print("excessive load cell force detected\n");
      input[0] = '1';
      return;
    }
    // load_cell[i] = inputVoltage;
    // load_cell[i] = adcResult;
  }
}


// get the number of motors currently attached based on user 
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
void checkMotorAState(){
  for(int i = 0; i<3; i++){
    if (torque_command[i] < 0){
      torque_command[i] = 0;
    }
  }

  switch (num_motors){
    case 1:
      motor1.MotorInAState(false);
      break;
    case 2:
      motor1.MotorInAState(false);
      motor2.MotorInAState(false);
      break;
    case 3:
      motor1.MotorInAState(false);
      motor2.MotorInAState(false);
      // motor 3 always opposite direction due to physical orientation
      motor3.MotorInAState(false);
      break;
    default:
      Serial.print("incompatible number of motors\n");
      break;
  }
}




/*------------------------------------------------------------------------------
 * CommandTorque commands set of up to three motors given vector of torque commands
 */
bool CommandTorque() {
    for (int i = 0; i < (torque_command.size()-1); i++){
      if (abs(torque_command[i]) > abs(maxTorque)) {
          Serial.println("Move rejected, invalid torque requested");
          return false;
      }
    }

    // Find the scaling factor of our torque range mapped to the PWM duty cycle
    // range (255 is the max duty cycle).
    double scaleFactor = 255 / maxTorque;

    // Scale the torque command to our duty cycle range.
    std::vector<float> dutyRequest(3);
    dutyRequest = {abs(torque_command[0])*scaleFactor, abs(torque_command[1])*scaleFactor, abs(torque_command[2])*scaleFactor};

    // Set input A to match the direction of torque.
    checkMotorAState();

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
//------------------------------------------------------------------------------
