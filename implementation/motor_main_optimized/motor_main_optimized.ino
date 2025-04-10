#include "ClearCore.h"
#include <SimpleKalmanFilter.h>
// Undefine preprocessor min/max macros to avoid conflicts with std min/max
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#include <vector>

// Constants
#define INPUT_A_FILTER 2
#define baudRate 115200
#define adcResolution 12
#define LOAD_CELL_MAX_FORCE 220.462 // lbs
#define IN_BUFFER_LEN 32

// Motor connectors
#define motor1 ConnectorM0
#define motor2 ConnectorM1
#define motor3 ConnectorM2

// Global variables
double max_allowable_force = 80; // lbs
const int analogPins[3] = {A10, A11, A12};
double scale_factor[3] = {0.977662904235, 0.974399048854, 0.975303160436};
double offset[3] = {1.5, 3.3, 1.34};
double load_cell[3] = {0, 0, 0};

// Control parameters
double ku=3.0;
double pu=.110;

double kp[3] = {ku/1.7, ku/1.7, ku/1.7};
double ki[3] = {pu/2, pu/2, pu/2};
double kd[3] = {pu/7, pu/7, pu/7};
double F_err_prev[3] = {0.0, 0.0, 0.0};
unsigned long prev_time[3] = {0, 0, 0};
double integral[3] = {0.0, 0.0, 0.0};
float torque_command[3] = {0.0, 0.0, 0.0}; // Changed from vector to array

// State variables
int num_motors = 0;
double maxTorque = 100;
double maxTorqueMag = 14.0; // Nm
char input[IN_BUFFER_LEN+1];
int prev_input1, input1 = 0;
double t_init = 0;
bool test_start = false;
bool ramp_done = false;

// Kalman filters for load cell smoothing
SimpleKalmanFilter kalmanFilters[3] = {
  SimpleKalmanFilter(0.5, 0.5, 0.01),
  SimpleKalmanFilter(0.5, 0.5, 0.01),
  SimpleKalmanFilter(0.5, 0.5, 0.01)
};

// Function to read load cell values and apply Kalman filtering
void readLoadCell() {
  int adcResult;
  double inputVoltage, force;
 
  for (int i = 0; i < 3; i++) {
    adcResult = analogRead(analogPins[i]);
    inputVoltage = 10.0 * adcResult / ((1 << adcResolution) - 1);
    force = scale_factor[i] * ((inputVoltage / 10) * LOAD_CELL_MAX_FORCE) - offset[i];
    load_cell[i] = kalmanFilters[i].updateEstimate(force);

    // Safety check - disable motors if load is excessive
    if (load_cell[i] > max_allowable_force) {
      Serial.print("excessive load cell force detected\n");
      input[0] = '1';
      return;
    }
  }
}

// Optimized inline force to torque conversion
inline double force2Torque(double force) {
  // Simplified calculation with constant multiplier
  return (force * 1.25 / 21.241) / maxTorqueMag * 100;
}

// Enable or disable motors based on num_motors value
void multipleMotorEnable(bool request) {
  motor1.EnableRequest(request);
  if (num_motors > 1) motor2.EnableRequest(request);
  if (num_motors > 2) motor3.EnableRequest(request);
}

// Command torque to motors
bool CommandTorque() {
  // Check if torque commands are within limits
  for (int i = 0; i < num_motors; i++) {
    if (abs(torque_command[i]) > maxTorque) {
      Serial.println("Move rejected, invalid torque requested");
      return false;
    }
    // Ensure torque command is non-negative (direction handled by MotorInAState)
    if (torque_command[i] < 0) torque_command[i] = 0;
  }

  // Calculate duty cycle once
  double scaleFactor = 255 / maxTorque;
  float duty[3];
 
  // Set all motors to same direction (motor3 handled separately inside the switch below)
  motor1.MotorInAState(false);
  motor2.MotorInAState(false);
  motor3.MotorInAState(false);

  // Calculate duty cycles
  for (int i = 0; i < num_motors; i++) {
    duty[i] = abs(torque_command[i]) * scaleFactor;
  }

  // Delay for Input A filter
  delay(20 + INPUT_A_FILTER);

  // Command motors and check HLFB
  motor1.MotorInBDuty(duty[0]);
  if (num_motors > 1) motor2.MotorInBDuty(duty[1]);
  if (num_motors > 2) motor3.MotorInBDuty(duty[2]);

  // Brief delay for HLFB to transition
  delay(1);

  // Check motor status
  bool fault = false;
  if (motor1.HlfbState() != MotorDriver::HLFB_ASSERTED) fault = true;
  if (num_motors > 1 && motor2.HlfbState() != MotorDriver::HLFB_ASSERTED) fault = true;
  if (num_motors > 2 && motor3.HlfbState() != MotorDriver::HLFB_ASSERTED) fault = true;

  if (fault) {
    Serial.println("Motor Fault or Overspeed Timeout Detected!");
    delay(200);
    multipleMotorEnable(false);
    input[0] = '1';
    return false;
  }
 
  return true;
}

// Ramp up motor to commanded force gradually
void ramp_up_motor(float *inputs) {
  bool ramped[3] = {false, false, false};
 
  // Initialize torque commands to zero
  for (int i = 0; i < 3; i++) {
    torque_command[i] = 0.0;
  }
 
  Serial.print("Motors ramping\n");
 
  while (!(ramped[0] || num_motors == 1) ||
         !(ramped[1] || num_motors <= 1) ||
         !(ramped[2] || num_motors <= 2)) {
   
    readLoadCell();
   
    for (int i = 0; i < num_motors; i++) {
      if (abs(load_cell[i] - inputs[i+1]) < 5.0) {
        ramped[i] = true;
      } else {
        torque_command[i] += (load_cell[i] < inputs[i+1]) ? 1.0 : -1.0;
      }
    }
   
    CommandTorque();
   
    // Check for abort condition
    if (input[0] == '1') {
      Serial.print("Motor ramp failed\n");
      return;
    }
   
    delay(200);
  }
 
  Serial.print("Motors ramped to commanded torque\n");
}

// Get number of motors from user input
void getNumMotors() {
  bool valid_input = false;
  int i = 0;
  char rc;
  char motor_num_in[IN_BUFFER_LEN+1];
 
  while (!valid_input) {
    Serial.print("input number of motors\n");
   
    // Wait for input
    while (Serial.available() == 0) {
      continue;
    }
   
    // Read input
    i = 0;
    while (Serial.available() > 0) {
      rc = Serial.read();
      motor_num_in[i++] = rc;
    }
    motor_num_in[i] = '\0';
   
    // Convert and validate
    int rint = atoi(motor_num_in);
    if (rint >= 1 && rint <= 3) {
      num_motors = rint;
      valid_input = true;
    } else {
      Serial.print("input should be 1, 2, or 3\n");
    }
  }
}

void setup() {
  // Set motor mode
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_A_DIRECT_B_PWM);

  // Initialize serial communication
  Serial.begin(baudRate);
  uint32_t timeout = 5000;
  uint32_t startTime = millis();
  while (!Serial && millis() - startTime < timeout) {
    continue;
  }

  // Set ADC resolution
  analogReadResolution(adcResolution);

  // Get number of motors from user
  getNumMotors();

  // Initialize state
  input[0] = '1';
 
  // Disable motors initially
  multipleMotorEnable(false);
  Serial.println(String(num_motors) + " Motor(s) Ready");
}

void loop() {
  // Read serial input
  if (Serial.available() > 0) {
    int i = 0;
    char rc;
   
    while (Serial.available() > 0) {
      rc = Serial.read();
      if (rc != '\n') {
        input[i++] = rc;
        if (i >= IN_BUFFER_LEN) break;
      } else {
        input[i] = '\0';
        break;
      }
    }
  }

  // Parse comma-separated inputs
  prev_input1 = input1;
 
  // Parse input string
  char input_temp[IN_BUFFER_LEN+1];
  strcpy(input_temp, input);
 
  float float_inputs[4] = {0};
  int i = 0;
 
  char *token = strtok(input_temp, ",");
  while (token != nullptr && i < 4) {
    float_inputs[i++] = atof(token);
    token = strtok(nullptr, ",");
  }
 
  // Extract state command
  input1 = float_inputs[0];

  // Read load cell values
  readLoadCell();
 
  // Calculate torque commands with PID control
  if (ramp_done) {
    unsigned long current_time = millis();
   
    for (int i = 0; i < num_motors; i++) {
      // Base torque command
      torque_command[i] = force2Torque(float_inputs[i+1]);
     
      // PID control calculations
      double F_err = float_inputs[i+1] - load_cell[i];
      double dt = (current_time - prev_time[i]) / 1000.0;
      double F_err_dot = (F_err - F_err_prev[i]) / dt;
     
      // Reset integral term on small errors
      /* if (abs(F_err) < 0.1) {
        integral[i] = 0;
      } else { */
        integral[i] += F_err * dt;
      //}
      //Serial.println(F_err);
     
      // Calculate PID adjustment
      double Tau_m_adj = kp[i] * F_err + ki[i] * integral[i] + kd[i] * F_err_dot;

      /* if(F_err<0 && Tau_m_adj<0 && F_err_dot<0)
      {
        Tau_m_adj=0;
      }
      if(F_err>0 && Tau_m_adj>0 && F_err_dot>0)
      {
        Tau_m_adj=0;
      } */

      if(force2Torque(Tau_m_adj+torque_command[i])>max_allowable_force)
      {
        Tau_m_adj=0;
      }
     
     
      // Add adjustment to base torque command
      torque_command[i] += force2Torque(Tau_m_adj);
     
      // Update previous values
      F_err_prev[i] = F_err;
      prev_time[i] = current_time;
    }
  }
 
  // State machine
  switch (input1) {
    case 0:
      // Reboot system
      NVIC_SystemReset();
      break;
     
    case 1:
      // Idle state - disable motors
      test_start = false;
      ramp_done = false;
     
      if (motor1.EnableRequest()) {
        multipleMotorEnable(false);
        Serial.print("motor(s) disabled\n");
      }
      break;
     
    case 2:
      // Enable motors but don't print data
      test_start = false;
     
      if (!motor1.EnableRequest()) {
        multipleMotorEnable(true);
        Serial.print("motor(s) enabled\n");
      }
     
      if (!ramp_done) {
        ramp_up_motor(float_inputs);
        ramp_done = true;
      } else {
        CommandTorque();
      }
      break;
     
    case 3:
      // Cannot jump directly to state 3 from 1
      if (prev_input1 == 1) {
        Serial.print("Invalid jump\n");
        input[0] = '1';
        break;
      }
     
      // Start timing if needed
      if (!test_start) {
        t_init = millis();
        test_start = true;
      }
     
      // Command motors
      CommandTorque();
     
      // Print data based on number of motors
      switch (num_motors) {
        case 1:
          // Serial.println(String(millis() - t_init) + ", " + String(load_cell[0]) + "\n");
          Serial.println(String(millis() - t_init) + ", " + String(load_cell[0]) + ", " + String(load_cell[0]) + ", " + String(load_cell[0]) + "\n");
          break;
        case 2:
          Serial.println(String(millis() - t_init) + ", " + String(load_cell[0]) + ", " + String(load_cell[1]) + "\n");
          break;
        case 3:
          Serial.println(String(millis() - t_init) + ", " + String(load_cell[0]) + ", " + String(load_cell[1]) + ", " + String(load_cell[2]) + "\n");
          break;
      }
      break;
     
    default:
      // Invalid state, set back to idle
      Serial.print("Invalid input\n");
      input[0] = '1';
      break;
  }
 
  // Small delay for stability
  delay(1);
}
