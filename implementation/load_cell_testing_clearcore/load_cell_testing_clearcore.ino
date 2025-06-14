/*
  input load cell analog data, add offset and scaling factor and run moving average on it
 */

#include <RunningAverage.h>   
#include <SimpleKalmanFilter.h>

// Defines the bit-depth of the ADC readings (8-bit, 10-bit, or 12-bit)
// Supported adcResolution values are 8, 10, and 12
#define adcResolution 12

// Select the baud rate to match the target device.
#define baudRate 115200

// pounds
#define LOAD_CELL_MAX_FORCE 220.462

SimpleKalmanFilter simpleKalmanFilter(0.5, 0.5, 0.01);
// moving average object
RunningAverage avgWeight(10);   

void setup() {
    // Put your setup code here, it will only run once:

    // Initialize the serial port for printing analog voltage readings and wait
    // up to 5 seconds for a port to open. Serial communication is not required
    // for this example to run, however without it only the coarse LED meter
    // can be used to read the analog signal.
    Serial.begin(baudRate);
    uint32_t timeout = 5000;
    uint32_t startTime = millis();
    while (!Serial && millis() - startTime < timeout) {
        continue;
    }

    // Since analog inputs default to analog input mode, there's no need to
    // call pinMode().

    // Set the resolution of the ADC.
    analogReadResolution(adcResolution);

}

void loop() {
    // Put your main code here, it will run repeatedly:

    // A10, A11, or A12
    int adcResult1 = analogRead(A10);
    // Convert the reading to a voltage.
    double inputVoltage1 = 10.0 * adcResult1 / ((1 << adcResolution) - 1);
    
    // load cell 1
    double offset1 = 1.5;
    double scale_factor1 = 0.977662904235;
    // 0.98837964


        // A10, A11, or A12
    int adcResult2 = analogRead(A12);
    // Convert the reading to a voltage.
    double inputVoltage2 = 10.0 * adcResult2 / ((1 << adcResolution) - 1);

    // load cell 2
    double offset2 = 3.3;
    double scale_factor2 = 0.974399048854;

    // load cell 3
    // double offset = 1.34;
    // double scale_factor = 0.975303160436;
// 0.9817549956559514

    // TODO: offset and scale factor
    double corrected_force1 = scale_factor1*((inputVoltage1/10)*LOAD_CELL_MAX_FORCE) - offset1;
    avgWeight.addValue(corrected_force1);
    float avg1 = avgWeight.getAverage();

    double corrected_force2 = scale_factor2*((inputVoltage2/10)*LOAD_CELL_MAX_FORCE) - offset2;
    float avg2 = simpleKalmanFilter.updateEstimate(corrected_force1);

    Serial.print(String(millis()) + ", " + String(corrected_force1) + ", " + String(avg1) + ", " + String(avg2) + "\n");

    // Wait a second before the next reading.
    delay(1);
}
