/*
  input load cell analog data, add offset and scaling factor and run moving average on it
 */

#include <RunningAverage.h>   

// Defines the bit-depth of the ADC readings (8-bit, 10-bit, or 12-bit)
// Supported adcResolution values are 8, 10, and 12
#define adcResolution 12

// Select the baud rate to match the target device.
#define baudRate 115200

// pounds
#define LOAD_CELL_MAX_FORCE 220.462

// moving average object
RunningAverage avgWeight(2000);   

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
    int adcResult = analogRead(A10);
    // Convert the reading to a voltage.
    double inputVoltage = 10.0 * adcResult / ((1 << adcResolution) - 1);
    
    // load cell 1
    double offset = 1.5;
    double scale_factor = 0.977662904235;
    // 0.98837964

    // load cell 2
    // double offset = 3.3;
    // double scale_factor = 0.974399048854;

    // load cell 3
    // double offset = 1.34;
    // double scale_factor = 0.975303160436;
// 0.9817549956559514

    // TODO: offset and scale factor
    double corrected_force = scale_factor*((inputVoltage/10)*LOAD_CELL_MAX_FORCE) - offset;
    avgWeight.addValue(corrected_force);
    float avg = avgWeight.getAverage();
    // Display the voltage reading to the USB serial port.
    Serial.print(corrected_force);
    Serial.print("\n");
    Serial.print(avg);
    
    Serial.print("\n\n");

    // Wait a second before the next reading.
    delay(1);
}
