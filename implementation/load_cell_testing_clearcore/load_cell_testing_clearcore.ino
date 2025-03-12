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
#define scale_factor 1.1316

// moving average object
RunningAverage avgWeight(50);   

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

    // Read the analog input (A-9 through A-12 may be configured as analog
    // inputs).
    // TODO: not reading correctly when ground hooked into A12, only A12-A11
    int adcResult1 = analogRead(A12);
    int adcResult2 = analogRead(A11);
    int adcResult = adcResult1 - adcResult2;
    // Convert the reading to a voltage.
    double inputVoltage = 10.0 * adcResult / ((1 << adcResolution) - 1);
    
    double tare = 1.4;
    // TODO: offset and scale factor
    double meas_load_cell_force = (inputVoltage/10)*LOAD_CELL_MAX_FORCE - tare;
    float corrected_force = meas_load_cell_force*scale_factor;
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
