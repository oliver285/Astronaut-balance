/*
 * Title: ReadAnalogInput
 *
 * Objective:
 *    This example demonstrates how to read the analog voltage of an analog
 *    input. ClearCore analog inputs are natively compatible with 0-10V signals,
 *    and 0-20mA signals with addition of an external resistor (see ClearCore
 *    manual link below for wiring).
 *
 * Description:
 *    This example sets up pin A-12 as an analog input, queries the value on
 *    that connector every second, and calculates the input voltage. This
 *    calculated voltage is written/displayed to the USB serial port.
 *    Connectors IO-0 through IO-5 will act as a coarse meter of the voltage
 *    read-in by turning on their outputs and LEDs (with more than 0V and less
 *    than 2V, IO-0 will be on as an output. With ~2V and less than 4V, IO-0 and
 *    IO-1 will be on; and so on until, with near 10V read in, IO-0 through IO-5
 *    will be turned on).
 *
 * Requirements:
 * ** An analog input source connected to A-12.
 * ** Optional: LEDs connected to IO-0 through IO-5 to act as a more prominent
 *    voltage meter than the ClearCore's onboard LEDs.
 *
 * Links:
 * ** ClearCore Documentation: https://teknic-inc.github.io/ClearCore-library/
 * ** ClearCore Manual: https://www.teknic.com/files/downloads/clearcore_user_manual.pdf
 *
 *
 * Copyright (c) 2020 Teknic Inc. This work is free to use, copy and distribute under the terms of
 * the standard MIT permissive software license which can be found at https://opensource.org/licenses/MIT
 */

// Defines the bit-depth of the ADC readings (8-bit, 10-bit, or 12-bit)
// Supported adcResolution values are 8, 10, and 12
#define adcResolution 12

// Select the baud rate to match the target device.
#define baudRate 115200

// pounds
#define LOAD_CELL_MAX_FORCE 220.462

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

    // // Make a voltage meter display with the I/O pins.
    // pinMode(IO0, OUTPUT);
    // pinMode(IO1, OUTPUT);
    // pinMode(IO2, OUTPUT);
    // pinMode(IO3, OUTPUT);
    // pinMode(IO4, OUTPUT);
    // pinMode(IO5, OUTPUT);

    // // Clear out the state of our voltage meter to start.
    // digitalWrite(IO0, false);
    // digitalWrite(IO1, false);
    // digitalWrite(IO2, false);
    // digitalWrite(IO3, false);
    // digitalWrite(IO4, false);
    // digitalWrite(IO5, false);



    // Since analog inputs default to analog input mode, there's no need to
    // call pinMode().

    // Set the resolution of the ADC.
    analogReadResolution(adcResolution);
}

void loop() {
    // Put your main code here, it will run repeatedly:

    // Read the analog input (A-9 through A-12 may be configured as analog
    // inputs).
    int adcResult1 = analogRead(A12);
    int adcResult2 = analogRead(A11);
    int adcResult = adcResult1 - adcResult2;
    // Convert the reading to a voltage.
    double inputVoltage = 10.0 * adcResult / ((1 << adcResolution) - 1);

    // Alternatively, you can use the following to get a measurement in
    // volts:
    // double inputVoltage = analogRead(A12, MILLIVOLTS) / 1000.0;
    double load_cell_force = (inputVoltage/10)*LOAD_CELL_MAX_FORCE;
    // Display the voltage reading to the USB serial port.
    Serial.print(load_cell_force);
        Serial.print("\n");


    // Wait a second before the next reading.
    delay(200);
}
