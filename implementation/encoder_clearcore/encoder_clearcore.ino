#include "ClearCore.h"

// Define pins for encoder inputs
const int encoderPinA = IO0;  // Encoder A connected to IO-0
const int encoderPinB = IO1;  // Encoder B connected to IO-1

// Encoder tracking variables
volatile long encoderPosition = 0;
volatile bool pinAState = false;
volatile bool pinBState = false;
volatile bool lastPinAState = false;
volatile bool lastPinBState = false;
volatile unsigned long stateChangeCount = 0;

// For direct signal monitoring
int lastDisplayedA = -1;
int lastDisplayedB = -1;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(2000);
 
  Serial.println("ClearCore Encoder Diagnostic Tool");
  Serial.println("================================");
 
  // Set encoder pins as inputs (try both with and without pullup)
  pinMode(encoderPinA, INPUT);  // Changed from INPUT_PULLUP
  pinMode(encoderPinB, INPUT);  // Changed from INPUT_PULLUP
 
  // Read and store initial states
  pinAState = digitalRead(encoderPinA);
  pinBState = digitalRead(encoderPinB);
  lastPinAState = pinAState;
  lastPinBState = pinBState;
 
  // Serial.println("Monitoring encoder signals directly...");
  // Serial.println("Turn the encoder and watch for signal changes.");
  // Serial.println("A B | Count");
  // Serial.println("----+------");
}

void loop() {
  // Read current state
  pinAState = digitalRead(encoderPinA);
  pinBState = digitalRead(encoderPinB);
 
  // Check for state changes WITHOUT using interrupts
  if (pinAState != lastPinAState || pinBState != lastPinBState) {
    // Increment state change counter
    stateChangeCount++;
   
    // Simple encoder logic - increment on rising edge of pin A
    if (pinAState && !lastPinAState) {
      // Check direction based on pin B
      if (!pinBState) {
        encoderPosition++;
      } else {
        encoderPosition--;
      }
    }
   
    // Store current states as last states
    lastPinAState = pinAState;
    lastPinBState = pinBState;
   
    // // Print the current state
    // Serial.print(pinAState ? "1" : "0");
    // Serial.print(" ");
    // Serial.print(pinBState ? "1" : "0");
    // Serial.print(" | ");
    // Serial.print(stateChangeCount);
    // Serial.print(" | Pos: ");
    // Serial.println(encoderPosition);
  }
 
  // Regular updates for signal monitoring
  bool stateChanged = false;
  int currentA = digitalRead(encoderPinA);
  int currentB = digitalRead(encoderPinB);
 
  if (currentA != lastDisplayedA || currentB != lastDisplayedB) {
    lastDisplayedA = currentA;
    lastDisplayedB = currentB;
    stateChanged = true;
  }
 
  // Calculate and display position every second
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate >= 1000 || stateChanged) {
    lastUpdate = millis();
   
    float degrees = (float)encoderPosition * 360.0 / 600;
    while (degrees < 0) degrees += 360.0;
    while (degrees >= 360.0) degrees -= 360.0;
   
    // Serial.print("Summary - Count: ");
    // Serial.print(stateChangeCount);
    // Serial.print(" | Position: ");
    // Serial.print(encoderPosition);
    // Serial.print(" | Angle: ");
    Serial.println(degrees);
    // Serial.print("° | A:");
    // Serial.print(currentA);
    // Serial.print(" B:");
    // Serial.println(currentB);
  }
 
  // Small delay for stability
  // delay(1);
}