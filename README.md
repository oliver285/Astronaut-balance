🚀 Astronaut Balancing System
The Astronaut Balancing System is an embedded mechatronics project designed to simulate gravitational force and aid astronaut postural training in microgravity. The system uses tethered torque-controlled motors, real-time sensor feedback, and dynamic force estimation to model balance mechanics and muscle feedback during extended space missions.

This project is currently under active development, with a focus on testing single-motor behavior and progressing toward a 3-motor configuration.

🧠 Core Objectives
Simulate gravitational load via a 3-motor tether system

Provide astronauts with postural feedback in simulated microgravity

Measure and adjust tether lengths, forces, and angles in real time

Interface with an external GUI for control, monitoring, and data logging

⚙️ Current Features
✅ Embedded Motor Control
Real-time torque control via ClearCore motor driver (Arduino/C++)

Encoder integration to track angular position and tether displacement

Load cell readings for force measurement

✅ Sensor Feedback Loop
Uses load cells and rotary encoders to determine tether length and applied force

Computes estimated gravitational force replication

Performs force-to-torque conversion in software

✅ Data Logging & UI Integration
Integrates with custom Python GUI (developed by Ethan) via serial communication

Logs the following to CSV:

Tether lengths

Applied force error

Angle error

Mass estimation error

🧪 Development Roadmap
🔧 Phase 1: Single-Motor Characterization (In Progress)
 Estimate force-to-torque and encoder-to-length relationships mathematically

 Integrate encoder + load cell into single motor control loop

 Validate tether length computation from encoder data

 Validate force measurement from load cell output

🔧 Phase 2: Full 3-Motor Implementation
 Expand verified single-motor setup to 3-motor tether control

 Implement motor state switching from computer interface (e.g., Idle → Engage → Release)

 Add safety cutoff logic and torque ramping

🔧 Phase 3: UI + CSV Integration
 Extend Ethan's GUI to support:

Real-time state switching

Manual override & live telemetry

 CSV logging of:

Tether lengths

Force & angle errors

Mass estimation error

📦 Planned Additions
PID control for closed-loop balancing

Kalman filter for force estimation smoothing

Battery backup or isolated power system for portable use

Multi-axis simulation (hip, shoulder tethering)

🧠 Notes
This system is designed to contribute to real-time countermeasures against balance deterioration and muscle atrophy during space missions. The final design will be tested in conjunction with a wearable harness and controlled under varied simulated gravity vectors.
