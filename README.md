# BOLT: Bike Overwatch & Location Tracker

## Overview
BOLT (Bike Overwatch & Location Tracker) is an advanced bike security system that provides real-time tracking and theft alerts. Designed for cyclists in cities with high bike theft rates, BOLT integrates motion detection, GPS tracking, and cloud-based notifications to enhance bike security.

## Features
- **Real-time Monitoring**: Detects unauthorized bike movement using an accelerometer.
- **Instant Alerts**: Sends email notifications with GPS coordinates when suspicious movement is detected.
- **Three Security Modes**:
  - **Locked Mode**: Actively monitors movement.
  - **Alert Mode**: Triggers an alert and sends GPS coordinates.
  - **Unlocked Mode**: Allows free movement without alerts.
- **Wireless Communication**: Uses AWS IoT and HTTP requests for cloud connectivity.
- **User Interface**:
  - IR remote-controlled state transitions.
  - OLED display for system status.
  - Reset button to clear alerts.

## System Components
### Hardware
- **CC3200 LaunchPad**: Microcontroller for processing and communication.
- **GT-U7 NEO-6M GPS Module**: Captures real-time location.
- **Accelerometer (Bosch in-built)**: Detects unauthorized movement.
- **IR Receiver Module (Vishay TSOP31336)**: Receives input from an IR remote.
- **128x128 OLED Display (Adafruit SSD1351)**: Displays system states.
- **AT&T S10-S3 Universal Remote**: Used to switch between states.
- **Wireless Communication**: AWS IoT SNS for email alerts.
- **Reset Button (GPIO13 - SW3)**: Resets the system in Alert mode.

## Software Implementation
- **Microcontroller Setup**:
  - Configures GPIO, UART, I2C, SPI for different peripherals.
  - Manages state transitions between **Locked**, **Unlocked**, and **Alert** modes.
- **Motion Detection**:
  - Monitors accelerometer data.
  - Triggers alert when predefined motion threshold is exceeded.
- **GPS Tracking**:
  - Parses `$GPRMC` NMEA sentences.
  - Extracts latitude and longitude for email alerts.
- **Wireless Communication**:
  - Secure AWS IoT connection.
  - Sends GPS coordinates via HTTP POST request.
- **Infrared (IR) Remote Control**:
  - Decodes 48-bit IR signals.
  - Matches last 16 bits to predefined commands for locking and unlocking.
- **OLED Display**:
  - Displays system status and alerts.

## Circuit Connections
- **Accelerometer**: Connected via I2C (SCL: Pin 1, SDA: Pin 2).
- **GPS Module**: Uses UART1 (TX: Pin 58, RX: Pin 59).
- **IR Receiver**: Connected to GPIO Pin 3.
- **OLED Display**: SPI communication (CS: Pin 18, DC: Pin 45, MOSI: Pin 7, CLK: Pin 5, RESET: Pin 8).



## Setup Instructions
### Hardware Setup
1. Connect the CC3200 LaunchPad to your computer via USB.
2. Assemble the circuit as per the circuit diagram.
3. Power the GPS module and accelerometer using 3.3V from the LaunchPad.

### Software Installation
1. Install **Code Composer Studio (CCS)** or **Energia IDE**.
2. Install the necessary libraries for IR decoding, OLED display, and AWS IoT communication.
3. Upload the firmware to CC3200 using CCS/Energia.
4. Configure AWS IoT credentials in the firmware.

### Running the System
1. Power on the device.
2. Use the IR remote to switch between **Locked** and **Unlocked** modes.
3. When motion is detected, an alert is triggered, and GPS coordinates are emailed.
4. Use the reset button to clear the alert.

## Usage Guide
- **Lock the Bike**: Press the designated button on the IR remote.
- **Unlock the Bike**: Use the IR remote again.
- **Receive Alerts**: Check your email when unauthorized movement is detected.
- **Reset Alert**: Press the reset button after retrieving your bike.

## Circuit Connections
- **Accelerometer**: Connected via I2C (SCL: Pin 1, SDA: Pin 2).
- **GPS Module**: Uses UART1 (TX: Pin 58, RX: Pin 59).
- **IR Receiver**: Connected to GPIO Pin 3.
- **OLED Display**: SPI communication (CS: Pin 18, DC: Pin 45, MOSI: Pin 7, CLK: Pin 5, RESET: Pin 8).

## Challenges Faced
- **GPS Data Parsing**: Extracting meaningful data from NMEA sentences.
- **AWS IoT Connectivity**: Maintaining stable cloud communication.
- **Motion Detection Calibration**: Preventing false positives by adjusting sensitivity.

## Future Enhancements
- **Biometric Authentication**: Add fingerprint verification.
- **Mobile App Integration**: Remote tracking and alerts via a smartphone app.
- **Geofencing**: Implement Google Maps API to set predefined security perimeters.

## Contributors
- **Anirudh Venkatachalam**  
  - GitHub: [github.com/anirudhvee](https://github.com/anirudhvee)  
  - LinkedIn: [linkedin.com/in/anirudhvee](https://linkedin.com/in/anirudhvee)  
- **Manasvini Narayanan**  
  - GitHub: [github.com/mana-nara](https://github.com/mana-nara)  
  - LinkedIn: [linkedin.com/in/mana-nara](https://www.linkedin.com/in/mana-nara/)  

## Project Website
[Visit the BOLT Web App](https://bolt-eec172.web.app/)




