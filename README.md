
# Automatic Medicine Delivery System

This repository contains the code, hardware connection instructions, and documentation for the Automatic Medicine Delivery System. This system utilizes an Arduino Mega to control various sensors and actuaries including a TCS34725 RGB sensor, servo motors, an L293D motor driver, and line tracking sensors.

## Project Overview

The Automatic Medicine Delivery System is designed to automate the delivery process within healthcare facilities using robotic technology. Key components of this system include color sensing for medicine verification, servo motors for handling operations, and line tracking for navigation.

## Hardware Setup

### TCS34725 RGB Color Sensor Connection
Connect the TCS34725 RGB sensor with the Arduino Mega as follows:
1. **GND (Ground)**: Connect the GND pin on the color sensor module to any GND pin on the Arduino.
2. **SDA (Serial Data)**: Connect Pin A4/20 on the Arduino to the SDA pin on the color sensor.
3. **SCL (Serial Clock)**: Connect Pin A5/21 on the Arduino to the SCL pin on the color sensor.
4. **3.3V Power**: Connect the sensor pin labelled 3.3V to the 3.3V pin on the Arduino.

### Servo Motor Connections
Connect three servo motors to the Arduino Mega for actuation:
1. **Red Servo (Signal)**: Connect to Digital Pin 9.
2. **Green Servo (Signal)**: Connect to Digital Pin 10.
3. **Blue Servo (Signal)**: Connect to Digital Pin 11.
4. **Power (VCC)**: Connect the power lines of all servos to the 5V pin on the Arduino Mega.
5. **Ground (GND)**: Connect the ground lines of all servos to any GND pin on the Arduino Mega.

### L293D Motor Driver Connection
Set up the L293D motor driver by connecting:
1. **12V**: Connect to a 12V power supply.
2. **GND (Ground)**: Connect to the ground of the power source.
3. **5V**: Connect to the 5V pin on the Arduino Mega.
4. **Motors**: Connect two motors to M1 and M2, and the other two to M3 and M4.
5. **Control Pins**:
   - IN1 to Arduino Mega Pin 24
   - IN2 to Pin 25
   - IN3 to Pin 26
   - IN4 to Pin 27

### Line Tracking Sensor Connections
Connect the line tracking sensors to the Arduino Mega for navigation:
1. **VCC**: Connect to the 5V pin on the Arduino.
2. **GND (Ground)**: Connect to any GND pin on the Arduino.
3. **Sensor Pins**:
   - LEFT_SENSORPIN to Digital Pin 42
   - CENTER_SENSORPIN to Digital Pin 44
   - RIGHT_SENSORPIN to Digital Pin 46

## Software Installation

upload the Arduino sketch provided in the repository to your Arduino Mega using the Arduino IDE.

## Usage

After setting up the hardware and uploading the code:
- Power up the Arduino Mega.
- The system will start to operate automatically, navigating along predefined paths and using the RGB sensor to verify medicine packages.
- Monitor the operations via the serial monitor in the Arduino IDE.
