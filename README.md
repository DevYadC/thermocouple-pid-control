# PID Temperature Control for 24V Coil Heater

## Overview

This project uses a Proportional-Integral-Derivative (PID) controller to regulate the temperature of a 24V coil heater. The PID controller continuously adjusts the heater's output to maintain a desired setpoint temperature.

## Components

- **Microcontroller**: Arduino 
- **Heater**: 24V coil heater
- **Temperature Sensor**: Analog thermocouple temperature sensor connected to `A0`
- **Output Control**: Heater control connected to digital pin `D3`

## Code Description

The code reads the temperature from the analog sensor, calculates the error between the desired temperature setpoint and the actual temperature, and adjusts the heater's output using a PID controller to minimize the error.

### Pin Configuration

- `INPUT_PIN` (A0): Reads the analog temperature sensor value.
- `OUTPUT_PIN` (D3): Controls the heater output using PWM.

### PID Parameters

- `kp`: Proportional gain (set to `0.00` initially, can be adjusted).
- `ki`: Integral gain (set to `0.05` initially, can be adjusted).
- `kd`: Derivative gain (set to `0.0003` initially, can be adjusted).
- `setpoint`: Desired temperature setpoint (set to `200.00`).

### Key Variables

- `dt`: Time difference between successive readings.
- `integral`: Accumulated integral of the error.
- `previous_error`: Error from the previous cycle.
- `prevOutput`: Previous output value.

### Functions

- `double pid(double error)`: Computes the PID output based on the error.
- `void setup()`: Initializes the serial communication and the output pin.
- `void loop()`: Main loop where the temperature is read, error is calculated, PID output is computed, and the heater output is adjusted.

## Usage

1. **Setup the Hardware**:
   - Connect the analog temperature sensor to the `A0` pin.
   - Connect the heater control to the `D3` pin.
   - Ensure the 24V coil heater is properly connected and powered.

2. **Upload the Code**:
   - Use the Arduino IDE to upload the provided code to your microcontroller.

3. **Run the System**:
   - Monitor the serial output for the setpoint, actual temperature, error, and output values.

## Code

```cpp
const int INPUT_PIN = A0;
const int OUTPUT_PIN = D3;

double dt, last_time;
double integral, previous_error, prevOutput = 0;
double kp = 0.00, ki = 0.05, kd = 0.0003;
const double setpoint = 700.00;

double pid(double error) {
  double proportional = error;
  integral += error * dt;
  double derivative = (error - previous_error) / dt;
  previous_error = error;
  double output = (kp * proportional) + (ki * integral) + (kd * derivative);
  return output;
}

void setup() {
  last_time = 0;
  Serial.begin(9600);
  analogWrite(OUTPUT_PIN, 0);
}

void loop() {
  double now = millis();
  dt = (now - last_time) / 1000.0;
  last_time = now;

  double actual = analogRead(INPUT_PIN);
  double error = setpoint - actual;

  double output = pid(error);
  output = constrain(output, 0, 255);
  analogWrite(OUTPUT_PIN, output);

  Serial.print(setpoint);
  Serial.print("\t");
  Serial.print(actual);
  Serial.print("\t");
  Serial.print(error);
  Serial.print("\t");
  Serial.println(output * 20);

  delay(300);
}
