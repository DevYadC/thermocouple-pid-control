const int INPUT_PIN = A0; // thermocouple reading
const int OUTPUT_PIN = DD3; //voltage to coil heater

double dt, last_time;
double integral, previous_error, prevOutput = 0;
double kp = 0.00, ki = 0.05, kd = 0.0003; 
const double setpoint = 200.00; // celcius

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

