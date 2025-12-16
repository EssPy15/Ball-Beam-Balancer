#include <Servo.h>
#include <PID_v1.h>

// --- Pin Definitions ---
const int trigPin = 9;
const int echoPin = 10;
const int servoPin = 11;

// --- PID constants ---
float Kp = 30.0;
float Ki = 10.0;
float Kd = 6.0;

// --- Variables ---
double Setpoint, Input, Output, ServoOutput;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
Servo myServo;

// --- Function Prototype ---
float readPosition();

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  myServo.attach(servoPin);

  // --- Initialize servo at 90° (neutral position) ---
  myServo.write(90);
  delay(1000);

  // --- PID setup ---
  Setpoint = 22.0;                // Target distance (midpoint)
  Input = readPosition();         // Initial sensor read
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-40, 40); // PID output range

  Serial.println("System initialized. Starting control loop...");
  Serial.println("Dist(cm)\tError\tPID_Out\tServoCmd\tServoRead");
}

void loop() {
  delay(100);

  // --- Read current distance from ultrasonic sensor ---
  Input = readPosition();

  if (Input > 0) {
    // --- Compute PID ---
    myPID.Compute();

    // --- Correct control direction ---
    // Closer than 22cm → increase servo angle
    // Farther than 22cm → decrease servo angle
    if(Input > 21.5 && Input < 23)
    ServoOutput = 90;
    else ServoOutput = 90 + Output;

    // --- Constrain servo angle safely ---
    ServoOutput = constrain(ServoOutput, 0, 170);

    // --- Write angle to servo ---
    myServo.write(ServoOutput);

    // --- Compute error ---
    double error = Setpoint - Input;

    // --- Print sensor + servo info ---
    Serial.print(Input, 2);         // Distance reading
    Serial.print("\t");
    Serial.print(error, 2);         // Error
    Serial.print("\t");
    Serial.print(Output, 2);        // PID output
    Serial.print("\t");
    Serial.print(ServoOutput, 2);   // Servo command
    Serial.print("\t");
    Serial.println(myServo.read()); // Feedback
  }
}

// --- Ultrasonic distance reading function ---
float readPosition() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);  // 30 ms timeout
  float distance = duration * 0.034 / 2;          // Convert to cm

  return distance;
}
