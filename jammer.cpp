// Pin definitions
#define LEFT_SENSOR_PIN 32
#define MIDDLE_SENSOR_PIN 33
#define RIGHT_SENSOR_PIN 34

#define MOTOR_A_PWM 18
#define MOTOR_B_PWM 19
#define MOTOR_A_IN1 2
#define MOTOR_A_IN2 4
#define MOTOR_B_IN1 16
#define MOTOR_B_IN2 17

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Set sensor pins as input
  pinMode(LEFT_SENSOR_PIN, INPUT);
  pinMode(MIDDLE_SENSOR_PIN, INPUT);
  pinMode(RIGHT_SENSOR_PIN, INPUT);

  // Set motor pins as output
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  // Initialize motors
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
}

void loop() {
  // Read sensor values
  int leftSensor = digitalRead(LEFT_SENSOR_PIN);
  int middleSensor = digitalRead(MIDDLE_SENSOR_PIN);
  int rightSensor = digitalRead(RIGHT_SENSOR_PIN);

  // Line following logic
  if (middleSensor == LOW) {
    // Move forward
    moveForward();
  } else if (leftSensor == LOW) {
    // Turn left
    turnLeft();
  } else if (rightSensor == LOW) {
    // Turn right
    turnRight();
  } else {
    // Stop if no line detected
    stopMotors();
  }
}

void moveForward() {
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  analogWrite(MOTOR_A_PWM, 255);

  digitalWrite(MOTOR_B_IN1, HIGH);
  digitalWrite(MOTOR_B_IN2, LOW);
  analogWrite(MOTOR_B_PWM, 255);
}

void turnLeft() {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  analogWrite(MOTOR_A_PWM, 255);

  digitalWrite(MOTOR_B_IN1, HIGH);
  digitalWrite(MOTOR_B_IN2, LOW);
  analogWrite(MOTOR_B_PWM, 255);
}

void turnRight() {
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  analogWrite(MOTOR_A_PWM, 255);

  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, HIGH);
  analogWrite(MOTOR_B_PWM, 255);
}

void stopMotors() {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  analogWrite(MOTOR_A_PWM, 0);

  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
  analogWrite(MOTOR_B_PWM, 0);
}

