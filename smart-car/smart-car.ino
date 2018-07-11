#define SENSOR_PIN_LEFT A1
#define SENSOR_PIN_RIGHT A2
#define POTENTIOMETER_PIN A3

#define LED_PIN_LEFT 5
#define LED_PIN_RIGHT 4


//#define BRIGHTNESS_MAX 900 replaced by potentiometer
#define BRIGHTNESS_MIN 100
#define BRIGHTNESS_MIN_DELTA 30
#define MIN_DISTANCE 10 
#define DRIVE_DURATION 100

int sensorValueLeft = 0;
int sensorValueRight = 0;
int maxSinceStart = 0;
int potentiometerValue = 0;
int obstacleDistance = 0;

#include <HC_SR04.h>

#define TRIGGER_PIN 3
#define ECHO_PIN 2
#define ECHO_INT 0   // The HC_SR04 echo pin is connected to Arduino pin 2 which is interrupt id 0)  

HC_SR04 sensor(TRIGGER_PIN, ECHO_PIN, ECHO_INT);  // Create the sensor object

enum direction_t {
  STOP,
  LOST,
  LEFT,
  RIGHT,
  FORWARD
};

// when lost: should we check if there was light before (since restart)
// and drive back if so ? (max since start > MIN )
const direction_t getDirection(int left, int right, int brightnessMax) {
  
  if (left <= BRIGHTNESS_MIN && right <= BRIGHTNESS_MIN) {
    return LOST;
  } else if (left > brightnessMax && right > brightnessMax) {
    return STOP; // goal reached
    // else if only one is > MAX
  } else if (abs(left - right) < BRIGHTNESS_MIN_DELTA) {
    return FORWARD;
  } else if (left > right + BRIGHTNESS_MIN_DELTA) {
    return LEFT;
  } else if (right > left + BRIGHTNESS_MIN_DELTA) {
    return RIGHT;
  }
}



//----------------
/*
Adafruit Arduino - Lesson 15. Bi-directional Motor
*/

//const int enablePin = 11; needs PWM

const int motorPin1 = 6;
const int motorPin2 = 7;
const int motorPin3 = 8;
const int motorPin4 = 9;
const int motorPinEN1 = 10;
const int motorPinEN2 = 11;
const int defaultSpeed = 125;

const String directionToString(const direction_t direction) {
  switch (direction) {
    case LEFT:
      return String("LEFT");
    case RIGHT:
      return String("RIGHT");
    case FORWARD:
      return String("FORWARD");
    case LOST:
      return String("LOST");
    case STOP:
      return String("STOP");
    default:
      return String("Unknown");
  }
}

void switchOffLeft() {
  digitalWrite(LED_PIN_LEFT, LOW);
}

void switchOffRight() {
  digitalWrite(LED_PIN_RIGHT, LOW);
}

void indicateLeft() {
  switchOffRight();
  digitalWrite(LED_PIN_LEFT, HIGH);
}

void indicateRight() {
  switchOffLeft();
  digitalWrite(LED_PIN_RIGHT, HIGH);
}

void indicateForward() {
  digitalWrite(LED_PIN_LEFT, HIGH);
  digitalWrite(LED_PIN_RIGHT, HIGH);
}

void indicateStop() {
  switchOffRight();
  switchOffLeft();
}

void indicatePanic() {
  for (int i = 0; i < 5; i++) {
    indicateForward();
    delay(900);
    indicateStop();
  }
}

/*
 * Input1 Input2 Result
 *   0      0      Stop
 *   0      1      Anti Clockwise
 *   1      0      Clockwise
 *   1      1      Stop
 */
void setSpeedMotor1(const unsigned int value){
  if(value >= 255){
    analogWrite(motorPinEN1, 255);
  }else{
    analogWrite(motorPinEN1, value);
  }
}

void setSpeedMotor2(const unsigned int value){
  if(value >= 255){
    analogWrite(motorPinEN2, 255);
  }else{
    analogWrite(motorPinEN2, value);
  }
}


// FIXME use enable pin: off, adjust, on
void driveForwards(unsigned long duration){
  Serial.println("Starting motor 1");
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin3, HIGH);
  delay(duration);
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin3, LOW);
}

// FIXME use enable pin: off, adjust, on
void driveBackwards(unsigned long duration){
  Serial.println("Starting motor 2");
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin4, HIGH);
  delay(duration);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin4, LOW);
}

void turnLeft(){
  Serial.println("turning left");
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin3, HIGH);
  delay(DRIVE_DURATION);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
}

void turnRight(){
  Serial.println("turning right");
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin4, HIGH);
  delay(DRIVE_DURATION);
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin4, LOW);
}

const unsigned int getDistance(){
  sensor.start();                      // Start the sensor
  while(!sensor.isFinished()) {};   // If the sensor does not have a reading, continue
  const int range = sensor.getRange();
  Serial.println(String(range) + " cm");
  return range;
}

#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);
long timer = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets();
  
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  
  pinMode(SENSOR_PIN_LEFT, INPUT);
  pinMode(SENSOR_PIN_RIGHT, INPUT);

  pinMode(LED_PIN_LEFT, OUTPUT);
  pinMode(LED_PIN_RIGHT, OUTPUT);

  pinMode(POTENTIOMETER_PIN, INPUT);
  sensor.begin();
  
  setSpeedMotor1(defaultSpeed);
  setSpeedMotor2(defaultSpeed);
}

void loop() {
  mpu6050.update();
  long delta = millis() - timer;
  if(delta > 1000){
    Serial.println("=======================================================");
    Serial.print("temp : ");Serial.println(mpu6050.getTemp());
    Serial.print("accX : ");Serial.print(mpu6050.getAccX());
    Serial.print("\taccY : ");Serial.print(mpu6050.getAccY());
    Serial.print("\taccZ : ");Serial.println(mpu6050.getAccZ());
    timer = millis();
  }else{
//    Serial.println("Waiting...");
//    Serial.println(timer);
//    Serial.println(delta);
//    delay(200);  
  }
  
  sensorValueLeft    = analogRead(SENSOR_PIN_LEFT);
  sensorValueRight   = analogRead(SENSOR_PIN_RIGHT);
  potentiometerValue = analogRead(POTENTIOMETER_PIN);
  obstacleDistance   = getDistance();
  direction_t direction = getDirection(sensorValueLeft, sensorValueRight, potentiometerValue);
  
  const String directionName = directionToString(direction);

  Serial.println(String("L: ") + sensorValueLeft + ",R: " + sensorValueRight + ", P " +  potentiometerValue + 
                        ", D: " + directionName + ", R: " + obstacleDistance);

  if(obstacleDistance < MIN_DISTANCE){
    Serial.println("Close obstacle found, stopping.");
    indicateStop();
    delay(500);
    indicateLeft();
    turnLeft();
    return;
  }else{
    Serial.println("NO obstacle found");
  }

  switch (direction) {
    case LEFT:
      indicateLeft();
      turnLeft();
      break;
    case RIGHT:
      indicateRight();
      turnRight();
      break;
    case FORWARD:
      indicateForward();
      driveForwards(DRIVE_DURATION * 2);
      break;
    case LOST:
      indicatePanic();
      break;
    case STOP:
      indicateStop();
      break;
    default:
      indicatePanic();
      break;
  }
  delay(200);
}
