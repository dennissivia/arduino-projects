#include "config.h"
#include <ArduinoSTL.h>

using namespace std;

enum pulse_value_t {
  DEGREE,
  PULSE
};

std::vector<String> legPositions;
std::vector<String> legList;
std::vector<std::pair<int, int> > legSequence;
std::vector<String> legCommands;



const std::vector<int> rightRearJoints = {cRRCoxaPin , cRRFemurPin , cRRTibiaPin};
const std::vector<int> rightMidJoints = {cRMCoxaPin , cRMFemurPin , cRMTibiaPin};
const std::vector<int> rightFrontJoints = {cRFCoxaPin , cRFFemurPin , cRFTibiaPin};

const std::vector<int> leftRearJoints = {cLRCoxaPin , cLRFemurPin , cLRTibiaPin};
const std::vector<int> leftMidJoints = {cLMCoxaPin , cLMFemurPin , cLMTibiaPin};
const std::vector<int> leftFrontJoints = {cLFCoxaPin , cLFFemurPin , cLFTibiaPin};

const std::vector<vector<int>> legJoints ={
  rightRearJoints ,rightMidJoints, rightFrontJoints,leftRearJoints, leftMidJoints,leftFrontJoints
};


/*
  defines used to dynamically extract all joints for a specific axis
  using index % 3 == MY_MOD
*/
#define Y_MOD 0
#define Z_MOD 1
#define X_MOD 2

/* std::vector<int> xPins; */
/* std::vector<int> zPins; */
/* std::vector<int> yPins; */

// hard coded version for performance advantages
const vector<int> xPins = {cRRTibiaPin, cRMTibiaPin, cRFTibiaPin, cLRTibiaPin, cLMTibiaPin, cLFTibiaPin};
const vector<int> zPins = {cRRFemurPin, cRMFemurPin, cRFFemurPin, cLRFemurPin, cLMFemurPin, cLFFemurPin};
const vector<int> yPins = {cRRCoxaPin , cRMCoxaPin , cRFCoxaPin , cLRCoxaPin , cLMCoxaPin , cLFCoxaPin};

/* const int initPositions[18][2]  = { */
const vector<pair<int,int>> initPositions  = {
  { cRRCoxaPin, cRRInitPosY},
  { cRRFemurPin, cRRInitPosZ },
  { cRRTibiaPin, cRRInitPosX },

  { cRMCoxaPin, cRMInitPosY },
  { cRMFemurPin, cRMInitPosZ },
  { cRMTibiaPin, cRMInitPosX },

  { cRFCoxaPin, cRFInitPosY },
  { cRFFemurPin, cRFInitPosZ },
  { cRFTibiaPin, cRFInitPosX },

  { cLRCoxaPin, cLRInitPosY },
  { cLRFemurPin, cLRInitPosZ },
  { cLRTibiaPin, cLRInitPosX },

  { cLMCoxaPin, cLMInitPosY },
  { cLMFemurPin, cLMInitPosZ },
  { cLMTibiaPin, cLMInitPosX },

  { cLFCoxaPin, cLFInitPosY },
  { cLFFemurPin, cLFInitPosZ },
  { cLFTibiaPin, cLFInitPosX }
};


const unsigned int degreeToPulse(const int degree){
  const double valPerDegree = 1000.0 / 90.0;
  const double val = (degree * valPerDegree) + PulseOffset;
  // double -> int
  return int(val);
}

String moveCommandFor(int servo, int pulse, int time) {
  String str = String("#") + servo + "P" + pulse + " T" + time;
  return str;
}

void move(int servo, int pulse, int time, const boolean blocking ) {
  String str = moveCommandFor(servo, pulse, time);
  Serial.println(str);
  Serial1.println(str);
  if (blocking) {
    delay(time);
  }
}

// X Y Z axis for robots
//X = Tibai
//Y = Coxa (Hip Horizontal)
//Z = Femur (Hip Vertical)
void moveSingleServo(const int servo, const int degree,  const int duration = 1000, const boolean blocking = true) {
  move(servo, degreeToPulse(degree), duration, blocking);
}

const String getCommandSequence(const vector<int> pins, const vector<int> values, const int duration, const pulse_value_t pulseValueType) {
  String cmd = "";

  for (std::size_t i = 0, e = pins.size(); i != e; ++i) {
    if(pulseValueType == DEGREE){
      cmd += String("#") + pins[i] + String("P") + degreeToPulse(values[i]);
    }else{
      cmd += String("#") + pins[i] + String("P") + values[i];
    }
  }
  cmd += String("T") + duration;
  return cmd;
}

void moveCommandGroup(const vector<int> pins, const vector<int> values, const int duration, const boolean blocking = true, const pulse_value_t pulseValueType = DEGREE) {
  String cmd = getCommandSequence(pins, values, duration, pulseValueType);
  Serial.println("Executing command group:");
  Serial.println(cmd);
  SSCSerial.println(cmd);
  if(blocking){
    delay(duration);
  }
}

void defaultPosition() {
  vector<int> pins;
  vector<int> values;

  std::transform( initPositions.begin(), initPositions.end(), std::back_inserter( pins ), [](pair<int,int> pair_) { return pair_.first; });
  std::transform( initPositions.begin(), initPositions.end(), std::back_inserter( values ), [](pair<int,int> pair_) { return pair_.second; });

  moveCommandGroup(pins, values, 2000);
}

void sitDown() {
  DebugSerial.println("Starting sit down series");
  defaultPosition();
  DebugSerial.println("Done with sit down series");
}


// we could use std::transform and a lambda with modulus to extract the x,y,z vectors
// that way, we could create a command group instead of non blocking single commands
void initYAxis() {
  Serial.println("Running initYAxis");
  std::vector<int> yValues;

  for (std::size_t i = 0, e = initPositions.size(); i != e; ++i) {
    if(i % 3 == Y_MOD){
      std::vector<int> yPins;
      yPins.push_back(initPositions[i].first);
      yValues.push_back(initPositions[i].second);
    }
  }
  moveCommandGroup(yPins, yValues, 400);
}

// template programming seems to be not working / limited...
/* template<typename T> */
/* const std::vector<T> combine(const std::vector<T> a, const std::vector<T> b){ */
 const std::vector<int> combine(const std::vector<int> a, const std::vector<int> b){
   std::vector<int> c;
   c.reserve( a.size() + b.size() ); // preallocate memory
   c.insert( c.end(), a.begin(), a.end() );
   c.insert( c.end(), b.begin(), b.end() );
   return c;
}

void standUp(const int xVal = 45, const int zVal = 10) {
  DebugSerial.println("Starting stand up  series");
  initYAxis();

  const int lowPinSign =  +1;
  const int highPinSign = -1;
  const int lowPinMax = 15;

  std::vector<int> xValues;
  std::vector<int> zValues;

  // Unfortunately std::function is not yet available in ArduinoSTL ...
  /* std::function<int(int)> f =[lowPinMax, lowPinSign, highPinSign](int pin) { */
  /*   if (pin <= lowPinMax) { */
  /*     return xVal * lowPinSign; */
  /*   } else { */
  /*     return xVal * highPinSign; */
  /*   } */
  /* }; */
  /* std::transform( xPins.begin(), xPins.end(), std::back_inserter( xValues ), f); */
  /* std::transform( zPins.begin(), zPins.end(), std::back_inserter( zValues ), f); */

  for (std::size_t i = 0, e = xPins.size(); i != e; ++i) {
    if (xPins[i] <= lowPinMax) {
      xValues.push_back(xVal * lowPinSign);
    } else {
      xValues.push_back(xVal * highPinSign);
    }
    if (zPins[i] <= lowPinMax) {
      zValues.push_back(zVal * lowPinSign);
    } else {
      zValues.push_back(zVal * highPinSign);
    }
  }

  moveCommandGroup(combine(xPins, zPins), combine(xValues, zValues), 500);
  DebugSerial.println("Done with stand up series");
}

void freeServos() {
  vector<int> zeros(3,0);
  Serial.println("Resettings servos");
  for (auto const& leg : legJoints) {
    moveCommandGroup(leg, zeros, 200, true, PULSE);
  }
}

void initializeVectors() {
  /* for (std::size_t i = 0, e = initPositions.size(); i != e; ++i) { */
  /*   if(i % 3 == X_MOD){ */
  /*     xPins.push_back(initPositions[i].first); */
  /*   }else if(i % 3 == Y_MOD){ */
  /*     yPins.push_back(initPositions[i].first); */
  /*   }else if(i % 3 == Z_MOD){ */
  /*     zPins.push_back(initPositions[i].first); */
  /*   } */
  /* } */
}

void setup() {
  initializeVectors();
  freeServos();
  DebugSerial.begin(115200);
  SSCSerial.begin(115200);
  sitDown();
  delay(3000);
}

void loop() {
  // x, z
  standUp(70, 40);
  standUp(45, 10);
  standUp(20, -20);
  standUp(0, -50);
  standUp(20, -20);
  standUp(45, 10);
  sitDown();
  freeServos();
  while(true){}
}
