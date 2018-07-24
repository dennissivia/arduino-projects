#include "config.h"
#include <ArduinoSTL.h>

using namespace std;

enum joint_type_t {
  COXA,
  FEMUR,
  TIBIA
};

enum site_t {
  LEFT,
  RIGHT
};

enum orientation_t {
  REAR,
  MID,
  FRONT
};

// Example: Rear / Right
struct position_t {
  site_t site;
  orientation_t orientation;
};

struct joint_t {
  joint_type_t joint_type;
  unsigned int pin;
};

struct leg_t{
  vector<joint_t> joints;
  String name;
};

enum pulse_value_t {
  DEGREE,
  PULSE
};

const int yFFront  =  20;
const int yFCenter =   0;
const int yFBack   = -20;

const int yCFront  =  20;
const int yCCenter =   0;
const int yCBack   = -20;

const int yRFront  =  20;
const int yRCenter =   0;
const int yRBack   = -20;

std::vector<String> legList;
std::vector<std::pair<int, int> > legSequence;
std::vector<String> legCommands;

const std::vector<int> rightRearJoints  = {cRRCoxaPin , cRRFemurPin , cRRTibiaPin};
const std::vector<int> rightMidJoints   = {cRMCoxaPin , cRMFemurPin , cRMTibiaPin};
const std::vector<int> rightFrontJoints = {cRFCoxaPin , cRFFemurPin , cRFTibiaPin};
const std::vector<int> leftRearJoints   = {cLRCoxaPin , cLRFemurPin , cLRTibiaPin};
const std::vector<int> leftMidJoints    = {cLMCoxaPin , cLMFemurPin , cLMTibiaPin};
const std::vector<int> leftFrontJoints  = {cLFCoxaPin , cLFFemurPin , cLFTibiaPin};

const std::vector<vector<int>> legJoints ={
  rightRearJoints ,rightMidJoints, rightFrontJoints, leftRearJoints, leftMidJoints, leftFrontJoints
};

// z = 2/3 x
// x = 3/2 z
vector<pair<int, int>> legPositions = {
  {30,   20}, // lowest
  {15,    0},
  {-15, -20},
  {-30, -40},
  {-45, -50} //highest
};

/*
  defines used to dynamically extract all joints for a specific axis
  using index % 3 == MY_MOD
*/
#define Y_MOD 0
#define Z_MOD 1
#define X_MOD 2

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


// ------------------------------------
/*
  Experiment
*/

// first pro: we are not assuming the same size for pins and values anymore
const String getCommandSequence(const vector<std::pair<int, int>> pairs, const int duration, const pulse_value_t pulseValueType){
  String cmd = "";
  for(const auto& pair: pairs){
    if(pulseValueType == DEGREE){
      cmd += String("#") + pair.first + String("P") + degreeToPulse(pair.second);
    }else{
      cmd += String("#") + pair.first + String("P") + pair.second;
    }
  }
  cmd += String("T") + duration;
  return cmd;
}

// ------------------------------------
const String getCommandSequence(const vector<int> pins, const vector<int> values, const int duration, const pulse_value_t pulseValueType, const boolean performCorrection) {
  String cmd = "";
  int value;

  for (std::size_t i = 0, e = pins.size(); i != e; ++i) {
    if(performCorrection){
      value = sideCorrection(pins[i], values[i]);
    }else{
      value = values[i];
    }

    if(pulseValueType == DEGREE){
      cmd += String("#") + pins[i] + String("P") + degreeToPulse(value);
    }else{
      cmd += String("#") + pins[i] + String("P") + value;
    }
  }
  cmd += String("T") + duration;
  return cmd;
}

void moveCommandGroup(const vector<int> pins, const vector<int> values, const int duration, const boolean blocking = true, const pulse_value_t pulseValueType = DEGREE, const boolean performCorrection = false) {
  String cmd = getCommandSequence(pins, values, duration, pulseValueType, performCorrection);
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

  moveCommandGroup(pins, values, 2000, true);
}

void sleepPosistion() {
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

// move the corpus up and down based on X and Z values
void adjustHeight(const int xVal = 45, const int zVal = 10) {
  DebugSerial.println("Starting stand up  series");

  const int lowPinSign =  +1;
  const int highPinSign = -1;
  const int lowPinMax = 15;

  // Unfortunately std::function is not yet available in ArduinoSTL ...
  // thus we cannot store a lambda to apply it multiple times in in transform calls
  std::vector<int> xValues;
  std::vector<int> zValues;

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

  moveCommandGroup(combine(xPins, zPins), combine(xValues, zValues), 200);
  DebugSerial.println("Done with stand up series");
  delay(800);
}

void freeServos() {
  vector<int> zeros(3,0);
  Serial.println("Resettings servos");
  for (auto const& leg : legJoints) {
    moveCommandGroup(leg, zeros, 200, true, PULSE);
  }
}

void initializeVectors() {
}

// we currently assume the order: COXA, FEMUR, TIBIA until we have proper structs
const vector<int> extractJoint(const vector<vector<int>> legs, const joint_type_t joint_type){
  std::vector<int> joints;
  for(auto const& leg: legs){
    switch(joint_type){
    case COXA:
      joints.push_back(leg[0]);
      break;
    case FEMUR:
      joints.push_back(leg[1]);
      break;
    case TIBIA:
      joints.push_back(leg[2]);
      break;
    default:
      break;
    }
  }
  return joints;
}

const int sideCorrection(int pin, int value){
  // remove duplication of these values...
  // a normalize function for values per PIN ?
  // or move it to the command group ?
  const int lowPinSign =  +1;
  const int highPinSign = -1;
  const int lowPinMax = 15;

  if (pin <= lowPinMax) {
    return (value * lowPinSign);
  } else {
    return (value * highPinSign);
  }
}


void walkForward(const int initialX, const int initialZ, const unsigned int rounds){
  tripodGait(initialX, initialZ, rounds);
}

/*
  |            (Tripod A)            |              (Tripod B)
  State Vertical Servo  | Horizontal Servo |  Vertical Servo |  Horizontal Servo
  --------------------------------------------------------------------------------
  INIT  |    Low        | Front            | Mid             | Rear
    0   |    Low        | Front to Center  | Mid to High     | Rear to Center
    1   |    Low        | Center to Rear   | High to Mid     | Center to Front
    2   |    Low        | Rear             | Mid to Low      | Front
    3   |    Low to Mid | Rear             | Low             | Front
    4   |  Mid to High  | Rear to Center   | Low             | Front to Center
    5   |  High to Mid  | Center to Front  | Low             | Center to Rear
    6   |  Mid to Low   | Front            | Low             | Rear
    7   |    Low        | Front            | Low(to Mid)     | Rear
  END   |    Low        | Front            | Mid             | Rear
*/

void tripodGait(const int initialX, const int initialZ, const unsigned int rounds){
  const int zLow     = initialZ; // -60; MAX // make sure we are really low to stabilize the lifted legs
  const int zMid     = initialZ + 15;
  const int zHigh    = initialZ + 30; // 60;

  const int xClose   = initialX;
  const int xFar     = initialX + 15;

  //  {Left Front Leg, Left Rear Leg, Right Center Leg}
  const vector<vector<int>> tripodA = {leftFrontJoints, rightMidJoints, leftRearJoints };
  // horizontal
  const vector<int> tripodAYAxis = extractJoint(tripodA, COXA);
  // vertical
  const vector<int> tripodAZAxis =  extractJoint(tripodA, FEMUR);

  //  {Left Center Leg, Right Front Leg, Right Rear Leg}
  const vector<vector<int>> tripodB = { rightRearJoints, leftMidJoints, rightFrontJoints };
  // horizontal
  const vector<int> tripodBYAxis =extractJoint(tripodB, COXA);
  // vertical
  const vector<int> tripodBZAxis =extractJoint(tripodB, FEMUR);

  vector<int> sequenceTripodAZ = {
    zLow, zLow, zLow, zMid,
    zHigh, zMid, zLow, zLow
  };
  vector<int> sequenceTripodBZ = {
    zHigh, zMid, zLow, zLow,
    zLow, zLow, zLow, zMid
  };



  // horizontal
  // LF, RM, LR
  /* const vector<vector<int>> tripodA = {leftFrontJoints, rightMidJoints, leftRearJoints }; */
  // should we generate this with a Y Axis helper?
  vector<vector<int>> sequenceTripodAY = {
    {yFCenter, yCCenter, yRCenter},
    {yFBack,   yCBack,   yRBack},
    {yFBack,   yCBack,   yRBack},
    {yFBack,   yCBack,   yRBack},
    {yFCenter, yCCenter, yRCenter}, // starting point?
    {yFFront,  yCFront,  yRFront},
    {yFFront,  yCFront,  yRFront},
    {yFFront,  yCFront,  yRFront}
  };

  // RR, LM, RF
  /* const vector<vector<int>> tripodB = { rightRearJoints, leftMidJoints, rightFrontJoints }; */
  vector<vector<int>> sequenceTripodBY = {
    {yRCenter, yCCenter, yRCenter},
    {yRFront,  yCFront,  yRFront},
    {yRFront,  yCFront,  yRFront},
    {yRFront,  yCFront,  yRFront},
    {yRCenter, yCCenter, yRCenter}, // starting point
    {yRBack,   yCBack,   yRBack},
    {yRBack,   yCBack,   yRBack},
    {yRBack,   yCBack,   yRBack}
  };

  const int pulseWidth = 200;
  const unsigned int sequenceLength = std::min(sequenceTripodAZ.size(), sequenceTripodBZ.size());

  for(int r = 0; r < rounds; ++r) {
    for(std::size_t step = 0; step != sequenceLength; ++step) {
      std::vector<int> aValues(sequenceLength, sequenceTripodAZ[step]);
      std::vector<int> bValues(sequenceLength, sequenceTripodBZ[step]);

      Serial.println("Walking step " + String(step) +" in progress");

      moveCommandGroup(tripodAZAxis, aValues, pulseWidth, false, DEGREE, true);
      moveCommandGroup(tripodBZAxis, bValues, pulseWidth, false, DEGREE, true);
      delay(pulseWidth/10); // make sure the leg is lifted / lowered before moving COXA
      moveCommandGroup(tripodAYAxis, sequenceTripodAY[step], pulseWidth, false, DEGREE, true);
      moveCommandGroup(tripodBYAxis, sequenceTripodBY[step], pulseWidth, false, DEGREE, true);

      delay(pulseWidth); // make sure the leg is lifted / lowered before moving COXA
    }
  }
}

void endSequence(){
  initYAxis();
  sleepPosistion();
  freeServos();
}

void upDownSequence(){
  Serial.println("Starting upDown");
  initYAxis();
  /* standup(x, z) */

  /*
   * better approach:
   * lift tripod A (z axis)
   * adjust x/y /
   * down tripod A
   * Repeat with Tripod B
   */
  vector<pair<int, int>> reverseLegPositions = legPositions;
  std::reverse(reverseLegPositions.begin(), reverseLegPositions.end());


  for(const auto& position : legPositions){
    adjustHeight(position.first, position.second);
    delay(1000);
  }

  for(const auto& position : reverseLegPositions){
    adjustHeight(position.first, position.second);
    delay(1000);
  }

  Serial.println("Fisished upDown");
}

void setup() {
  DebugSerial.begin(DEBUG_BAUD);
  SSCSerial.begin(SSC_BAUD);
  initializeVectors();
  freeServos();
  // initial startup delay
  /* delay(2000); */
  // play tone, lid LED whatever

  sleepPosistion();
  while(true){}
  /* delay(2000); */
  /* upDownSequence(); */
  /* while(true){} */
  /* delay(2000); */
}

void loop() {
  const unsigned int pauseTime = 2000;
  // x, z
  adjustHeight(40, 30);
  /* adjustHeight(15, 0); */
  /* adjustHeight(-15, -20); */
  delay(pauseTime);

  // -------------- test walking ----------------
  Serial.println("Starting walking sequence");
  /* firstStep(0); */
  walkForward(30, 20, 6);
  Serial.println("done with walking sequence");

  /* // reset */
  delay(pauseTime);
  initYAxis();
  adjustHeight(30, 20);
  // -------------- end of walking ----------------
  endSequence();
  while(true){}
}
