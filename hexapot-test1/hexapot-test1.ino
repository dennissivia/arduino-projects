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

std::vector<String> legPositions;
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

  moveCommandGroup(pins, values, 1000);
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

  moveCommandGroup(combine(xPins, zPins), combine(xValues, zValues), 200);
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

void walkForward(const int initialZ){
  //  {Left Front Leg, Left Rear Leg, Right Center Leg}
  const vector<vector<int>> tripodA = {leftFrontJoints, leftRearJoints, rightMidJoints};
  // horizontal
  const vector<int> tripodAYAxis = extractJoint(tripodA, COXA);
  // vertical
  const vector<int> tripodAZAxis =  extractJoint(tripodA, FEMUR);

  //  {Left Center Leg, Right Front Leg, Right Rear Leg}
  const vector<vector<int>> tripodB = { leftMidJoints, rightFrontJoints, rightRearJoints};
  // horizontal
  const vector<int> tripodBYAxis =extractJoint(tripodB, COXA);
  // vertical
  const vector<int> tripodBZAxis =extractJoint(tripodB, FEMUR);


  /* const int leftHigh = 1000; */
  /* const int leftMid = 1400; */
  /* const int leftLow = 1800; */
  /* const int rightHigh = 2000; */
  /* const int rightMid  = 1600; */
  /* const int rightLow  = 1200; */
  /* const int pulseVS = 3000; */

  const int yFFront  = -10; // 40; MAX
  const int yFCenter = -30; // -20;
  const int yFBack   = -50; // -90; MAX

  const int yCFront  = 25; //50;
  const int yCCenter = 0;
  const int yCBack   = 25; //-50;

  const int yRFront  = -30; // 0;
  const int yRCenter = -60; //-45;
  const int yRBack   = -90; //-90;

  /* const int zLow     = -20; // -60; */
  /* const int zMid     =  0; */
  /* const int zHigh    = 20; // 60; */
  const int zLow     = initialZ; // -60;
  const int zMid     =  0;
  const int zHigh    = initialZ + 60; // 60;

  /* {Left Front Leg, Left Rear Leg, Right Center Leg} */
  /* const vector<vector<int>> tripodA = {leftFrontJoints, leftRearJoints, rightMidJoints}; */
  /* const vector<vector<int>> tripodB = { leftMidJoints, rightFrontJoints, rightRearJoints}; */

  vector<int> sequenceTripodAZ = {zLow, zLow, zLow, zLow, zLow, zMid, zHigh, zMid, zLow, zLow};
  vector<int> sequenceTripodBZ = {zLow, zMid, zHigh, zMid, zLow, zLow, zLow, zLow, zLow, zLow};

  // should we generate this with a Y Axis helper?
  vector<vector<int>> sequenceTripodAY = {
    // should be front, but its INIT
    {yFCenter, yRCenter, yCCenter},
    {yFCenter, yRCenter, yCCenter},
    {yFBack,   yRBack,   yCBack},
    {yFBack,   yRBack,   yCBack},
    {yFBack,   yRBack,   yCBack},
    {yFCenter, yRCenter, yCCenter},
    {yFFront,  yRFront,  yCFront},
    {yFFront,  yRFront,  yCFront},
    {yFFront,  yRFront,  yCFront},
    {yFCenter, yRCenter, yCCenter}
  };
  vector<vector<int>> sequenceTripodBY = {
    // should be back, but its INIT
    {yCCenter, yFCenter, yRCenter},
    {yCCenter, yFCenter, yRCenter},
    {yCFront, yFFront, yRFront},
    {yCFront, yFFront, yRFront},
    {yCFront, yFFront, yRFront},
    {yCCenter, yFCenter, yRCenter},
    {yCBack, yFBack, yRBack},
    {yCBack, yFBack, yRBack},
    {yCBack, yFBack, yRBack},
    {yCCenter, yFCenter, yRCenter}
  };

  const unsigned int sequenceLength = std::min(sequenceTripodAZ.size(), sequenceTripodBZ.size());
  for (std::size_t i = 0; i != sequenceLength; ++i) {
    std::vector<int> aValues(sequenceLength, sequenceTripodAZ[i]);
    std::vector<int> bValues(sequenceLength, sequenceTripodBZ[i]);

    /* std::vector<int> allValues = combine(aValues, bValues); */
    /* std::vector<int> allJoints = combine(tripodAZAxis, tripodBZAxis); */
    Serial.println("Walking step " + String(i) +" in progress");
    moveCommandGroup(tripodAZAxis, aValues, 200, false, DEGREE, true);
    moveCommandGroup(tripodBZAxis, bValues, 200, false, DEGREE, true);
    moveCommandGroup(tripodAYAxis, sequenceTripodAY[i], 200, false, DEGREE, true);
    moveCommandGroup(tripodBYAxis, sequenceTripodBY[i], 200, false, DEGREE, true);
    delay(300);
  }
}

void setup() {
  initializeVectors();
  freeServos();
  DebugSerial.begin(115200);
  SSCSerial.begin(115200);
  sitDown();
  /* while(true){} */
  delay(3000);
}

void loop() {
  int pauseTime = 3000;
  // x, z
  standUp(60, 40);
  standUp(40, 10);
  standUp(30, 0); // could be the walking position
  standUp(15, -20);
  delay(pauseTime);

  // -------------- test walking ----------------
  Serial.println("Starting walking sequence");
  walkForward(-20);
  walkForward(-20);
  walkForward(-20);
  Serial.println("done with walking sequence");

  /* // reset */
  delay(pauseTime);
  initYAxis();
  standUp(15, -20);
  // -------------- end of walking ----------------

  standUp(10, -35);
  standUp(0, -50);

  delay(pauseTime / 2 );

  // go back to sleep aka relax
  standUp(20, -20);
  standUp(45, 10);
  sitDown();
  freeServos();
  while(true){}
}
