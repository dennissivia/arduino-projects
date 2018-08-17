#include "config.h"
#include <ArduinoSTL.h>
#include <math.h>
#include <assert.h>

#include <Wire.h>
#include "i2c.h"

// IMU-Sensor
#include "i2c_MPU9250.h"
MPU9250 mpu9250;


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

#define LEG_AMOUNT 6
std::vector<double> currentFeetPosX(LEG_AMOUNT);
std::vector<double> currentFeetPosY(LEG_AMOUNT);
std::vector<double> currentFeetPosZ(LEG_AMOUNT);

// globals to store IK calculation results
std::vector<double> nextLegPosX(LEG_AMOUNT);
std::vector<double> nextLegPosY(LEG_AMOUNT);
std::vector<double> nextLegPosZ(LEG_AMOUNT);

std::vector<double> xxxdistCoxaToFeet(LEG_AMOUNT);

std::vector<int> servoCoxaAngle(LEG_AMOUNT);
std::vector<int> servoFemurAngle(LEG_AMOUNT);
std::vector<int> servoTibiaAngle(LEG_AMOUNT);

std::vector<double> xxxtotalX(LEG_AMOUNT);
std::vector<double> xxxtotalY(LEG_AMOUNT);

std::vector<double> xxxdistBodyCenterToFeet(LEG_AMOUNT);
std::vector<double> xxxangleBodyCenterX(LEG_AMOUNT);

std::vector<double> bodyIKX(LEG_AMOUNT);
std::vector<double> bodyIKY(LEG_AMOUNT);
std::vector<double> bodyIKZ(LEG_AMOUNT);

std::vector<double> bodyOffsetsX(LEG_AMOUNT);
std::vector<double > bodyOffsetsY(LEG_AMOUNT);

std::vector<double> legIKTibia(LEG_AMOUNT);
std::vector<double> legIKFemur(LEG_AMOUNT);
std::vector<double> legIKCoxa(LEG_AMOUNT);

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
  DebugSerial.println(str);
  SSCSerial.println(str);
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
#if DEBUG_SSC
  Serial.println("Executing command group:");
  Serial.println(cmd);
#endif
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
  initializeBodyOffset();
  initializeFeetPositions();
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
  const int zMid     = initialZ + 30;
  const int zHigh    = initialZ + 45; // 60;

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


void shutDownSequence(){
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

void playStartTone(){
  Serial.println("Tone coming");
  vector<pair<short, int>> notes = {{60, 2000},
                                    {80, 2250} ,
                                    {100, 2500}};
  for(const auto& pair: notes){
    tone(BUZZER_PIN, pair.second);
    delay(pair.first);
    noTone(BUZZER_PIN);
  }
  Serial.println("Tone done");
}

void performMPUMeasurement(){
  static float xyz_GyrAccMag[9];

  mpu9250.getMeasurement(xyz_GyrAccMag);

  Serial.print("XYZ ACC g[");
  Serial.print(xyz_GyrAccMag[0],2);
  Serial.print(";");
  Serial.print(xyz_GyrAccMag[1],2);
  Serial.print(";");
  Serial.print(xyz_GyrAccMag[2],2);
  Serial.print("]");

  Serial.print(" \t GYR dps[");
  Serial.print(xyz_GyrAccMag[4],2);
  Serial.print(";");
  Serial.print(xyz_GyrAccMag[5],2);
  Serial.print(";");
  Serial.print(xyz_GyrAccMag[6],2);
  Serial.print("]");

  Serial.print(" \t T: ");
  Serial.print(xyz_GyrAccMag[3],2);
  Serial.print(" C");

  Serial.println("");
  delay(20);
}
void setupMPU(){
  Serial.print("Probe MPU9250: ");
  switch (mpu9250.initialize()) {
    case 0: Serial.println("MPU-Sensor missing"); while(1) {};
    case 1: Serial.println("Found unknown Sensor."); break;
    case 2: Serial.println("MPU6500 found."); break;
    case 3: Serial.println("MPU9250 found!"); break;
  }

  Serial.print("Probe AK8963: ");
  if (i2c.probe(0x0C))
    Serial.println("AK8963 found!");
  else
    Serial.println("AK8963 missing");
}

void rotateBody(int rounds = 1){
  const vector<vector<int>> tripodA = {leftFrontJoints, rightMidJoints, leftRearJoints };
  const vector<int> tripodAYAxis = extractJoint(tripodA, COXA);

  const vector<vector<int>> tripodB = { rightRearJoints, leftMidJoints, rightFrontJoints };
  const vector<int> tripodBYAxis =extractJoint(tripodB, COXA);

  moveCommandGroup(tripodAYAxis, {-cLFCoxaMax1, cRMCoxaMax1, -cLRCoxaMax1},1000, true);
  moveCommandGroup(tripodBYAxis, {cRRCoxaMax1, -cLMCoxaMax1, cRFCoxaMax1},1000, true);

  moveCommandGroup(combine(tripodAYAxis, tripodBYAxis),
                   {-cLFCoxaMin1, cRMCoxaMin1, -cLRCoxaMin1, cRRCoxaMin1, -cLMCoxaMin1, cRFCoxaMin1},
                   1000, true);
}

void initializeBodyOffset(){
  bodyOffsetsX = {
    0, // offsetXL1,
    0, // offsetXL2,
    0, // offsetXL3,
    0, // offsetXL4,
    0, // offsetXL5,
    0, // offsetXL6
  };

  bodyOffsetsY = {
    0, // offsetYL1,
    0, // offsetYL2,
    0, // offsetYL3,
    0, // offsetYL4,
    0, // offsetYL5,
    0, // offsetYL6
  };

}

/* #define PI 3.1415926535897932384626433832795 */
/* #define HALF_PI 1.5707963267948966192313216916398 */
/* #define TWO_PI 6.283185307179586476925286766559 */
/* #define DEG_TO_RAD 0.017453292519943295769236907684886 */
/* #define RAD_TO_DEG 57.295779513082320876798154814105 */
std::vector<std::vector<double>> initialFeetPos(double angle = INITIAL_FRONT_REAR_ANGLE ){
  Serial.println("============");
  Serial.println(angle);
  Serial.println(angle* DEG_TO_RAD);
  Serial.println(cos(angle*DEG_TO_RAD));
  Serial.println(COXA_LENGTH + FEMUR_LENGTH);
  Serial.println(cos(angle*DEG_TO_RAD) * (COXA_LENGTH + FEMUR_LENGTH));
  Serial.println("============");

  double xL1 = cos(angle*DEG_TO_RAD) * (COXA_LENGTH + FEMUR_LENGTH);
  double yL1 = sin(angle*DEG_TO_RAD) * (COXA_LENGTH + FEMUR_LENGTH);
  double zL1 = TIBIA_LENGTH;

  double xL2 = COXA_LENGTH + FEMUR_LENGTH;
  double yL2 = 0;
  double zL2 = TIBIA_LENGTH;

  double xL3 = cos(angle*DEG_TO_RAD) * (COXA_LENGTH + FEMUR_LENGTH);
  double yL3 = sin(-angle*DEG_TO_RAD) * (COXA_LENGTH + FEMUR_LENGTH);
  double zL3 = TIBIA_LENGTH;

  double xL4 = -cos(angle*DEG_TO_RAD) * (COXA_LENGTH + FEMUR_LENGTH);
  double yL4 = sin(-angle*DEG_TO_RAD) * (COXA_LENGTH + FEMUR_LENGTH);
  double zL4 = TIBIA_LENGTH;

  double xL5 = -(COXA_LENGTH + FEMUR_LENGTH);
  double yL5 = 0;
  double zL5 = TIBIA_LENGTH;

  double xL6 = -cos(angle*DEG_TO_RAD) * (COXA_LENGTH + FEMUR_LENGTH);
  double yL6 = sin(angle*DEG_TO_RAD) * (COXA_LENGTH + FEMUR_LENGTH);
  double zL6 = TIBIA_LENGTH;

  // this vector and the following loop are wasteful, but for now it
  // will make the code easier to read and thus easier to generalize,
  // before we optimize it.
  std::vector<std::vector<double>> feetPositions = {
    {xL1, yL1, zL1},
    {xL2, yL2, zL2},
    {xL3, yL3, zL3},
    {xL4, yL4, zL4},
    {xL5, yL5, zL5},
    {xL6, yL6, zL6},
  };

  for(size_t i = 0; i < LEG_AMOUNT; i++){
    currentFeetPosX[i] = feetPositions[i][0];
    currentFeetPosY[i] = feetPositions[i][1];
    currentFeetPosZ[i] = feetPositions[i][2];
  }

  return feetPositions;
}

void initializeFeetPositions(){
  // we ignore the return value and count on the side effect for now...
  // XXX fixme ASAP
  initialFeetPos();
}


// legID (the leg we want to calculate the coordinates for)
// posY (the disired Y coordinate (input) for the given leg)
void bodyIK(unsigned short legID, int posX, int posY, int rotX, int rotY, int rotZ){
  double totalY = currentFeetPosY[legID] + bodyOffsetsY[legID] + posY;
  double totalX = currentFeetPosX[legID] + bodyOffsetsX[legID] + posX;

  // calculate the hypothenuse for the target point
  // DistBodyCenterFeet_1	 = sqrt(TotalY_1^2 + TotalX_1^2)
  double distBodyCenterToFeet = sqrt(square(totalY) + square(totalX));

  // calculate the atan2 (angle) of the hypothenuse
  // AngleBodyCenterX_1	  = PI/2 - atan2(TotalY_1, TotalX_1)
  double angleBodyCenterX = HALF_PI - atan2(totalY, totalX);
  /* double angleBodyCenterX = HALF_PI - abs(atan2(totalY, totalX)); */

#if DEBUG_IK
  Serial.println("***********");
  Serial.println(atan2(totalY, totalX));
  Serial.println(angleBodyCenterX);
  Serial.println("***********");
#endif

  // XXX I swapped the - totalX and - totalY .. not sure if that is correct
  // calculate the new X/Y coordinates taking into account the degree of Y rotation
  // BodyIKX_1	cos(AngleBodyCenterX_1 + (RotY * PI/180)) * DistBodyCenterFeet_1 - TotalX_1
  bodyIKX[legID] = cos(angleBodyCenterX + (rotY * DEG_TO_RAD)) * distBodyCenterToFeet - totalY;
  // BodyIKY_1	(sin(AngleBodyCenterX_1 + (RotY  * PI/180)) * DistBodyCenterFeet_1) - TotalY_1
  bodyIKY[legID] = sin(angleBodyCenterX + (rotY * DEG_TO_RAD)) * distBodyCenterToFeet - totalX;
/* BodyIKZ_1	RollZ_1 + PitchZ_1 */


  // calculate the new Z coordinate based on the pitch and roll inputs
  // RollZ_1	tan(RotZ * PI/180) * TotalX_1
  double rollZ = tan(rotZ * DEG_TO_RAD) * totalX;
  // PitchZ_1	tan(RotX * PI/180) * TotalY_1
  double pitchZ = tan(rotX * DEG_TO_RAD) * totalY;
  bodyIKZ[legID] = rollZ + pitchZ;


  // debugging
  xxxtotalX[legID] = totalX;
  xxxtotalY[legID] = totalY;
  xxxdistBodyCenterToFeet[legID] = distBodyCenterToFeet;
}

// See: https://oscarliang.com/inverse-kinematics-and-trigonometry-basics/
// for the solution of the a1 + a2 = a and beta and gamma angles
void legIK(unsigned short legID, unsigned long posX, unsigned long posY, unsigned long posZ){
  nextLegPosX[legID] = currentFeetPosX[legID] + posX + bodyIKX[legID];
  nextLegPosY[legID] = currentFeetPosY[legID] + posY + bodyIKY[legID];
  nextLegPosZ[legID] = currentFeetPosZ[legID] + posZ + bodyIKZ[legID];
  double distCoxaToFeet     = sqrt(square(nextLegPosX[legID]) + square(nextLegPosY[legID]));

  // debugging
  xxxdistCoxaToFeet[legID] = distCoxaToFeet;

  /* double l1      = nextLegPosY[legID]; */
  double l1      = distCoxaToFeet;
  double zOffset = nextLegPosZ[legID];
  double l2 = l1 - COXA_LENGTH;

  // XXX should double l = distCoxaToFeet; ?????
  double l = sqrt(square(l2) + square(zOffset));

  double l_2 = square(l);
  double t_2 = square(TIBIA_LENGTH);
  double f_2 = square(FEMUR_LENGTH);


  // sin = opp/hyp
  // cos = adj/hyp
  // tan = opp/adj
  // XXX we can compute a1 with acos or atan
  double a1 = atan(l2 / zOffset);
  /* double a1 = acos(zOffset / l); */
  double a2 = acos((square(TIBIA_LENGTH) - square(FEMUR_LENGTH) - l_2) / ( -2 * l * FEMUR_LENGTH));
  double a  = a1 + a2; // the full angle alpha
  // acos((IKSW_1^2 - TibiaLength^2 - FemurLength^2)/(-2 * FemurLength * TibiaLength))
  double b  = acos((l_2 - square(TIBIA_LENGTH) - square(FEMUR_LENGTH)) / ( -2 * FEMUR_LENGTH * TIBIA_LENGTH));

  double g  = atan2(nextLegPosY[legID], nextLegPosX[legID]);

  legIKTibia[legID] = -1 * (90 - b * RAD_TO_DEG);
  legIKFemur[legID] = 90 - a * RAD_TO_DEG;
  // XXX we should add our real initial values here (asap)
  int initialCoxaAngle = 0;
  legIKCoxa[legID]  = initialCoxaAngle + g * RAD_TO_DEG;

#if DEBUG_IK
  Serial.println("alpha1: " + String(a1));
  Serial.println("alpha2: " + String(a2));
  Serial.println("alpha: "  + String(a));
  Serial.println("beta: "   + String(b));
  Serial.println("gamma (rad):"  + String(g));
  Serial.println("gamma (deg): "  + String(g * RAD_TO_DEG));
#endif
  return;
}

void debugVector(std::vector<double> vec){
  for(const auto& val: vec){
    Serial.print(val);
    Serial.print(", ");
  }
  Serial.println("");
}
void debugVector(std::vector<unsigned int> vec){
  Serial.println("Debugging vector:");
  for(const auto& val: vec){
    Serial.print(val);
    Serial.print(", ");
  }
  Serial.println("");
}


void servoAngles(unsigned short legID){
  vector<int> initialCoxaAngles     = {
    cRFCoxaAngle1,
    cRMCoxaAngle1,
    cRRCoxaAngle1,
    cLRCoxaAngle1,
    cLMCoxaAngle1,
    cLFCoxaAngle1
  };
  vector<int> servoCoxaAngleOffsets = {-60, 0 , 60, 120, -180, -120 };
  /* vector<int> servoCoxaAngleOffsets(6,0); */

  int coxaAngle = (int(legIKCoxa[legID] + servoCoxaAngleOffsets[legID]) % 360) + initialCoxaAngles[legID];
  servoCoxaAngle[legID]  = int(coxaAngle);
  servoFemurAngle[legID] = int(legIKFemur[legID]);
  servoTibiaAngle[legID] = int(legIKTibia[legID]);
  return;
}

void testIK(double posX = 0, double posY = 0, double posZ = 0, int rotX = 0, int rotY = 0, int rotZ = 0){
#if DEBUG_IK
  Serial.println("-----------------------");
  Serial.println("** current Feet position vectors");
  debugVector(currentFeetPosX);
  debugVector(currentFeetPosY);
  debugVector(currentFeetPosZ);
#endif

  for(size_t i = 0; i < LEG_AMOUNT; i++){
    bodyIK(i, posX, posY, rotX, rotY, rotZ);
  }
#if DEBUG_IK
  Serial.println("** dist body to feet (bodyIK)");
  debugVector(xxxdistBodyCenterToFeet);

  Serial.println("** body IK total X/Y vectors");
  debugVector(xxxtotalX);
  debugVector(xxxtotalY);

  Serial.println("** body IK vectors for leg");
  debugVector(bodyIKX);
  debugVector(bodyIKY);
  debugVector(bodyIKZ);
#endif

  for(size_t i = 0; i < LEG_AMOUNT; i++){
    legIK(i, posX, posY, posZ);
  }

#if DEBUG_IK
  Serial.println("** next leg-position vectors");
  debugVector(nextLegPosX);
  debugVector(nextLegPosY);
  debugVector(nextLegPosZ);

  Serial.println("** leg-ik vectors");
  debugVector(legIKCoxa);
  debugVector(legIKFemur);
  debugVector(legIKTibia);
  Serial.println("-----------------------");
#endif

  for(size_t i = 0; i < LEG_AMOUNT; i++){
    servoAngles(i);
  }

#if DEBUG_IK
  String text;
  for(size_t i = 0; i < LEG_AMOUNT; i++){
    text = String("Servo angle Coxa[") + i + "] = " + servoCoxaAngle[i];
    Serial.println(text);
    text = String("Servo angle Femur[") + i + "] = " + servoFemurAngle[i];
    Serial.println(text);
    text = String("Servo angle Tibia[") + i + "] = " + servoTibiaAngle[i];
    Serial.println(text);
  }
#endif

  vector<int> coxaPins  = {cRFCoxaPin, cRMCoxaPin, cRRCoxaPin, cLRCoxaPin, cLMCoxaPin, cLFCoxaPin};
  vector<int> femurPins = {cRFFemurPin, cRMFemurPin, cRRFemurPin, cLRFemurPin, cLMFemurPin, cLFFemurPin };
  vector<int> tibiaPins = {cRFTibiaPin, cRMTibiaPin, cRRTibiaPin, cLRTibiaPin, cLMTibiaPin, cLFTibiaPin};

  moveCommandGroup(coxaPins,  servoCoxaAngle,  500, false, DEGREE, true);
  // XXX we have to remind the servo angles, since the IK gives us the change in angle??
  moveCommandGroup(femurPins, servoFemurAngle, 500, false, DEGREE, true);
  moveCommandGroup(tibiaPins, servoTibiaAngle, 500, false, DEGREE, true);
  delay(500);
}

/* void applyIKtoServo(){ */
/*   vector<int> coxaPins  = {cRFCoxaPin, cRMCoxaPin, cRRCoxaPin, cLRCoxaPin, cLMCoxaPin, cLFCoxaPin}; */
/*   vector<int> femurPins = {cRFFemurPin, cRMFemurPin, cRRFemurPin, cLRFemurPin, cLMFemurPin, cLFFemurPin }; */
/*   vector<int> tibiaPins = {cRFTibiaPin, cRMTibiaPin, cRRTibiaPin, cLRTibiaPin, cLMTibiaPin, cLFTibiaPin}; */

/*   moveCommandGroup(coxaPins,  servoCoxaAngle,  500, false, DEGREE, true); */
/*   // XXX we have to remind the servo angles, since the IK gives us the change in angle?? */
/*   moveCommandGroup(femurPins, servoFemurAngle, 500, false, DEGREE, true); */
/*   moveCommandGroup(tibiaPins, servoTibiaAngle, 500, false, DEGREE, true); */
/*   /1* while(true); *1/ */
/*   delay(800); */
/* } */

void moveSingleLeg(int legID, int posX, int posY, int posZ){
  vector<int> femurPins = {cRFFemurPin, cRMFemurPin, cRRFemurPin, cLRFemurPin, cLMFemurPin, cLFFemurPin };

  moveSingleServo(femurPins[legID], servoFemurAngle[legID] + posZ, 200, true);
  moveSingleServo(femurPins[legID], servoFemurAngle[legID],  200, true);
}

void setup() {
  DebugSerial.begin(DEBUG_BAUD);
  SSCSerial.begin(SSC_BAUD);
  initializeVectors();
  freeServos();
  setupMPU();
  moveSingleServo(cRFCoxaPin, 0, 300, true);
  moveSingleServo(cRMCoxaPin, 0, 300, true);
  moveSingleServo(cRRCoxaPin, 0, 300, true);
  moveSingleServo(cLRCoxaPin, 0, 300, true);
  moveSingleServo(cLMCoxaPin, 0, 300, true);
  moveSingleServo(cLFCoxaPin, 0, 300, true);

  // initial startup delay
  delay(500);
  playStartTone();
  // play tone, lid LED whatever
  performMPUMeasurement();
  sleepPosistion();
  /* while(true){} */
}

void loop() {
  const unsigned int pauseTime = 2000;
  const unsigned int legLiftHeight = 40;

  // x, z
  /* adjustHeight(50, 30); */
  /* rotateBody(1); */
  const int defaultHeight = 0;
  while(true){
    performMPUMeasurement();
    delay(1000);

    testIK(0,0,defaultHeight,0,0,0);
    delay(500);
    playStartTone();
    for(const auto& leg: {0,1,2}){
      moveSingleLeg(leg, 0,0,legLiftHeight);
    }
    for(const auto& leg: {3,4,5}){
      moveSingleLeg(leg, 0,0,-legLiftHeight);
    }

    // Up Down
    // 0 -> right angle to ground
    // 50 -> lowest position possible
    for(const auto& val: {30, 20, 0 ,20, 30 }){
      playStartTone();
      testIK(0,0,val,0,0,0);
      delay(200);
    }

    // not sure what this is used for...
    // test did not have any insights so far
    // for(const auto& val: {20, 10, 0 -10, 0 }){
    //  playStartTone();
    //  testIK(0,0,20,0,val,0);
    //  delay(500);
   // }

    // lean left right
    for(const auto& val: {10, 0, -10, 0 }){
      playStartTone();
      testIK(0,0,defaultHeight,0,0,val);
      delay(200);
    }

    // lean front back
    for(const auto& val: {10, 25, 10, 0, -10, -25, 0 }){
      playStartTone();
      testIK(0,0,defaultHeight,val,0,0);
      delay(200);
    }

    playStartTone();
    testIK(0,0,defaultHeight,-10,0,0);
    playStartTone();
    moveCommandGroup({cRMCoxaPin, cLMCoxaPin}, {20, -20}, 1000, true);
    delay(200);

    moveCommandGroup({cRFFemurPin, cLFFemurPin}, {80, -80}, 1000, true);
    delay(200);
    moveCommandGroup({cRFFemurPin, cLFFemurPin}, {10, -10}, 1000, true);

    playStartTone();
    testIK(0, 0, 50);
    delay(1000);

    break;
  }


  // understand the concept of gaits with IK, first!
  // -------------- test walking ----------------
  /* testIK(0,0,0,0,0,0); */
  /* Serial.println("Starting walking sequence"); */
  /* walkForward(30, 20, 6); */
  /* Serial.println("done with walking sequence"); */
  /* // reset */
  /* delay(pauseTime); */
  // -------------- end of walking ----------------
  shutDownSequence();
  while(true){}
}
