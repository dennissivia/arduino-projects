
#define DebugSerial       Serial
#define SSCSerial         Serial1

#define SSCMaxPulse  2500
#define SSCMinPulse  500
#define SSCBase  (SSCMaxPulse - SSCMinPulse) / 2
#define PulseOffset  SSCMaxPulse - SSCBase
#define ServoMin  -1 * SSCBase
#define ServoMax  +1 * SSCBase


#define SSC_BAUD         115200
#define DEBUG_BAUD       115200
//--------------------------------------------------------------------

//[Arduino Pin Numbers]
#define BUZZER_PIN    22
#define ECHO_PIN       2
#define TRIG_PIN       3
#define LEFT_LED_PIN   9
#define RIGHT_LED_PIN 10


//--------------------------------------------------------------------
//[SSC PIN NUMBERS]
#define cRRCoxaPin      0   //Rear Right leg Hip Horizontal
#define cRRFemurPin     1   //Rear Right leg Hip Vertical
#define cRRTibiaPin     2   //Rear Right leg Knee

#define cRMCoxaPin      4   //Middle Right leg Hip Horizontal
#define cRMFemurPin     5   //Middle Right leg Hip Vertical
#define cRMTibiaPin     6   //Middle Right leg Knee

#define cRFCoxaPin      8   //Front Right leg Hip Horizontal
#define cRFFemurPin     9   //Front Right leg Hip Vertical
#define cRFTibiaPin     10   //Front Right leg Knee

#define cLRCoxaPin      16   //Rear Left leg Hip Horizontal
#define cLRFemurPin     17   //Rear Left leg Hip Vertical
#define cLRTibiaPin     18   //Rear Left leg Knee

#define cLMCoxaPin      20   //Middle Left leg Hip Horizontal
#define cLMFemurPin     21   //Middle Left leg Hip Vertical
#define cLMTibiaPin     22   //Middle Left leg Knee

#define cLFCoxaPin      24   //Front Left leg Hip Horizontal
#define cLFFemurPin     25   //Front Left leg Hip Vertical
#define cLFTibiaPin     26   //Front Left leg Knee
//--------------------------------------------------------------------
//[MIN/MAX ANGLES]
/* #define CHexInitZ	  85  // MIN: -90 // MAX 90 */
/* #define CHexInitYC   0  // MIN: -80 // MAX: 70 */
/* #define CHexInitYR   0  // MIN: -65 // MAX 65 */
/* #define CHexInitYF   0  // MIN: -90 // MAX: 50 */
/* #define CHexInitX	  60  // MIN -90 // MAX 82 */

//Mechanical limits of the Right Rear Leg
#define cRRCoxaMin1     -65
#define cRRCoxaMax1     65
#define cRRFemurMin1    -90
#define cRRFemurMax1    90
#define cRRTibiaMin1    -90
#define cRRTibiaMax1    82

//Mechanical limits of the Right Middle Leg
#define cRMCoxaMin1     -80
#define cRMCoxaMax1     70
#define cRMFemurMin1    -90
#define cRMFemurMax1    90
#define cRMTibiaMin1    -90
#define cRMTibiaMax1    82

//Mechanical limits of the Right Front Leg
#define cRFCoxaMin1     -90
#define cRFCoxaMax1     50
#define cRFFemurMin1    -90
#define cRFFemurMax1    90
#define cRFTibiaMin1    -90
#define cRFTibiaMax1    82

//Mechanical limits of the Left Rear Leg
#define cLRCoxaMin1     -65
#define cLRCoxaMax1     65
#define cLRFemurMin1    -90
#define cLRFemurMax1    90
#define cLRTibiaMin1    -90
#define cLRTibiaMax1    82

//Mechanical limits of the Left Middle Leg
#define cLMCoxaMin1     -80
#define cLMCoxaMax1     70
#define cLMFemurMin1    -90
#define cLMFemurMax1    90
#define cLMTibiaMin1    -90
#define cLMTibiaMax1    82

//Mechanical limits of the Left Front Leg
#define cLFCoxaMin1     -90
#define cLFCoxaMax1     50
#define cLFFemurMin1    -90
#define cLFFemurMax1    90
#define cLFTibiaMin1    -90
#define cLFTibiaMax1    82

//--------------------------------------------------------------------
//[BODY DIMENSIONS]
// These are necessary for inverse kinemetics for the leg joints
#define cCoxaLength     35  // mm
#define cFemurLength    100 // mm
#define cTibiaLength    120 // mm
//
#define cRRCoxaAngle1   -60     //Default Coxa setup angle, decimals = 1
#define cRMCoxaAngle1    0      //Default Coxa setup angle, decimals = 1
#define cRFCoxaAngle1    60     //Default Coxa setup angle, decimals = 1
#define cLRCoxaAngle1    -60    //Default Coxa setup angle, decimals = 1
#define cLMCoxaAngle1    0      //Default Coxa setup angle, decimals = 1
#define cLFCoxaAngle1    60     //Default Coxa setup angle, decimals = 1
//
//#define cRROffsetX      -69     //Distance X from center of the body to the Right Rear coxa
//#define cRROffsetZ      119     //Distance Z from center of the body to the Right Rear coxa
//#define cRMOffsetX      -138    //Distance X from center of the body to the Right Middle coxa
//#define cRMOffsetZ      0       //Distance Z from center of the body to the Right Middle coxa
//#define cRFOffsetX      -69     //Distance X from center of the body to the Right Front coxa
//#define cRFOffsetZ      -119    //Distance Z from center of the body to the Right Front coxa
//
//#define cLROffsetX      69      //Distance X from center of the body to the Left Rear coxa
//#define cLROffsetZ      119     //Distance Z from center of the body to the Left Rear coxa
//#define cLMOffsetX      138     //Distance X from center of the body to the Left Middle coxa
//#define cLMOffsetZ      0       //Distance Z from center of the body to the Left Middle coxa
//#define cLFOffsetX      69      //Distance X from center of the body to the Left Front coxa
//#define cLFOffsetZ      -119    //Distance Z from center of the body to the Left Front coxa

//--------------------------------------------------------------------
// http://developercenter.robotstudio.com/BlobProxy/manuals/IRC5FlexPendantOpManual/images/xx0300000495-45330.png
// X Y Z axis for robots
//X = Tibai
//Y = Hip Horizontal
//Z = Hip Vertical

//[START POSITIONS FEET]
#define CHexInitZ	  85  // MIN: -90 // MAX 90
#define CHexInitYC   0  // MIN: -80 // MAX: 70
#define CHexInitYR   0  // MIN: -65 // MAX 65
#define CHexInitYF   0  // MIN: -90 // MAX: 50
#define CHexInitX	  60  // MIN -90 // MAX 82


#define cRRInitPosX     CHexInitX      //Start positions of the Right Rear leg
#define cRRInitPosY     CHexInitYR
#define cRRInitPosZ     CHexInitZ

#define cRMInitPosX     CHexInitX      //Start positions of the Right Middle leg
#define cRMInitPosY     CHexInitYC
#define cRMInitPosZ     CHexInitZ

#define cRFInitPosX     CHexInitX      //Start positions of the Right Front leg
#define cRFInitPosY     CHexInitYF
#define cRFInitPosZ     CHexInitZ


#define cLRInitPosX     -CHexInitX      //Start positions of the Left Rear leg
#define cLRInitPosY     -CHexInitYR
#define cLRInitPosZ     -CHexInitZ

#define cLMInitPosX     -CHexInitX      //Start positions of the Left Middle leg
#define cLMInitPosY     CHexInitYC
#define cLMInitPosZ     -CHexInitZ

#define cLFInitPosX     -CHexInitX      //Start positions of the Left Front leg
#define cLFInitPosY     -CHexInitYF
#define cLFInitPosZ     -CHexInitZ
//--------------------------------------------------------------------
