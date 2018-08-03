
#define DebugSerial       Serial
#define SSCSerial         Serial1

#define SSCMaxPulse  2500
#define SSCMinPulse  500
#define SSCBase  (SSCMaxPulse - SSCMinPulse) / 2
#define PulseOffset  SSCMaxPulse - SSCBase
#define ServoMin  -1 * SSCBase
#define ServoMax  +1 * SSCBase


#define DEBUG 1
#define SSC_BAUD         115200
#define DEBUG_BAUD       115200
//--------------------------------------------------------------------

//[Arduino Pin Numbers]
#define BUZZER_PIN    10
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
#define COXA_LENGTH   28 // 35  // mm
#define FEMUR_LENGTH 100 // mm
#define TIBIA_LENGTH 120 // mm

#define INITIAL_FRONT_REAR_ANGLE 60.0

#define cRFCoxaAngle1    0     //Default Coxa setup angle, decimals = 1
#define cRMCoxaAngle1    0      //Default Coxa setup angle, decimals = 1
#define cRRCoxaAngle1    0     //Default Coxa setup angle, decimals = 1
#define cLRCoxaAngle1    0    //Default Coxa setup angle, decimals = 1
#define cLMCoxaAngle1    0      //Default Coxa setup angle, decimals = 1
#define cLFCoxaAngle1    0     //Default Coxa setup angle, decimals = 1

/* #define cRFCoxaAngle1    60     //Default Coxa setup angle, decimals = 1 */
/* #define cRMCoxaAngle1    0      //Default Coxa setup angle, decimals = 1 */
/* #define cRRCoxaAngle1   -60     //Default Coxa setup angle, decimals = 1 */
/* #define cLRCoxaAngle1    -60    //Default Coxa setup angle, decimals = 1 */
/* #define cLMCoxaAngle1    0      //Default Coxa setup angle, decimals = 1 */
/* #define cLFCoxaAngle1    60     //Default Coxa setup angle, decimals = 1 */


#define offsetXL1 42  // FrontRight
#define offsetXL2 62  // Center
#define offsetXL3 42  // Rear-Right
#define offsetXL4 -42 // Rear-Left
#define offsetXL5 -62 // Center-Left
#define offsetXL6 -42 // Front-Left

#define offsetYL1 82   // Front-Right
#define offsetYL2 0    // Center Right
#define offsetYL3 -82  // Rear-Right
#define offsetYL4 -82  // Rear-Left
#define offsetYL5 0    // Center-Left
#define offsetYL6 82   // Front-Left


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
