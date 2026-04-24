// Main Code for Knightsaurus. Install onto the Arduino UNO R4 WIFI Microcontroller.
// Team Knightsaurus: Spring 2026.

// This code reads in controller button presses (via the ESP32 SPI) and commands Knightsaurus
// to do the proper function.

// Functions: Forward Walking, Rough Terrain Walking, Backwards Walking (in Progress),
// Turning, Emotes (Dance, Lunge, Power Stance), Roar, Move Head, Move Tail.

// Background Functions: Battery Monitoring, Processing SPI (RC via ESP32) Commands, Updating OLED Display.

#include <SPI.h>
#include <Adafruit_PWMServoDriver.h>
#include <DFRobotDFPlayerMini.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include "00_LegStruct.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Use Wire because the screen is on A4/A5 with the PCA9685
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, -1);

// Definitions for the Display:
const char* currentModeString = "IDLE";
const char* currentGaitString = "NONE";
const char* currentEmoteString = "NONE";
const char* currentTurnValue = "NONE";


#define CS 10

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

DFRobotDFPlayerMini myDFPlayer;

// Servos:
#define SERVO1_CHANNEL  0
#define SERVO2_CHANNEL  1
#define SERVO3_CHANNEL  2
#define SERVO4_CHANNEL  3
#define SERVO5_CHANNEL  4
#define SERVO6_CHANNEL  5
#define SERVO7_CHANNEL  6
#define SERVO8_CHANNEL  7
#define SERVO9_CHANNEL  8
#define SERVO10_CHANNEL  9
#define SERVO11_CHANNEL  10
#define SERVO12_CHANNEL  11

// Based on the 45 kg Servos Used:
#define SERVO_MIN_PULSE  150
#define SERVO_MAX_PULSE  600

volatile uint8_t currentCommand = 0; 

// Servos for Leg Joints:
uint8_t leg1[3] = {0, 1, 2};  // SERVO1, SERVO2, SERVO3 -> Front Left
uint8_t leg2[3] = {3, 4, 5};  // SERVO4, SERVO5, SERVO6 -> Front Right
uint8_t leg3[3] = {6, 7, 8};  // SERVO7, SERVO8, SERVO9 -> Back Left
uint8_t leg4[3] = {9, 10, 11};  // SERVO10, SERVO11, SERVO12 -> Back Right


// Intiialize for Reading the LiPo Battery Percentage
#define lipoPin A0

// Used for the voltage divider:
const float R1 = 100000.0; // 100 kOhm resistor 1
const float R2 = 33000.0; // 30 kOhm resistor 2
int rawPinData = 0;
int rawV = 0;
float pinV = 0; // Stepped down voltage at the pin.
float batteryV = 0; // Battery voltage.
bool isPlaying = false;
int soundNum = 0;
float batteryVFiltered = 0; // Used for battery smoothing.


// Lookup table for LiPo percentage (Table to get the percentage since LiPo percentage is nonlinear)
struct Point { float voltage; int percent; };
Point table[] = {
  {16.8, 100}, {16.4, 90}, {16.0, 80}, {15.6, 70},
  {15.2, 60}, {14.8, 50}, {14.4, 40}, {14.0, 30},
  {13.6, 20}, {13.2, 10}, {13.0, 0}
};
const int tableSize = sizeof(table) / sizeof(table[0]); // Provides the size of the LiPo percentage table


// Dino Eyes (LEDs) Initialization:
const int LED_PIN    = 4;
const int JAW_SERVO_PIN  = 5;

// Jaw Servo Initialization:
const int JAW_CLOSED = 82;
const int JAW_OPEN   = 110;

Servo jawServo;

// Triggers Rough Terrain Mode
bool roughTerrainMode = false;

// Tail Servo Intialization:
int tailServoPos = 90; // Initialize tail servo position to zero degrees (90 degrees)
const int TAIL_ZERO = 90; // 90 degrees -> new 0 degrees
const int TAIL_FULL_LEFT = 70; // Max left position for the tail. 70 degrees -> new -20 degrees
const int TAIL_FULL_RIGHT = 110; // Max right position for the tail. 110 degrees -> new 20 degress
const int TAIL_STEP_SIZE = 1;  // How many degrees/steps the tail moves per iteration (used for controlling speed)

const int TAIL_SERVO_PIN = 9;
Servo tailServo;

// Neck Servo Initialization:
int neckServoPos = 90; // Initialize neck servo position to zero degrees (90 degrees)
const int NECK_ZERO = 90; // 90 degrees -> new 0 degrees
const int NECK_FULL_LEFT = 70; // Max left position for the neck. 70 degrees -> new -20 degrees
const int NECK_FULL_RIGHT = 110; // Max right position for the neck. 110 degrees -> new 20 degress
const int NECK_STEP_SIZE = 1;  // How many degrees/steps the neck moves per iteration (used for controlling speed)

const int NECK_SERVO_PIN = 6;
Servo neckServo;

// Initializing Emotes:

// Emote 1:
bool emote1Active = false;
bool emote1Resetting = false;
unsigned long emote1Start = 0;
const unsigned long EMOTE1_DURATION = 3000; // 3 seconds
int tailPhase = 0;

// Emote 2:
bool emote2Active = false;
bool emote2Resetting = false;
const unsigned long EMOTE2_DURATION = 4500; // 3 seconds
int emote2LegIndex = 0;     // 0–3 for legs 1→2→3→4
int emote2LegPhase = 0;     // 0 = lift, 1 = slam, 2 = return, 3 = next leg
unsigned long emote2Start = 0;

unsigned long lastHeadTailUpdate = 0;
const unsigned long HEAD_TAIL_INTERVAL = 120;
int emote2HeadTailPhase = 0;

// Emote 3:
bool emote3Active = false;
unsigned long emote3Start = 0;
const unsigned long EMOTE3_DURATION = 3500; // 3.5 seconds
bool emote3Resetting = false;

int emote3Phase = 0; // 0 = roar pose, 1 = stomp, 2 = tail whip, 3 = reset
unsigned long lastWhipUpdate = 0;
const unsigned long WHIP_INTERVAL = 60; // fast tail whip
int whipDirection = 0; // 0 = left, 1 = right


/*

Leg Layout (top down view):

.          Front

=============================
|           |  |            |
|           |  |            |
|     1     |  |     2      |
|           |  |            |
|           |  |            |
=============================
.    ||      ||      ||
.    ||      ||      ||
.    ||      ||      ||
.    ||      ||      ||
.    ||      ||      ||
.    ||      ||      ||
=============================
|           |  |            |
|           |  |            |
|     3     |  |     4      |
|           |  |            |
|           |  |            |
=============================

.           Back

*/

// Leg 1 Servo positions ------------------------------------------------------
const int SERVO1_Zero = 90; // new 0
const int POS_A_SERVO1 = 110; // new 20 deg ==> 10 deg in real life
const int POS_B_SERVO1 = 140; // new 50 deg ==> 25 deg in real life
const int POS_C_SERVO1 = 170; // new 80 deg ==> 40 deg in real life

const int SERVO2_Zero = 180; // new 0
const int POS_A_SERVO2 = 130; // new 50 deg ==> 25 deg in real life
const int POS_B_SERVO2 = 88; // new 92 deg ==> 46 deg in real life
const int POS_C_SERVO2 = 60; // new 140 deg ==> 70 deg in real life     40

// Leg 2 Servo positions ------------------------------------------------------

const int SERVO1_2_Zero = 90; // new 0
const int POS_A_SERVO1_2 = 70; // new -20 deg ==> -10 deg in real life
const int POS_B_SERVO1_2 = 40; // new -50 deg ==> -25 deg in real life
const int POS_C_SERVO1_2 = 10; // new -80 deg ==> -40 deg in real life

const int SERVO2_2_Zero = 90; // new 0
const int POS_A_SERVO2_2 = 140; // new -50 deg ==> 25 deg in real life
const int POS_B_SERVO2_2 = 182; // new -92 deg ==> -46 deg in real life
const int POS_C_SERVO2_2 = 210; // new -140 deg ==> -70 deg in real life     230


// Leg 3 Servo positions ------------------------------------------------------

const int SERVO1_3_Zero = 90; // new 0
const int POS_A_SERVO1_3 = 70; // new -20 deg ==> -10 deg in real life
const int POS_B_SERVO1_3 = 40; // new -50 deg ==> -25 deg in real life
const int POS_C_SERVO1_3 = 10; // new -80 deg ==> -40 deg in real life

const int SERVO2_3_Zero = 90; // new 0
const int POS_A_SERVO2_3 = 140; // new -50 deg ==> 25 deg in real life
const int POS_B_SERVO2_3 = 182; // new -92 deg ==> --46 deg in real life
const int POS_C_SERVO2_3 = 210; // new -140 deg ==> -70 deg in real life     230

// Leg 4 Servo positions ------------------------------------------------------

const int SERVO1_4_Zero = 90; // new 0
const int POS_A_SERVO1_4 = 110; // new 20 deg ==> 10 deg in real life
const int POS_B_SERVO1_4 = 140; // new 50 deg ==> 25 deg in real life
const int POS_C_SERVO1_4 = 170; // new 80 deg ==> 40 deg in real life

const int SERVO2_4_Zero = 180; // new 0
const int POS_A_SERVO2_4 = 130; // new 50 deg ==> 25 deg in real life
const int POS_B_SERVO2_4 = 88; // new 92 deg ==> 46 deg in real life
const int POS_C_SERVO2_4 = 60; // new 140 deg ==> 70 deg in real life     40


// Servo 3 positions -------------------------------------------------------------
const int SERVO3_HOME_2_3   = 0;
const int SERVO3_HOME_1_4   = 80;
const int TURN_YAW = 8;
int SERVO3_POS  = SERVO3_HOME_2_3;

// Hip-Yaw Neutral Angles (Servo 3 on each leg)
const int HY_FL_NEUTRAL = SERVO3_HOME_1_4;  // Leg 1 → 90°
const int HY_FR_NEUTRAL = SERVO3_HOME_2_3;  // Leg 2 → 0°
const int HY_RL_NEUTRAL = SERVO3_HOME_2_3;  // Leg 3 → 0°
const int HY_RR_NEUTRAL = SERVO3_HOME_1_4;  // Leg 4 → 90°

// Hip-Yaw servo channels (Servo 3 on each leg)
const int hipYawChannel[4] = {
    2,   // Leg 1
    5,   // Leg 2
    8,   // Leg 3
    11   // Leg 4
};

int hipYawNeutral[4] = {
    HY_FL_NEUTRAL,
    HY_FR_NEUTRAL,
    HY_RL_NEUTRAL,
    HY_RR_NEUTRAL
};

// Left legs = 1 & 4 → -1
// Right legs = 2 & 3 → +1

const int hipYawOutward[4] = { -1, +1, +1, -1 };


// Auto-Tuning Parameters:
float SPEED_SCALE = 0.8;        // 1.0 = normal speed

// Load scaling per leg (used for fine tuning)
const float LEG1_LOAD_SCALE = 1.00;
const float LEG2_LOAD_SCALE = 1.00;
const float LEG3_LOAD_SCALE = 1.00;
const float LEG4_LOAD_SCALE = 1.00;

// Sync pause system
bool gaitPaused = false;
unsigned long pauseStart = 0;
const unsigned long PAUSE_DURATION = 100; // ms pause between diagonal pairs

// Helper to scale stride toward neutral B
int scaledAngle(int neutralB, int target, float scale) {
  return neutralB + (int)((target - neutralB) * scale);
}



// Initialize States for the Non-Blocking Gait Engine:
bool walkingForward = false;  // Flag for checking if the robot is currently walking forwards.
bool walkingBackward = false; // Flag for checking if the robot is currently walking backwards.
int servo1Pos = POS_B_SERVO1; // Initializes servo 1 position for the front legs
int servo2Pos = POS_B_SERVO2; // Initializes servo 2 position for the front legs
int servo1Pos_Back = POS_B_SERVO1_4; // Initializes servo 1 position for the back legs
int servo2Pos_Back = POS_B_SERVO2_4; // Initializes servo 2 position for the back legs
bool walkingShutdown = false; // Flag to finish gait movement when DPad is released.

// Turning Global Variable (Used to activate the turn)
float turnValue = 0.0;   // -1 = left, +1 = right, 0 = straight

bool killSwitchFlag = false; // Flag for checking if the robot kill switch should be activated.

int gaitPhase = 0; // Initializes the gate phase.


// Servo Arrays (0 is hip, 1 is knee)
uint8_t leg1Servos[2] = { leg1[0], leg1[1] };
uint8_t leg2Servos[2] = { leg2[0], leg2[1] };
uint8_t leg3Servos[2] = { leg3[0], leg3[1] };
uint8_t leg4Servos[2] = { leg4[0], leg4[1] };


// Helper Function for Setting the Servo Angles:
void setServoAngle270(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 270);
  uint16_t pulse = map(angle, 0, 270, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  pwm.setPWM(channel, 0, pulse);
}

// Structs defining the legs:
// Layout -> hip, knee, target hip, target knee, hipServo, kneeServo, active flag
// Defined in 00_LegStruct.h
LegState leg1State = {
  POS_B_SERVO1, POS_B_SERVO2,
  POS_B_SERVO1, POS_B_SERVO2,
  leg1Servos[0], leg1Servos[1],
  false
};

LegState leg2State = {
  POS_B_SERVO1_2, POS_B_SERVO2_2,
  POS_B_SERVO1_2, POS_B_SERVO2_2,
  leg2Servos[0], leg2Servos[1],
  false
};

LegState leg3State = {
  POS_B_SERVO1_3, POS_B_SERVO2_3,
  POS_B_SERVO1_3, POS_B_SERVO2_3,
  leg3Servos[0], leg3Servos[1],
  false
};

LegState leg4State = {
  POS_B_SERVO1_4, POS_B_SERVO2_4,
  POS_B_SERVO1_4, POS_B_SERVO2_4,
  leg4Servos[0], leg4Servos[1],
  false
};

// Phases for the leg pairs (forwards and backwards)
uint8_t phase14 = 0;
uint8_t phase23 = 0;
uint8_t phase14_back = 0;
uint8_t phase23_back = 0;

unsigned long lastStepTime = 0;
unsigned long stepInterval = 12;


// Moves one leg toward its target position.
// Returns true when BOTH hip and knee have reached target.
bool stepLegTowardTarget(LegState &L) {
  if (!L.active) return true;   // If leg is inactive, treat as "already done"

  const int BASE_STEP_SIZE = 3;
  int STEP_SIZE = max(1, (int)(BASE_STEP_SIZE * SPEED_SCALE));  
  // SPEED_SCALE allows global speed control

  // Move hip and knee toward their targets
  if (L.hip < L.targetHip)  L.hip += STEP_SIZE;
  if (L.hip > L.targetHip)  L.hip -= STEP_SIZE;
  if (L.knee < L.targetKnee) L.knee += STEP_SIZE;
  if (L.knee > L.targetKnee) L.knee -= STEP_SIZE;

  // Prevent overshoot by clamping to exact target
  if ((L.hip < L.targetHip && L.hip + STEP_SIZE > L.targetHip) ||
      (L.hip > L.targetHip && L.hip - STEP_SIZE < L.targetHip)) {
    L.hip = L.targetHip;
  }
  if ((L.knee < L.targetKnee && L.knee + STEP_SIZE > L.targetKnee) ||
      (L.knee > L.targetKnee && L.knee - STEP_SIZE < L.targetKnee)) {
    L.knee = L.targetKnee;
  }

  // Send updated angles to servos
  setServoAngle270(L.hipServo,  L.hip);
  setServoAngle270(L.kneeServo, L.knee);

  // Return true only when both joints have reached their targets
  return (L.hip == L.targetHip && L.knee == L.targetKnee);
}


// Gait phases for Leg 1 + Leg 4 (diagonal pair) for Forward Walking.
// Scaled angle can be used to reduce torque if needed (for a leg)
void setTargetsPhase14() {
  switch (phase14) {

    case 0: // Both legs move to neutral B
      leg1State.targetHip  = POS_B_SERVO1;
      leg1State.targetKnee = POS_B_SERVO2;
      leg4State.targetHip  = POS_B_SERVO1_4;
      leg4State.targetKnee = POS_B_SERVO2_4;
      break;

    case 1: // Leg1 knee B→C (scaled), Leg4 hip B→C (scaled)
      leg1State.targetHip  = leg1State.hip;
      leg1State.targetKnee = scaledAngle(POS_B_SERVO2, POS_C_SERVO2, LEG1_LOAD_SCALE);

      leg4State.targetHip  = scaledAngle(POS_B_SERVO1_4, POS_C_SERVO1_4, LEG4_LOAD_SCALE);
      leg4State.targetKnee = leg4State.knee;
      break;

    case 2: // Leg1 hip B→A (scaled), Leg4 knee B→A (scaled)
      leg1State.targetHip  = scaledAngle(POS_B_SERVO1, POS_A_SERVO1, LEG1_LOAD_SCALE);
      leg1State.targetKnee = leg1State.knee;

      leg4State.targetHip  = leg4State.hip;
      leg4State.targetKnee = scaledAngle(POS_B_SERVO2_4, POS_A_SERVO2_4, LEG4_LOAD_SCALE);
      break;

    case 3: // Leg1 knee C→A (scaled), Leg4 hip C→A (scaled)
      leg1State.targetHip  = leg1State.hip;
      leg1State.targetKnee = scaledAngle(POS_C_SERVO2, POS_A_SERVO2, LEG1_LOAD_SCALE);

      leg4State.targetHip  = scaledAngle(POS_C_SERVO1_4, POS_A_SERVO1_4, LEG4_LOAD_SCALE);
      leg4State.targetKnee = leg4State.knee;
      break;

    case 4: // Leg1 hip A→C (scaled), Leg4 knee A→C (scaled)
      leg1State.targetHip  = scaledAngle(POS_A_SERVO1, POS_C_SERVO1, LEG1_LOAD_SCALE);
      leg1State.targetKnee = leg1State.knee;

      leg4State.targetHip  = leg4State.hip;
      leg4State.targetKnee = scaledAngle(POS_A_SERVO2_4, POS_C_SERVO2_4, LEG4_LOAD_SCALE);
      break;

    case 5: // Leg1 knee A→C (scaled), Leg4 hip A→B (scaled)
      leg1State.targetHip  = leg1State.hip;
      leg1State.targetKnee = scaledAngle(POS_A_SERVO2, POS_C_SERVO2, LEG1_LOAD_SCALE);

      leg4State.targetHip  = scaledAngle(POS_A_SERVO1_4, POS_B_SERVO1_4, LEG4_LOAD_SCALE);
      leg4State.targetKnee = leg4State.knee;
      break;

    case 6: // Leg1 hip C→B (scaled), Leg4 holds
      leg1State.targetHip  = scaledAngle(POS_C_SERVO1, POS_B_SERVO1, LEG1_LOAD_SCALE);
      leg1State.targetKnee = leg1State.knee;
      break;

    case 7: // Leg1 knee C→B (scaled), Leg4 knee C→B (scaled)
      leg1State.targetHip  = leg1State.hip;
      leg1State.targetKnee = scaledAngle(POS_C_SERVO2, POS_B_SERVO2, LEG1_LOAD_SCALE);

      leg4State.targetHip  = leg4State.hip;
      leg4State.targetKnee = scaledAngle(POS_C_SERVO2_4, POS_B_SERVO2_4, LEG4_LOAD_SCALE);
      break;
  }
}



// Gait phases for Leg 2 + Leg 3 (diagonal pair) for Forward Walking.
// Scaled angle can be used to reduce torque if needed (for a leg)
void setTargetsPhase23() {
  switch (phase23) {

    case 0: // Both to neutral B
      leg2State.targetHip  = POS_B_SERVO1_2;
      leg2State.targetKnee = POS_B_SERVO2_2;
      leg3State.targetHip  = POS_B_SERVO1_3;
      leg3State.targetKnee = POS_B_SERVO2_3;
      break;

    case 1: // Leg2 knee B→C (scaled), Leg3 hip B→C (scaled)
      leg2State.targetHip  = leg2State.hip;
      leg2State.targetKnee = scaledAngle(POS_B_SERVO2_2, POS_C_SERVO2_2, LEG2_LOAD_SCALE);

      leg3State.targetHip  = scaledAngle(POS_B_SERVO1_3, POS_C_SERVO1_3, LEG3_LOAD_SCALE);
      leg3State.targetKnee = leg3State.knee;
      break;

    case 2: // Leg2 hip B→A (scaled), Leg3 knee B→A (scaled)
      leg2State.targetHip  = scaledAngle(POS_B_SERVO1_2, POS_A_SERVO1_2, LEG2_LOAD_SCALE);
      leg2State.targetKnee = leg2State.knee;

      leg3State.targetHip  = leg3State.hip;
      leg3State.targetKnee = scaledAngle(POS_B_SERVO2_3, POS_A_SERVO2_3, LEG3_LOAD_SCALE);
      break;

    case 3: // Leg2 knee C→A (scaled), Leg3 hip C→A (scaled)
      leg2State.targetHip  = leg2State.hip;
      leg2State.targetKnee = scaledAngle(POS_C_SERVO2_2, POS_A_SERVO2_2, LEG2_LOAD_SCALE);

      leg3State.targetHip  = scaledAngle(POS_C_SERVO1_3, POS_A_SERVO1_3, LEG3_LOAD_SCALE);
      leg3State.targetKnee = leg3State.knee;
      break;

    case 4: // Leg2 hip A→C (scaled), Leg3 knee A→C (scaled)
      leg2State.targetHip  = scaledAngle(POS_A_SERVO1_2, POS_C_SERVO1_2, LEG2_LOAD_SCALE);
      leg2State.targetKnee = leg2State.knee;

      leg3State.targetHip  = leg3State.hip;
      leg3State.targetKnee = scaledAngle(POS_A_SERVO2_3, POS_C_SERVO2_3, LEG3_LOAD_SCALE);
      break;

    case 5: // Leg2 knee A→C (scaled), Leg3 hip A→B (scaled)
      leg2State.targetHip  = leg2State.hip;
      leg2State.targetKnee = scaledAngle(POS_A_SERVO2_2, POS_C_SERVO2_2, LEG2_LOAD_SCALE);

      leg3State.targetHip  = scaledAngle(POS_A_SERVO1_3, POS_B_SERVO1_3, LEG3_LOAD_SCALE);
      leg3State.targetKnee = leg3State.knee;
      break;

    case 6: // Leg2 hip C→B (scaled)
      leg2State.targetHip  = scaledAngle(POS_C_SERVO1_2, POS_B_SERVO1_2, LEG2_LOAD_SCALE);
      leg2State.targetKnee = leg2State.knee;
      break;

    case 7: // Leg2 & Leg3 knee C→B (scaled)
      leg2State.targetHip  = leg2State.hip;
      leg2State.targetKnee = scaledAngle(POS_C_SERVO2_2, POS_B_SERVO2_2, LEG2_LOAD_SCALE);

      leg3State.targetHip  = leg3State.hip;
      leg3State.targetKnee = scaledAngle(POS_C_SERVO2_3, POS_B_SERVO2_3, LEG3_LOAD_SCALE);
      break;
  }
}

// Gait phases for Leg 1 + Leg 4 (diagonal pair) for Backward Walking.
// Scaled angle can be used to reduce torque if needed (for a leg)
void setTargetsPhase14_Back() {
  switch (phase14_back) {

    case 0: // Both legs move to neutral B
      leg1State.targetHip  = POS_B_SERVO1;
      leg1State.targetKnee = POS_B_SERVO2;
      leg4State.targetHip  = POS_B_SERVO1_4;
      leg4State.targetKnee = POS_B_SERVO2_4;
      break;

    case 1: // Leg1 knee B→A (lift), Leg4 hip B→A (lift)
      leg1State.targetHip  = leg1State.hip;
      leg1State.targetKnee = scaledAngle(POS_B_SERVO2, POS_A_SERVO2, LEG1_LOAD_SCALE);

      leg4State.targetHip  = scaledAngle(POS_B_SERVO1_4, POS_A_SERVO1_4, LEG4_LOAD_SCALE);
      leg4State.targetKnee = leg4State.knee;
      break;

    case 2: // Leg1 hip B→C (back swing), Leg4 knee B→C (back swing)
      leg1State.targetHip  = scaledAngle(POS_B_SERVO1, POS_C_SERVO1, LEG1_LOAD_SCALE);
      leg1State.targetKnee = leg1State.knee;

      leg4State.targetHip  = leg4State.hip;
      leg4State.targetKnee = scaledAngle(POS_B_SERVO2_4, POS_C_SERVO2_4, LEG4_LOAD_SCALE);
      break;

    case 3: // Leg1 knee A→C (plant), Leg4 hip A→C (plant)
      leg1State.targetHip  = leg1State.hip;
      leg1State.targetKnee = scaledAngle(POS_A_SERVO2, POS_C_SERVO2, LEG1_LOAD_SCALE);

      leg4State.targetHip  = scaledAngle(POS_A_SERVO1_4, POS_C_SERVO1_4, LEG4_LOAD_SCALE);
      leg4State.targetKnee = leg4State.knee;
      break;

    case 4: // Leg1 hip C→A (push forward), Leg4 knee C→A (push forward)
      leg1State.targetHip  = scaledAngle(POS_C_SERVO1, POS_A_SERVO1, LEG1_LOAD_SCALE);
      leg1State.targetKnee = leg1State.knee;

      leg4State.targetHip  = leg4State.hip;
      leg4State.targetKnee = scaledAngle(POS_C_SERVO2_4, POS_A_SERVO2_4, LEG4_LOAD_SCALE);
      break;

    case 5: // Leg1 knee C→A (lift), Leg4 hip C→B (return)
      leg1State.targetHip  = leg1State.hip;
      leg1State.targetKnee = scaledAngle(POS_C_SERVO2, POS_A_SERVO2, LEG1_LOAD_SCALE);

      leg4State.targetHip  = scaledAngle(POS_C_SERVO1_4, POS_B_SERVO1_4, LEG4_LOAD_SCALE);
      leg4State.targetKnee = leg4State.knee;
      break;

    case 6: // Leg1 hip A→B (return)
      leg1State.targetHip  = scaledAngle(POS_A_SERVO1, POS_B_SERVO1, LEG1_LOAD_SCALE);
      leg1State.targetKnee = leg1State.knee;
      break;

    case 7: // Leg1 knee A→B, Leg4 knee A→B
      leg1State.targetHip  = leg1State.hip;
      leg1State.targetKnee = scaledAngle(POS_A_SERVO2, POS_B_SERVO2, LEG1_LOAD_SCALE);

      leg4State.targetHip  = leg4State.hip;
      leg4State.targetKnee = scaledAngle(POS_A_SERVO2_4, POS_B_SERVO2_4, LEG4_LOAD_SCALE);
      break;
  }
}

// Gait phases for Leg 2 + Leg 3 (diagonal pair) for Backward Walking.
// Scaled angle can be used to reduce torque if needed (for a leg)
void setTargetsPhase23_Back() {
  switch (phase23_back) {

    case 0: // Both to neutral B
      leg2State.targetHip  = POS_B_SERVO1_2;
      leg2State.targetKnee = POS_B_SERVO2_2;
      leg3State.targetHip  = POS_B_SERVO1_3;
      leg3State.targetKnee = POS_B_SERVO2_3;
      break;

    case 1: // Leg2 knee B→A (lift), Leg3 hip B→A (lift)
      leg2State.targetHip  = leg2State.hip;
      leg2State.targetKnee = scaledAngle(POS_B_SERVO2_2, POS_A_SERVO2_2, LEG2_LOAD_SCALE);

      leg3State.targetHip  = scaledAngle(POS_B_SERVO1_3, POS_A_SERVO1_3, LEG3_LOAD_SCALE);
      leg3State.targetKnee = leg3State.knee;
      break;

    case 2: // Leg2 hip B→C (back swing), Leg3 knee B→C (back swing)
      leg2State.targetHip  = scaledAngle(POS_B_SERVO1_2, POS_C_SERVO1_2, LEG2_LOAD_SCALE);
      leg2State.targetKnee = leg2State.knee;

      leg3State.targetHip  = leg3State.hip;
      leg3State.targetKnee = scaledAngle(POS_B_SERVO2_3, POS_C_SERVO2_3, LEG3_LOAD_SCALE);
      break;

    case 3: // Leg2 knee A→C (plant), Leg3 hip A→C (plant)
      leg2State.targetHip  = leg2State.hip;
      leg2State.targetKnee = scaledAngle(POS_A_SERVO2_2, POS_C_SERVO2_2, LEG2_LOAD_SCALE);

      leg3State.targetHip  = scaledAngle(POS_A_SERVO1_3, POS_C_SERVO1_3, LEG3_LOAD_SCALE);
      leg3State.targetKnee = leg3State.knee;
      break;

    case 4: // Leg2 hip C→A (push forward), Leg3 knee C→A (push forward)
      leg2State.targetHip  = scaledAngle(POS_C_SERVO1_2, POS_A_SERVO1_2, LEG2_LOAD_SCALE);
      leg2State.targetKnee = leg2State.knee;

      leg3State.targetHip  = leg3State.hip;
      leg3State.targetKnee = scaledAngle(POS_C_SERVO2_3, POS_A_SERVO2_3, LEG3_LOAD_SCALE);
      break;

    case 5: // Leg2 knee C→A (lift), Leg3 hip C→B (return)
      leg2State.targetHip  = leg2State.hip;
      leg2State.targetKnee = scaledAngle(POS_C_SERVO2_2, POS_A_SERVO2_2, LEG2_LOAD_SCALE);

      leg3State.targetHip  = scaledAngle(POS_C_SERVO1_3, POS_B_SERVO1_3, LEG3_LOAD_SCALE);
      leg3State.targetKnee = leg3State.knee;
      break;

    case 6: // Leg2 hip A→B (return)
      leg2State.targetHip  = scaledAngle(POS_A_SERVO1_2, POS_B_SERVO1_2, LEG2_LOAD_SCALE);
      leg2State.targetKnee = leg2State.knee;
      break;

    case 7: // Leg2 & Leg3 knee A→B (return)
      leg2State.targetHip  = leg2State.hip;
      leg2State.targetKnee = scaledAngle(POS_A_SERVO2_2, POS_B_SERVO2_2, LEG2_LOAD_SCALE);

      leg3State.targetHip  = leg3State.hip;
      leg3State.targetKnee = scaledAngle(POS_A_SERVO2_3, POS_B_SERVO2_3, LEG3_LOAD_SCALE);
      break;
  }
}

// Gait phases for Leg 1 + Leg 4 (diagonal pair) for Rough Terrain.
// Scaled angle can be used to reduce torque if needed (for a leg)
void setTargetsPhase14_RoughTerrain() {
  switch (phase14) {

    case 0: // Phase 1: All B
      leg1State.targetHip  = POS_B_SERVO1;
      leg1State.targetKnee = POS_B_SERVO2;
      leg4State.targetHip  = POS_B_SERVO1_4;
      leg4State.targetKnee = POS_B_SERVO2_4;
      break;

    case 1: // Phase 2: knees B→C
      leg1State.targetHip  = POS_B_SERVO1;
      leg1State.targetKnee = scaledAngle(POS_B_SERVO2, POS_C_SERVO2, LEG1_LOAD_SCALE);

      leg4State.targetHip  = POS_B_SERVO1_4;
      leg4State.targetKnee = scaledAngle(POS_B_SERVO2_4, POS_C_SERVO2_4, LEG4_LOAD_SCALE);
      break;

    case 2: // Phase 3: L1 hip B→A, L4 hip B→C
      leg1State.targetHip  = scaledAngle(POS_B_SERVO1, POS_A_SERVO1, LEG1_LOAD_SCALE);
      leg1State.targetKnee = POS_C_SERVO2;

      leg4State.targetHip  = scaledAngle(POS_B_SERVO1_4, POS_C_SERVO1_4, LEG4_LOAD_SCALE);
      leg4State.targetKnee = POS_C_SERVO2_4;
      break;

    case 3: // Phase 4: L4 knee C→A
      leg1State.targetHip  = POS_A_SERVO1;
      leg1State.targetKnee = POS_C_SERVO2;

      leg4State.targetHip  = POS_C_SERVO1_4;
      leg4State.targetKnee = scaledAngle(POS_C_SERVO2_4, POS_A_SERVO2_4, LEG4_LOAD_SCALE);
      break;

    case 4: // Phase 5: L1 knee C→A, L4 hip C→A
      leg1State.targetHip  = POS_A_SERVO1;
      leg1State.targetKnee = scaledAngle(POS_C_SERVO2, POS_A_SERVO2, LEG1_LOAD_SCALE);

      leg4State.targetHip  = scaledAngle(POS_C_SERVO1_4, POS_A_SERVO1_4, LEG4_LOAD_SCALE);
      leg4State.targetKnee = POS_A_SERVO2_4;
      break;

    case 5: // Phase 6: L1 hip A→C, L4 knee A→C
      leg1State.targetHip  = scaledAngle(POS_A_SERVO1, POS_C_SERVO1, LEG1_LOAD_SCALE);
      leg1State.targetKnee = POS_A_SERVO2;

      leg4State.targetHip  = POS_A_SERVO1_4;
      leg4State.targetKnee = scaledAngle(POS_A_SERVO2_4, POS_C_SERVO2_4, LEG4_LOAD_SCALE);
      break;

    case 6: // Phase 7: L1 knee A→B, L4 hip A→B
      leg1State.targetHip  = POS_C_SERVO1;
      leg1State.targetKnee = scaledAngle(POS_A_SERVO2, POS_B_SERVO2, LEG1_LOAD_SCALE);

      leg4State.targetHip  = scaledAngle(POS_A_SERVO1_4, POS_B_SERVO1_4, LEG4_LOAD_SCALE);
      leg4State.targetKnee = POS_C_SERVO2_4;
      break;

    case 7: // Phase 8: L1 hip C→B, L4 knee C→B
      leg1State.targetHip  = scaledAngle(POS_C_SERVO1, POS_B_SERVO1, LEG1_LOAD_SCALE);
      leg1State.targetKnee = POS_C_SERVO2;

      leg4State.targetHip  = POS_B_SERVO1_4;
      leg4State.targetKnee = scaledAngle(POS_C_SERVO2_4, POS_B_SERVO2_4, LEG4_LOAD_SCALE);
      break;

    case 8: // Phase 9: return to B
      leg1State.targetHip  = POS_B_SERVO1;
      leg1State.targetKnee = scaledAngle(POS_C_SERVO2, POS_B_SERVO2, LEG1_LOAD_SCALE);

      leg4State.targetHip  = POS_B_SERVO1_4;
      leg4State.targetKnee = POS_B_SERVO2_4;
      break;
  }
}

// Gait phases for Leg 2 + Leg 3 (diagonal pair) for Rough Terrain.
// Scaled angle can be used to reduce torque if needed (for a leg)
void setTargetsPhase23_RoughTerrain() {
  switch (phase23) {

    case 0: // Phase 1
      leg2State.targetHip  = POS_B_SERVO1_2;
      leg2State.targetKnee = POS_B_SERVO2_2;
      leg3State.targetHip  = POS_B_SERVO1_3;
      leg3State.targetKnee = POS_B_SERVO2_3;
      break;

    case 1: // Phase 2
      leg2State.targetHip  = POS_B_SERVO1_2;
      leg2State.targetKnee = scaledAngle(POS_B_SERVO2_2, POS_C_SERVO2_2, LEG2_LOAD_SCALE);

      leg3State.targetHip  = POS_B_SERVO1_3;
      leg3State.targetKnee = scaledAngle(POS_B_SERVO2_3, POS_C_SERVO2_3, LEG3_LOAD_SCALE);
      break;

    case 2: // Phase 3
      leg2State.targetHip  = scaledAngle(POS_B_SERVO1_2, POS_A_SERVO1_2, LEG2_LOAD_SCALE);
      leg2State.targetKnee = POS_C_SERVO2_2;

      leg3State.targetHip  = scaledAngle(POS_B_SERVO1_3, POS_C_SERVO1_3, LEG3_LOAD_SCALE);
      leg3State.targetKnee = POS_C_SERVO2_3;
      break;

    case 3: // Phase 4
      leg2State.targetHip  = POS_A_SERVO1_2;
      leg2State.targetKnee = POS_C_SERVO2_2;

      leg3State.targetHip  = POS_C_SERVO1_3;
      leg3State.targetKnee = scaledAngle(POS_C_SERVO2_3, POS_A_SERVO2_3, LEG3_LOAD_SCALE);
      break;

    case 4: // Phase 5
      leg2State.targetHip  = POS_A_SERVO1_2;
      leg2State.targetKnee = scaledAngle(POS_C_SERVO2_2, POS_A_SERVO2_2, LEG2_LOAD_SCALE);

      leg3State.targetHip  = scaledAngle(POS_C_SERVO1_3, POS_A_SERVO1_3, LEG3_LOAD_SCALE);
      leg3State.targetKnee = POS_A_SERVO2_3;
      break;

    case 5: // Phase 6
      leg2State.targetHip  = scaledAngle(POS_A_SERVO1_2, POS_C_SERVO1_2, LEG2_LOAD_SCALE);
      leg2State.targetKnee = POS_A_SERVO2_2;

      leg3State.targetHip  = POS_A_SERVO1_3;
      leg3State.targetKnee = scaledAngle(POS_A_SERVO2_3, POS_C_SERVO2_3, LEG3_LOAD_SCALE);
      break;

    case 6: // Phase 7
      leg2State.targetHip  = POS_C_SERVO1_2;
      leg2State.targetKnee = scaledAngle(POS_A_SERVO2_2, POS_B_SERVO2_2, LEG2_LOAD_SCALE);

      leg3State.targetHip  = scaledAngle(POS_A_SERVO1_3, POS_B_SERVO1_3, LEG3_LOAD_SCALE);
      leg3State.targetKnee = POS_C_SERVO2_3;
      break;

    case 7: // Phase 8
      leg2State.targetHip  = scaledAngle(POS_C_SERVO1_2, POS_B_SERVO1_2, LEG2_LOAD_SCALE);
      leg2State.targetKnee = POS_C_SERVO2_2;

      leg3State.targetHip  = POS_B_SERVO1_3;
      leg3State.targetKnee = scaledAngle(POS_C_SERVO2_3, POS_B_SERVO2_3, LEG3_LOAD_SCALE);
      break;

    case 8: // Phase 9
      leg2State.targetHip  = POS_B_SERVO1_2;
      leg2State.targetKnee = scaledAngle(POS_C_SERVO2_2, POS_B_SERVO2_2, LEG2_LOAD_SCALE);

      leg3State.targetHip  = POS_B_SERVO1_3;
      leg3State.targetKnee = POS_B_SERVO2_3;
      break;
  }
}


// Update walk function with turning integrated. 
// This is the primary walking function for Knightsaurus.
void updateWalk() {

  if (emote1Active || emote2Active || emote3Active) return; // prevents walking/turning during emote

  // Note: Turning Overrides Walking!

  // Left Turn:
  if (turnValue <= -0.1) {

      // Disable gait
      leg1State.active = false;
      leg2State.active = false;
      leg3State.active = false;
      leg4State.active = false;

      // ---- Lift Legs 1 & 4 ----
      setServoAngle270(leg1Servos[0], POS_C_SERVO1);      // Leg 1 hip
      setServoAngle270(leg1Servos[1], POS_C_SERVO2);     // Leg 1 knee

      setServoAngle270(leg4Servos[0], POS_C_SERVO1_4);    // Leg 4 hip
      setServoAngle270(leg4Servos[1], POS_C_SERVO2_4);   // Leg 4 knee

      delay(250);

      // ---- Push outward Legs 2 & 3 ----
      int yaw2 = hipYawNeutral[1] + hipYawOutward[1] * TURN_YAW;
      int yaw3 = hipYawNeutral[2] + hipYawOutward[2] * TURN_YAW;

      setServoAngle270(hipYawChannel[1], yaw2);
      setServoAngle270(hipYawChannel[2], yaw3);

      delay(250);

     // ---- put Legs 1 & 4 down ----
      setServoAngle270(leg1Servos[0], POS_B_SERVO1);      // Leg 1 hip
      setServoAngle270(leg1Servos[1], POS_B_SERVO2);     // Leg 1 knee

      setServoAngle270(leg4Servos[0], POS_B_SERVO1_4);    // Leg 4 hip
      setServoAngle270(leg4Servos[1], POS_B_SERVO2_4);   // Leg 4 knee

      delay(250);

      // ---- lift Legs 2 & 3 (hip + knee to C) ----
      setServoAngle270(leg2Servos[0], POS_C_SERVO1_2);
      setServoAngle270(leg2Servos[1], POS_C_SERVO2_2);

      setServoAngle270(leg3Servos[0], POS_C_SERVO1_3);
      setServoAngle270(leg3Servos[1], POS_C_SERVO2_3);

      delay(250);

      // legs 2 and 3 hips back to 0
      setServoAngle270(5, SERVO3_HOME_2_3);
      setServoAngle270(8, SERVO3_HOME_2_3);

      delay(250);

      // ---- Legs 2 & 3 back to B ----
      setServoAngle270(leg2Servos[0], POS_B_SERVO1_2);
      setServoAngle270(leg2Servos[1], POS_B_SERVO2_2);

      setServoAngle270(leg3Servos[0], POS_B_SERVO1_3);
      setServoAngle270(leg3Servos[1], POS_B_SERVO2_3);

      delay(250);

      return;
  }


  // Right Turn:
  if (turnValue >= 0.1) {

      // Disable gait
      leg1State.active = false;
      leg2State.active = false;
      leg3State.active = false;
      leg4State.active = false;

      // ---- Lift Legs 2 & 3 ----
      setServoAngle270(leg2Servos[0], POS_C_SERVO1_2);
      setServoAngle270(leg2Servos[1], POS_C_SERVO2_2);

      setServoAngle270(leg3Servos[0], POS_C_SERVO1_3);
      setServoAngle270(leg3Servos[1], POS_C_SERVO2_3);

      delay(250);

      // ---- Push outward Legs 1 & 4 ----
      int yaw1 = hipYawNeutral[0] + hipYawOutward[0] * TURN_YAW;
      int yaw4 = hipYawNeutral[3] + hipYawOutward[3] * TURN_YAW;

      setServoAngle270(hipYawChannel[0], yaw1);
      setServoAngle270(hipYawChannel[3], yaw4);

      delay(250);

      // ---- Legs 2 & 3 down ----
      setServoAngle270(leg2Servos[0], POS_B_SERVO1_2);
      setServoAngle270(leg2Servos[1], POS_B_SERVO2_2);

      setServoAngle270(leg3Servos[0], POS_B_SERVO1_3);
      setServoAngle270(leg3Servos[1], POS_B_SERVO2_3);

      delay(250);

      // ---- Legs 1 & 4 up ----
      setServoAngle270(leg1Servos[0], POS_C_SERVO1);
      setServoAngle270(leg1Servos[1], POS_C_SERVO2);

      setServoAngle270(leg4Servos[0], POS_C_SERVO1_4);
      setServoAngle270(leg4Servos[1], POS_C_SERVO2_4);

      delay(250);

      setServoAngle270(11, SERVO3_HOME_1_4);   
      setServoAngle270(2, SERVO3_HOME_1_4);

      delay(250);

      // ---- Legs 1 & 4 back down ----
      setServoAngle270(leg1Servos[0], POS_B_SERVO1);
      setServoAngle270(leg1Servos[1], POS_B_SERVO2);

      setServoAngle270(leg4Servos[0], POS_B_SERVO1_4);
      setServoAngle270(leg4Servos[1], POS_B_SERVO2_4);

      delay(250);

      return;
  }



  // If not turning → reset hip yaw + continue walking
  for (int i = 0; i < 4; i++) {
      setServoAngle270(hipYawChannel[i], hipYawNeutral[i]);
  }


  // Backward Walking Engine:
  if (walkingBackward) {

      unsigned long now = millis();

      if (gaitPaused) {
          if (now - pauseStart < PAUSE_DURATION) return;
          gaitPaused = false;
      }

      if (now - lastStepTime < stepInterval) return;
      lastStepTime = now;

      // Start backward gait with legs 2 & 3 (reverse order)
      if (!leg1State.active && !leg2State.active &&
          !leg3State.active && !leg4State.active) {

          leg2State.active = true;
          leg3State.active = true;
          phase23_back = 0;
          setTargetsPhase23_Back();
      }

      bool done1 = stepLegTowardTarget(leg1State);
      bool done4 = stepLegTowardTarget(leg4State);
      bool done2 = stepLegTowardTarget(leg2State);
      bool done3 = stepLegTowardTarget(leg3State);

    // Legs 2 & 3 phase
    if (leg2State.active && leg3State.active && done2 && done3) {
        phase23_back++;
        if (phase23_back > 7) {
            leg2State.active = false;
            leg3State.active = false;

            gaitPaused = true;
            pauseStart = millis();

            // ===== NEW SHUTDOWN BLOCK (mirrors forward walking) =====
            if (walkingShutdown) {
                walkingBackward = false;
                walkingShutdown = false;

                // Return all legs to B stance
                leg1State.targetHip  = POS_B_SERVO1;
                leg1State.targetKnee = POS_B_SERVO2;

                leg2State.targetHip  = POS_B_SERVO1_2;
                leg2State.targetKnee = POS_B_SERVO2_2;

                leg3State.targetHip  = POS_B_SERVO1_3;
                leg3State.targetKnee = POS_B_SERVO2_3;

                leg4State.targetHip  = POS_B_SERVO1_4;
                leg4State.targetKnee = POS_B_SERVO2_4;

                return;
            }
            // ========================================================

            // Continue backward gait normally
            leg1State.active = true;
            leg4State.active = true;
            phase14_back = 0;
            setTargetsPhase14_Back();
        } else {
            setTargetsPhase23_Back();
        }
    }


      // Legs 1 & 4 phase
      if (leg1State.active && leg4State.active && done1 && done4) {
          phase14_back++;
          if (phase14_back > 7) {
              leg1State.active = false;
              leg4State.active = false;

              gaitPaused = true;
              pauseStart = millis();

              if (walkingShutdown) {
                  walkingBackward = false;
                  walkingShutdown = false;

                  // Return to B stance
                  leg1State.targetHip  = POS_B_SERVO1;
                  leg1State.targetKnee = POS_B_SERVO2;

                  leg2State.targetHip  = POS_B_SERVO1_2;
                  leg2State.targetKnee = POS_B_SERVO2_2;

                  leg3State.targetHip  = POS_B_SERVO1_3;
                  leg3State.targetKnee = POS_B_SERVO2_3;

                  leg4State.targetHip  = POS_B_SERVO1_4;
                  leg4State.targetKnee = POS_B_SERVO2_4;
              } else {
                  leg2State.active = true;
                  leg3State.active = true;
                  phase23_back = 0;
                  setTargetsPhase23_Back();
              }
          } else {
              setTargetsPhase14_Back();
          }
      }

      return;
  }

  // Forward (Primary) Walking Gait Engine:

  if (!walkingForward) return;

  unsigned long now = millis();

  // Checks if the gait is paused, preventing movement. Used to pause between phases/steps.
  if (gaitPaused) {
      if (now - pauseStart < PAUSE_DURATION) {
          return;
      }
      gaitPaused = false;
  }

  // Keeps track of the step timing.
  if (now - lastStepTime < stepInterval) return;
  lastStepTime = now;

  // If no legs are active -> begin by activating leg 1 and leg 4.
  if (!leg1State.active && !leg4State.active &&
      !leg2State.active && !leg3State.active) {

      leg1State.active = true;
      leg4State.active = true;
      phase14 = 0;
      if (roughTerrainMode)
      {
        setTargetsPhase14_RoughTerrain();
      } 
      else
      {
        setTargetsPhase14();
      }

  }

  // Sets the target step for each leg.
  bool done1 = stepLegTowardTarget(leg1State);
  bool done4 = stepLegTowardTarget(leg4State);
  bool done2 = stepLegTowardTarget(leg2State);
  bool done3 = stepLegTowardTarget(leg3State);

  // If currently leg 1 and leg 4 are active -> iterate to the next movement phase.
  if (leg1State.active && leg4State.active && done1 && done4) {
      phase14++;
      // If leg 1 and leg 4 completed reached their movement goals -> activate leg 2 and leg 3.
      if (phase14 > 7) {
          leg1State.active = false;
          leg4State.active = false;

          gaitPaused = true;
          pauseStart = millis();

          leg2State.active = true;
          leg3State.active = true;
          phase23 = 0;
          if (roughTerrainMode)
          {
            setTargetsPhase23_RoughTerrain();
          }
          else
          {
            setTargetsPhase23();
          }

      } else {
         if (roughTerrainMode)
          {
            setTargetsPhase14_RoughTerrain();
          } 
          else
          {
            setTargetsPhase14();
          }
      }
  }

  // If currently leg 2 and leg 3 are active -> iterate to the next movement phase.
  if (leg2State.active && leg3State.active && done2 && done3) {
      phase23++;
      // If leg 2 and leg 3 completed reached their movement goals -> activate leg 1 and leg 4.
      if (phase23 > 7) {
          leg2State.active = false;
          leg3State.active = false;

          gaitPaused = true;
          pauseStart = millis();

          // If the DPad is released, continuing moving through the walking gait until the robot is at its
          // rest position (knee and hip servos at B). This prevents imbalance.
          if (walkingShutdown) {
              walkingForward = false;
              walkingShutdown = false;

              leg1State.targetHip  = POS_B_SERVO1;
              leg1State.targetKnee = POS_B_SERVO2;

              leg2State.targetHip  = POS_B_SERVO1_2;
              leg2State.targetKnee = POS_B_SERVO2_2;

              leg3State.targetHip  = POS_B_SERVO1_3;
              leg3State.targetKnee = POS_B_SERVO2_3;

              leg4State.targetHip  = POS_B_SERVO1_4;
              leg4State.targetKnee = POS_B_SERVO2_4;
          } else {
              leg1State.active = true;
              leg4State.active = true;
              phase14 = 0;
              if (roughTerrainMode)
              {
                setTargetsPhase14_RoughTerrain();
              } 
              else
              {
                setTargetsPhase14();
              }
          }
      } else {
        if (roughTerrainMode)
        {
          setTargetsPhase23_RoughTerrain();
        }
        else
        {
          setTargetsPhase23();
        }
      }
  }
}


// Remote Controller Based Kill Switch for Knightsaurus.
// CRITICAL NOTE: The Battery will not switch off with this kill switch.
// The Battery must be shut off using the LiPo Switch mounted on the chassis box
// when it is safe to do so.
void killSwitch()
{
  // if the 'screenshot' button is pressed -> shutdown robot
  if (!killSwitchFlag) return; // Leaves the function if the kill switch is not activated

  // One Method (Kind of Archaic but it works):
  // When Screenshot is pressed, turn off the Uno ->
  // -> Turn off the ESP32 -> Physically press the off switch

  // Method Two: Have arduino shut off the battery using relays, transistors, diodes,
  // a fuse, and a relay
  
  // Add in a switch to turn off the LiPo

  // Exits the Arduino Program.
  //exit(0);
}

// Gets the battery percentage based on the LiPo voltage at an analog pin
int getBatteryPercent(float v) {
  if (v >= table[0].voltage) return 100; // If battery is full
  if (v <= table[tableSize - 1].voltage) return 0; // If battery is empty

  // Calculates the battery percentage using the LiPo table (see the struct at the top of the code)
  for (int i = 0; i < tableSize - 1; i++) {
    // Enters when the voltage is between two values in the table.
    if (v <= table[i].voltage && v > table[i + 1].voltage) {
      // Interpolates to estimate the exact battery percentage

      float v1 = table[i].voltage;
      float v2 = table[i + 1].voltage;
      int p1 = table[i].percent;
      int p2 = table[i + 1].percent;

      return p1 + (v - v1) * (p2 - p1) / (v2 - v1); // Returns the interpolated value.
    }
  }
  return 0;
}

// Draws the battery icon on the OLED Display
void drawBatteryIcon(int x, int y, int percent) {
  // Battery outline
  display.drawRect(x, y, 30, 14, SSD1306_WHITE);   // main body
  display.fillRect(x + 30, y + 4, 3, 6, SSD1306_WHITE); // positive terminal

  // Clamp percent
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;

  // Fill width (max 26 pixels inside)
  int fillWidth = map(percent, 0, 100, 0, 26);

  // Battery fill
  display.fillRect(x + 2, y + 2, fillWidth, 10, SSD1306_WHITE);
}

// Displays the battery informaiton on the OLED Display.
void showBattery(float voltage, int percent) {
  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE);   // <-- REQUIRED

  display.setCursor(0, 0);
  display.setTextSize(1);
  display.print("Battery:");

  display.setCursor(0, 12);
  display.setTextSize(2);
  display.print(batteryVFiltered, 2);
  display.print("V");

  display.setCursor(0, 36);
  display.setTextSize(2);
  display.print(percent);
  display.print("%");

  drawBatteryIcon(90, 0, percent);

  display.display();
}

// Draws the top bar on the OLED Display.
void drawTopBar(int percent, const char* mode) 
{
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    display.setCursor(0, 0);
    display.print("MODE: ");
    display.print(mode);

    drawBatteryIcon(70, 0, percent);   // moved left to make room

    display.setCursor(110, 0);         // FIXED
    display.print(percent);
    display.print("%");
}

// Draws the Main HUD Information on the OLED Display.
void drawMainHUD(float voltage, const char* gait, const char* turn, const char* emote) {
    // Blue zone (16–63 px)
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    display.setCursor(0, 20);
    display.print("Voltage: ");
    display.print(voltage, 2);
    display.print("V");

    display.setCursor(0, 32);
    display.print("Gait: ");
    display.print(gait);

    display.setCursor(0, 44);
    display.print("Turn: ");
    display.print(turn);

    display.setCursor(0, 56);
    display.print("Emote: ");
    display.print(emote);
}

// Displays the HUD on the OLED Display.
void showHUD(float voltage, int percent, const char* mode, const char* gait, const char* turn, const char* emote) {
    display.clearDisplay();

    drawTopBar(percent, mode);
    drawMainHUD(voltage, gait, turn, emote);

    display.display();
}


// Movement Command Functions for Knightsaurus:


void turnLeft() {
  // Activates left turning in the updateWalk function.
  turnValue = -1.0;
}

void turnRight() {
  // Activates right turning in the updateWalk function.
  turnValue = 1.0;
}

void turnStop() {
  // Deactivates turning in the updateWalk function.
  turnValue = 0.0;
}

// Function to move the tail left.
void tailLeft()
{
  // Moves the servo left until it is at its max threshold.
  if (tailServoPos > TAIL_FULL_LEFT)
  {
    tailServoPos -= TAIL_STEP_SIZE;
    tailServo.write(tailServoPos);
  }
}

// Function to the move the tail right.
void tailRight()
{
  // Moves the servo right until it is at its max threshold.
  if (tailServoPos < TAIL_FULL_RIGHT)
  {
    tailServoPos += TAIL_STEP_SIZE;
    tailServo.write(tailServoPos);
  }
}

// Function to center the tail.
void tailCenter()
{
  // Moves the servo right until it is at its max threshold.
  if (tailServoPos < TAIL_ZERO)
  {
    tailServoPos += TAIL_STEP_SIZE;
  }
  else if (tailServoPos > TAIL_ZERO)
  {
    tailServoPos -= TAIL_STEP_SIZE;
  }

  // Clamp to exact center
  if (abs(tailServoPos - TAIL_ZERO) < TAIL_STEP_SIZE)
  {
      tailServoPos = TAIL_ZERO;
  }

  tailServo.write(tailServoPos);
}

// Function to move the head left.
void headLeft()
{
  // Moves the servo left until it is at its max threshold.
  if (neckServoPos > NECK_FULL_LEFT)
  {
    neckServoPos -= NECK_STEP_SIZE;
    neckServo.write(neckServoPos);
  }
}

// Function to move the head right.
void headRight()
{
  // Moves the servo right until it is at its max threshold.
  if (neckServoPos < NECK_FULL_RIGHT)
  {
    neckServoPos += NECK_STEP_SIZE;
    neckServo.write(neckServoPos);
  }
}

// Function to center the head.
void neckCenter()
{
  if (neckServoPos < NECK_ZERO)
  {
    neckServoPos += NECK_STEP_SIZE;
  }
  else if (neckServoPos > NECK_ZERO)
  {
    neckServoPos -= NECK_STEP_SIZE;
  }

  // Clamp to exact center
  if (abs(neckServoPos - NECK_ZERO) < NECK_STEP_SIZE)
  {
      neckServoPos = NECK_ZERO;
  }

  neckServo.write(neckServoPos);
}

// Function to move the jaw of Knightsaurus.
void jawMove()
{
  jawServo.write(JAW_OPEN);    // 110 deg while holding button 
}

// This is Knightsaurus' "roar" function (plays SFX and moves jaw).
void soundEffect(int soundNum)
{
  // Call the Jaw Movement Function
  jawMove();
  myDFPlayer.play(soundNum);
  //isPlaying = true;
  delay(3000);
  jawServo.write(JAW_CLOSED);
}

// Emote 1 (FLEX): Crouch the hind legs, raise the fore legs, wag tail, and roar.
void startEmote1() {
    emote1Active = true;
    emote1Start = millis();

    // Disable walking
    walkingForward = false;
    walkingBackward = false;
    walkingShutdown = false;

    // Set leg targets for the pose
    leg1State.targetHip  = SERVO1_Zero;
    leg1State.targetKnee = SERVO2_Zero;

    leg2State.targetHip  = SERVO1_2_Zero;
    leg2State.targetKnee = SERVO2_2_Zero;

    leg3State.targetHip  = POS_C_SERVO1_3; // crouched
    leg3State.targetKnee = POS_C_SERVO2_3;

    leg4State.targetHip  = POS_C_SERVO1_4; // crouched
    leg4State.targetKnee = POS_C_SERVO2_4;

    // Activate all legs so step engine moves them
    leg1State.active = true;
    leg2State.active = true;
    leg3State.active = true;
    leg4State.active = true;

    tailPhase = 0;
}

// Update (go through) emote 1 until it is finished.
void updateEmote1() 
{
    if (!emote1Active && !emote1Resetting) return;

    unsigned long now = millis();

  if (emote1Resetting)
  {
      bool d1 = stepLegTowardTarget(leg1State);
      bool d2 = stepLegTowardTarget(leg2State);
      bool d3 = stepLegTowardTarget(leg3State);
      bool d4 = stepLegTowardTarget(leg4State);

      if (d1 && d2 && d3 && d4) {

          // === FULL GAIT ENGINE RESET ===
          leg1State.active = false;
          leg2State.active = false;
          leg3State.active = false;
          leg4State.active = false;

          phase14 = 0;
          phase23 = 0;
          phase14_back = 0;
          phase23_back = 0;

          gaitPaused = false;
          walkingShutdown = false;

          lastStepTime = millis();

          for (int i = 0; i < 4; i++) {
              setServoAngle270(hipYawChannel[i], hipYawNeutral[i]);
          }

          emote1Resetting = false;
      }

      return;
  }


    // Move legs toward their targets
    stepLegTowardTarget(leg1State);
    stepLegTowardTarget(leg2State);
    stepLegTowardTarget(leg3State);
    stepLegTowardTarget(leg4State);
    if (emote1Active)
    {
      // Tail wag state machine
      switch (tailPhase) 
      {
          case 0:
              tailLeft();
              if (tailServoPos == TAIL_FULL_LEFT) tailPhase = 1;
              break;

          case 1:
              tailRight();
              if (tailServoPos == TAIL_FULL_RIGHT) tailPhase = 2;
              break;

          case 2:
              tailCenter();
              if (tailServoPos == TAIL_ZERO) tailPhase = 0;
              break;
      }
    }
  // End emote after duration
  if (now - emote1Start > EMOTE1_DURATION) 
  {
    // Call the sound effect and roar.
    soundEffect(1);


    // Begin reset phase
    emote1Active = false;
    emote1Resetting = true;

    // Set B stance targets
    leg1State.targetHip  = POS_B_SERVO1;
    leg1State.targetKnee = POS_B_SERVO2;

    leg2State.targetHip  = POS_B_SERVO1_2;
    leg2State.targetKnee = POS_B_SERVO2_2;

    leg3State.targetHip  = POS_B_SERVO1_3;
    leg3State.targetKnee = POS_B_SERVO2_3;

    leg4State.targetHip  = POS_B_SERVO1_4;
    leg4State.targetKnee = POS_B_SERVO2_4;

    // Activate legs so step engine moves them
    leg1State.active = true;
    leg2State.active = true;
    leg3State.active = true;
    leg4State.active = true;

  }



}


// Emote 2 (DANCE) - Quickly moves the legs (in a cycle) up and down: 
void startEmote2() {
    emote2Active = true;
    emote2Resetting = false;
    emote2Start = millis();

    emote2LegIndex = 0;
    emote2LegPhase = 0;
    emote2HeadTailPhase = 0;

    // Disable walking
    walkingForward = false;
    walkingBackward = false;
    walkingShutdown = false;

    // Only one leg moves at a time
    leg1State.active = false;
    leg2State.active = false;
    leg3State.active = false;
    leg4State.active = false;
}


// Function to update emote 2.
void updateEmote2() {
    if (!emote2Active && !emote2Resetting) return;

    unsigned long now = millis();

    // Reset Phase:
    if (emote2Resetting)
    {
        bool d1 = stepLegTowardTarget(leg1State);
        bool d2 = stepLegTowardTarget(leg2State);
        bool d3 = stepLegTowardTarget(leg3State);
        bool d4 = stepLegTowardTarget(leg4State);

        if (d1 && d2 && d3 && d4) {

            // === FULL GAIT ENGINE RESET ===
            leg1State.active = false;
            leg2State.active = false;
            leg3State.active = false;
            leg4State.active = false;

            phase14 = 0;
            phase23 = 0;
            phase14_back = 0;
            phase23_back = 0;

            gaitPaused = false;
            walkingShutdown = false;

            lastStepTime = millis();

            for (int i = 0; i < 4; i++) {
                setServoAngle270(hipYawChannel[i], hipYawNeutral[i]);
            }

            emote2Resetting = false;
        }

        return;
    }


    // Head and Tail Swing:
    if (now - lastHeadTailUpdate > HEAD_TAIL_INTERVAL) {
        lastHeadTailUpdate = now;

        if (emote2HeadTailPhase == 0) {
            headLeft();
            tailLeft();
            if (neckServoPos == NECK_FULL_LEFT)
                emote2HeadTailPhase = 1;
        } else {
            headRight();
            tailRight();
            if (neckServoPos == NECK_FULL_RIGHT)
                emote2HeadTailPhase = 0;
        }
    }


    // Leg "Stomping" (Dancing) Movement Gait:

    LegState* L;
    int hipA, kneeA, hipC, kneeC, hipB, kneeB;

    switch (emote2LegIndex) {
        case 0: L = &leg1State; hipA=POS_A_SERVO1; kneeA=POS_A_SERVO2; hipC=POS_C_SERVO1; kneeC=POS_C_SERVO2; hipB=POS_B_SERVO1; kneeB=POS_B_SERVO2; break;
        case 1: L = &leg2State; hipA=POS_A_SERVO1_2; kneeA=POS_A_SERVO2_2; hipC=POS_C_SERVO1_2; kneeC=POS_C_SERVO2_2; hipB=POS_B_SERVO1_2; kneeB=POS_B_SERVO2_2; break;
        case 2: L = &leg3State; hipA=POS_A_SERVO1_3; kneeA=POS_A_SERVO2_3; hipC=POS_C_SERVO1_3; kneeC=POS_C_SERVO2_3; hipB=POS_B_SERVO1_3; kneeB=POS_B_SERVO2_3; break;
        case 3: L = &leg4State; hipA=POS_A_SERVO1_4; kneeA=POS_A_SERVO2_4; hipC=POS_C_SERVO1_4; kneeC=POS_C_SERVO2_4; hipB=POS_B_SERVO1_4; kneeB=POS_B_SERVO2_4; break;
    }

    // Only this leg moves
    L->active = true;

    bool done = stepLegTowardTarget(*L);

    switch (emote2LegPhase) {

        case 0: // LIFT
            L->targetHip = hipA;
            L->targetKnee = kneeA;
            if (done) emote2LegPhase = 1;
            break;

        case 1: // SLAM
            L->targetHip = hipC;
            L->targetKnee = kneeC;
            if (done) emote2LegPhase = 2;
            break;

        case 2: // RETURN TO B
            L->targetHip = hipB;
            L->targetKnee = kneeB;
            if (done) emote2LegPhase = 3;
            break;

        case 3: // NEXT LEG
            L->active = false;
            emote2LegIndex++;
            if (emote2LegIndex > 3) emote2LegIndex = 0;
            emote2LegPhase = 0;
            break;
    }

    // Ends the emote and resets Knightsaurus to rest position:
    if (now - emote2Start > EMOTE2_DURATION) {
        emote2Active = false;
        emote2Resetting = true;

        // Set all legs to B stance
        leg1State.targetHip = POS_B_SERVO1;
        leg1State.targetKnee = POS_B_SERVO2;

        leg2State.targetHip = POS_B_SERVO1_2;
        leg2State.targetKnee = POS_B_SERVO2_2;

        leg3State.targetHip = POS_B_SERVO1_3;
        leg3State.targetKnee = POS_B_SERVO2_3;

        leg4State.targetHip = POS_B_SERVO1_4;
        leg4State.targetKnee = POS_B_SERVO2_4;

        leg1State.active = true;
        leg2State.active = true;
        leg3State.active = true;
        leg4State.active = true;

        tailCenter();
        neckCenter();
    }
}


// Emote 3 (Lunge): Roar's With Pose -> Front Legs Stomp -> Tail Whips Left and Right -> Resets Back to Rest
void startEmote3() {
    emote3Active = true;
    emote3Start = millis();
    emote3Phase = 0;
    whipDirection = 0;

    // Disable walking
    walkingForward = false;
    walkingBackward = false;
    walkingShutdown = false;

    // Activate all legs
    leg1State.active = true;
    leg2State.active = true;
    leg3State.active = true;
    leg4State.active = true;
}

// Update Emote 3
void updateEmote3() 
{
  // Only run if active OR resetting
  if (!emote3Active && !emote3Resetting) return;

  unsigned long now = millis();

  // Reset the emote phases:
  if (emote3Resetting)
  {
      bool d1 = stepLegTowardTarget(leg1State);
      bool d2 = stepLegTowardTarget(leg2State);
      bool d3 = stepLegTowardTarget(leg3State);
      bool d4 = stepLegTowardTarget(leg4State);

      if (d1 && d2 && d3 && d4) {

          // === FIX: FULL GAIT ENGINE RESET ===
          leg1State.active = false;
          leg2State.active = false;
          leg3State.active = false;
          leg4State.active = false;

          phase14 = 0;
          phase23 = 0;
          phase14_back = 0;
          phase23_back = 0;

          gaitPaused = false;
          walkingShutdown = false;

          lastStepTime = millis();

          for (int i = 0; i < 4; i++) {
              setServoAngle270(hipYawChannel[i], hipYawNeutral[i]);
          }

          emote3Resetting = false;
      }

      return;
  }


  // Activates the emote phases:

  // Move legs toward targets
  bool d1 = stepLegTowardTarget(leg1State);
  bool d2 = stepLegTowardTarget(leg2State);
  bool d3 = stepLegTowardTarget(leg3State);
  bool d4 = stepLegTowardTarget(leg4State);

  switch (emote3Phase) {

      // Roar Pose
      case 0:
          // Front legs lift
          leg1State.targetHip  = POS_A_SERVO1;
          leg1State.targetKnee = POS_A_SERVO2;

          leg2State.targetHip  = POS_A_SERVO1_2;
          leg2State.targetKnee = POS_A_SERVO2_2;

          // Back legs crouch
          leg3State.targetHip  = POS_C_SERVO1_3;
          leg3State.targetKnee = POS_C_SERVO2_3;

          leg4State.targetHip  = POS_C_SERVO1_4;
          leg4State.targetKnee = POS_C_SERVO2_4;

          headRight();
          tailCenter();

          if (d1 && d2 && d3 && d4)
              emote3Phase = 1;
          break;


      // Stomp
      case 1:
          // Front legs slam down
          leg1State.targetHip  = POS_C_SERVO1;
          leg1State.targetKnee = POS_C_SERVO2;

          leg2State.targetHip  = POS_C_SERVO1_2;
          leg2State.targetKnee = POS_C_SERVO2_2;

          // Back legs return to B
          leg3State.targetHip  = POS_B_SERVO1_3;
          leg3State.targetKnee = POS_B_SERVO2_3;

          leg4State.targetHip  = POS_B_SERVO1_4;
          leg4State.targetKnee = POS_B_SERVO2_4;

          headLeft();
          tailCenter();

          if (d1 && d2 && d3 && d4)
              emote3Phase = 2;
          break;


      // Tail Whip
      case 2:
          // Hold strong stance
          leg1State.targetHip  = POS_C_SERVO1;
          leg1State.targetKnee = POS_C_SERVO2;

          leg2State.targetHip  = POS_C_SERVO1_2;
          leg2State.targetKnee = POS_C_SERVO2_2;

          leg3State.targetHip  = POS_B_SERVO1_3;
          leg3State.targetKnee = POS_B_SERVO2_3;

          leg4State.targetHip  = POS_B_SERVO1_4;
          leg4State.targetKnee = POS_B_SERVO2_4;

          // Fast tail whip
          if (now - lastWhipUpdate > WHIP_INTERVAL) {
              lastWhipUpdate = now;

              if (whipDirection == 0) {
                  tailLeft();
                  headLeft();
                  if (tailServoPos <= TAIL_FULL_LEFT)
                      whipDirection = 1;
              } else {
                  tailRight();
                  headRight();
                  if (tailServoPos >= TAIL_FULL_RIGHT)
                      whipDirection = 0;
              }
          }

          // Move to reset phase near the end
          if (now - emote3Start > EMOTE3_DURATION - 800)
              emote3Phase = 3;

          break;


      // Ends the emote and resets Knightsaurus to rest position:
      case 3:
          // Set B stance targets
          leg1State.targetHip  = POS_B_SERVO1;
          leg1State.targetKnee = POS_B_SERVO2;

          leg2State.targetHip  = POS_B_SERVO1_2;
          leg2State.targetKnee = POS_B_SERVO2_2;

          leg3State.targetHip  = POS_B_SERVO1_3;
          leg3State.targetKnee = POS_B_SERVO2_3;

          leg4State.targetHip  = POS_B_SERVO1_4;
          leg4State.targetKnee = POS_B_SERVO2_4;

          tailCenter();
          neckCenter();

          // Switch to reset mode
          emote3Active = false;
          emote3Resetting = true;

          break;
  }

  // Safety: if duration exceeded, force reset
  if (now - emote3Start > EMOTE3_DURATION) {
      emote3Active = false;
      emote3Resetting = true;
  }
}


// Function for processing the remote controller commands (via the SPI Chars sent from ESP32)
void processGamepad(char cmd) {

  //== XBOX Screenshot trigger button = 0x0040 ==//
    if (cmd == 'J') {
    // code for when J button is pushed
    
    // Activates Kill Switch Function
    killSwitchFlag = true;


  }

  //== XBOX A button = 0x0001 ==//
  if (cmd == 'O') {
    // code for when A button is pushed
    
    // Call Function Sound Effect
    soundEffect(random(1, 16));
  }
  else {
    // code for when A button is released
  }

  //== XBOX X button = 0x0004 ==//
  if (cmd == 'X') {
    // code for when X button is pushed
    startEmote1();
  }
  else {
  // code for when X button is released
  }

  //== XBOX Y button = 0x0008 ==//
  if (cmd == 'Y') {
    // code for when Y button is pushed
    startEmote2();
  }
  else {
    // code for when Y button is released
  }

  //== XBOX B button = 0x0002 ==//
  if (cmd == 'Z') {
    // code for when B button is pushed
    startEmote3();
  }
  else {
    // code for when B button is released
  }

  //==  XBOX DPAD UP button = 0x01==//
  if (cmd == 'B') {
    Serial.println("Walking forward..."); // Used for Debugging.
    walkingForward = true;
    walkingShutdown = false;
  }
  else if (cmd == 'V')
  {
    // code for when dpad up button is released
    walkingShutdown = true;
  }

  //==  XBOX Dpad DOWN button = 0x02==//
  if (cmd == 'C') {
    // code for when dpad down button is pushed
    Serial.println("Walking backwards..."); // Used for Debugging.
    walkingBackward = true;
    walkingShutdown = false;
  }
  else if (cmd == 'Z')
  {
    // code for when dpad down button is released
    walkingShutdown = true;
  }

  //== XBOX Dpad LEFT button = 0x08 ==//
  if (cmd == 'D') {
      // D-pad LEFT pressed
      turnLeft();
  }
  else if (cmd == 'K') {
      // D-pad LEFT released
      turnStop();
  }

  //== XBOX Dpad RIGHT button = 0x04 ==//
  if (cmd == 'E') {
      // D-pad RIGHT pressed
      turnRight();
  }
  else if (cmd == 'L') {
      // D-pad RIGHT released
      turnStop();
  }

  //== XBOX RB trigger button = 0x0020 ==//
  if (cmd == 'F') {
    // code for when RB button is pushed

    // Call Head Right Function
    headRight();
  }
  else {
    // code for when RB button is released
  }

  //== XBOX RT trigger button = 0x0080 ==//
  if (cmd == 'G') {
    // code for when RT button is pushed

    // Call Tail Right Function
    tailRight();
  }
  else {
    // code for when RT button is released
  }

  //== XBOX LB trigger button = 0x0010 ==//
  if (cmd == 'H') {
    // code for when LB button is pushed

    // Call Head Left Function
    headLeft();
  }
  else {
    // code for when LB button is released
  }

  //== XBOX LT trigger button = 0x0040 ==//
  if (cmd == 'I') {
    // code for when LT button is pushed

    // Call Tail Left Function
    tailLeft();
  }
  else {
    // code for when LT button is released
  }

  //== R3 Button Pressed ==//
  if (cmd == '?')
  {
    // Toggles on and off rough terrain mode.
    roughTerrainMode = !roughTerrainMode;   // toggle mode
  }

}

// Setup Function for Knightsaurus. Initializes everything.
void setup() {

  Serial.begin(115200); // Serial Baudrate used for SPI and I2C Communications.

  // SPI Initialization:
  SPI.begin(); 
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);

  // Initializes the Head and Tail Servos (UNO):
  pwm.begin();
  pwm.setPWMFreq(50);

  // Initializes the OLED Display:
  Wire1.begin();  // A4/A5
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();


  // Set all legs to B (neutral stance)
  setServoAngle270(leg1Servos[0], POS_B_SERVO1);
  setServoAngle270(leg1Servos[1], POS_B_SERVO2);

  setServoAngle270(leg2Servos[0], POS_B_SERVO1_2);
  setServoAngle270(leg2Servos[1], POS_B_SERVO2_2);

  setServoAngle270(leg3Servos[0], POS_B_SERVO1_3);
  setServoAngle270(leg3Servos[1], POS_B_SERVO2_3);

  setServoAngle270(leg4Servos[0], POS_B_SERVO1_4);
  setServoAngle270(leg4Servos[1], POS_B_SERVO2_4);


// Used for Debugging:
/*
  // sets all to 0
  setServoAngle270(leg1Servos[0], SERVO1_Zero);
  setServoAngle270(leg1Servos[1], SERVO2_Zero);

  setServoAngle270(leg2Servos[0], SERVO1_2_Zero);
  setServoAngle270(leg2Servos[1], SERVO1_2_Zero);

  setServoAngle270(leg3Servos[0], SERVO1_3_Zero);
  setServoAngle270(leg3Servos[1], SERVO2_3_Zero);

  setServoAngle270(leg4Servos[0], SERVO1_4_Zero);
  setServoAngle270(leg4Servos[1], SERVO2_4_Zero);
*/

  // Initialize the hips to test.
  setServoAngle270(2, SERVO3_HOME_1_4);
  setServoAngle270(5, SERVO3_HOME_2_3);
  setServoAngle270(8, SERVO3_HOME_2_3);
  setServoAngle270(11, SERVO3_HOME_1_4);


  // Give servos time to physically reach B stance
  delay(400); 

  // Sync internal state of each leg to match physical B stance
  leg1State.hip  = POS_B_SERVO1;
  leg1State.knee = POS_B_SERVO2;

  leg2State.hip  = POS_B_SERVO1_2;
  leg2State.knee = POS_B_SERVO2_2;

  leg3State.hip  = POS_B_SERVO1_3;
  leg3State.knee = POS_B_SERVO2_3;

  leg4State.hip  = POS_B_SERVO1_4;
  leg4State.knee = POS_B_SERVO2_4;


  // Reset gait engine state
  gaitPhase = 0;
  lastStepTime = millis();
  walkingForward = false;   // MUST be false at boot


  // Initializes the LED Eyes:
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Initializes the Jaw Servo:
  jawServo.attach(JAW_SERVO_PIN);
  jawServo.write(JAW_CLOSED);

  // Initializes the Tail Servo:
  tailServo.attach(TAIL_SERVO_PIN);
  tailServo.write(TAIL_ZERO);

  // Initializes the Neck Servo:
  neckServo.attach(NECK_SERVO_PIN);
  neckServo.write(NECK_ZERO);

  // Second Serial Value used for the DF Player:
  Serial1.begin(9600); // Baudrate of 9600 for the DF Player Communications

  // Initializes the DF Player:
  if (!myDFPlayer.begin(Serial1)) {
    Serial.println("DFPlayer Mini not found.");
  } else {
    Serial.println("DFPlayer Mini ready.");
    myDFPlayer.volume(30);
  }

  // Startup Sound Effect
  soundEffect(2);
}


// The Main Loop Function for Knightsaurus. Runs Continuously.
void loop() {

  // Monitor the battery percentage:
  rawV = analogRead(lipoPin);
  pinV = (rawV / 4095.0) * 5.0;      // pin voltage
  batteryV = pinV * 18.75;           // your calibrated factor

  // Low-pass filter (0.9 = smooth, 0.1 = responsiveness)
  batteryVFiltered = 0.9 * batteryVFiltered + 0.1 * batteryV;

  int percent = getBatteryPercent(batteryVFiltered);
  

  // Serial prints for debugging and monitoring the battery percentage.
  Serial.print("Battery Voltage: ");
  Serial.print(batteryVFiltered, 2);
  Serial.print(" V   |   ");
  Serial.print("Battery: ");
  Serial.print(percent);
  Serial.println("%");


  // Reads the SPI information from the ESP32 and communicates to ensure still connected.
  digitalWrite(CS, LOW);
  byte incoming = SPI.transfer(0x00);   // read byte from ESP32
  digitalWrite(CS, HIGH);

  // Prints to make check connection with ESP32:
  Serial.print("Master received: "); 
  Serial.println((char)incoming);

  // Processes the characters sent from the ESP32 to read the controller inputs:
  processGamepad((char)incoming); // Every character sent is mapped to a certain button.

  // Run non-blocking gait engine
  updateWalk();

  // Emotes:
  updateEmote1();
  updateEmote2();
  updateEmote3();

  // Codes for updating the display:
  // MODE
  if (emote1Active || emote2Active || emote3Active) {
      currentModeString = "EMOTE";
  }
  else if (walkingForward) {
      currentModeString = "WALK";
  }
  else if (turnValue == 0.0) {
      currentModeString = "TURN";
  }
  else {
      currentModeString = "IDLE";
  }

  // GAIT
  currentGaitString = "WALK";   // You can change this if you add more gaits

  // TURN VALUE
  if (turnValue <= -0.1)
  {
    currentTurnValue = "LEFT"; 
  }
  else if (turnValue >= 0.1)
  {
    currentTurnValue = "RIGHT";
  }
  else
  {
    currentTurnValue = "NONE";
  }

  // EMOTE
  if (emote1Active) currentEmoteString = "FLEX";
  else if (emote2Active) currentEmoteString = "DANCE";
  else if (emote3Active) currentEmoteString = "LUNGE";
  else currentEmoteString = "NONE";

  static unsigned long lastDisplayUpdate = 0;
  // Update the OLED every 500 ms
  if (millis() - lastDisplayUpdate > 500) {
      showHUD(
          batteryVFiltered,
          percent,
          currentModeString,
          currentGaitString,
          currentTurnValue,
          currentEmoteString
      );
      lastDisplayUpdate = millis();
  }


}