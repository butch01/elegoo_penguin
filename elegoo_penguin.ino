#include "Arduino.h"
// #include "Oscillator.h"
#include <Servo.h>
#include <NeoSWSerial.h>
#include "MsTimer2.h"
#include <EEPROM.h>
#include <FastCRC.h>
#include "globalDefines.h"
#include "ServoKeyframeAnimatorGroup.h"
#include "ServoKeyframeAnimator.h"
#include <ArduinoLog.h>
#include <EnhancedServo.h>
#include "MY1690_16S.h"
#include "RobotMoves01.h"
#include "moveConstants.h"
#include "soundConstants.h"

unsigned long packageNumber=0;

// 0 normal run
// 1 test, deactivate ble processing, just running test samples
#define RUNMODE 0
// vars for test mode


#define THUMBSTICK_DEADZONE 5
#define THUMBSTICK_MAX_INCREMENT_SPEED 5

//ServoKeyframeAnimatorGroup keyframeServoGroupLegs;

#define NUMBER_OF_SERVOGROUPS 1
#define NUMBER_OF_SERVOGROUP_LEGS_SERVOS 4
//#define NUMBER_OF_SERVOGROUP_LEGS_SERVOS 1
#define SERVO_GROUP_LEGS 0
// #define SERVO_GROUP_ARMS 1

bool isCenterKeyAlreadyPressed = false;


// Array of ServoKeyframeAnimator for the 4 leg servos
ServoKeyframeAnimator  keyframeAnimatorLegs[NUMBER_OF_SERVOGROUP_LEGS_SERVOS];
EnhancedServo servosLegs[NUMBER_OF_SERVOGROUP_LEGS_SERVOS];
//ServoKeyframeAnimatorGroup keyframeServoGroupLegs(keyframeAnimatorLegs, servoGroupLegs, NUMBER_OF_SERVOGROUP_LEGS_SERVOS);
ServoKeyframeAnimatorGroup servoGroups[NUMBER_OF_SERVOGROUPS];
unsigned char servoGroupLastMove[NUMBER_OF_SERVOGROUPS];

bool isCurrentlyUpdatingTrim = false;
bool isKeepServosLastPositions = true; // says if Servos should return to center (false) or if servos should stay in their last position (true)


RobotMoves01 movesLegs4Servos;


signed char servoGroupMoveIteration[NUMBER_OF_SERVOGROUPS];
signed char servoGroupMoveIterationDirection[NUMBER_OF_SERVOGROUPS];
bool isCenter=false;

unsigned char lastMove=0;


/* Serial Bluetooth Communication Control Command Data Frame*/

//// Hand Tour APP Control Interface Left Domain Key
//#define BTN_UP    'f'
//#define BTN_DOWN  'b'
//#define BTN_LEFT  'l'
//#define BTN_RIGHT 'i'
//#define BTN_IDLE  's'
//
//// Right Domain Key of Hand-Tour APP Control Interface
//#define BTN_MUSIC    '1'
//#define BTN_DANCE    '2'
//#define BTN_OBSTACLE '3'
//#define BTN_VOL_ADD  '4'
//#define BTN_VOL_SUB  '5'
//#define BTN_FOLLOW   '6'
//
//#define BTN_RR_ADD   '7'
//#define BTN_RL_ADD   '8'
//#define BTN_YR_ADD   '9'
//#define BTN_YL_ADD   '0'
//
//#define BTN_RR_SUB   'a'
//#define BTN_RL_SUB   'c'
//#define BTN_YR_SUB   'd'
//#define BTN_YL_SUB   'e'



/*
         ---------------
        |     O   O     |
        |---------------|
YR 2<== |               | <== YL 3
         ---------------
            ||     ||
            ||     ||
RR 0==^   -----   ------  v== RL 1
         |-----   ------|
*/

#define YL_PIN 10
#define YR_PIN 9
#define RL_PIN 12
#define RR_PIN 6

#define SERVO_RR 0
#define SERVO_RL 1
#define SERVO_YR 2
#define SERVO_YL 3

/* Hardware interface mapping*/

// now using software serial for ble, so i can debug easily via usb
#define BLE_SERIAL_NAME bleSerial
#define BLE_SERIAL_RX 3
#define BLE_SERIAL_TX 2
#define BLE_SERIAL_BAUD 19200
#define BLE_DELAY_BETWEEN_AT_COMMANDS 200
NeoSWSerial BLE_SERIAL_NAME(BLE_SERIAL_RX, BLE_SERIAL_TX);


// movement
//unsigned long timePrevKey=0; // time when the move started
//bool isInMove=false; // if we are currently in a move

// Protocol definition
#define PROT_ARRAY_LENGTH 17
#define PROT_STICK_LX 0
#define PROT_STICK_LY 1
#define PROT_STICK_RX 2
#define PROT_STICK_RY 3
#define PROT_BTN_SQUARE 4
#define PROT_BTN_CROSS 5
#define PROT_BTN_TRIANGLE 6
#define PROT_BTN_CIRCLE 7
#define PROT_BTN_L1 8
#define PROT_BTN_L2 9
#define PROT_BTN_R1 10
#define PROT_BTN_R2 11
#define PROT_UP 12
#define PROT_DONW 13
#define PROT_LEFT 14
#define PROT_RIGHT 15
#define PROT_DIGITAL_BUTTONS 16
#define PROT_PACKET_NUMBER 16	// packetnumber, currently replaces DIGITAL_BUTTONS which are no longer sent
#define PROT_DIGITAL_START 0
#define PROT_DIGITAL_SELECT 1
#define PROT_DIGITAL_STICK_BTN_L 2
#define PROT_DIGITAL_STICK_BTN_R 3

uint8_t rawMessage[PROT_ARRAY_LENGTH];
uint8_t message[PROT_ARRAY_LENGTH];
uint8_t messageCRC=0;
uint8_t messageBytesRead = 0 ; // we will mark message only as valid, if we read at least a complete message

bool isMessageValid=false;
unsigned long messageReceivedTime = 0; // will be used for timeout

#define PROT_TIMEOUT 10000 // timeout in milliseconds for protocol
const unsigned long protocolTimeout = PROT_TIMEOUT;

// CRC
FastCRC8 CRC8;


#define AUDIO_SOFTWARE_RX A2 //Software implementation of serial interface (audio module driver interface)
#define AUDIO_SOFTWARE_TX A3
NeoSWSerial mp3Serial(AUDIO_SOFTWARE_RX, AUDIO_SOFTWARE_TX);
MY1690_16S mp3( &mp3Serial);



// seems not to be in use anymore
//#define RECV_PIN 3

// ultrasonic interface
#define ECHO_PIN 4
#define TRIG_PIN 5

//Infrared Controller Interface (distance check L / R)
#define ST188_R_PIN A1
#define ST188_L_PIN A0

#define VOLTAGE_MEASURE_PIN A4 //Voltage Detection Interface

#define INDICATOR_LED_PIN A5 //LED Indicator Interface

#define MY1690_PIN 8
#define HT6871_PIN 7

#define INTERVALTIME 10.0
#define CENTRE 90
#define AMPLITUDE 30
#define ULTRA_HIGH_RATE 0.3
#define HIGH_RATE 0.5
#define MID_RATE 0.7
#define LOW_RATE 1.0
#define ULTRA_LOW_RATE 1.5

//void Test_voltageMeasure(void);

//unsigned long moveTime;
//unsigned long ledBlinkTime;
unsigned long voltageMeasureTime;
unsigned long infraredMeasureTime;
unsigned char LED_value = 255;

char danceNum = 0;

//int distanceUltrasonic = 0;
//int st188Val_L;
//int st188Val_R;
long int ST188Threshold;
long int ST188RightDataMin;
long int ST188LeftDataMin;
int UltraThresholdMin = 7;
int UltraThresholdMax = 20;
//Oscillator servo[4];


#define ULRTASONIC_TIMEOUT 20000
#define ULTRASONIC_DISTANCE_MAX 200
#define DISTANCE_SENSOR_VALUE_ARRAY_SIZE 5
#define DISTANCE_IR_STOP 5
#define DISTANCE_IR_START 5
#define DISTANCE_ULTRASONIC_START 13
#define DISTANCE_ULTRASONIC_STOP 8


#define SERVO_RL_IR_ANGLE_LIMIT 0
#define SERVO_YL_IR_ANGLE_LIMIT 0
#define SERVO_RR_IR_ANGLE_LIMIT 0
#define SERVO_YR_IR_ANGLE_LIMIT 0


unsigned int distanceUltrasonic[DISTANCE_SENSOR_VALUE_ARRAY_SIZE];
unsigned int distanceIRLeft[DISTANCE_SENSOR_VALUE_ARRAY_SIZE];
unsigned int distanceIRRight[DISTANCE_SENSOR_VALUE_ARRAY_SIZE];





//enum MODE
//{
//    IDLE,
//    BLUETOOTH,
//    OBSTACLE,
//    FOLLOW,
//    MUSIC,
//    DANCE,
//    VOLUME
//} mode = IDLE; // Right Domain Key of Hand-Tour APP Control Interface

enum BTMODE
{
    FORWARD,
    BACKWARDS,
    TURNRIGHT,
    TURNLEFT,
    STOP,
} BTmode = STOP; // Hand Tour APP Control Interface Left Domain Key

unsigned char lastCommand=STOP;

int musicIndex = 2;
int musicNumber = 4;
int danceIndex = 2;
bool danceFlag = false;
unsigned long preMp3Millis;

unsigned long preMp3MillisStop_OBSTACLE;
unsigned long preMp3MillisStop_FOLLOW;
int t = 495;
double pause = 0;
char irValue = '\0';
bool serial_flag = false;

//
//bool delays(unsigned long ms)
//{
//    for (unsigned long i = 0; i < ms; i++)
//    {
//        if (serial_flag)
//        {
//            return true;
//        }
//        delay(1);
//    }
//    return false;
//}
/*
   Implementation of MP3 Driver
//*/


//
//bool oscillate(int A[4], int O[4], int T, double phase_diff[4])
//{
//    for (int i = 0; i < 4; i++)
//    {
//        servo[i].SetO(O[i]);
//        servo[i].SetA(A[i]);
//        servo[i].SetT(T);
//        servo[i].SetPh(phase_diff[i]);
//    }
//    double ref = millis();
//    for (double x = ref; x < T + ref; x = millis())
//    {
//        for (int i = 0; i < 4; i++)
//        {
//            servo[i].refresh();
//        }
//        if (irValue)
//            return true;
//    }
//    return false;
//}

unsigned long final_time;
unsigned long interval_time;
int oneTime;
int iteration;
float increment[4];
int oldPosition[] = {CENTRE, CENTRE, CENTRE, CENTRE};



/*
    Setting the 90-degree position of the steering gear to make the penguin stand on its feet
*/
void home(int delayms=0)
{
	for (unsigned char i=0; i< NUMBER_OF_SERVOGROUP_LEGS_SERVOS; i++)
	{
		Serial.println(i);
		Log.notice(F("home: setting servoGroups[SERVO_GROUP_LEGS].getServoKeyframeAnimator(%d)->setServoAbsolutePosition(90)" CR), i);
//		servosLegs[i].enhancedWrite(90,0,180);
		servoGroups[SERVO_GROUP_LEGS].getServoKeyframeAnimator(i)->setServoAbsolutePosition(90);

	}
	delay(delayms);

}
//
//bool moveNServos(int time, int newPosition[])
//{
//    for (int i = 0; i < 4; i++)
//    {
//        increment[i] = ((newPosition[i]) - oldPosition[i]) / (time / INTERVALTIME);
//    }
//    final_time = millis() + time;
//    iteration = 1;
//    while (millis() < final_time)
//    {
//        interval_time = millis() + INTERVALTIME;
//        oneTime = 0;
//        while (millis() < interval_time)
//        {
//            if (oneTime < 1)
//            {
//                for (int i = 0; i < 4; i++)
//                {
//                    servo[i].SetPosition(oldPosition[i] + (iteration * increment[i]));
//                }
//                iteration++;
//                oneTime++;
//            }
//            if (serial_flag)
//                return true;
//        }
//    }
//
//    for (int i = 0; i < 4; i++)
//    {
//        oldPosition[i] = newPosition[i];
//    }
//    return false;
//}
//
///*
// Walking control realization:
//*/
//bool walk(int steps, int T, int dir)
//{
//
//    int move1[] = {90, 90 + 35, 90 + 15, 90 + 15};
//    int move2[] = {90 + 25, 90 + 30, 90 + 15, 90 + 15};
//    int move3[] = {90 + 20, 90 + 20, 90 - 15, 90 - 15};
//    int move4[] = {90 - 35, 90, 90 - 15, 90 - 15};
//    int move5[] = {90 - 40, 90 - 30, 90 - 15, 90 - 15};
//    int move6[] = {90 - 20, 90 - 20, 90 + 15, 90 + 15};
//
//    int move21[] = {90, 90 + 35, 90 - 15, 90 - 15};
//    int move22[] = {90 + 25, 90 + 30, 90 - 15, 90 - 15};
//    int move23[] = {90 + 20, 90 + 20, 90 + 15, 90 + 15};
//    int move24[] = {90 - 35, 90, 90 + 15, 90 + 15};
//    int move25[] = {90 - 40, 90 - 30, 90 + 15, 90 + 15};
//    int move26[] = {90 - 20, 90 - 20, 90 - 15, 90 - 15};
//
//    if (dir == 1) //Walking forward
//    {
//        for (int i = 0; i < steps; i++)
//            if (
//                moveNServos(T * 0.2, move1) ||
//                delays(t / 10) ||
//                moveNServos(T * 0.2, move2) ||
//                delays(t / 10) ||
//                moveNServos(T * 0.2, move3) ||
//                delays(t / 5) ||
//                moveNServos(T * 0.2, move4) ||
//                delays(t / 2) ||
//                moveNServos(T * 0.2, move5) ||
//                delays(t / 5) ||
//                moveNServos(T * 0.2, move6) ||
//                delays(t / 5))
//                return true;
//    }
//    else //Walking backward
//    {
//        for (int i = 0; i < steps; i++)
//            if (
//                moveNServos(T * 0.2, move21) ||
//                delays(t / 10) ||
//                moveNServos(T * 0.2, move22) ||
//                delays(t / 10) ||
//                moveNServos(T * 0.2, move23) ||
//                delays(t / 5) ||
//                moveNServos(T * 0.2, move24) ||
//                delays(t / 2) ||
//                moveNServos(T * 0.2, move25) ||
//                delays(t / 5) ||
//                moveNServos(T * 0.2, move26))
//                return true;
//    }
//
//    return false;
//}
///*
//    Realization of Turn Control
//*/
//bool turn(int steps, int T, int dir)
//{
//    int move1[] = {90 - 55, 90 - 20, 90 + 20, 90 + 20};
//    int move2[] = {90 - 20, 90 - 20, 90 + 20, 90 - 20};
//    int move3[] = {90 + 20, 90 + 55, 90 + 20, 90 - 20};
//    int move4[] = {90 + 20, 90 + 20, 90 - 20, 90 + 20};
//    int move5[] = {90 - 55, 90 - 20, 90 - 20, 90 + 20};
//    int move6[] = {90 - 20, 90 - 20, 90 + 20, 90 - 20};
//    int move7[] = {90 + 20, 90 + 55, 90 + 20, 90 - 20};
//    int move8[] = {90 + 20, 90 + 20, 90 - 20, 90 + 20};
//
//    int move21[] = {90 + 20, 90 + 55, 90 + 20, 90 + 20};
//    int move22[] = {90 + 20, 90 + 20, 90 + 20, 90 - 20};
//    int move23[] = {90 - 55, 90 - 20, 90 + 20, 90 - 20};
//    int move24[] = {90 - 20, 90 - 20, 90 - 20, 90 - 20};
//    int move25[] = {90 + 20, 90 + 55, 90 - 20, 90 + 20};
//    int move26[] = {90 + 20, 90 + 20, 90 + 20, 90 - 20};
//    int move27[] = {90 - 55, 90 - 20, 90 + 20, 90 - 20};
//    int move28[] = {90 - 20, 90 - 20, 90 - 20, 90 - 20};
//
//    if (dir == 1)
//    {
//        for (int i = 0; i < steps; i++)
//            if (
//                moveNServos(T * 0.2, move1) ||
//                delays(t / 5) ||
//                moveNServos(T * 0.2, move2) ||
//                delays(t / 5) ||
//                moveNServos(T * 0.2, move3) ||
//                delays(t / 5) ||
//                moveNServos(T * 0.2, move4) ||
//                delays(t / 5) ||
//                moveNServos(T * 0.2, move5) ||
//                delays(t / 5) ||
//                moveNServos(T * 0.2, move6) ||
//                delays(t / 5) ||
//                moveNServos(T * 0.2, move7) ||
//                delays(t / 5) ||
//                moveNServos(T * 0.2, move8) ||
//                delays(t / 5))
//                return true;
//    }
//    else
//    {
//        for (int i = 0; i < steps; i++)
//            if (
//                moveNServos(T * 0.2, move21) ||
//                delays(t / 5) ||
//                moveNServos(T * 0.2, move22) ||
//                delays(t / 5) ||
//                moveNServos(T * 0.2, move23) ||
//                delays(t / 5) ||
//                moveNServos(T * 0.2, move24) ||
//                delays(t / 5) ||
//                moveNServos(T * 0.2, move25) ||
//                delays(t / 5) ||
//                moveNServos(T * 0.2, move26) ||
//                delays(t / 5) ||
//                moveNServos(T * 0.2, move27) ||
//                delays(t / 5) ||
//                moveNServos(T * 0.2, move28) ||
//                delays(t / 5))
//                return true;
//    }
//
//    return false;
//}
//
///* Turn right*/
//bool moonWalkRight(int steps, int T)
//{
//    int A[4] = {25, 25, 0, 0};
//    int O[4] = {-15, 15, 0, 0};
//    double phase_diff[4] = {DEG2RAD(0), DEG2RAD(180 + 120), DEG2RAD(90), DEG2RAD(90)};
//
//    for (int i = 0; i < steps; i++)
//        if (oscillate(A, O, T, phase_diff))
//            return true;
//    return false;
//}
//
///* Turn left*/
//bool moonWalkLeft(int steps, int T)
//{
//    int A[4] = {25, 25, 0, 0};
//    int O[4] = {-15, 15, 0, 0};
//    double phase_diff[4] = {DEG2RAD(0), DEG2RAD(180 - 120), DEG2RAD(90), DEG2RAD(90)};
//
//    for (int i = 0; i < steps; i++)
//        if (oscillate(A, O, T, phase_diff))
//            return true;
//    return false;
//}
//
//bool crusaito(int steps, int T)
//{
//    int A[4] = {25, 25, 30, 30};
//    int O[4] = {-15, 15, 0, 0};
//    double phase_diff[4] = {DEG2RAD(0), DEG2RAD(180 + 120), DEG2RAD(90), DEG2RAD(90)};
//    for (int i = 0; i < steps; i++)
//        if (oscillate(A, O, T, phase_diff))
//            return true;
//    if (home())
//        return true;
//    return false;
//}
//bool swing(int steps, int T)
//{
//    int A[4] = {25, 25, 0, 0};
//    int O[4] = {-15, 15, 0, 0};
//    double phase_diff[4] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(90), DEG2RAD(90)};
//
//    for (int i = 0; i < steps; i++)
//        if (oscillate(A, O, T, phase_diff))
//            return true;
//    return false;
//}
//
//bool upDown(int steps, int T)
//{
//    int A[4] = {25, 25, 0, 0};
//    int O[4] = {-15, 15, 0, 0};
//    double phase_diff[4] = {DEG2RAD(180), DEG2RAD(0), DEG2RAD(270), DEG2RAD(270)};
//    for (int i = 0; i < steps; i++)
//        if (oscillate(A, O, T, phase_diff))
//            return true;
//    if (home())
//        return true;
//    return false;
//}
//
//bool flapping(int steps, int T)
//{
//    int A[4] = {15, 15, 8, 8};
//    int O[4] = {-A[0], A[1], 0, 0};
//    double phase_diff[4] = {DEG2RAD(0), DEG2RAD(180), DEG2RAD(-90), DEG2RAD(90)};
//
//    for (int i = 0; i < steps; i++)
//        if (oscillate(A, O, T, phase_diff))
//            return true;
//    return false;
//}
//
//bool run(int steps, int T)
//{
//    int A[4] = {10, 10, 10, 10};
//    int O[4] = {0, 0, 0, 0};
//    double phase_diff[4] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(90), DEG2RAD(90)};
//
//    for (int i = 0; i < steps; i++)
//        if (oscillate(A, O, T, phase_diff))
//            return true;
//    return false;
//}
//
//bool backyard(int steps, int T)
//{
//    int A[4] = {15, 15, 30, 30};
//    int O[4] = {0, 0, 0, 0};
//    double phase_diff[4] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(-90), DEG2RAD(-90)};
//
//    for (int i = 0; i < steps; i++)
//        if (oscillate(A, O, T, phase_diff))
//            return true;
//    return false;
//}
//
//bool backyardSlow(int steps, int T)
//{
//    int A[4] = {15, 15, 30, 30};
//    int O[4] = {0, 0, 0, 0};
//    double phase_diff[4] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(-90), DEG2RAD(-90)};
//
//    for (int i = 0; i < steps; i++)
//        if (oscillate(A, O, T, phase_diff))
//            return true;
//    return false;
//}
//
//bool goingUp(int tempo)
//{
//    int move1[] = {50, 130, 90, 90};
//    if (moveNServos(tempo * HIGH_RATE, move1) ||
//        delays(tempo / 2) ||
//        home())
//        return true;
//    return false;
//}
//
//bool drunk(int tempo)
//{
//    int move1[] = {70, 70, 90, 90};
//    int move2[] = {110, 110, 90, 90};
//    int move3[] = {70, 70, 90, 90};
//    int move4[] = {110, 110, 90, 90};
//    if (moveNServos(tempo * MID_RATE, move1) ||
//        moveNServos(tempo * MID_RATE, move2) ||
//        moveNServos(tempo * MID_RATE, move3) ||
//        moveNServos(tempo * MID_RATE, move4) ||
//        home())
//        return true;
//    return false;
//}
//
//bool noGravity(int tempo)
//{
//    int move1[] = {120, 140, 90, 90};
//    int move2[] = {120, 30, 90, 90};
//    int move3[] = {120, 120, 90, 90};
//    int move4[] = {120, 30, 120, 120};
//    int move5[] = {120, 30, 60, 60};
//    if (moveNServos(tempo * MID_RATE, move1) ||
//        delays(tempo) ||
//        moveNServos(tempo * MID_RATE, move2) ||
//        moveNServos(tempo * MID_RATE, move3) ||
//        moveNServos(tempo * MID_RATE, move2) ||
//        delays(tempo) ||
//        moveNServos(tempo * LOW_RATE, move4) ||
//        delays(tempo) ||
//        moveNServos(tempo * LOW_RATE, move5) ||
//        delays(tempo) ||
//        moveNServos(tempo * LOW_RATE, move4) ||
//        delays(tempo) ||
//        home())
//        return true;
//    return false;
//}
//
//bool kickLeft(int tempo)
//{
//    int move1[] = {120, 140, 90, 90};
//    int move2[] = {120, 90, 90, 90};
//    int move3[] = {120, 120, 90, 90};
//    int move4[] = {120, 90, 120, 120};
//    int move5[] = {120, 120, 60, 60};
//    if (moveNServos(tempo * MID_RATE, move1) ||
//        delays(tempo) ||
//        moveNServos(tempo * MID_RATE, move2) ||
//        delays(tempo / 4) ||
//        moveNServos(tempo * MID_RATE, move3) ||
//        delays(tempo / 4) ||
//        moveNServos(tempo * LOW_RATE, move4) ||
//        delays(tempo / 4) ||
//        moveNServos(tempo * LOW_RATE, move5) ||
//        delays(tempo / 4) ||
//        home())
//        return true;
//    return false;
//}
//
//bool kickRight(int tempo)
//{
//    int move1[] = {40, 60, 90, 90};
//    int move2[] = {90, 60, 90, 90};
//    int move3[] = {60, 60, 90, 90};
//    int move4[] = {90, 60, 120, 120};
//    int move5[] = {60, 60, 60, 60};
//    if (moveNServos(tempo * MID_RATE, move1) ||
//        delays(tempo) ||
//        moveNServos(tempo * MID_RATE, move2) ||
//        delays(tempo / 4) ||
//        moveNServos(tempo * MID_RATE, move3) ||
//        delays(tempo / 4) ||
//        moveNServos(tempo * LOW_RATE, move4) ||
//        delays(tempo / 4) ||
//        moveNServos(tempo * LOW_RATE, move5) ||
//        delays(tempo / 4) ||
//        home())
//        return true;
//    return false;
//}
//
//bool legRaise(int tempo, int dir)
//{
//    if (dir)
//    {
//        int move1[] = {70, 70, 60, 60};
//        if (moveNServos(tempo * MID_RATE, move1) || delays(tempo))
//            return true;
//    }
//    else
//    {
//        int move1[] = {110, 110, 120, 120};
//        if (moveNServos(tempo * MID_RATE, move1) || delays(tempo))
//            return true;
//    }
//    if (home())
//        return true;
//    return false;
//}
//
//bool legRaise1(int tempo, int dir)
//{
//    if (dir)
//    {
//        int move1[] = {50, 60, 90, 90};
//        int move2[] = {60, 60, 120, 90};
//        int move3[] = {60, 60, 60, 90};
//        if (moveNServos(tempo * MID_RATE, move1) ||
//            delays(tempo) ||
//            moveNServos(tempo * LOW_RATE, move2) ||
//            delays(tempo / 4) ||
//            moveNServos(tempo * LOW_RATE, move3) ||
//            delays(tempo / 4) ||
//            moveNServos(tempo * LOW_RATE, move2) ||
//            delays(tempo / 4) ||
//            moveNServos(tempo * LOW_RATE, move3) ||
//            delays(tempo / 4))
//            return true;
//    }
//    else
//    {
//        int move1[] = {120, 130, 90, 90};
//        int move2[] = {120, 120, 90, 60};
//        int move3[] = {120, 120, 90, 120};
//        if (moveNServos(tempo, move1) ||
//            delays(tempo) ||
//            moveNServos(tempo * MID_RATE, move2) ||
//            delays(tempo / 4) ||
//            moveNServos(tempo * MID_RATE, move3) ||
//            delays(tempo / 4) ||
//            moveNServos(tempo * MID_RATE, move2) ||
//            delays(tempo / 4) ||
//            moveNServos(tempo * MID_RATE, move3) ||
//            delays(tempo / 4))
//            return true;
//    }
//    if (home())
//        return true;
//    return false;
//}
//
//bool legRaise2(int steps, int tempo, int dir)
//{
//    if (dir)
//    {
//        int move1[] = {20, 60, 90, 90};
//        int move2[] = {20, 90, 120, 90};
//        for (int i = 0; i < steps; i++)
//        {
//            if (moveNServos(tempo * 0.7, move1) ||
//                delays(tempo / 4) ||
//                moveNServos(tempo * 0.7, move2) ||
//                delays(tempo / 4))
//                return true;
//        }
//    }
//    else
//    {
//        int move1[] = {120, 160, 90, 90};
//        int move2[] = {90, 160, 90, 60};
//        for (int i = 0; i < steps; i++)
//        {
//            if (moveNServos(tempo * 0.7, move1) ||
//                delays(tempo / 4) ||
//                moveNServos(tempo * 0.7, move2) ||
//                delays(tempo / 4))
//                return true;
//        }
//    }
//    if (home())
//        return true;
//    return false;
//}
//
//bool legRaise3(int steps, int tempo, int dir)
//{
//    if (dir)
//    {
//        int move1[] = {20, 60, 90, 90};
//        int move2[] = {20, 90, 90, 90};
//        for (int i = 0; i < steps; i++)
//        {
//            if (moveNServos(tempo * 0.5, move1) ||
//                delays(tempo / 4) ||
//                moveNServos(tempo * 0.5, move2) ||
//                delays(tempo / 4))
//                return true;
//        }
//    }
//    else
//    {
//        int move1[] = {120, 160, 90, 90};
//        int move2[] = {90, 160, 90, 90};
//        for (int i = 0; i < steps; i++)
//        {
//            if (moveNServos(tempo * 0.5, move1) ||
//                delays(tempo / 4) ||
//                moveNServos(tempo * 0.5, move2) ||
//                delays(tempo / 4))
//                return true;
//        }
//    }
//    if (home())
//        return true;
//    return false;
//}
//
//bool legRaise4(int tempo, int dir)
//{
//    if (dir)
//    {
//        int move1[] = {20, 60, 90, 90};
//        int move2[] = {20, 90, 90, 90};
//
//        if (moveNServos(tempo * MID_RATE, move1) ||
//            delays(tempo / 4) ||
//            moveNServos(tempo * MID_RATE, move2) ||
//            delays(tempo / 4))
//            return true;
//    }
//    else
//    {
//        int move1[] = {120, 160, 90, 90};
//        int move2[] = {90, 160, 90, 90};
//        if (moveNServos(tempo * MID_RATE, move1) ||
//            delays(tempo / 4) ||
//            moveNServos(tempo * MID_RATE, move2) ||
//            delays(tempo / 4))
//            return true;
//    }
//    if (home())
//        return true;
//    return false;
//}
//
//bool sitdown()
//{
//    int move1[] = {150, 90, 90, 90};
//    int move2[] = {150, 30, 90, 90};
//    if (moveNServos(t * ULTRA_LOW_RATE, move1) ||
//        delays(t / 2) ||
//        moveNServos(t * ULTRA_LOW_RATE, move2) ||
//        delays(t / 2) ||
//        home())
//        return true;
//    return false;
//}
//
//bool lateral_fuerte(boolean dir, int tempo)
//{
//    if (dir)
//    {
//        int move1[] = {CENTRE - 2 * AMPLITUDE, CENTRE - AMPLITUDE, CENTRE, CENTRE};
//        int move2[] = {CENTRE + AMPLITUDE, CENTRE - AMPLITUDE, CENTRE, CENTRE};
//        int move3[] = {CENTRE - 2 * AMPLITUDE, CENTRE - AMPLITUDE, CENTRE, CENTRE};
//        if (moveNServos(tempo * LOW_RATE, move1) || delays(tempo * 2) ||
//            moveNServos(tempo * ULTRA_HIGH_RATE, move2) || delays(tempo / 2) ||
//            moveNServos(tempo * ULTRA_HIGH_RATE, move3) || delays(tempo))
//            return true;
//    }
//    else
//    {
//        int move1[] = {CENTRE + AMPLITUDE, CENTRE + 2 * AMPLITUDE, CENTRE, CENTRE};
//        int move2[] = {CENTRE + AMPLITUDE, CENTRE - AMPLITUDE, CENTRE, CENTRE};
//        int move3[] = {CENTRE + AMPLITUDE, CENTRE + 2 * AMPLITUDE, CENTRE, CENTRE};
//        if (moveNServos(tempo * LOW_RATE, move1) || delays(tempo * 2) ||
//            moveNServos(tempo * ULTRA_HIGH_RATE, move2) || delays(tempo / 2) ||
//            moveNServos(tempo * ULTRA_HIGH_RATE, move3) || delays(tempo))
//            return true;
//    }
//    if (home())
//        return true;
//    return false;
//}
//
//bool primera_parte()
//{
//    int move2[4] = {90, 90, 90, 90};
//    if (lateral_fuerte(1, t) ||
//        moveNServos(t * 0.5, move2) ||
//        lateral_fuerte(0, t) ||
//        moveNServos(t * 0.5, move2) ||
//        lateral_fuerte(1, t) ||
//        moveNServos(t * 0.5, move2) ||
//        lateral_fuerte(0, t) ||
//        home())
//        return true;
//    return false;
//}
//
//bool segunda_parte()
//{
//    int move1[4] = {90, 90, 80, 100};
//    int move2[4] = {90, 90, 100, 80};
//    for (int x = 0; x < 3; x++)
//    {
//        for (int i = 0; i < 3; i++)
//        {
//            pause = millis();
//            if (moveNServos(t * 0.15, move1) ||
//                moveNServos(t * 0.15, move2))
//                return true;
//            while (millis() < (pause + t))
//            {
//                if (irValue)
//                    return true;
//            }
//        }
//    }
//    if (home())
//        return true;
//    return false;
//}
///*Dance action part*/
//
//void dance()
//{
//    primera_parte();
//    segunda_parte();
//    moonWalkLeft(4, t * 2);
//    moonWalkRight(4, t * 2);
//    moonWalkLeft(4, t * 2);
//    moonWalkRight(4, t * 2);
//    primera_parte();
//
//    for (int i = 0; i < 16; i++)
//    {
//        flapping(1, t / 4);
//        delays(3 * t / 4);
//    }
//
//    moonWalkRight(4, t * 2);
//    moonWalkLeft(4, t * 2);
//    moonWalkRight(4, t * 2);
//    moonWalkLeft(4, t * 2);
//
//    drunk(t * 4);
//    drunk(t * 4);
//    drunk(t * 4);
//    drunk(t * 4);
//    kickLeft(t);
//    kickRight(t);
//    drunk(t * 8);
//    drunk(t * 4);
//    drunk(t / 2);
//    delays(t * 4);
//
//    drunk(t / 2);
//
//    delays(t * 4);
//    walk(2, t * 3, 1);
//    home();
//    backyard(2, t * 2);
//    home();
//    goingUp(t * 2);
//    goingUp(t * 1);
//    noGravity(t);
//
//    delays(t);
//    primera_parte();
//    for (int i = 0; i < 32; i++)
//    {
//        flapping(1, t / 2);
//        delays(t / 2);
//    }
//
//    for (int i = 0; i < 4; i++)
//        servo[i].SetPosition(90);
//}
//
//void dance2()
//{
//    if (lateral_fuerte(1, t) ||
//        lateral_fuerte(0, t) ||
//        drunk(t / 2) ||
//        drunk(t) ||
//        kickLeft(t) ||
//        kickRight(t) ||
//        walk(2, t * 3, 1) ||
//        home() ||
//        backyard(2, t * 4) ||
//        noGravity(t) ||
//        lateral_fuerte(1, t) ||
//        lateral_fuerte(0, t) ||
//        segunda_parte() ||
//        upDown(5, 500))
//        return;
//}
//
//void dance3()
//{
//    if (sitdown() ||
//        legRaise(t, 1) ||
//        swing(5, t) ||
//        legRaise1(t, 1) ||
//        walk(2, t * 3, 1) ||
//        home() ||
//        noGravity(t) ||
//        kickRight(t) ||
//        goingUp(t) ||
//        kickLeft(t) ||
//        legRaise4(t, 1) ||
//        backyard(2, t * 4) ||
//        drunk(t) ||
//        lateral_fuerte(1, 500) ||
//        lateral_fuerte(0, 500) ||
//        sitdown())
//        return;
//}
//
//void dance4()
//{
//    if (flapping(1, t) ||
//        drunk(t) ||
//        kickLeft(t) ||
//        walk(2, t * 3, 1) ||
//        home() ||
//        lateral_fuerte(0, t) ||
//        sitdown() ||
//        legRaise(t, 1) ||
//        swing(5, t) ||
//        backyard(2, t * 4) ||
//        goingUp(t) ||
//        noGravity(t) ||
//        upDown(5, t) ||
//        legRaise1(t, 1) ||
//        legRaise2(4, t, 0) ||
//        kickRight(t) ||
//        goingUp(t) ||
//        legRaise3(4, t, 1) ||
//        kickLeft(t) ||
//        legRaise4(t, 1) ||
//        segunda_parte() ||
//        sitdown())
//        return;
//}
//void start()
//{
//    mp3.stopPlay();
//    delay(10);
//    mp3.stopPlay();
//    delay(10);
//    mp3.stopPlay();
//    delay(10);
//    mp3.playSong(1, mp3.volume);
//    startDance();
//    mp3.stopPlay();
//    delay(10);
//    mp3.stopPlay();
//    delay(10);
//    mp3.stopPlay();
//    delay(10);
//    servoAttach();
//}
//
//void startDance()
//{
//    servoAttach();
//    lateral_fuerte(1, t);
//    lateral_fuerte(0, t);
//    goingUp(t);
//    servoDetach();
//}
//
///* Realization of Obstacle Avoidance Mode*/
//void obstacleMode()
//{
//    bool turnFlag = true;
//    servoDetach();
//    //delay(500);
//    distance_value = getDistance();
//    /*  Serial.print("distance_obs: ");
//    Serial.println(distance_value);
//*/
//    if (distance_value >= 1 && distance_value <= 500)
//    {
//        st188Val_L = analogRead(ST188_L_PIN);
//        st188Val_R = analogRead(ST188_R_PIN);
//        if (st188Val_L >= 400 && st188Val_R >= 400)
//        {
//            servoAttach();
//            walk(3, t * 4, -1);
//            if (turnFlag)
//            {
//                turn(3, t * 4, 1);
//            }
//            else
//            {
//                turn(3, t * 4, -1);
//            }
//            servoDetach();
//        }
//        else if (st188Val_L >= 400 && st188Val_R < 400)
//        {
//            turnFlag = true;
//            servoAttach();
//            turn(3, t * 4, 1);
//            servoDetach();
//        }
//        else if (st188Val_L < 400 && st188Val_R >= 400)
//        {
//            turnFlag = false;
//            servoAttach();
//            turn(3, t * 4, -1);
//            servoDetach();
//        }
//        else if (st188Val_L < 400 && st188Val_R < 400)
//        {
//            if (distance_value < 5)
//            {
//                servoAttach();
//                walk(3, t * 3, -1);
//                if (turnFlag)
//                {
//                    turn(3, t * 4, 1);
//                }
//                else
//                {
//                    turn(3, t * 4, -1);
//                }
//                servoDetach();
//            }
//            else if (distance_value >= 5 && distance_value <= 20)
//            {
//                servoAttach();
//                if (turnFlag)
//                {
//                    turn(1, t * 4, 1);
//                }
//                else
//                {
//                    turn(1, t * 4, -1);
//                }
//                servoDetach();
//            }
//            else
//            {
//                servoAttach();
//                walk(1, t * 3, 1);
//                servoDetach();
//            }
//        }
//    }
//    else
//    {
//        servoAttach();//
//        home();
//        servoDetach();
//    }
//}
//
///* Follow-up mode implementation*/
//void followMode()
//{
//    servoDetach();
//    //delay(500);
//    distance_value = getDistance();
//    /*  Serial.print("distance_follow:");
//    Serial.println(distance_value);
//*/
//    if (distance_value >= 1 && distance_value <= 500)
//    {
//        st188Val_L = analogRead(ST188_L_PIN);
//        st188Val_R = analogRead(ST188_R_PIN);
//
//        /*
//        Serial.print(st188Val_L);
//        Serial.print('\t');
//        Serial.print(st188Val_R);
//        Serial.println();
//       */
//        if (st188Val_L >= 400 && st188Val_R >= 400)
//        {
//            servoAttach();
//            walk(1, t * 3, 1);
//            servoDetach();
//        }
//        else if (st188Val_L >= 400 && st188Val_R < 400)
//        {
//            servoAttach();
//            turn(1, t * 4, -1);
//            servoDetach();
//        }
//        else if (st188Val_L < 400 && st188Val_R >= 400)
//        {
//            servoAttach();
//            turn(1, t * 4, 1);
//            servoDetach();
//        }
//        else if (st188Val_L < 400 && st188Val_R < 400)
//        {
//            if (distance_value > 20)
//            {
//                servoAttach();
//                home();
//                servoDetach();
//            }
//            else
//            {
//                servoAttach();
//                walk(1, t * 3, 1);
//                servoDetach();
//            }
//        }
//    }
//    else
//    {
//        servoAttach();
//        home();
//        servoDetach();
//    }
//}
//
//void st188Adjust(int dis)
//{
//    if (millis() - infraredMeasureTime > 1000 && dis > 20 && dis < 200 && analogRead(ST188_L_PIN) < 300 && analogRead(ST188_R_PIN) < 300)
//    {
//        unsigned long st188RightData = 0;
//        unsigned long st188LeftData = 0;
//        for (int n = 0; n < 10; n++)
//        {
//            st188LeftData += analogRead(ST188_L_PIN);
//            st188RightData += analogRead(ST188_R_PIN);
//        }
//        ST188LeftDataMin = st188LeftData / 10;
//        ST188RightDataMin = st188RightData / 10;
//        ST188Threshold = ST188LeftDataMin - ST188RightDataMin;
//        infraredMeasureTime = millis();
//    }
//}
//

void servoAttach()
{
//    Log.notice(F("servoAttach start" CR));
//    DEBUG_SERIAL_NAME.println(trim_rr);
//    servosLegs[0].setTrim(trim_rr);
//
//    DEBUG_SERIAL_NAME.println(trim_rl);
//    servosLegs[1].setTrim(trim_rl);
//
//    DEBUG_SERIAL_NAME.println(trim_yr);
//    servosLegs[2].setTrim(trim_yr);
//
//    DEBUG_SERIAL_NAME.println(trim_yl);
//    servosLegs[3].setTrim(trim_yl);

//    for (unsigned char i=0; i< NUMBER_OF_SERVOGROUP_LEGS_SERVOS; i++)
//    {
//    	servosLegs[i].setTrim(servoGroupLegsTrim[i]);
//    	Log.verbose(F("servoAttach: servosLegs[%d].setTrim=%d" CR), i, servoGroupLegsTrim[i]);
//    }


    servosLegs[0].attach(RR_PIN);
    servosLegs[1].attach(RL_PIN);
    servosLegs[2].attach(YR_PIN);
    servosLegs[3].attach(YL_PIN);
//    Log.notice(F("servoAttach end" CR));
}

void servoDetach()
{
	servosLegs[0].detach();
	servosLegs[1].detach();
	servosLegs[2].detach();
	servosLegs[3].detach();
}

/* Realization of Ultrasound Ranging*/
int getDistance()
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    return (int)pulseIn(ECHO_PIN, HIGH) / 58;
}



unsigned int getDistanceFilterAverage(unsigned int* array)
{
	unsigned int avg=0;
	for (unsigned int i=0; i< DISTANCE_SENSOR_VALUE_ARRAY_SIZE; i++)
	{
		avg = avg +  array[i];
	}
	return avg /DISTANCE_SENSOR_VALUE_ARRAY_SIZE ;
}


/**
 * mapps the IR sensor to centimeters. range is 1 - 15
 */
unsigned char mapIRSensorToCentimeter(unsigned int sensorValue)
{
	unsigned char mappedValue=0;

	if (sensorValue > 993 )
	{
		mappedValue = 1;
	}
	else if (sensorValue > 980 )
	{
		mappedValue = 2;
	}
	else if (sensorValue > 520 )
	{
		mappedValue = 3;
	}
	else if (sensorValue > 283 )
	{
		mappedValue = 4;
	}
	else if (sensorValue > 199 )
	{
		mappedValue = 5;
	}
	else if (sensorValue > 140)
	{
		mappedValue = 6;
	}
	else if (sensorValue > 105)
	{
		mappedValue = 7;
	}
	else if (sensorValue > 90)
	{
		mappedValue = 8;
	}
	else if (sensorValue > 83)
	{
		mappedValue = 9;
	}
	else if (sensorValue > 65)
	{
		mappedValue = 10;
	}
	else if (sensorValue > 56)
	{
		mappedValue = 11;
	}
	else if (sensorValue > 45 )
	{
		mappedValue = 12;
	}
	else if (sensorValue > 38)
	{
		mappedValue = 13;
	}
	else if (sensorValue > 33 )
	{
		mappedValue = 14;
	}
	else
	{
		mappedValue = 15;
	}

	return mappedValue;
}



void updateDistanceFilterArrays()
{

	long duration;
	int distance;
	unsigned char start=0;

	if (DISTANCE_SENSOR_VALUE_ARRAY_SIZE > 1)
	{
		unsigned int avgLeft=0;
		unsigned int avgCenter=0;
		unsigned int avgRight=0;

		// calculate the average of all values to first element
		for (unsigned int i=0; i< DISTANCE_SENSOR_VALUE_ARRAY_SIZE; i++)
		{
			avgLeft=avgLeft+distanceIRLeft[i];
			avgCenter=avgCenter+distanceUltrasonic[i];
			avgRight=avgRight+distanceIRRight[i];
		}
		// save avg to first element
		distanceIRLeft[0]=avgLeft / DISTANCE_SENSOR_VALUE_ARRAY_SIZE;
		distanceUltrasonic[0]=avgCenter / DISTANCE_SENSOR_VALUE_ARRAY_SIZE;
		distanceIRRight[0]=avgRight / DISTANCE_SENSOR_VALUE_ARRAY_SIZE ;
		start=1;
	}
	else
	{
		start = 0;
	}

	// now update all but the first element
	for (unsigned i=start; i< DISTANCE_SENSOR_VALUE_ARRAY_SIZE; i++)
	{

		// Clears the trigPin
		digitalWrite(TRIG_PIN, LOW);
		delayMicroseconds(2);
		// Sets the trigPin on HIGH state for 10 micro seconds
		digitalWrite(TRIG_PIN, HIGH);
		delayMicroseconds(10);
		digitalWrite(TRIG_PIN, LOW);
		// Reads the echoPin, returns the sound wave travel time in microseconds
		duration = pulseIn(ECHO_PIN, HIGH, ULRTASONIC_TIMEOUT);

		// Calculating the distance in cm
		distanceUltrasonic[i]= duration*0.034/2;
		if (distanceUltrasonic[i] == 0 || distanceUltrasonic[i]  > ULTRASONIC_DISTANCE_MAX)
		{
			distanceUltrasonic[i] = ULTRASONIC_DISTANCE_MAX;
		}
		distanceIRLeft[i]=mapIRSensorToCentimeter(analogRead(ST188_L_PIN));
		distanceIRRight[i]=mapIRSensorToCentimeter(analogRead(ST188_R_PIN));
	}
}
//
///* Control command acquisition implementation*/
//void getCommand()
//{
//
//	if (Serial.available())
//    {
//        irValue = Serial.read();
//        if (irValue && irValue != '\0')
//        {
//            // Serial.print("new data: ");
//            // Serial.println(irValue);
//            serial_flag = true;
//        }
//        else
//        {
//            // Serial.print("error data: ");
//            // Serial.println(irValue);
//            irValue = '\0';
//        }
//    }
//
////	readPacketFromBLE();
//
//    Test_voltageMeasure();// Realization of Voltage Detection
//}


/**
 * returns the trim values which are saved in eeprom
 */
signed char getEEPROMTrim(unsigned char servo)
{
	unsigned char trimValue=0;
	switch (servo)
	{
	case SERVO_YL:
		trimValue = EEPROM.read((int) SERVO_YL);
		break;

	case SERVO_YR:
		trimValue = EEPROM.read((int) SERVO_YR);
		break;

	case SERVO_RL:
		trimValue = EEPROM.read((int) SERVO_RL);
		break;

	case SERVO_RR:
		trimValue = EEPROM.read((int) SERVO_RR);
		break;
	}

	// undo offset of 90 degrees or set to 0
	if (trimValue == 255)
	{
		trimValue=0;
	}
	else
	{
		trimValue = trimValue - 90;
	}

//	Log.trace(F("getEEPROMTrim(%d) returning %d" CR), servo, trimValue);
	return trimValue;

}



void servoInit()
{
	Log.notice(F("servoInit: reading trim from EEPROM" CR));
	for (unsigned int i=0; i< NUMBER_OF_SERVOGROUP_LEGS_SERVOS; i++)
	{
		servosLegs[i].setTrim(getEEPROMTrim(i));
		Log.trace(F("servoGroupLegsTrim[%d] =  %d;" CR), i, servosLegs[i].getTrim() );
	}

    Log.notice(F("servoInit: finished" CR));
}


/**
 * shifts all bytes one step left in array. [0] will become value of messageCRC
 */
void leftShiftMessageBytes ()
{
	for (int i=sizeof(rawMessage)-1; i>0 ;i--)
	{
		rawMessage[i] = rawMessage[i-1];
	}
	rawMessage[0] = messageCRC;
}

void swapMessage()
{
	for (unsigned char  i=0; i < sizeof(rawMessage); i++)
	{
		message[i] = rawMessage[(sizeof(rawMessage) -1 -i)];
	}

}


void readPacketFromBLE()
{

	if (BLE_SERIAL_NAME.available() > 0)
	{
		// shift old bytes
		leftShiftMessageBytes();

		// read new byte
		messageCRC = (uint8_t) BLE_SERIAL_NAME.read();
		messageBytesRead++;

		// check if we have read enough bytes (more than protocol length)
		if ( messageBytesRead > sizeof(rawMessage))
		{


			swapMessage();

			// calculate crc8
			if (messageCRC == CRC8.smbus(message, sizeof(message)))
			{
				// mark message as valid
				isMessageValid = true;
				packageNumber++;
				////Serial.print("isMessageValid=");
				////Serial.println(isMessageValid);

				// update timestamp
				messageReceivedTime = millis();

				// reset BytesReadCounter
				messageBytesRead = 0;

				////Serial.write(message, sizeof(message));
				////Serial.println("CRC OK");
			}
			else
			{
				////Serial.println("CRC FAILURE");
			}
		}

	}
}


void clearMessage()
{
	for (unsigned char i=0; i< sizeof(rawMessage); i++)
	{
		rawMessage[i]=0;
		message[i]=0;
	}
}


/*
 * setup default values
 */
void setupDefaultValues()
{
	// servo groups
	for (unsigned char i=0; i< NUMBER_OF_SERVOGROUPS; i++)
	{
		servoGroupMoveIteration[i] = -1;
		servoGroupMoveIterationDirection[i] = 1;  // may be 1 (forward) or -1 (reverse) or 0 (stop)
		servoGroupLastMove[i]=0;
	}

	for (unsigned char i=0; i< DISTANCE_SENSOR_VALUE_ARRAY_SIZE; i++)
	{
		distanceUltrasonic[i]=0;
		distanceIRLeft[i]=0;
		distanceIRRight[i]=0;
	}

}


/**
 * setup the servos
 */
void setupServos()
{
	Log.notice(F("setup servos..." CR));
	for (unsigned char i=0; i< NUMBER_OF_SERVOGROUP_LEGS_SERVOS; i++)
	{
		servosLegs[i].setMaxValue(90+50);
		servosLegs[i].setMinValue(90-50);
		Log.error(" servos [%d], min %d max %d" CR, i, servosLegs[i].getMinValue(), servosLegs[i].getMaxValue());

	}

	servoInit();
	servoAttach();
//	home(200);
}


// setup the hm10 controller
void setupBLEHM10()
{

	bleSerial.println(F("AT+CHAR0xFFE1"));
	delay(BLE_DELAY_BETWEEN_AT_COMMANDS);

	bleSerial.println(F("AT+POWE2"));
	delay(BLE_DELAY_BETWEEN_AT_COMMANDS);

	bleSerial.println(F("AT+NAMEROBOT"));
	delay(BLE_DELAY_BETWEEN_AT_COMMANDS);

	bleSerial.println(F("AT+ROLE0"));
	delay(BLE_DELAY_BETWEEN_AT_COMMANDS);

	bleSerial.println(F("AT+RESET"));
	delay(1000);

	Log.notice(F("AT+START\n"));
	bleSerial.println(F("AT+START"));
	delay(BLE_DELAY_BETWEEN_AT_COMMANDS);

	Log.notice(F("setupBLEHM10 done\n"));

	// clear message
	clearMessage();

}

void setup()
{

	// init mp3-player
	mp3Serial.begin(9600);
	mp3.init(HT6871_PIN);


	DEBUG_SERIAL_NAME.begin(DEBUG_SERIAL_BAUD);



    bleSerial.begin(BLE_SERIAL_BAUD);
    bleSerial.listen();

    Log.begin   (LOG_LEVEL_VERBOSE, &DEBUG_SERIAL_NAME);


    Log.notice(F("starting\n"));

    pinMode(ECHO_PIN, INPUT);
    pinMode(TRIG_PIN, OUTPUT);


    //Log.notice(F("Distance %d" CR), myGetDistance());

//     ServoKeyframeAnimator test;
//    delay (10000);
//    Serial.print(F("MEM: "));
//    Serial.println(freeMemory());


    setupBLEHM10();

    // setup the default values, initialize arrays, ...
    setupDefaultValues();

    // setup the servos
    setupServos();
//    Serial.print(F("MEM: "));
//    Serial.println(freeMemory());

 //   keyframeServoGroupLegs = new ServoKeyframeAnimatorGroup(4);
//    mp3Serial.begin(9600);

//    pinMode(INDICATOR_LED_PIN, OUTPUT);
//    pinMode(VOLTAGE_MEASURE_PIN, INPUT);
//
//    analogWrite(INDICATOR_LED_PIN, LED_value);
//    mp3.init();
//    MsTimer2::set(50, getCommand);
//    MsTimer2::start();


 //   home();
//	kickLeft(t);
//	kickRight(t);
//	delay(500);
//	walk(1,t,0);
//	home();
//    servoDetach();
//    delays(2000);
//    start();
    clearMessage();


//    for (unsigned int i=0; i< NUMBER_OF_SERVOGROUP_LEGS_SERVOS; i++)
//    {
//    	keyframeAnimatorLegs[i].setKeyframeMode(5+i);
//    	Log.error("keyframe[%d] is set to %d" CR, i, 5+i);
//
//    }


    for (unsigned int i=0; i< NUMBER_OF_SERVOGROUP_LEGS_SERVOS; i++)
	{
		;
		Log.error("keyframeAnimator[%d] has keyframe value of %d" CR, i, keyframeAnimatorLegs[i].getKeyframeMode() );
	}

    servoGroups[SERVO_GROUP_LEGS].init(keyframeAnimatorLegs, servosLegs, NUMBER_OF_SERVOGROUP_LEGS_SERVOS);
//
//
//	for (unsigned int i=0; i< NUMBER_OF_SERVOGROUP_LEGS_SERVOS; i++)
//	{
//		//keyframeAnimatorLegs[i].setKeyframeMode(5+i);
//		servoGroups[0].getServoKeyframeAnimator(i)->setKeyframeMode(10+i);
//		Log.error("keyframe[%d] is set to %d" CR, i, 10+i);
//
//	}
//    for (unsigned int i=0; i< NUMBER_OF_SERVOGROUP_LEGS_SERVOS; i++)
//	{
//		;
//		Log.error("keyframe[%d]  has value of %d" CR, i, keyframeAnimatorLegs[i].getKeyframeMode() );
//	}
//
//
//    for (unsigned int i=0; i< NUMBER_OF_SERVOGROUP_LEGS_SERVOS; i++)
//    {
//    	Log.error("debug keyframemode [%d] = %d" CR, i, servoGroups[SERVO_GROUP_LEGS].getKeyframeAnimatorKeyframeMode(i));
//    }
//

    for (unsigned int i=0; i< NUMBER_OF_SERVOGROUP_LEGS_SERVOS; i++)
    {
//    	servoGroups[SERVO_GROUP_LEGS].set
    	Log.error("debug keyframemode [%d] = %d" CR, i, servoGroups[SERVO_GROUP_LEGS].getKeyframeAnimatorKeyframeMode(i));
	}


    home(200);

    //(ServoKeyframeAnimator) servoGroups[SERVO_GROUP_LEGS].getServoKeyframeAnimator(i)

    //servoGroups[SERVO_GROUP_LEGS]= new ServoKeyframeAnimatorGroup(keyframeAnimatorLegs, servosLegs, NUMBER_OF_SERVOGROUP_LEGS_SERVOS);
    //servoGroups[SERVO_GROUP_LEGS].init(NUMBER_OF_SERVOGROUP_LEGS_SERVOS);
//    keyframeServoGroupLegs.init(NUMBER_OF_SERVOGROUP_LEGS_SERVOS);
    Log.trace("servoNum=%d duration=%d" CR, servoGroups[SERVO_GROUP_LEGS].getNumberOfServos(), servoGroups[SERVO_GROUP_LEGS].getMoveDuration());


    //Log.trace("adresses keyframeAnimatorLegs %d, keyframeAnimatorLegs[0]=%d keyframeAnimatorLegs[1]=%d, keyframeAnimatorLegs[2]=%d, keyframeAnimatorLegs[3]=%d" CR, &keyframeAnimatorLegs, &keyframeAnimatorLegs[0],&keyframeAnimatorLegs[1],&keyframeAnimatorLegs[2], &keyframeAnimatorLegs[3]);
    //Log.trace("adresses keyframemmode [0]=%d [1]=%d, [2]=%d, [3]=%d" CR, servoGroups[SERVO_GROUP_LEGS].getServoKeyframeAnimator(0).getKeyframeModeAddress(), servoGroups[SERVO_GROUP_LEGS].getServoKeyframeAnimator(1).getKeyframeModeAddress(), servoGroups[SERVO_GROUP_LEGS].getServoKeyframeAnimator(2).getKeyframeModeAddress(), servoGroups[SERVO_GROUP_LEGS].getServoKeyframeAnimator(3).getKeyframeModeAddress());

//    for (int i=0; i<4; i++)
//	{
//    	servoGroupLegsTrim[i]=0;
//    	saveTrimValuesInEEPROM();
//    }
//
//    delay (10000);

    wrapperPlayMp3(SOUND_TITLE_ENABLED, SOUND_DEFAULT_VOLUME, false);
    //wrapperPlayMp3(10, 10);
    Log.notice(F("setup done" CR));

}

/*
 Voltage detection implementation function:

Acquisition of battery voltage, preset abnormal voltage threshold, control LED flashing to remind users of charging
*/
void Test_voltageMeasure(void) //Realization of Voltage Detection
{

    static boolean voltageMeasure_flag = true;
    static boolean is_flag = true;
    if (millis() - voltageMeasureTime > 10000)
    {
        double volMeasure = analogRead(VOLTAGE_MEASURE_PIN) * 4.97 / 1023;
        //Log.warning(F("Battery voltage: %F"CR), volMeasure);
        //Serial.println(volMeasure)

        //if (volMeasure < 3.70 || volMeasure >= 4.97)//Detection of power supply voltage below or above the set value is regarded as an abnormal phenomenon
        if (volMeasure < 3.70 )//Detection of power supply voltage below or above the set value is regarded as an abnormal phenomenon
        {
            voltageMeasure_flag = false;
        }
        else
        {
            voltageMeasure_flag = true;
        }
        voltageMeasureTime = millis();
    }

    if (voltageMeasure_flag)
    {
        digitalWrite(INDICATOR_LED_PIN, HIGH);
    }
    else
    {
        if (is_flag)
        {
            is_flag = false;
            digitalWrite(INDICATOR_LED_PIN, HIGH);
        }
        else
        {
            is_flag = true;
            digitalWrite(INDICATOR_LED_PIN, LOW);
        }
    }
}
//    /*
//	Maximum Load Test
//*/
//void BurnBrain_Test(void)
//{
//	servoDetach();
//	delays(10);
//	mp3.stopPlay();
//	delays(10);
//	mp3.playSong(danceIndex, mp3.volume);
//	servoAttach();
//	dance2();
//	getDistance();
//	analogRead(ST188_L_PIN);
//	analogRead(ST188_R_PIN);
//}



bool isTimeout()
{
	long timeDiff = millis() - protocolTimeout - messageReceivedTime;
//	//Serial.print("timeoutCalc: ");
//	//Serial.println(timeDiff);

	if (timeDiff > 0)
	{
		return true;
	}
	else
	{
		return false;
	}

}


// controls the servos directly via 2 thumbsticks.
void controlServosDirectly()
{
	 // rotate
//	servosLegs[SERVO_YL].enhancedWrite(map(message[PROT_STICK_LX],0,255,0,180));
//
//	servosLegs[SERVO_YR].enhancedWrite(map(message[PROT_STICK_RX],0,255,0,180));
//	// yaw
//	servosLegs[SERVO_RL].enhancedWrite(map(message[PROT_STICK_LY],0,255,0,180));
//	servosLegs[SERVO_RR].enhancedWrite(map(message[PROT_STICK_RY],0,255,0,180));
	servoGroups[SERVO_GROUP_LEGS].getServoKeyframeAnimator(SERVO_YL)->setServoAbsolutePosition(map(message[PROT_STICK_LX],0,255,0,180));
	servoGroups[SERVO_GROUP_LEGS].getServoKeyframeAnimator(SERVO_YR)->setServoAbsolutePosition(map(message[PROT_STICK_RX],0,255,0,180));
	servoGroups[SERVO_GROUP_LEGS].getServoKeyframeAnimator(SERVO_RL)->setServoAbsolutePosition(map(message[PROT_STICK_LY],0,255,0,180));
	servoGroups[SERVO_GROUP_LEGS].getServoKeyframeAnimator(SERVO_RR)->setServoAbsolutePosition(map(message[PROT_STICK_RY],0,255,0,180));
	servoGroups[SERVO_GROUP_LEGS].driveServosToCalculatedPosition();

}



/**
 * when thumbstick is in increment mode, then this function takes the thumbstick position and calculates the servo position delta
 * returns - signed servo position delta
 */
signed char calculateThumbstickIncrement(unsigned char thumbPosition)
{
	signed char increment=0;
	if (thumbPosition > 127 + THUMBSTICK_DEADZONE)
	{
		increment = map (thumbPosition, 127, 255, 1, THUMBSTICK_MAX_INCREMENT_SPEED);
	}
	else if (thumbPosition < 127 - THUMBSTICK_DEADZONE)
	{
		increment = map (thumbPosition, 127, 0, -1, -THUMBSTICK_MAX_INCREMENT_SPEED);
	}

	Log.trace(F("inc = %d" CR), increment);
	return increment;
}





void controlServosByIncrement()
{
	servoGroups[SERVO_GROUP_LEGS].getServoKeyframeAnimator(SERVO_YL)-> setServoAbsolutePositionChange(calculateThumbstickIncrement(message[PROT_STICK_LX]));
	servoGroups[SERVO_GROUP_LEGS].getServoKeyframeAnimator(SERVO_YR)-> setServoAbsolutePositionChange(calculateThumbstickIncrement(message[PROT_STICK_RX]));
	servoGroups[SERVO_GROUP_LEGS].getServoKeyframeAnimator(SERVO_RL)-> setServoAbsolutePositionChange(calculateThumbstickIncrement(message[PROT_STICK_LY]));
	servoGroups[SERVO_GROUP_LEGS].getServoKeyframeAnimator(SERVO_RR)-> setServoAbsolutePositionChange(calculateThumbstickIncrement(message[PROT_STICK_RY]));

	servoGroups[SERVO_GROUP_LEGS].driveServosToCalculatedPosition();
}

void loop()
{
	updateDistanceFilterArrays();

	//Log.notice("L=%d C=%d R=%d" CR, getDistanceFilterAverage(distanceIRLeft), getDistanceFilterAverage(distanceUltrasonic), getDistanceFilterAverage(distanceIRRight));
	printDistanceFilterAverage();
	printCurrentServoPositions();
	DEBUG_SERIAL_NAME.print("inMove: ");
	DEBUG_SERIAL_NAME.print(servoGroups[SERVO_GROUP_LEGS].isInMove());
	DEBUG_SERIAL_NAME.print("\n");

//	DEBUG_SERIAL_NAME.println("%");
	Test_voltageMeasure();


//	if (!isInMove)
//	{
//		Serial.println("DELAY!!!!!!!!!!!!!");
//		delay(2000);
//	}
	#if RUNMODE == 1
//	if (sweepDirection==0)
//	{
		//myMoveTest(SERVO_GROUP_LEGS, 10);
		//genericMove(MOVE_01_TEST, SERVO_GROUP_LEGS, 10);

	//genericMove(MOVE_01_WALKFORWARD, SERVO_GROUP_LEGS, 10);
	myObstacleMode();

//
//	}
//	else
//	{
//		Serial.println("sweep 1");
//		//currentServoPos = smoothedServoPosition(duration, key1, key0, currentServoPos);
//	}
//
//	if (!isInMove)
//	{
//		// change direction
//		if (sweepDirection == 0)
//		{
//			sweepDirection=1;
//		}
//		else
//		{
//			sweepDirection=0;
//		}
//
//	}
	#endif



	#if RUNMODE == 0


//	Log.trace("BLE_SERIAL_NAME.available %d" CR, BLE_SERIAL_NAME.available());
	// hopefully only the last package stays if more than 1 is in buffer
	while  (BLE_SERIAL_NAME.available() > 0)
	{
		readPacketFromBLE();
	}


	if (isTimeout())
	{
		// set message as invalid
		////Serial.println("timeout");
		isMessageValid = false;
	}

	if (isMessageValid)
	{

		serial_flag=false;
		DEBUG_SERIAL_NAME.print(message[PROT_STICK_LX]);
		DEBUG_SERIAL_NAME.print(" ");
		DEBUG_SERIAL_NAME.print(message[PROT_STICK_LY]);
		DEBUG_SERIAL_NAME.print(" ");
		DEBUG_SERIAL_NAME.print(message[PROT_STICK_RX]);
		DEBUG_SERIAL_NAME.print(" ");
		DEBUG_SERIAL_NAME.print(message[PROT_STICK_RY]);
		DEBUG_SERIAL_NAME.print(" T:");
		DEBUG_SERIAL_NAME.print(message[PROT_BTN_TRIANGLE]);
		DEBUG_SERIAL_NAME.print(" X:");
		DEBUG_SERIAL_NAME.print(message[PROT_BTN_CROSS]);
		DEBUG_SERIAL_NAME.print(" S:");
		DEBUG_SERIAL_NAME.print(message[PROT_BTN_SQUARE]);
		DEBUG_SERIAL_NAME.print(" O:");
		DEBUG_SERIAL_NAME.print(message[PROT_BTN_CIRCLE]);
		DEBUG_SERIAL_NAME.print("\n");


		bool highLevelControlPressed=false;

		// store trim?
		if (message[PROT_BTN_L1] && message[PROT_BTN_L2] && message[PROT_BTN_TRIANGLE])
		{
			if (isCurrentlyUpdatingTrim == false)
			{
				isCurrentlyUpdatingTrim=true;
				updateTrimValuesFromCurrentPositions();
				saveTrimValuesInEEPROM();
			}

		}

		// control servos directly with thumbsticks
		else
		{
			isCurrentlyUpdatingTrim=false;
			if (message[PROT_BTN_R2] > 0)
			{

				controlServosDirectly();
				isKeepServosLastPositions=true;

				// set move to manual. Otherwise servos will not return smoothly to center after Pressing the center key.
				for (unsigned int i=0; i< NUMBER_OF_SERVOGROUPS; i++)
				{
					servoGroupLastMove[i] = MOVE_01_MANUAL;
				}
			}

			// control servos with thumbsticks, but increase / decrease. No change on Thumbstick release
			else if (message[PROT_BTN_L2] > 0)
			{

				controlServosByIncrement();
				isKeepServosLastPositions=true;

				// set move to manual. Otherwise servos will not return smoothly to center after Pressing the center key.
				for (unsigned int i=0; i< NUMBER_OF_SERVOGROUPS; i++)
				{
					servoGroupLastMove[i] = MOVE_01_MANUAL;
				}
			}

			else
			{
				// high level controls, no direct control via sticks
				// return to center
				if (message[PROT_BTN_R1] > 0)
				{
					if (!isCenterKeyAlreadyPressed)
					{
						isKeepServosLastPositions=false;
						// update target positions to current values
						servoGroups[SERVO_GROUP_LEGS].updateTargetToCurrent();
						isCenterKeyAlreadyPressed=true;
					}

				}
				else
				{
					// center key is no longer pressed
					isCenterKeyAlreadyPressed=false;


					// move forward
					if (message[PROT_BTN_TRIANGLE] > 0 )
					{
						if (message[PROT_BTN_TRIANGLE] < 50)
						{
		//					DEBUG_SERIAL_NAME.println("walk f");
							//walk(1, t*1.5,1);
							genericMove(MOVE_01_WALKFORWARD, SERVO_GROUP_LEGS, 10);
						}
						else
						{
		//					DEBUG_SERIAL_NAME.println("run f");
							//walk(1, t/2,1);
							genericMove(MOVE_01_WALKFORWARD, SERVO_GROUP_LEGS, 20);
						}
						highLevelControlPressed=true;
						lastCommand=FORWARD;
					}

					// move backwards
					else
					{
						if (message[PROT_BTN_CROSS] > 0)

						{
			//					DEBUG_SERIAL_NAME.println("walk b");
							genericMove(MOVE_01_WALKTBACKWARDS, SERVO_GROUP_LEGS, 15);
							lastCommand=BACKWARDS;
							highLevelControlPressed=true;

						}

					// turn left
						else
						{
							if (message[PROT_BTN_SQUARE] > 0)

							{
				//						DEBUG_SERIAL_NAME.println("left");
								genericMove(MOVE_01_TURN_LEFT, SERVO_GROUP_LEGS, 15);
								lastCommand=TURNLEFT;
								highLevelControlPressed=true;
							}


						// turn right
							else
							{
								if (message[PROT_BTN_CIRCLE] > 0)
								{
					//							DEBUG_SERIAL_NAME.println("right");
									genericMove(MOVE_01_TURN_RIGHT, SERVO_GROUP_LEGS, 15);
									lastCommand=TURNRIGHT;
									highLevelControlPressed=true;
								}
							}
	//						Log.trace("highLevelControlPressed=%T, lastCommand=%d" CR, highLevelControlPressed, lastCommand);
						}
					}
					// no high level button pressed -> home
					if (!highLevelControlPressed)
					{

						if (lastCommand != STOP)
						{
							lastCommand = STOP;
							servoGroupMoveIteration[SERVO_GROUP_LEGS]=-1;
						}
						if (!isKeepServosLastPositions)
						{
							// genericMove(MOVE_01_CENTER, SERVO_GROUP_LEGS, 25);
							genericMove(MOVE_01_CENTER, SERVO_GROUP_LEGS, 10);
						}
					}
					else
					{
						// high level command received. Return to center after move iteration is done when stopping.
						isKeepServosLastPositions=false;
					}
	//				}
				}

			}
		}

		Log.verbose("%d %d %d %d" CR, servoGroups[0].getServoKeyframeAnimator(0)->getServoCurrentPositon(), servoGroups[0].getServoKeyframeAnimator(1)->getServoCurrentPositon(), servoGroups[0].getServoKeyframeAnimator(2)->getServoCurrentPositon(), servoGroups[0].getServoKeyframeAnimator(3)->getServoCurrentPositon());
//		Log.verbose("%d %d %d %d wT: %d %d %d %d , ServoTrim: %d %d %d %d, servoRead %d %d %d %d " CR, servoGroups[0].getServoKeyframeAnimator(0)->getServoCurrentPositon(), servoGroups[0].getServoKeyframeAnimator(1)->getServoCurrentPositon(), servoGroups[0].getServoKeyframeAnimator(2)->getServoCurrentPositon(),servoGroups[0].getServoKeyframeAnimator(3)->getServoCurrentPositon(),          servoGroups[0].getServoKeyframeAnimator(0)->getServoCurrentPositon() + servosLegs[0].getTrim(), servoGroups[0].getServoKeyframeAnimator(1)->getServoCurrentPositon() + servosLegs[1].getTrim(), servoGroups[0].getServoKeyframeAnimator(2)->getServoCurrentPositon() + servosLegs[2].getTrim(), servoGroups[0].getServoKeyframeAnimator(3)->getServoCurrentPositon()  + servosLegs[3].getTrim(), servosLegs[0].getTrim(), servosLegs[1].getTrim(), servosLegs[2].getTrim(), servosLegs[3].getTrim(), servosLegs[0].read(), servosLegs[1].read(), servosLegs[2].read(), servosLegs[3].read());



//		clearMessage();

		// invalidate message, because it has been processed
		isMessageValid = false;
	}
//	else
//	{
////		DEBUG_SERIAL_NAME.println("no valid msg");
//	}


 /*
	if (irValue != '\0')// Bluetooth serial port data stream on app side (character acquisition is completed in timer 2)
	{
		serial_flag = false;
		switch (irValue)
		{
		case BTN_UP:
			mp3.stopPlay();
			mode = BLUETOOTH;
			BTmode = FORWARD;
			break;
		case BTN_DOWN:
			mp3.stopPlay();
			mode = BLUETOOTH;
			BTmode = BACKWARDS;
			break;
		case BTN_LEFT:
			mp3.stopPlay();
			mode = BLUETOOTH;
			BTmode = TURNLEFT;
			break;
		case BTN_RIGHT:
			mp3.stopPlay();
			mode = BLUETOOTH;
			BTmode = TURNRIGHT;
			break;
		case BTN_IDLE:
			mp3.stopPlay();
			mode = IDLE;
			servoAttach();
			homes(200);
			servoDetach();
			break;
		case BTN_MUSIC:
			servoDetach();
			mp3.stopPlay();
			mode = MUSIC;
			musicIndex++;
			if (musicIndex > musicNumber)
			{
				musicIndex = 2;
			}
			mp3.playSong(musicIndex, mp3.volume);
			preMp3Millis = millis();
			break;
		case BTN_DANCE:
			mode = DANCE;
			danceFlag = true;
			danceIndex++;
			if (danceIndex > 4)
			{
				danceIndex = 2;
			}
			mp3.playSong(danceIndex, mp3.volume);
			break;

		case BTN_OBSTACLE:
			delays(10);
			mp3.stopPlay();
			delays(10);
			mp3.playSong(7, mp3.volume);
			mode = OBSTACLE;
			preMp3MillisStop_OBSTACLE = 0;
			preMp3MillisStop_OBSTACLE = millis();
			break;
		case BTN_VOL_ADD:
			mp3.volumePlus();
			mp3.volume += 1;
			if (mp3.volume >= 30)
			{
				mp3.volume = 30;
			}
			break;
		case BTN_VOL_SUB:
			mp3.volumeDown();
			mp3.volume -= 1;
			if (mp3.volume <= 0)
			{
				mp3.volume = 0;
			}
			break;
		case BTN_FOLLOW:
			delays(10);
			mp3.stopPlay();
			delays(10);
			mp3.playSong(6, mp3.volume);
			mode = FOLLOW;
			preMp3MillisStop_FOLLOW = 0;
			preMp3MillisStop_FOLLOW = millis();
			break;
		case BTN_RR_ADD:
			trim_rr++;
			trim_rr = constrain(trim_rr, -90, 90);
			EEPROM.write(addr_trim_rr, trim_rr + 90);
			servoAttach();
			homes(100);
			servoDetach();
			//Serial.println(trim_rr);
			break;
		case BTN_RL_ADD:
			trim_rl++;
			trim_rl = constrain(trim_rl, -90, 90);
			EEPROM.write(addr_trim_rl, trim_rl + 90);
			servoAttach();
			homes(100);
			servoDetach();
			//Serial.println(trim_rl);
			break;
		case BTN_YR_ADD:
			trim_yr++;
			trim_yr = constrain(trim_yr, -90, 90);
			EEPROM.write(addr_trim_yr, trim_yr + 90);
			servoAttach();
			homes(100);
			servoDetach();
			//Serial.println(trim_yr);
			break;
		case BTN_YL_ADD:
			trim_yl++;
			trim_yl = constrain(trim_yl, -90, 90);
			EEPROM.write(addr_trim_yl, trim_yl + 90);
			servoAttach();
			homes(100);
			servoDetach();
			//Serial.println(trim_yl);
			break;
		case BTN_RR_SUB:
			trim_rr--;
			trim_rr = constrain(trim_rr, -90, 90);
			EEPROM.write(addr_trim_rr, trim_rr + 90);
			servoAttach();
			homes(100);
			servoDetach();
			//Serial.println(trim_rr);
			break;
		case BTN_RL_SUB:
			trim_rl--;
			trim_rl = constrain(trim_rl, -90, 90);
			EEPROM.write(addr_trim_rl, trim_rl + 90);
			servoAttach();
			homes(100);
			servoDetach();
			//Serial.println(trim_rl);
			break;
		case BTN_YR_SUB:
			trim_yr--;
			trim_yr = constrain(trim_yr, -90, 90);
			EEPROM.write(addr_trim_yr, trim_yr + 90);
			servoAttach();
			homes(100);
			servoDetach();
			//Serial.println(trim_yr);
			break;
		case BTN_YL_SUB:
			trim_yl--;
			trim_yl = constrain(trim_yl, -90, 90);
			EEPROM.write(addr_trim_yl, trim_yl + 90);
			servoAttach();
			homes(100);
			servoDetach();
			//Serial.println(trim_yl);
			break;
		default:
			break;
		}
		if (serial_flag == false)
		{
			irValue = '\0'; // Data Command Clearing Serial Cache
		}
	}

	switch (mode)
	{
	case IDLE:
		mp3.stopPlay();
		servoDetach();
		break;
	case BLUETOOTH:
		switch (BTmode)
		{
		case FORWARD:
			servoAttach();
			walk(1, t * 3, 1);
			servoDetach();
			break;
		case BACKWARDS:
			servoAttach();
			walk(1, t * 3, -1);
			servoDetach();
			break;
		case TURNRIGHT:
			servoAttach();
			turn(1, t * 4, 1);
			servoDetach();
			break;
		case TURNLEFT:
			servoAttach();
			turn(1, t * 4, -1);
			servoDetach();
			break;
		default:
			break;
		}
		break;
	case OBSTACLE:
		if (millis() - preMp3MillisStop_OBSTACLE > 1200) // Judgment timestamp: Turn off music playback (avoid random music broadcasting)
		{
			mp3.stopPlay();
		}

			servoAttach();
			obstacleMode();
			servoDetach();

			break;
		case FOLLOW:

			if (millis() - preMp3MillisStop_FOLLOW > 1200) // Judgment timestamp: Turn off music playback (avoid random music broadcasting)
			{
				mp3.stopPlay();
		}
			servoAttach();
			followMode();
			servoDetach();
			break;
		case MUSIC:
			if (millis() - preMp3Millis > 1000)
			{
				preMp3Millis = millis();
				if (mp3.getPlayStatus() == mp3.playStatus[0])
				{
					musicIndex++;
					if (musicIndex > musicNumber)
					{
						musicIndex = 2;
					}
					mp3.playSong(musicIndex, mp3.volume);
				}
			}
			break;
		case DANCE:
			if (danceFlag == true)
			{
				servoDetach();
				delays(10);
				mp3.stopPlay();
				delays(10);
				mp3.playSong(danceIndex, mp3.volume);
				servoAttach();
				switch (danceIndex)
				{
				case 2:
					dance2();
					break;
				case 3:
					dance3();
					break;
				case 4:
					dance4();
					break;
				default:
					break;
				}
				homes(200);
				servoDetach();
				danceFlag = false;
				mp3.stopPlay();
				delay(10);
				mp3.stopPlay();
				delay(10);
				mp3.stopPlay();
				delay(10);
				danceNum = 0;
			}
			break;
		default:
			break;
	}
	*/
	#endif
}



/**
 * calculates the move iteration number wich should be processed
 * iterationMode may be  ITERATION_CONTINUATION_MODE_LOOP, ITERATION_MODE_REVERSE or ITERATION_MODE_ONCE
 * iteration is reference to the iteration variable for the servo group
 */
void calculateMoveIterationId (unsigned char iterationMode, signed char sizeOfIterations, signed char &iteration,  signed char &iterationDirection)
{
	#if DEBUG_CALCULATE_MOVE_ITERATION_ID == 1
		Log.trace(F("calculateMoveIterationId: args: %d, %d, %d, %d"CR), iterationMode, sizeOfIterations, iteration, iterationDirection);
	#endif
	// -1 is the start when idling
	if (iteration==-1)
	{
		// go iterations forward, start with 0 in the end
		iterationDirection=1;
//		Log.trace("XXXXXX 0" CR);
	}
	else
	{
		// lower end, already going backward
		if (iteration == 0 && iterationDirection == -1)
		{
			// go forward again, start with iteration 1 because we are already at 0
			iterationDirection = 1;
//			Log.trace("XXXXXX 1" CR);
		}
		else
		{
			if (iteration +1 == sizeOfIterations)
			{
				// maximum is reached (iterations counts from 0)
				if (iterationMode == ITERATION_CONTINUATION_MODE_LOOP)
				{
					// loop mode, start from beginning, keep direction forward -> start at 0 at the end
					iteration=-1;
//					Log.trace("XXXXXX 2" CR);
				}
				else
				{
					if (iterationMode == ITERATION_CONTINUATION_MODE_REVERSE)
					{
						// end is reached and reverse is set, so reverse direction
						iterationDirection = -1;
//						Log.trace("XXXXXX 3" CR);
					}
					else
					{
						if (iterationMode == ITERATION_CONTINUATION_MODE_ONCE_NO_WAIT || iterationMode == ITERATION_CONTINUATION_MODE_ONCE_WAIT)
						{
							// end is reached, stop is set. so keep there
							iterationDirection=0;
//							Log.trace("XXXXXX 4" CR);
						}
					}
				}
			}
		}
	}

	iteration=iteration + iterationDirection;

	#if DEBUG_CALCULATE_MOVE_ITERATION_ID == 1
		Log.trace(F("calculateMoveIterationId: iteration=%d, iterationDirection=%d"CR), iteration, iterationDirection);
	#endif


}


/**
 * fixes speed if outside bounds.
 * speed: 0 - 255 is allowed, will be mapped to 1 - 255 to fix division by 0 issue
 */
unsigned char checkAndCorrectSpeedBounds(unsigned char speed)
{
	if (speed == 0)
	{
		speed = 1;
	}
//	else
//	{
//		if (speed > 255)
//		{
//			speed = 255;
//		}
//	}

	return speed;

}



/**
 * new genericMoveFunction
 */
void genericMove(unsigned char moveId, unsigned char servoGroupId, unsigned int speed)
{
//	Log.trace(F("MOVE is: %d, old is %d" CR), moveId, servoGroupLastMove[servoGroupId]);
	bool isNewMove=false;
	//	unsigned char oldMoveIteration = servoGroupMoveIteration[servoGroupId] ;
	//Log.trace(F("genericMove -  start: isInMove%T" CR), servoGroups[servoGroupId].isInMove());
	if ( ! servoGroups[servoGroupId].isInMove())
	{

		if (servoGroupLastMove[servoGroupId] != moveId)
		{
			// we have a new move now. set iteration to the beginning (-1)
			servoGroupMoveIteration[servoGroupId]=-1;
			isNewMove = true;

		}
		else
		{
			isNewMove=false;
		}


		// remember the move id
		servoGroupLastMove[servoGroupId] = moveId;

//		// special case for center move. do Nothing if end is reached
//		if (moveId == MOVE_01_CENTER && servoGroups[servoGroupId].getServoKeyframeAnimator(0)->getServoCurrentPositon() == servoGroups[servoGroupId].getServoKeyframeAnimator(0)->getServoTargetPositon())
//		{
//			Log.trace("XXXXXXXXX DO NOTHING" CR);
//			// do nothing. We are already in center Position
//		}
//		else
//		{
			// we are not in center position or we are not in center move, process normally



		// increase the iteration if we will stay in bounds. otherwise recycle from beginning
		//calculateMoveIterationId( ITERATION_MODE_LOOP, 4, servoGroupMoveIteration[servoGroupId], servoGroupMoveIterationDirection[servoGroupId]);
		calculateMoveIterationId( movesLegs4Servos.getContinuationMode(moveId), movesLegs4Servos.getNumberOfIterations(moveId), servoGroupMoveIteration[servoGroupId], servoGroupMoveIterationDirection[servoGroupId]);

		// now copy the keyframe of move for the calculated iteration to mykeyframe
		unsigned char myKeyframe[NUMBER_OF_SERVOGROUP_LEGS_SERVOS +1];

		movesLegs4Servos.getKeyframe(moveId, servoGroupMoveIteration[servoGroupId], myKeyframe);
		if (!isNewMove)
		{
			// check the continuation mode. Rewrite duration / time if it is set to ITERATION_CONTINUATION_MODE_ONCE_NO_WAIT
			if (movesLegs4Servos.getContinuationMode(moveId) == ITERATION_CONTINUATION_MODE_ONCE_NO_WAIT )
			{
//				Log.trace(F("XXXXXXXXXXXXXXX" CR));
				// check if target position for all servos is reached.
				if (servoGroups[servoGroupId].isTargetPositionOfKeyframeReached())
				{
					// set duration to 0 ms. By that the Move is over within the next loop iteration.
					myKeyframe[0]=0;
				}
			}
		}



		DEBUG_SERIAL_NAME.print("genericMove - myKeyframe values: ");
		for (unsigned char i=0; i< NUMBER_OF_SERVOGROUP_LEGS_SERVOS +1; i++ )
		{
			DEBUG_SERIAL_NAME.print(myKeyframe[i]);
			DEBUG_SERIAL_NAME.print(" ");
		}
		DEBUG_SERIAL_NAME.print("\n");

		// duration in keyframe is 0.01s. Multiply by 10 to have millis
		unsigned int baseDurationInMs = ((unsigned int) myKeyframe[0])*10;

		unsigned char keyframeOnlyServos[servoGroups[servoGroupId].getNumberOfServos()];

		DEBUG_SERIAL_NAME.print("genericMove - keyframeOnlyServos[] = ");
		for (unsigned int i=0; i < servoGroups[servoGroupId].getNumberOfServos(); i++)
		{
			keyframeOnlyServos[i] = myKeyframe[i+1];

			DEBUG_SERIAL_NAME.print(keyframeOnlyServos[i]);
			DEBUG_SERIAL_NAME.print(" ");
		}
		DEBUG_SERIAL_NAME.print("\n");

		// the faster the speed, the less the time given for a move.

		servoGroups[servoGroupId].setServoMoveDuration(baseDurationInMs / (checkAndCorrectSpeedBounds(speed) / 10 ));

		servoGroups[servoGroupId].setServoPositionsNextKeyframe(keyframeOnlyServos);

//		Log.trace("setting keyframemmode:");
		for (unsigned int i=0; i< servoGroups[servoGroupId].getNumberOfServos(); i++)
		{
//				Log.trace("[%d]=%d ",i, KEYFRAME_MODE_SMOOTH);
			servoGroups[servoGroupId].getServoKeyframeAnimator(i)->setKeyframeMode(KEYFRAME_MODE_SMOOTH);
		}
//		Log.trace("\n");
	}

//		Log.trace("getting keyframemmode:");
//		for (unsigned int i=0; i< servoGroups[servoGroupId].getNumberOfServos(); i++)
//		{
//			Log.trace("[%d]=%d ",i, servoGroups[servoGroupId].getServoKeyframeAnimator(i)->getKeyframeMode()) ;
//		}
//		Log.trace("\n");

		// calculate the new servo position
		servoGroups[servoGroupId].calculateServoPositions();

		// drive the servos
//		for (unsigned char i=0; i < keyframeServoGroupLegs.getNumberOfServos(); i++)
//		{
//			servoGroupLegs[i].enhancedWrite(keyframeServoGroupLegs.getCalculatedServoPositionById(i) , 0, 180);
//		}
		servoGroups[servoGroupId].driveServosToCalculatedPosition();
//		Log.trace("adresses keyframemmode [0]=%d [1]=%d, [2]=%d, [3]=%d" CR, servoGroups[SERVO_GROUP_LEGS].getServoKeyframeAnimator(0).getKeyframeModeAddress(), servoGroups[SERVO_GROUP_LEGS].getServoKeyframeAnimator(1).getKeyframeModeAddress(), servoGroups[SERVO_GROUP_LEGS].getServoKeyframeAnimator(2).getKeyframeModeAddress(), servoGroups[SERVO_GROUP_LEGS].getServoKeyframeAnimator(3).getKeyframeModeAddress());
//	}
	lastMove=moveId;

}
//
///**
// * function for move testing
// *  servoGroupId: ID of ServoGroup
// *  speed: for movement.
// */
//void myMoveTest(unsigned char servoGroupId, unsigned int speed)
//{
//	unsigned char oldMoveIteration = servoGroupMoveIteration[servoGroupId] ;
//
//	//Log.notice(F("myMoveTest: keyframeServoGroupLegs.isInMove=%T\n"), keyframeServoGroupLegs.isInMove());
//
//	unsigned int base_duration = 1500;
//	if ( ! keyframeServoGroupLegs.isInMove())
//	{
//		// increase the iteration if we will stay in bounds. otherwise recycle from beginning
//		calculateMoveIterationId( ITERATION_MODE_LOOP, 4, servoGroupMoveIteration[servoGroupId], servoGroupMoveIterationDirection[servoGroupId]);
//
//		// define all the moves
////		unsigned char moves[][4]={
////			{90, 90 + 35, 90 + 15, 90 + 15},
////			{90 + 25, 90 + 30, 90 + 15, 90 + 15},
////			{90 + 20, 90 + 20, 90 - 15, 90 - 15},
////			{90 - 35, 90, 90 - 15, 90 - 15},
////			{90 - 40, 90 - 30, 90 - 15, 90 - 15},
////			{90 - 20, 90 - 20, 90 + 15, 90 + 15}
//		unsigned char moves[][4]={
////			{90, 90, 90 , 90 },
////			{90-80, 90, 90 , 90},
////			{90, 90, 90 , 90 },
////			{90+80, 90, 90 , 90},
////
////			{90, 90, 90 , 90 },
////			{90, 90-80, 90 , 90},
////			{90, 90, 90 , 90 },
////			{90, 90+80, 90 , 90},
////
////			{90, 90, 90 , 90 },
////			{90, 90, 90-80 , 90},
////			{90, 90, 90 , 90 },
////			{90, 90, 90+80 , 90},
//
//			{90, 90, 90 , 90 -80},
//			{90, 90, 90 , 90},
//			{90, 90, 90 , 90 },
//			{90, 90, 90 , 90+80}
//
//
//		};
//
//		// the faster the speed, the less the time given for a move.
//
//		keyframeServoGroupLegs.setServoMoveDuration(base_duration / (checkAndCorrectSpeedBounds(speed) / 10 ));
//
//		keyframeServoGroupLegs.setServoPositionsNextKeyframe(moves[servoGroupMoveIteration[servoGroupId]]);
//
//		for (unsigned int i=0; i< NUMBER_OF_SERVOGROUP_LEGS_SERVOS; i++)
//		{
//			keyframeServoGroupLegs.getServoKeyframeAnimator(i).setKeyframeMode(KEYFRAME_MODE_SMOOTH);
//		}
//
//	}
//
//	// calculate the new servo position
//	keyframeServoGroupLegs.calculateServoPositions();
//
//	for (unsigned char i=0; i < keyframeServoGroupLegs.getNumberOfServos(); i++)
//	{
//		servosLegs[i].enhancedWrite(keyframeServoGroupLegs.getCalculatedServoPositionById(i) , 0, 180);
//	}
//
//	//Log.trace(F("myMoveTest: move=%T i=%d d=%d"CR), keyframeServoGroupLegs.isInMove(),  servoGroupMoveIteration[servoGroupId], servoGroupMoveIterationDirection[servoGroupId] );
//}

/**
 * this function stores the trim values in eeprom permanently.
 * The trim value will only be written to EEPROM, if it differs.
 */
void saveTrimValuesInEEPROM()
{
	Log.trace(F("storeTrimValuesInEEPROM" CR));
	for (unsigned char i=0; i< NUMBER_OF_SERVOGROUP_LEGS_SERVOS; i++)
	{
		Log.trace(F("eeprom=%d == servoGroupLegsTrim=%d ?" CR), getEEPROMTrim(i), servosLegs[i].getTrim());
		if (getEEPROMTrim(i) != servosLegs[i].getTrim())
		{
			Log.trace(F("save Servo Trim %d = %d (%d)" CR), i,servosLegs[i].getTrim(),  servosLegs[i].getTrim() + 90 );
			EEPROM.write((int) i, servosLegs[i].getTrim() + 90);
			Log.trace(F("verify Servo Trim %d = %d (%d) = %d" CR), i,servosLegs[i].getTrim(),  servosLegs[i].getTrim() + 90, getEEPROMTrim(i) );
		}
	}
//
//
//	if (getEEPROMTrim(SERVO_RL) != trim_rl)
//	{
//		Log.trace(F("save RL"));
//		EEPROM.write((int) SERVO_RL, trim_rl + 90);
//	}
//	if (getEEPROMTrim(SERVO_RR) != trim_rr)
//	{
//		Log.trace(F("save RR"));
//		EEPROM.write((int) SERVO_RR, trim_rr + 90);
//	}
//	if (getEEPROMTrim(SERVO_YL) != trim_yl)
//	{
//		Log.trace(F("save YL"));
//		EEPROM.write((int) SERVO_YL, trim_yl + 90);
//	}
//	if (getEEPROMTrim(SERVO_YR) != trim_yr)
//	{
//		Log.trace(F("save YR"));
//		EEPROM.write((int) SERVO_YR, trim_yr + 90);
//	}

}


void updateTrimValuesFromCurrentPositions()
{

	for (unsigned char i=0; i< NUMBER_OF_SERVOGROUP_LEGS_SERVOS; i++)
	{

		Log.verbose(F("updateTrimValuesFromCurrentPositions: i=%d, servoPos=%d, oldtrimRAM=%d, oldTrimEEPROM=%d" CR) , i, servoGroups[SERVO_GROUP_LEGS].getServoKeyframeAnimator(i)->getServoCurrentPositon(), servosLegs[i].getTrim(), EEPROM.read((int) i -90));
		//signed char newTrim = servosLegs[i].read() - servoGroups[SERVO_GROUP_LEGS].getServoKeyframeAnimator(i)->getServoCurrentPositon();
		signed char newTrim = servosLegs[i].read() - 90;
		Log.verbose(F("updateTrimValuesFromCurrentPositions: set trim for servo[%d] to %d" CR), i, servosLegs[i].getTrim());

		servosLegs[i].setTrim(newTrim);

		servoGroups[SERVO_GROUP_LEGS].getServoKeyframeAnimator(i)->setServoAbsolutePosition(90);
		servoGroups[SERVO_GROUP_LEGS].driveServosToCalculatedPosition();
	}


}

void wrapperPlayMp3(int title, unsigned char volume, bool stopCurrentPlay)
{
    mp3Serial.listen();

    // stop the play if it is set.
    // currently no status checking. Don't know if needed.
    if (stopCurrentPlay)
    {
    	mp3.stopPlay();
    }
    mp3.playSong(title, volume);
    BLE_SERIAL_NAME.listen();

}

#define SOUND_EVENT_NONE 0 // no sound
#define SOUND_EVENT_MIN_DISTANCE_REACHED 1

void playSoundEvent(unsigned int eventId)
{
	// not implemented yet
}


bool isNeedToCheckIRDistanceLeft()
{
	if (servoGroups[SERVO_GROUP_LEGS].getServoKeyframeAnimator(SERVO_RL)->getServoTargetPositon() > 90 + SERVO_RL_IR_ANGLE_LIMIT
			&& servoGroups[SERVO_GROUP_LEGS].getServoKeyframeAnimator(SERVO_YL)->getServoTargetPositon() > 90 + SERVO_YL_IR_ANGLE_LIMIT)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool isNeedToCheckIRDistanceRight()
{
	if (servoGroups[SERVO_GROUP_LEGS].getServoKeyframeAnimator(SERVO_RR)->getServoTargetPositon() > 90 + SERVO_RR_IR_ANGLE_LIMIT
			&& servoGroups[SERVO_GROUP_LEGS].getServoKeyframeAnimator(SERVO_YR)->getServoTargetPositon() > 90 + SERVO_YR_IR_ANGLE_LIMIT)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void myObstacleMode()
{

	unsigned char move = servoGroupLastMove[SERVO_GROUP_LEGS];
	unsigned char speed = 20;
	unsigned int sound = SOUND_EVENT_NONE;
	// if we are not in a move -> means we are at a keyframe
	if (! servoGroups[SERVO_GROUP_LEGS].isInMove())
	{
		unsigned int avgDistanceCenter=getDistanceFilterAverage(distanceUltrasonic);
		unsigned int avgDistanceLeft=getDistanceFilterAverage(distanceIRLeft);
		unsigned int avgDistanceRight=getDistanceFilterAverage(distanceIRRight);

		// todo: need to count steps

		if (move == MOVE_01_WALKFORWARD)
		{

			bool isToStop=false;
			// currently moving forward. ultrasonic sensor in all cases / all keyframes
			if (avgDistanceCenter < DISTANCE_ULTRASONIC_STOP)
			{
				// stop. return to center
				isToStop = true;
			}

			// check the space to the sides. But must only check if robot is not tilted too much because IR may measure the distance to the ground.
			// check left sensor if angle is not too steep.
			if (isNeedToCheckIRDistanceLeft && avgDistanceLeft < DISTANCE_IR_STOP)
			{
				 isToStop = true;
			}

			// check right sensor if angle is not too steep.
			if ( isNeedToCheckIRDistanceRight || avgDistanceRight < DISTANCE_IR_STOP)
			{
				 isToStop = true;
			}

			// stop / center if to close to obstacles
			if (isToStop)
			{
				move=MOVE_01_CENTER;
				sound=SOUND_EVENT_MIN_DISTANCE_REACHED;
			}

		}
		else if (move == MOVE_01_CENTER)
		{
			// MOVE_01_CENTER has only one keyframe, so no need to check in which iteration we are

			// decide in which direction to go. Take the direction with higher distance
			if (avgDistanceLeft > DISTANCE_IR_START && avgDistanceLeft > avgDistanceRight)
			{
				// more room to the left
				move = MOVE_01_TURN_LEFT;
			}
			else if  (avgDistanceRight > DISTANCE_IR_START && avgDistanceRight > avgDistanceLeft )
			{
				// more room to the right
				move = MOVE_01_TURN_RIGHT;
			}
			else if (avgDistanceCenter > DISTANCE_ULTRASONIC_START || avgDistanceLeft > DISTANCE_IR_START || avgDistanceRight > DISTANCE_IR_START)
			{
				// enough space in all direction. we can savely move forward
				move = MOVE_01_WALKFORWARD;

			}
			else
			{
				// not enough space to front, left, right -> move backwards
				move = MOVE_01_WALKTBACKWARDS;
			}

		}





		if (avgDistanceCenter > 15)
		{
			move=MOVE_01_WALKFORWARD;
			genericMove(MOVE_01_WALKFORWARD, SERVO_GROUP_LEGS, 20);
		}
		else if (avgDistanceLeft < DISTANCE_IR_STOP && avgDistanceLeft < avgDistanceRight)
		{
			move=MOVE_01_TURN_RIGHT;
			genericMove(MOVE_01_TURN_RIGHT, SERVO_GROUP_LEGS, 20);
		}
		else if (avgDistanceRight < DISTANCE_IR_STOP)
		{
			move=MOVE_01_TURN_LEFT;
			genericMove(MOVE_01_TURN_LEFT, SERVO_GROUP_LEGS, 20);
		}
		else
		{
			move=MOVE_01_WALKTBACKWARDS;

		}

	}

	// do the move
	genericMove(move, SERVO_GROUP_LEGS, speed);
	// remember last move
	servoGroupLastMove[SERVO_GROUP_LEGS] = move;


}

void printDistanceFilterAverage()
{
	DEBUG_SERIAL_NAME.print("D: ");
	DEBUG_SERIAL_NAME.print(getDistanceFilterAverage(distanceIRLeft));
	DEBUG_SERIAL_NAME.print(" ");
	DEBUG_SERIAL_NAME.print(getDistanceFilterAverage(distanceUltrasonic));
	DEBUG_SERIAL_NAME.print(" ");
	DEBUG_SERIAL_NAME.print(getDistanceFilterAverage(distanceIRRight));
	DEBUG_SERIAL_NAME.print(" ");

}

void printCurrentServoPositions()
{
	for (unsigned char servoGroupId = 0; servoGroupId < NUMBER_OF_SERVOGROUPS; servoGroupId++)
	{
		DEBUG_SERIAL_NAME.print("SG ");
		DEBUG_SERIAL_NAME.print(servoGroupId);
		DEBUG_SERIAL_NAME.print(": ");

		for (unsigned char servoId=0;  servoId < NUMBER_OF_SERVOGROUP_LEGS_SERVOS; servoId++)
		{
			DEBUG_SERIAL_NAME.print(servoGroups[servoGroupId].getServoKeyframeAnimator(servoId)->getServoCurrentPositon());
			DEBUG_SERIAL_NAME.print(" ");
		}

	}
}
