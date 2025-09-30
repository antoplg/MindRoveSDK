#include <DynamixelShield.h>
#include "origami_module.h"
#include "Timing.cpp"

// Old code. Don't know what it does exactly, but doesn't matter for now.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif

String DataBLE = "";

// Timing control
Timer Timer_01, Timer_02;

int postionContolLoop = 20, printLoop = 200; // [ms]
bool print = false;

//Initialize module class
int max_motor_speed = 200, min_motor_speed = -200; // [0.024 rad/s] or [0.229 rotation/minute] // [raw]

float min_position = 15*PI/180; // appx 0.26, before it was 5
float max_position = 60*PI/180; // appx 1.4 [rad], before it was 80

// Initialize
float setpointAngle=min_position, setpointVelocity = 0; // for sinusoidal angle

// --- CONTROL MODE ---
int controlMode = 4; // 0-> JOYSTICK; 1-> HBC DIAPHRAGM; 2-> HBC DIAPHRAGM & SOCKS; 3-> POSITION CONTROL; 4 -> EMG

// --- JOYSTICK CONTROL ---
// Set default joystick angles to minimum extension
float joystickAngles[3] = {min_position, min_position, min_position};
String inputStringJoystickBLE = "";
bool stringCompleteJoystick = false;

// --- HBC - DIAPHRAGM CONTROL ---
float diaphragmResistance = 10.5;
float diaphragmRESTLOWResistance = 10;
float diaphragmRESTResistance = 10.5;
float diaphragmRESTHIGHResistance = 11;
String inputStringDiaphragmBLE = "";
bool stringCompleteDiaphragm = false;

// --- HBC - DIAPHRAGM AND SOCKS CONTROL ---
String inputStringHBCBLE = "";
bool stringCompleteHBC = false;
float dataHBC[5] = {10.5, 1000, 1000, 1000, 1000}; // [diaphragm, left bottom resistance, left top resistance, right bottom resistance, right top resistance]
float sock_lower_bound = 400;
float sock_upper_bound = 700;

float diaphragm_scaled = 0;
float left_sock_scaled = 0;
float right_sock_scaled = 0;

// --- POSITON CONTROL ---
bool state = false;

// --- MOTORS FOR MODULE ---
const uint8_t DXL_ID_CNT = 3;
const uint8_t DXL_ID[DXL_ID_CNT] = {0,1,2}; // 6,7,8 for third module
float setAngle[DXL_ID_CNT]={min_position, min_position, min_position}; // 
float setVelocity[DXL_ID_CNT]={0,0,0}; // ,0,0,0

// --- GRIPPER MOTOR --- 
const uint8_t DXL_ID_CNT_TOTAL = 4;
const uint8_t GRIPPER_DXL_ID = 6;

float poseUnit = 0.001534355; // [rad/RAW_DATA]

Module module1(DXL_ID[0], DXL_ID[1], DXL_ID[2]);

float startTime, currentTime, previousTime;

const float DXL_PROTOCOL_VERSION = 2.0;

DynamixelShield dxl; // Serial 

//This namespace is required to use Control table item names
using namespace ControlTableItem;



// ----- SYNC READ and WRITE  - (from example code)
const uint8_t BROADCAST_ID = 254;

const uint16_t user_pkt_buf_cap = 128; // why 128? maybe not enough for 6 motors?
uint8_t user_pkt_buf[user_pkt_buf_cap];

// --- POSITION DATA TO READ --- 
// Starting address of the Data to read; Present Position = 132
const uint16_t SR_START_ADDR = 132;
// Length of the Data to read; Length of Position data of X series is 4 byte
const uint16_t SR_ADDR_LEN = 4;

// --- VELOCITY DATA TO WRITE --- 
// Starting address of the Data to write; Goal Velocity = 104
const uint16_t SW_START_ADDR = 104;
// Length of the Data to write; Length of Goal Velocity data of X series is 4 byte
const uint16_t SW_ADDR_LEN = 4;

typedef struct sr_data{
  int32_t present_position;
} __attribute__((packed)) sr_data_t;

typedef struct sw_data{
  int32_t goal_velocity;
} __attribute__((packed)) sw_data_t;


sr_data_t sr_data[DXL_ID_CNT_TOTAL];
DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr[DXL_ID_CNT_TOTAL];

sw_data_t sw_data[DXL_ID_CNT_TOTAL];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT_TOTAL];

// --------------


// https://stackoverflow.com/questions/9072320/split-string-into-string-array
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

float sinWaveGenerator(double offset, double amplitude, double period, double currentTime){
  return amplitude*sin(2*PI*currentTime/period) + offset;
}

// Function to Convert a Byte Array to a Float (4 Bytes)
float bytesToFloat(uint8_t *buffer, int startIndex) {
  float value;
  memcpy(&value, &buffer[startIndex], 4); // Copy 4 bytes into the double
  return value;
}