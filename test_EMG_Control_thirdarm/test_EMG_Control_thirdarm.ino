// Define the LED pins
#include <Arduino.h>
const int ledPins[] = {52, 40, 30, 26};
const int numLeds = 4; // Number of LEDs
float flex, ext;
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
// Timing control

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif

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

void setup() {
  // Initialize the serial communication
  DEBUG_SERIAL.begin(115200);
  inputString.reserve(200);

  // Initialize all LED pins as output
  for (int i = 0; i < numLeds; i++) {
    pinMode(ledPins[i], OUTPUT);
  }
}

void loop() {

  // Check if there is data available on the serial port
  while (DEBUG_SERIAL.available()>0) {
      inputString = DEBUG_SERIAL.readStringUntil('\n');
      stringComplete = true;
  }

  if(stringComplete){
          flex = getValue(inputString, ' ', 0).toFloat(); 
          ext = getValue(inputString, ' ', 1).toFloat(); 
          inputString = ""; //Reset string
          stringComplete = false;
        }

  digitalWrite(ledPins[0], flex > 0.5 ? HIGH : LOW);
  digitalWrite(ledPins[1], ext > 0.5 ? HIGH : LOW);
  DEBUG_SERIAL.print("<");
  DEBUG_SERIAL.print("yea");
  DEBUG_SERIAL.println(">");
}


// Function to convert a byte array to a float
float bytesToFloat(uint8_t *buffer, int startIndex) {
  double value;
  memcpy(&value, &buffer[startIndex], 4); // Copy 8 bytes into the float
  return value;
}

