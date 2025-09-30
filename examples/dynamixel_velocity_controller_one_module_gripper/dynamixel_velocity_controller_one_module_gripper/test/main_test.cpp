#include "main.h"
//#include "origamiJoystick.h"
#include <Arduino.h>


void setup() {
  DEBUG_SERIAL.begin(9600);
  while(!DEBUG_SERIAL); // Wait for the port to open

  DEBUG_SERIAL.println("Started the receiver (black board");
  Serial1.begin(9600);  // , SERIAL_8N1, RXD2, TXD2
  while(!Serial1);

  // --- INIT SERIAL COMMUNICATION TO ESP BLE MODULE ---
  //Serial2.begin(9600); 
  //while(!Serial2); // Wait for the port to open
  //DEBUG_SERIAL.println("Setup");

}

 // SoftwareSerial is half duplex
void loop() {
  
    // --- JOYSTICK CONTROL - 0 --- 
    if(controlMode==0){  

      
      //DEBUG_SERIAL.println(incoming);
      //DEBUG_SERIAL.println("loop");
      // read the joystick values from ESP BLE module
      while (Serial1.available()>0) {
        DataBLE = Serial1.readStringUntil('\n');
        //stringCompleteJoystick = true; // relict from manually searching for '\n', now using readStringUntil
        DEBUG_SERIAL.println(DataBLE);
      }

      //DEBUG_SERIAL.println("loop");

      /*
      if(stringCompleteJoystick){
        for(int p = 0; p < 3 ; p++){
          joystickAngles[p] = getValue(inputStringJoystick, ',', p).toFloat();
        } 
        //Reset string
        DataBLE = "";
        stringCompleteJoystick = false;
      }

      DEBUG_SERIAL.print(joystickAngles[0]); DEBUG_SERIAL.print(" "); DEBUG_SERIAL.print(joystickAngles[1]); DEBUG_SERIAL.print(" "); DEBUG_SERIAL.println(joystickAngles[2]);*/

      /*while (Serial2.available()>0) {
      char inChar = (char)Serial2.read();
      Serial.println("while");
      Serial.println(String(inChar));
      }*/
      //Serial.println("loop");
      //Serial.println(String(joystickAngles[0]) + "," + String(joystickAngles[1]) + "," + String(joystickAngles[2]));

    
    }

  } 