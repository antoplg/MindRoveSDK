#include "main.h"
#include <Arduino.h>
#include <kinematics.h>

// --- DEFINE KINEMATICS ---
BLA::Matrix<3, 3> jacobian_matrix = {
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0};

// --- DEFINE MODULE VARIABLES ---
BLA::Matrix<3> ee_velocities = {0.0, 0.0, 0.0};
BLA::Matrix<3> joint_velocities = {0.0, 0.0, 0.0};

// --- DEFINE GRIPPER VARIABLES ---
int gripperVel = 0;
int presentLoad = 0;


void setup() {
  // --- INIT SERIAL COMMUNICATION TO ESP BLE MODULE ---
  Serial.begin(115200);
  Serial.flush();  

  // --- INIT DYNAMIXEL ---
  // reserve 200 bytes for the inputString:
  inputStringJoystickBLE.reserve(200);
  inputStringDiaphragmBLE.reserve(200);

  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  //DEBUG_SERIAL.begin(9600);
  //while(!DEBUG_SERIAL); // Wait for the port to open

  dxl.begin(115200);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  
  // Turn off torque when configuring items in EEPROM area
  for(int i = 0; i < DXL_ID_CNT; i++) {
    dxl.torqueOff(DXL_ID[i]);
  }
  // Gripper
  dxl.torqueOff(GRIPPER_DXL_ID);

  // --- SYNC READ POSITION ---
  // Fill the members of structure to syncRead using external user packet buffer
  sr_infos.packet.p_buf = user_pkt_buf;
  sr_infos.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos.packet.is_completed = false;
  sr_infos.addr = SR_START_ADDR;
  sr_infos.addr_length = SR_ADDR_LEN;
  sr_infos.p_xels = info_xels_sr;
  sr_infos.xel_count = 0;

  // Module 
  for(int i = 0; i < DXL_ID_CNT; i++) {
    info_xels_sr[i].id = DXL_ID[i];
    info_xels_sr[i].p_recv_buf = (uint8_t*)&sr_data[i];
    sr_infos.xel_count++;
  }
  // Gripper
  info_xels_sr[3].id = GRIPPER_DXL_ID;
  info_xels_sr[3].p_recv_buf = (uint8_t*)&sr_data[3];
  sr_infos.xel_count++;

  sr_infos.is_info_changed = true;

  // --- SYNC WRITE VELOCITY ---
  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos.packet.p_buf = nullptr;
  sw_infos.packet.is_completed = false;
  sw_infos.addr = SW_START_ADDR;
  sw_infos.addr_length = SW_ADDR_LEN;
  sw_infos.p_xels = info_xels_sw;
  sw_infos.xel_count = 0;

  // Module
  for(int i = 0; i < DXL_ID_CNT; i++) {
    sw_data[i].goal_velocity = 0;  // sw_data[0].goal_velocity = 0;
    info_xels_sw[i].id = DXL_ID[i];
    info_xels_sw[i].p_data = (uint8_t*)&sw_data[i].goal_velocity;
    sw_infos.xel_count++;
  }
  // Gripper
  sw_data[3].goal_velocity = 0;  // sw_data[0].goal_velocity = 0;
  info_xels_sw[3].id = GRIPPER_DXL_ID;
  info_xels_sw[3].p_data = (uint8_t*)&sw_data[3].goal_velocity;
  sw_infos.xel_count++;

  sw_infos.is_info_changed = true;

  // --- MOTOR SETTINGS --- 
  // Module
  for(int i = 0; i < DXL_ID_CNT; i++) {
    // to reduce the dead zone
    dxl.writeControlTableItem(MOVING_THRESHOLD, DXL_ID[i], 2); // 0.229 rpm	0 ~ 1,023	
    dxl.writeControlTableItem(VELOCITY_LIMIT, DXL_ID[i], max_motor_speed); // 0.229 rpm	0 ~ 1,023	
    dxl.writeControlTableItem(CURRENT_LIMIT, DXL_ID[i], 1450);
    // Turn off the safety shutdown when reaching the stall torque (turning on: 00110101)
    //dxl.writeControlTableItem(SHUTDOWN, DXL_ID[i], 00010101);
    dxl.setOperatingMode(DXL_ID[i], OP_VELOCITY);
  }
  // Gripper
  dxl.writeControlTableItem(MOVING_THRESHOLD, GRIPPER_DXL_ID, 2); // 0.229 rpm	0 ~ 1,023	
  dxl.writeControlTableItem(VELOCITY_LIMIT, GRIPPER_DXL_ID, max_motor_speed); // 0.229 rpm	0 ~ 1,023	
  //dxl.writeControlTableItem(CURRENT_LIMIT, GRIPPER_DXL_ID, 1450);
  // Turn off the safety shutdown when reaching the stall torque (turning on: 00110101)
  //dxl.writeControlTableItem(SHUTDOWN, DXL_ID[i], 00010101);
  dxl.setOperatingMode(GRIPPER_DXL_ID, OP_VELOCITY);

  // Initialize the positions and velocities 
  for(int i = 0; i < 3; i++) { // 3 is number of legs per module
    module1.setpointLegAngle[i] = min_position;  
    //module2.setpointLegAngle[i] = min_position;
    module1.setpointLegAngleVelocity[i] = 0;
    //module2.setpointLegAngleVelocity[i] = 0;
  }

  // --- SETUP PID ---
  module1.setupPIDs();
  //module2.setupPIDs();

  // Position control  PID
  float kPPosBase = 350, kIPosBase = 350, kDPosBase = 3;  // double kPPosBase = 450, kIPosBase = 450, kDPosBase = 2;
  float kPPos2 = 200, kIPos2 = 300, kDPos2 = 3;  // double kPPos2 = 350, kIPos2 = 400, kDPos2 = 2;
  
  float kPVelBase = 400, kIVelBase = 600;  // double kPVelBase = 100, kIVelBase = 100;
  float kPVel2 = 800, kIVel2 = 1200;  // double kPVel2 = 100, kIVel2 = 100;

  float kPVelGripper = 100, kIVelGripper = 100; // double kPVelGripper = 100, kIVelGripper = 100;

  module1.position1PID->SetTunings(kPPosBase, kIPosBase, kDPosBase); // p, i, d
  module1.position2PID->SetTunings(kPPosBase, kIPosBase, kDPosBase);
  module1.position3PID->SetTunings(kPPosBase, kIPosBase, kDPosBase); // p,i,d
  //module2.position1PID->SetTunings(kPPos2, kIPos2, kDPos2);
  //module2.position2PID->SetTunings(kPPos2, kIPos2, kDPos2);
  //module2.position3PID->SetTunings(kPPos2, kIPos2, kDPos2); // p,i,d

  // --- TURN ON MOTOR TORQUE ---
  // Module
  for(int i = 0; i < DXL_ID_CNT; i++) {
    dxl.torqueOn(DXL_ID[i]);
  }
  // Gripper
  dxl.torqueOn(GRIPPER_DXL_ID);

  // --- WRTIE PID GAINS ---
  // Module
  dxl.writeControlTableItem(VELOCITY_P_GAIN, module1.motorID1, kPVelBase);  dxl.writeControlTableItem(VELOCITY_I_GAIN, module1.motorID1, kIVelBase);
  dxl.writeControlTableItem(VELOCITY_P_GAIN, module1.motorID2, kPVelBase);  dxl.writeControlTableItem(VELOCITY_I_GAIN, module1.motorID2, kIVelBase);
  dxl.writeControlTableItem(VELOCITY_P_GAIN, module1.motorID3, kPVelBase);  dxl.writeControlTableItem(VELOCITY_I_GAIN, module1.motorID3, kIVelBase);
  //dxl.writeControlTableItem(VELOCITY_P_GAIN, module2.motorID1, kPVel2);  dxl.writeControlTableItem(VELOCITY_I_GAIN, module2.motorID1, kIVel2);
  //dxl.writeControlTableItem(VELOCITY_P_GAIN, module2.motorID2, kPVel2);  dxl.writeControlTableItem(VELOCITY_I_GAIN, module2.motorID2, kIVel2);
  //dxl.writeControlTableItem(VELOCITY_P_GAIN, module2.motorID3, kPVel2);  dxl.writeControlTableItem(VELOCITY_I_GAIN, module2.motorID3, kIVel2);
  
  // Gripper
  dxl.writeControlTableItem(VELOCITY_P_GAIN, GRIPPER_DXL_ID, kPVelGripper);  dxl.writeControlTableItem(VELOCITY_I_GAIN, GRIPPER_DXL_ID, kIVelGripper);

  Timer_01.initialize(postionContolLoop);
  Timer_02.initialize(printLoop);

  startTime = (float)millis(); // [ms]

}

// SoftwareSerial is half duplex
void loop() {
  // --- POSITION CONTROL LOOP ---
  if(Timer_01.Tick()){ 
    //previousTime = currentTime; // [ms]
    currentTime = (float)millis() - startTime; // [ms]

    // --- SYNC READ DYNAMIXEL --- 
    uint8_t recv_cnt;
    recv_cnt = dxl.syncRead(&sr_infos);
    //DEBUG_SERIAL.print("recv_cnt: "); DEBUG_SERIAL.println(recv_cnt);

    // Changed here because of gripper
    if(recv_cnt == DXL_ID_CNT_TOTAL){
      module1.assignLegAngles(poseUnit*sr_data[0].present_position,
      poseUnit*sr_data[1].present_position, poseUnit*sr_data[2].present_position);

      //module2.assignLegAngles(poseUnit*sr_data[3].present_position,
      //poseUnit*sr_data[4].present_position, poseUnit*sr_data[5].present_position);
    }

    // --- JOYSTICK CONTROL - 0 --- 
    if(controlMode==0){  
      // read the joystick values from ESP BLE module
      while (Serial1.available()>0) {
        inputStringJoystickBLE = Serial1.readStringUntil('\n');
        stringCompleteJoystick = true; // relict from manually searching for '\n', now using readStringUntil
      }

      if(stringCompleteJoystick){
        for(int p = 0; p < 3 ; p++){
          joystickAngles[p] = getValue(inputStringJoystickBLE, ',', p).toFloat();
        } 
        //Reset string
        inputStringJoystickBLE = "";
        stringCompleteJoystick = false;
      }

      for(int i = 0; i < 3; i++) { // 3 is number of legs per module
        module1.setpointLegVelocityFF[i] = 0;
        //module2.setpointLegVelocityFF[i] = 0; //0; //
        
        module1.setpointLegAngle[i] = joystickAngles[i];
        //module2.setpointLegAngle[i] = joystickAngles[i]; 
      }

      // --- COMPUTE POSITON CONTROLLER ---
      module1.computePositionController(); // setpointLegVelocityFB is pid output, moduleLegAngle input, setpointLegAngle setpoint
      //module2.computePositionController(); // setpointLegVelocityFB is pid output, moduleLegAngle input, setpointLegAngle setpoint
    }

    // --- HUMAN BODY CONTROL - DIAPHRAGM ---
    else if(controlMode==1){  
      // read the diaphragm sensor values from ESP BLE module
      while (Serial1.available()>0) {
        inputStringDiaphragmBLE = Serial1.readStringUntil('\n');
        stringCompleteDiaphragm = true; // relict from manually searching for '\n', now using readStringUntil
      }
      if(stringCompleteDiaphragm){
        diaphragmResistance = inputStringDiaphragmBLE.toDouble();
        //Reset string
        inputStringDiaphragmBLE = "";
        stringCompleteDiaphragm = false;
      }

      float resistanceScaled = (diaphragmResistance - diaphragmRESTResistance) / (diaphragmRESTHIGHResistance - diaphragmRESTLOWResistance);
      // compute jacobian
      float motor_angles[3] = {module1.moduleLegAngle[0], module1.moduleLegAngle[1], module1.moduleLegAngle[2]};

      numericalJacobian(forwardKinematicsTwoModules, motor_angles, jacobian_matrix);
      
      // IF DIAPHRAGM NORMAL
      if (resistanceScaled > -0.5 && resistanceScaled < 0.5){
        // set velocity to zero
        for(int i = 0; i < 3; i++) { // 3 is number of legs per module
          module1.setpointLegVelocityFF[i] = 0;
          //module2.setpointLegVelocityFF[i] = 0; 
          module1.setpointLegVelocityFB[i] = 0;
          //module2.setpointLegVelocityFB[i] = 0; 
        }
      }
      else{
        // compute end effector velocities
        ee_velocities(0) = - 200 * resistanceScaled;  // height velocity in [mm/s] defined by diaphragm sensor
        ee_velocities(1) = 0;
        ee_velocities(2) = 0;
        joint_velocities = Inverse(jacobian_matrix) * ee_velocities;

        for(int i = 0; i < 3; i++) { // 3 is number of legs per module
          module1.setpointLegVelocityFF[i] = joint_velocities(i);
          //module2.setpointLegVelocityFF[i] = joint_velocities(i); 
          module1.setpointLegVelocityFB[i] = 0;
          //module2.setpointLegVelocityFB[i] = 0; 
        }
      } 
    }

    // --- HUMAN BODY CONTROL - DIPHRAGM & SOCKS ---
    else if(controlMode==2){ 
      // read the diaphragm sensor values from ESP BLE module
      while (Serial1.available()>0) {
        inputStringHBCBLE = Serial1.readStringUntil('\n');
        stringCompleteHBC = true; // relict from manually searching for '\n', now using readStringUntil
      }
      if(stringCompleteHBC){
        for(int p = 0; p < 5 ; p++){
          dataHBC[p] = getValue(inputStringHBCBLE, ',', p).toFloat();
        }
        //Reset string
        inputStringDiaphragmBLE = "";
        stringCompleteDiaphragm = false;
      }
      
      //DEBUG_SERIAL.println(inputStringHBCBLE);

      //DEBUG_SERIAL.print(dataHBC[0]); DEBUG_SERIAL.print(" "); DEBUG_SERIAL.print(dataHBC[2]); DEBUG_SERIAL.print(" "); DEBUG_SERIAL.print(dataHBC[4]); DEBUG_SERIAL.println(" ");
      // --- SCALE SENSOR DATA ---
      // Diaphragm sensor
      float diaphragm_scaled = (dataHBC[0] - diaphragmRESTResistance) / (diaphragmRESTHIGHResistance - diaphragmRESTLOWResistance);
      // Left sock sensor
      float left_sock_scaled = (dataHBC[2] - 550) / 300; 
      // Right bottom sock sensor
      float right_sock_scaled = (dataHBC[4] - 550) / 300;

      DEBUG_SERIAL.print(diaphragm_scaled); DEBUG_SERIAL.print(" ");
      DEBUG_SERIAL.print(left_sock_scaled); DEBUG_SERIAL.print(" ");
      DEBUG_SERIAL.print(right_sock_scaled); DEBUG_SERIAL.println(" ");

      // compute jacobian
      float motor_angles[3] = {module1.moduleLegAngle[0], module1.moduleLegAngle[1], module1.moduleLegAngle[2]};

      numericalJacobian(forwardKinematicsTwoModules, motor_angles, jacobian_matrix);
      
      // --- DIAPHRAGM INPUT ---
      if (diaphragm_scaled > -0.5 && diaphragm_scaled < 0.5){
        ee_velocities(0) = 0;  // height velocity in [mm/s] defined by diaphragm sensor
      }
      else{
        ee_velocities(0) = - 200 * diaphragm_scaled;  // height velocity in [mm/s] defined by diaphragm sensor
      }

      // --- LEFT SOCK INPUT ---
      if (left_sock_scaled > -0.5 && left_sock_scaled < 0.5){
        ee_velocities(1) = 0; // roll angular velocity in [rad/s] defined by left sock sensor
      }
      else if (left_sock_scaled < -0.5)
      {
        ee_velocities(1) = left_sock_scaled; // roll angular velocity in [rad/s] defined by left sock sensor
      }
      else if (left_sock_scaled > 0.5)
      {
        ee_velocities(1) = left_sock_scaled; // roll angular velocity in [rad/s] defined by left sock sensor
      }
      else{
        ee_velocities(1) = 0;
      }
      
      // --- RIGHT SOCK INPUT ---
      if (right_sock_scaled > -0.5 && right_sock_scaled < 0.5){
        ee_velocities(2) = 0; // roll angular velocity in [rad/s] defined by left sock sensor
      }
      else if (right_sock_scaled < -0.5)
      {
        ee_velocities(2) = right_sock_scaled * 0.5; // roll angular velocity in [rad/s] defined by left sock sensor
      }
      else if (right_sock_scaled > 0.5)
      {
        ee_velocities(2) = right_sock_scaled * 0.5; // roll angular velocity in [rad/s] defined by left sock sensor
      }
      else{
        ee_velocities(2) = 0;
      }

      //DEBUG_SERIAL.print(ee_velocities(0)); DEBUG_SERIAL.print(" "); DEBUG_SERIAL.print(ee_velocities(1)); DEBUG_SERIAL.print(" "); DEBUG_SERIAL.print(ee_velocities(2)); DEBUG_SERIAL.println(" ");
      // Compute the joint velocities with jacobian
      joint_velocities = Inverse(jacobian_matrix) * ee_velocities;

      for(int i = 0; i < 3; i++) { // 3 is number of legs per module
        module1.setpointLegVelocityFF[i] = joint_velocities(i);
        //module2.setpointLegVelocityFF[i] = joint_velocities(i); 
        module1.setpointLegVelocityFB[i] = 0;
        //module2.setpointLegVelocityFB[i] = 0; 
      } 
    }

    // --- POSITON CONTROL ---
    else if(controlMode==3){ 
      float timeSwing = 2000;
      if(currentTime<timeSwing){
        module1.setpointLegAngle[0] = 0.75; // [rad]
        module1.setpointLegAngle[1] = 0.08;
        module1.setpointLegAngle[2] = 0.90;

        //module2.setpointLegAngle[0] = 0.75; // [rad]
        //module2.setpointLegAngle[1] = 0.08;
        //module2.setpointLegAngle[2] = 0.90;
        }
      
      else if (currentTime<2*timeSwing) {
        module1.setpointLegAngle[0] = 0.64; // [rad]
        module1.setpointLegAngle[1] = 0.96;
        module1.setpointLegAngle[2] = 0.2;

        //module2.setpointLegAngle[0] = 0.61; // [rad]
        //module2.setpointLegAngle[1] = 0.97;
        //module2.setpointLegAngle[2] = 0.21;
      }
      else{
        startTime = (double)millis(); // [ms]
      }
      DEBUG_SERIAL.print(module1.moduleLegAngle[0]); DEBUG_SERIAL.print(" ");
      DEBUG_SERIAL.print(module1.setpointLegAngle[0]); DEBUG_SERIAL.print(" ");
      DEBUG_SERIAL.print(module1.moduleLegAngle[1]); DEBUG_SERIAL.print(" ");
      DEBUG_SERIAL.print(module1.setpointLegAngle[1]); DEBUG_SERIAL.print(" ");
      DEBUG_SERIAL.print(module1.moduleLegAngle[2]); DEBUG_SERIAL.print(" ");
      DEBUG_SERIAL.print(module1.setpointLegAngle[2]); DEBUG_SERIAL.println(" ");

      // --- COMPUTE POSITION CONTROLLER ---
      module1.computePositionController(); // setpointLegVelocityFB is pid output, moduleLegAngle input, setpointLegAngle setpoint
      //module2.computePositionController(); // setpointLegVelocityFB is pid output, moduleLegAngle input, setpointLegAngle setpoint
    }

    // --- EMG - ONE MODULE + GRIPPER CONTROL ---
    else if(controlMode==4){ 

      if (Serial.available() >= 8 ) { // Ensure enough bytes are available

        // Create a buffer to store the received data
        uint8_t buffer[8];

        // Read the entire vector into the buffer
        for (int i = 0; i < 8; i++) {
          buffer[i] = Serial.read();
        }

        // Invio delle variabili modificate indietro a Simulink
        x1 = bytesToFloat(buffer, 0);
        x2 = bytesToFloat(buffer, 4);
        Serial.print(x1);
        Serial.print(x2); //*/

        // Control LEDs based on x1 and x2
        digitalWrite(ledPins[0], x1 > 0.5 ? HIGH : LOW);
        digitalWrite(ledPins[1], x2 > 0.5 ? HIGH : LOW);

        digitalWrite(ledPins[2], LOW);

      } else {
        digitalWrite(ledPins[2], HIGH);  // Turn on LED 3
      } 

      // Read current load from gripper
      presentLoad = dxl.readControlTableItem(PRESENT_LOAD, GRIPPER_DXL_ID);
      
      // TODO ADD READING OF EMG SIGNAL HERE (RESULT IS GRIPPER VELOCITY)
      gripperVel = 0; 

      // Check torque limit for gripper - TODO CHANGE RIGHT SOCK SCALED TO EMG SIGNAL FOR CLOSING DIRECTION
      if (presentLoad > 160 && right_sock_scaled < -0.5) { 
        gripperVel = 0;
      }
      // TODO write velocity to gripper (serial - gripperVel)

      // TODO add min/max limit for gripper (with max_motor_speed)
      
      // Define velocities for module (for now 0, later controlled by EMG)
      ee_velocities(0) = 0; // height velocity in [mm/s] defined by EMG sensor
      ee_velocities(1) = 0; // roll 
      ee_velocities(2) = 0; // pitch
      // Compute the joint velocities with jacobian
      joint_velocities = Inverse(jacobian_matrix) * ee_velocities;

      // Set fixed position for module (only for now, later controlled by EMG)
      module1.setpointLegAngle[0] = 0.5; // [rad]
      module1.setpointLegAngle[1] = 0.5;
      module1.setpointLegAngle[2] = 0.5;
      // Compute the feedback term of the velocity controller for module
      module1.computePositionController();

      // Compute the feedforward term of the velocity controller for module
      for(int i = 0; i < 3; i++) { 
        //module1.setpointLegVelocityFF[i] = joint_velocities(i);
        //module2.setpointLegVelocityFF[i] = joint_velocities(i); 
        //module1.setpointLegVelocityFB[i] = 0;
        //module2.setpointLegVelocityFB[i] = 0; 
      } 

    }
       

    // --- COMPUTE VELOCITY CONTROLLER ---
    for(int i = 0; i < 3; i++) {
      // Feedback and feedforward are combined
      module1.setpointLegAngleVelocity[i] = module1.setpointLegVelocityFB[i] + module1.setpointLegVelocityFF[i]/0.024; // [raw]
      //module2.setpointLegAngleVelocity[i] = module2.setpointLegVelocityFB[i] + module2.setpointLegVelocityFF[i]/0.024; // [raw]
    }
      
    // MIN-MAX Velocity Check
    for(int i = 0; i < 3; i++) {
      if (module1.setpointLegAngleVelocity[i] > max_motor_speed) module1.setpointLegAngleVelocity[i]= max_motor_speed; // [raw]
      else if (module1.setpointLegAngleVelocity[i] < min_motor_speed) module1.setpointLegAngleVelocity[i]= min_motor_speed; // [raw]

      //if (module2.setpointLegAngleVelocity[i] > max_motor_speed) module2.setpointLegAngleVelocity[i]= max_motor_speed; // [raw]
      //else if (module2.setpointLegAngleVelocity[i] < min_motor_speed) module2.setpointLegAngleVelocity[i]= min_motor_speed; // [raw]
    }

    // MIN-MAX Position Check
    for(int i = 0; i < 3; i++) {
      // module 1
      if ((module1.moduleLegAngle[i] > max_position)&&(module1.setpointLegAngleVelocity[i]>0))
        module1.setpointLegAngleVelocity[i] = 0;
      else if((module1.moduleLegAngle[i] < min_position)&&(module1.setpointLegAngleVelocity[i]<0))
        module1.setpointLegAngleVelocity[i] = 0;        
      // module 2
      //if ((module2.moduleLegAngle[i] > max_position)&&(module2.setpointLegAngleVelocity[i]>0))
      //  module2.setpointLegAngleVelocity[i] = 0;
      //else if((module2.moduleLegAngle[i] < min_position)&&(module2.setpointLegAngleVelocity[i]<0))
      //  module2.setpointLegAngleVelocity[i] = 0;
    }

    // --- SYNC WRITE MOTOR COMMANDS ---
    // Module - Insert a new Goal Velocity to the SyncWrite Packet
    for(int i = 0; i < 3; i++) {
      sw_data[i].goal_velocity = module1.setpointLegAngleVelocity[i]; // [UNIT_RAW]
      //sw_data[i+3].goal_velocity = module2.setpointLegAngleVelocity[i]; // [UNIT_RAW]
    }
    // Gripper - Insert a new Goal Velocity to the SyncWrite Packet
    sw_data[3].goal_velocity = gripperVel;

    // Update the SyncWrite packet status
    sw_infos.is_info_changed = true;
    dxl.syncWrite(&sw_infos);

    }

    // --- SERIAL OUTPUT ---
    if(Timer_02.Tick()){  
      if (print){
      if(controlMode==0){            
        DEBUG_SERIAL.print(module1.moduleLegAngle[0]); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.setpointLegAngle[0]); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.moduleLegAngle[1]); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.setpointLegAngle[1]); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.moduleLegAngle[2]); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.setpointLegAngle[2]); DEBUG_SERIAL.print(" ");

        /*DEBUG_SERIAL.print(module2.moduleLegAngle[0]); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module2.setpointLegAngle[0]); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module2.moduleLegAngle[1]); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module2.setpointLegAngle[1]); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module2.moduleLegAngle[2]); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module2.setpointLegAngle[2]); DEBUG_SERIAL.println(" ");*/
 
      }
      else if(controlMode==1){
        DEBUG_SERIAL.print(module1.moduleLegAnglularVelocity[0]); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.setpointLegAngleVelocity[0]); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.moduleLegAnglularVelocity[1]); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.setpointLegAngleVelocity[1]); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.moduleLegAnglularVelocity[2]); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.println(module1.setpointLegAngleVelocity[2]); DEBUG_SERIAL.print(" ");
        //DEBUG_SERIAL.print(module2.moduleLegAnglularVelocity[0]); DEBUG_SERIAL.print(" ");
        //DEBUG_SERIAL.print(module2.setpointLegAngleVelocity[0]); DEBUG_SERIAL.print(" ");
        //DEBUG_SERIAL.print(module2.moduleLegAnglularVelocity[1]); DEBUG_SERIAL.print(" ");
        //DEBUG_SERIAL.print(module2.setpointLegAngleVelocity[1]); DEBUG_SERIAL.print(" ");
        //DEBUG_SERIAL.print(module2.moduleLegAnglularVelocity[2]); DEBUG_SERIAL.print(" ");
        //DEBUG_SERIAL.print(module2.setpointLegAngleVelocity[2]); DEBUG_SERIAL.println(" ");
      }
      else if(controlMode==2){
        DEBUG_SERIAL.print(module1.moduleLegAnglularVelocity[0]); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.setpointLegAngleVelocity[0]); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.moduleLegAnglularVelocity[1]); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.setpointLegAngleVelocity[1]); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.moduleLegAnglularVelocity[2]); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.println(module1.setpointLegAngleVelocity[2]); //DEBUG_SERIAL.print(" ");
        //DEBUG_SERIAL.print(module2.moduleLegAnglularVelocity[0]); DEBUG_SERIAL.print(" ");
        //DEBUG_SERIAL.print(module2.setpointLegAngleVelocity[0]); DEBUG_SERIAL.print(" ");
        //DEBUG_SERIAL.print(module2.moduleLegAnglularVelocity[1]); DEBUG_SERIAL.print(" ");
        //DEBUG_SERIAL.print(module2.setpointLegAngleVelocity[1]); DEBUG_SERIAL.print(" ");
        //DEBUG_SERIAL.print(module2.moduleLegAnglularVelocity[2]); DEBUG_SERIAL.print(" ");
        //DEBUG_SERIAL.print(module2.setpointLegAngleVelocity[2]); DEBUG_SERIAL.println(" ");
      }
      else if(controlMode==3){            
          DEBUG_SERIAL.print(module1.moduleLegAngle[0]); DEBUG_SERIAL.print(" ");
          DEBUG_SERIAL.print(module1.setpointLegAngle[0]); DEBUG_SERIAL.print(" ");
          DEBUG_SERIAL.print(module1.moduleLegAngle[1]); DEBUG_SERIAL.print(" ");
          DEBUG_SERIAL.print(module1.setpointLegAngle[1]); DEBUG_SERIAL.print(" ");
          DEBUG_SERIAL.print(module1.moduleLegAngle[2]); DEBUG_SERIAL.print(" ");
          DEBUG_SERIAL.print(module1.setpointLegAngle[2]); DEBUG_SERIAL.println(" ");
          

          /*DEBUG_SERIAL.print(module2.moduleLegAngle[0]); DEBUG_SERIAL.print(" ");
          DEBUG_SERIAL.print(module2.setpointLegAngle[0]); DEBUG_SERIAL.print(" ");
          DEBUG_SERIAL.print(module2.moduleLegAngle[1]); DEBUG_SERIAL.print(" ");
          DEBUG_SERIAL.print(module2.setpointLegAngle[1]); DEBUG_SERIAL.print(" ");
          DEBUG_SERIAL.print(module2.moduleLegAngle[2]); DEBUG_SERIAL.print(" ");
          DEBUG_SERIAL.print(module2.setpointLegAngle[2]); DEBUG_SERIAL.println(" ");*/
  
        }
      else if(controlMode==4){            
        DEBUG_SERIAL.print(module1.moduleLegAngle[0]); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.setpointLegAngle[0]); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.moduleLegAngle[1]); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.setpointLegAngle[1]); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.moduleLegAngle[2]); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.setpointLegAngle[2]); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(gripperVel); DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(presentLoad); DEBUG_SERIAL.println(" ");

      }
      }
    }
}