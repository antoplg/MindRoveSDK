#include "main.h"
#include <Arduino.h>
#include <math.h> 
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
int gripperVel    = 0;
int presentLoad   = 0;

// --- DEFINE SERIAL VARIABLES ---
bool fuzzy_control_enabled    = true;
bool inverted                 = false;
double flexext                = 0.0;
double cocon1                 = 0.0;
double cocon2                 = 0.0;
String inputString            = "";         // a String to hold incoming data
float received_val            = 0.0;
int direction                 = +1;
bool stringComplete           = false;      // whether the string is complete

// --- DEFINE CALIBRATION VARIABLES ---
float get_phase_ms(float v) {
  if (v >= 0.9f) return 1000;
  if (v >= 0.8f) return 1050;
  if (v >= 0.7f) return 1150;
  if (v >= 0.6f) return 1300;
  if (v >= 0.5f) return 1500;
  if (v >= 0.4f) return 1750;
  if (v >= 0.3f) return 2100;
  if (v >= 0.2f) return 2750;
  if (v >= 0.1f) return 5250;
  return 6500;
}

uint32_t phase_ms = 1500;     // default
enum CalibrationState {
        IDLE,
        MOVING_GRIPPER_FORWARD,
        MOVING_GRIPPER_BACKWARD,
        MOVING_SRL_FORWARD,
        MOVING_SRL_BACKWARD
      };
CalibrationState cal_state    = IDLE;
unsigned long lastActiveTime  = 0;
unsigned long t_start         = 0;
char header                   = '0'; 


// --- DEFINE FUZZY VARIABLES ---

// Soglie regolabili
const float DIFF_DEADBAND        = 0.10f; // gripper attivo solo oltre questa differenza
const float DIFF_DEADBAND_WIDTH  = 0.15f;
const float CO_MIN               = 0.15f; // entrambi > CO_MIN
const float CO_GATE_WIDTH        = 0.20f;
const float DIFF_MAX_FOR_SRL     = 0.15f; // SRL richiede differenza piccola
const float DIFF_SRL_WIDTH       = 0.15f;

// soglie per i 3 livelli di velocità del gripper (in base a |d|)
const float G_SMALL              = 0.15f;
const float G_MED                = 0.50f;
const float G_LARGE              = 0.75f;

// --- FUZZY HELPERS ---
static inline float clamp01(float x){ return x < 0 ? 0 : (x > 1 ? 1 : x); }

// Triangolo
static inline float tri(float x, float a, float b, float c){
  if (x <= a || x >= c) return 0.f;
  if (x == b) return 1.f;
  return (x < b) ? ((x - a) / (b - a)) : ((c - x) / (c - b));
}

// Spalla crescente (grade) 0→1
static inline float grade(float x, float a, float b){
  if (x <= a) return 0.f;
  if (x >= b) return 1.f;
  return (x - a) / (b - a);
}

// Spalla decrescente (rgrade) 1→0
static inline float rgrade(float x, float a, float b){
  if (x <= a) return 1.f;
  if (x >= b) return 0.f;
  return (b - x) / (b - a);
}

void setup() {
  // --- INIT SERIAL COMMUNICATION TO ESP BLE MODULE ---
  DEBUG_SERIAL.begin(115200);
  DEBUG_SERIAL.flush();

  // --- INIT DYNAMIXEL ---
  // reserve 200 bytes for the inputString:
  inputStringJoystickBLE.reserve(200);
  inputStringDiaphragmBLE.reserve(200);

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
  sr_infos.packet.p_buf         = user_pkt_buf;
  sr_infos.packet.buf_capacity  = user_pkt_buf_cap;
  sr_infos.packet.is_completed  = false;
  sr_infos.addr                 = SR_START_ADDR;
  sr_infos.addr_length          = SR_ADDR_LEN;
  sr_infos.p_xels               = info_xels_sr;
  sr_infos.xel_count            = 0;

  // Module
  for(int i = 0; i < DXL_ID_CNT; i++) {
    info_xels_sr[i].id          = DXL_ID[i];
    info_xels_sr[i].p_recv_buf  = (uint8_t*)&sr_data[i];
    sr_infos.xel_count++;
  }
  // Gripper
  info_xels_sr[3].id          = GRIPPER_DXL_ID;
  info_xels_sr[3].p_recv_buf  = (uint8_t*)&sr_data[3];
  sr_infos.xel_count++;

  sr_infos.is_info_changed    = true;

  // --- SYNC WRITE VELOCITY ---
  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos.packet.p_buf         = nullptr;
  sw_infos.packet.is_completed  = false;
  sw_infos.addr                 = SW_START_ADDR;
  sw_infos.addr_length          = SW_ADDR_LEN;
  sw_infos.p_xels               = info_xels_sw;
  sw_infos.xel_count            = 0;

  // Module
  for(int i = 0; i < DXL_ID_CNT; i++) {
    sw_data[i].goal_velocity  = 0;  // sw_data[0].goal_velocity = 0;
    info_xels_sw[i].id        = DXL_ID[i];
    info_xels_sw[i].p_data    = (uint8_t*)&sw_data[i].goal_velocity;
    sw_infos.xel_count++;
  }
  // Gripper
  sw_data[3].goal_velocity    = 0;  // sw_data[0].goal_velocity = 0;
  info_xels_sw[3].id          = GRIPPER_DXL_ID;
  info_xels_sw[3].p_data      = (uint8_t*)&sw_data[3].goal_velocity;
  sw_infos.xel_count++;

  sw_infos.is_info_changed    = true;

  // --- MOTOR SETTINGS ---
  // Module
  for(int i = 0; i < DXL_ID_CNT; i++) {
    // to reduce the dead zone
    dxl.writeControlTableItem(MOVING_THRESHOLD, DXL_ID[i], 2);              // 0.229 rpm	0 ~ 1,023
    dxl.writeControlTableItem(VELOCITY_LIMIT, DXL_ID[i], max_motor_speed);  // 0.229 rpm	0 ~ 1,023
    dxl.writeControlTableItem(CURRENT_LIMIT, DXL_ID[i], 1450);
    dxl.setOperatingMode(DXL_ID[i], OP_VELOCITY);
  }
  // Gripper
  dxl.writeControlTableItem(MOVING_THRESHOLD, GRIPPER_DXL_ID, 2);               // 0.229 rpm	0 ~ 1,023
  dxl.writeControlTableItem(VELOCITY_LIMIT, GRIPPER_DXL_ID, max_motor_speed);   // 0.229 rpm	0 ~ 1,023
  dxl.setOperatingMode(GRIPPER_DXL_ID, OP_VELOCITY);

  // Initialize the positions and velocities
  for(int i = 0; i < 3; i++) { // 3 is number of legs per module
    module1.setpointLegAngle[i]         = min_position;
    module1.setpointLegAngleVelocity[i] = 0;
  }

  // --- SETUP PID ---
  module1.setupPIDs();

  // Position control  PID
  float kPPosBase     = 350, kIPosBase      = 350, kDPosBase    = 3;  // double kPPosBase = 450, kIPosBase = 450, kDPosBase = 2;
  float kPVelBase     = 400, kIVelBase      = 600;                    // double kPVelBase = 100, kIVelBase = 100;
  float kPVelGripper  = 100, kIVelGripper   = 100;                    // double kPVelGripper = 100, kIVelGripper = 100;

  module1.position1PID->SetTunings(kPPosBase, kIPosBase, kDPosBase);  // p, i, d
  module1.position2PID->SetTunings(kPPosBase, kIPosBase, kDPosBase);  // p, i, d
  module1.position3PID->SetTunings(kPPosBase, kIPosBase, kDPosBase);  // p, i, d

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
    currentTime = (float)millis() - startTime; // [ms]

    // --- SYNC READ DYNAMIXEL ---
    uint8_t recv_cnt;
    recv_cnt = dxl.syncRead(&sr_infos);

    // Changed here because of gripper
    if(recv_cnt == DXL_ID_CNT_TOTAL){
      module1.assignLegAngles(poseUnit*sr_data[0].present_position,
      poseUnit*sr_data[1].present_position, poseUnit*sr_data[2].present_position);
    }

    // --- EMG - ONE MODULE + GRIPPER CONTROL ---
    
    if (DEBUG_SERIAL.available() > 0) {
      header = DEBUG_SERIAL.peek();  // peek non consuma il byte
    }
  else if (DEBUG_SERIAL.available() <= 0 && !(header == 'G' || header == 'S')) {
      header = '0';
    }

    if (header == 'G' || header == 'S') {
      if ((int)DEBUG_SERIAL.available() >= (int)(1 + sizeof(float))) {
        DEBUG_SERIAL.read(); // consuma il carattere
        DEBUG_SERIAL.readBytes((char*)&received_val, sizeof(float));
        // → qui fai partire la tua macchina a stati di calibrazione
        if (cal_state == IDLE) {
          if (header == 'G') {
            cal_state         = MOVING_GRIPPER_FORWARD;
            gripperVel        = + received_val * max_motor_speed;
            //phase_ms          = 1098.0f + 6920.0f * powf(1.0f - received_val, 4.13f);
            phase_ms          = get_phase_ms(received_val);
            t_start           = millis();  // AGGIUNGI QUESTO
          } else if (header == 'S') {
            cal_state         = MOVING_SRL_FORWARD;
            ee_velocities(0)  = + received_val * max_motor_speed;
            //phase_ms          = 1098.0f + 6920.0f * powf(1.0f - received_val, 4.13f);
            phase_ms          = get_phase_ms(received_val);            
            t_start           = millis();  // AGGIUNGI QUESTO
          }
        }
      }
      
      if (header == 'G') {

        switch (cal_state) {

          // Fase 1: muove in una direzione per 3 secondi
          case MOVING_GRIPPER_FORWARD:
            if (millis() - t_start < phase_ms) {

            } else {
              gripperVel  = - received_val * max_motor_speed;
              t_start     = millis();
              cal_state   = MOVING_GRIPPER_BACKWARD;
            }
            break;

          // Fase 2: muove nell'altra direzione per 3 secondi
          case MOVING_GRIPPER_BACKWARD:
            if (millis() - t_start < phase_ms) {

            } else {
              gripperVel  = 0;
              t_start     = millis();
              cal_state   = IDLE;
              DEBUG_SERIAL.println("OK");
            }
          break;
        } 
      }
  
      else if (header == 'S') {

        switch (cal_state) {

          // Fase 1: muove in una direzione per 3 secondi
          case MOVING_SRL_FORWARD:
            if (millis() - t_start < phase_ms) {

            } else {
              ee_velocities(0)    = - received_val * max_motor_speed;
              t_start             = millis();
              cal_state           = MOVING_SRL_BACKWARD;
            }
            break;

          // Fase 2: muove nell'altra direzione per 3 secondi
          case MOVING_SRL_BACKWARD:
            if (millis() - t_start < phase_ms) {

            } else {
              ee_velocities(0)    = 0;
              t_start             = millis();
              cal_state           = IDLE;
              DEBUG_SERIAL.println("OK");
            }
          break;
        }
      }
    }
    
    else if (header == 'C') {
      if (DEBUG_SERIAL.available() >= 4) {
        DEBUG_SERIAL.read();  // consuma il 'C'
        inputString     = DEBUG_SERIAL.readStringUntil('\n');
        flexext         = getValue(inputString, ' ', 0).toFloat();
        cocon1          = getValue(inputString, ' ', 1).toFloat();
        cocon2          = getValue(inputString, ' ', 2).toFloat();
        inputString     = "";
        
        DEBUG_SERIAL.print("  gripvel1: ");
        DEBUG_SERIAL.print(gripperVel);
        DEBUG_SERIAL.print(" Joint1Vel: ");   
        DEBUG_SERIAL.println(joint_velocities(1));
      }
      
      if (fuzzy_control_enabled == true) {
      //   // --- FUZZY CONTROLLER ---
      //   // ---------- FUZZY (come prima per d, a, c, d01 e membership) ----------
      //   float d   = ext - flex;               // -1..1
      //   float a   = fabsf(d);                 // |d|
      //   float c   = cocontraction;          // co-contrazione 0..1
      //   float d01 = 0.5f * (d + 1.0f);

      //   // Membership su d (per la direzione del gripper)
      //   float mu_neg    = tri(d01, 0.00f, 0.00f, 0.50f);  // flex > ext → chiudi
      //   float mu_zero   = tri(d01, 0.25f, 0.50f, 0.75f);  // vicino a 0 → hold
      //   float mu_pos    = tri(d01, 0.50f, 1.00f, 1.00f);  // ext > flex → apri

      //   // Membership su |d| (per “piccola differenza” SRL)
      //   float mu_a_small  = rgrade(a, DIFF_MAX_FOR_SRL, DIFF_MAX_FOR_SRL + DIFF_SRL_WIDTH); // 1 se |d| piccolo

      //   // Membership su co-contrazione
      //   float mu_c_low    = rgrade(c, CO_MIN,                     CO_MIN + CO_GATE_WIDTH);
      //   float mu_c_med    = tri   (c, CO_MIN,                     CO_MIN + CO_GATE_WIDTH, CO_MIN + 2*CO_GATE_WIDTH);
      //   float mu_c_high   = grade (c, CO_MIN + CO_GATE_WIDTH,     CO_MIN + 2*CO_GATE_WIDTH);
        
      //   // Consequents SRL (scala con la co-contrazione)
      //   float srl_slow  = (best_srl - 0.2) * max_motor_speed;
      //   float srl_med   = best_srl * max_motor_speed;
      //   float srl_fast  = (best_srl + 0.2) * max_motor_speed;

      //   // ========= GRIPPER con 3 livelli per direzione =========
      //   // Membership su |d| per lo SPEED del gripper
      //   float mu_d_small  = rgrade(a, G_SMALL, G_MED);
      //   float mu_d_med    = tri   (a, G_SMALL, G_MED, G_LARGE);
      //   float mu_d_large  = grade (a, G_MED,   G_LARGE);

      //   // Consequents velocità gripper (magnitudo)
      //   float g_slow  = (best_grip - 0.2) * max_motor_speed;
      //   float g_med   = best_grip * max_motor_speed;
      //   float g_fast  = (best_grip + 0.2) * max_motor_speed;

      //   // Magnitudo pesata (0..max) in base a |d|
      //   float Wmag      = mu_d_small + mu_d_med + mu_d_large; if (Wmag < 1e-6f) Wmag = 1.f;
      //   float g_mag     = (mu_d_small*g_slow + mu_d_med*g_med + mu_d_large*g_fast) / Wmag;

      //   // Direzione dal segno (mu_pos vs mu_neg), -1..+1
      //   float Wdir      = mu_pos + mu_neg; if (Wdir < 1e-6f) Wdir = 1.f;
      //   float dir_sign  = (mu_pos - mu_neg) / Wdir;  // negativo=chiudi, positivo=apri

      //   // Comando gripper “grezzo”
      //   float gripper_cmd = dir_sign * g_mag;

      //   // Deadband morbida sul gripper (se |d| piccolo → ≈0)
      //   float g_gate = grade(a, DIFF_DEADBAND, DIFF_DEADBAND + DIFF_DEADBAND_WIDTH);
      //   float g_cmd  = gripper_cmd * g_gate;   // applica la deadband

      //   // ========= SRL: pesi fuzzy dipendenti da co-contrazione =========
      //   // SRL attivo solo quando |d| è piccolo → includiamo mu_a_small nei pesi
      //   float w_srl_slow  = min(mu_a_small, mu_c_low);
      //   float w_srl_med   = min(mu_a_small, mu_c_med);
      //   float w_srl_fast  = min(mu_a_small, mu_c_high);
      //   float Wsrl        = w_srl_slow + w_srl_med + w_srl_fast; if (Wsrl < 1e-6f) Wsrl = 1.f;
      //   float srl_cmd_mag = (w_srl_slow*srl_slow + w_srl_med*srl_med + w_srl_fast*srl_fast) / Wsrl;

      //   // Gate esplicito SRL: (entrambi sopra soglia) AND (differenza piccola)
      //   float co_gate       = grade(c, CO_MIN, CO_MIN + CO_GATE_WIDTH);
      //   float diff_gate_srl = rgrade(a, DIFF_MAX_FOR_SRL, DIFF_MAX_FOR_SRL + DIFF_SRL_WIDTH);
      //   float srl_gate      = min(co_gate, diff_gate_srl);

      //   // ---------- Mutua esclusione + applicazione ----------
      //   if (srl_gate > 0.5f) {
      //     // SRL “wins”: gripper fermo
      //     gripperVel  = 0;
      //     float cmd   = srl_cmd_mag * srl_gate;

      //     if (cmd > 1.0f) {
      //       ee_velocities(0) = direction * cmd;

      //       // ---- controllo limiti posizione ----
      //       for (int i = 0; i < 3; i++) {
      //         if ((module1.moduleLegAngle[i] >= max_position) && (ee_velocities(0) > 0)) {
      //           direction      = -1;      // inverti direzione
      //           ee_velocities(0) = 0.0f;  // blocca al limite
      //         }
      //         else if ((module1.moduleLegAngle[i] <= min_position) && (ee_velocities(0) < 0)) {
      //           direction      = 1;
      //           ee_velocities(0) = 0.0f;
      //         }
      //       }
      //     }
      //   } else {
      //     // SRL non attivo → nessun movimento
      //     ee_velocities(0) = 0.0f;
      //     ee_velocities(1) = 0.0f;
      //     ee_velocities(2) = 0.0f;
      //     gripperVel       = (int)g_cmd;
      //   }
      }

      // ---------- SRL ----------      
      if (cocon1 > 0.2) {
          // SRL attivo → calcola velocità
          float vel_cmd = direction * cocon1 * max_motor_speed;

          // Controlla se sta andando oltre i limiti
          for (int i = 0; i < 3; i++) {
              if ((module1.moduleLegAngle[i] >= max_position) && (vel_cmd > 0)) {
                  direction = -1;   // inverti direzione
                  vel_cmd   = 0.0f; // ferma il movimento sul limite
              }
              else if ((module1.moduleLegAngle[i] <= min_position) && (vel_cmd < 0)) {
                  direction = 1;    // inverti direzione
                  vel_cmd   = 0.0f;
              }
          }
          ee_velocities(0) = direction * cocon1 * max_motor_speed;
      }
      else {
          // SRL non attivo → nessun movimento
          ee_velocities(0) = 0.0f;
          ee_velocities(1) = 0.0f;
          ee_velocities(2) = 0.0f;
      }

      // ---------- GRIPPER ----------     
      if (cocon1 < 0.2 && abs(flexext) > 0.2) {
          // Gripper attivo → calcola velocità
          gripperVel = flexext * max_motor_speed;
      }
      else {
          // Gripper non attivo → nessun movimento
          gripperVel = 0;
      }
    }

    // Set fixed position for module (only for now, later controlled by EMG)
    // module1.setpointLegAngle[0] = 0.1; // [rad]
    // module1.setpointLegAngle[1] = 0.1;
    // module1.setpointLegAngle[2] = 0.1;
    // Compute the feedback term of the velocity controller for module
    // module1.computePositionController();
    
    // compute jacobian
    float motor_angles[3] = {module1.moduleLegAngle[0], module1.moduleLegAngle[1], module1.moduleLegAngle[2]};
    numericalJacobian(forwardKinematicsOneModule, motor_angles, jacobian_matrix);

    // Compute the joint velocities with jacobian
    joint_velocities = Inverse(jacobian_matrix) * ee_velocities;

    // Compute the feedback term of the velocity controller for module
    module1.computePositionController();

    for(int i = 0; i < 3; i++) { // 3 is number of legs per module
      module1.setpointLegVelocityFF[i] = joint_velocities(i);
      module1.setpointLegVelocityFB[i] = 0;
    }

    // --- COMPUTE VELOCITY CONTROLLER ---
    for(int i = 0; i < 3; i++) {
      // Feedback and feedforward are combined
      module1.setpointLegAngleVelocity[i] = module1.setpointLegVelocityFB[i] + module1.setpointLegVelocityFF[i]/0.024; // [raw]
    }

    // MIN-MAX Velocity Check
    // SRL
    for(int i = 0; i < 3; i++) {
      if (module1.setpointLegAngleVelocity[i] > max_motor_speed) module1.setpointLegAngleVelocity[i]      = max_motor_speed; // [raw]
      else if (module1.setpointLegAngleVelocity[i] < min_motor_speed) module1.setpointLegAngleVelocity[i] = min_motor_speed; // [raw]
    }

    // MIN-MAX Position Check
    for(int i = 0; i < 3; i++) {
      // module 1
      if ((module1.moduleLegAngle[i] > max_position)&&(module1.setpointLegAngleVelocity[i]>0))
        module1.setpointLegAngleVelocity[i] = 0;
      else if((module1.moduleLegAngle[i] < min_position)&&(module1.setpointLegAngleVelocity[i]<0))
        module1.setpointLegAngleVelocity[i] = 0;
    }

    // GRIPPER
    if (gripperVel > max_motor_speed) gripperVel        = max_motor_speed; // [raw]
    else if (gripperVel < min_motor_speed) gripperVel   = min_motor_speed; // [raw]

    presentLoad = dxl.readControlTableItem(PRESENT_LOAD, GRIPPER_DXL_ID);

    // Check torque limit for gripper - TODO CHANGE RIGHT SOCK SCALED TO EMG SIGNAL FOR CLOSING DIRECTION
    if (presentLoad > 160) gripperVel                   = 0;

    // --- SYNC WRITE MOTOR COMMANDS ---
    // Module - Insert a new Goal Velocity to the SyncWrite Packet
    for(int i = 0; i < 3; i++) {
      sw_data[i].goal_velocity  = module1.setpointLegAngleVelocity[i]; // [UNIT_RAW]
    }

    // Gripper - Insert a new Goal Velocity to the SyncWrite Packet
    sw_data[3].goal_velocity    = gripperVel;

    // Update the SyncWrite packet status
    sw_infos.is_info_changed    = true;
    dxl.syncWrite(&sw_infos);
  }

// --- SERIAL OUTPUT ---
  if(Timer_02.Tick()){
    if (print){
      if(controlMode==0){
        DEBUG_SERIAL.print(module1.moduleLegAngle[0]);    DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.setpointLegAngle[0]);  DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.moduleLegAngle[1]);    DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.setpointLegAngle[1]);  DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.moduleLegAngle[2]);    DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.setpointLegAngle[2]);  DEBUG_SERIAL.println(" ");
      }
      else if(controlMode==1){
        DEBUG_SERIAL.print(module1.moduleLegAnglularVelocity[0]);   DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.setpointLegAngleVelocity[0]);    DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.moduleLegAnglularVelocity[1]);   DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.setpointLegAngleVelocity[1]);    DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.moduleLegAnglularVelocity[2]);   DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.setpointLegAngleVelocity[2]);    DEBUG_SERIAL.println(" ");
      }
      else if(controlMode==2){
        DEBUG_SERIAL.print(module1.moduleLegAnglularVelocity[0]);   DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.setpointLegAngleVelocity[0]);    DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.moduleLegAnglularVelocity[1]);   DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.setpointLegAngleVelocity[1]);    DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.moduleLegAnglularVelocity[2]);   DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.setpointLegAngleVelocity[2]);    DEBUG_SERIAL.println(" ");
      }
      else if(controlMode==3){
        DEBUG_SERIAL.print(module1.moduleLegAngle[0]);    DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.setpointLegAngle[0]);  DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.moduleLegAngle[1]);    DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.setpointLegAngle[1]);  DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.moduleLegAngle[2]);    DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.setpointLegAngle[2]);  DEBUG_SERIAL.println(" ");
      }
      else if(controlMode==4){
        DEBUG_SERIAL.print("PARSED -> grip: ");
        DEBUG_SERIAL.print(module1.moduleLegAnglularVelocity[0]);  
        DEBUG_SERIAL.print(" "); 
        DEBUG_SERIAL.print(module1.setpointLegAngleVelocity[0]);
        DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.moduleLegAnglularVelocity[1]);
        DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.setpointLegAngleVelocity[1]);    
        DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.moduleLegAnglularVelocity[2]);   
        DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(module1.setpointLegAngleVelocity[2]);    
        DEBUG_SERIAL.print(" ");
        DEBUG_SERIAL.print(joint_velocities(0)); 
        DEBUG_SERIAL.print(" Joint1Vel: ");   
        DEBUG_SERIAL.print(joint_velocities(1));
        DEBUG_SERIAL.print(" ");    
        DEBUG_SERIAL.print(joint_velocities(2));
        DEBUG_SERIAL.print("  gripperVel: ");
        DEBUG_SERIAL.println(gripperVel);
      }
    }
  }
}