#include <QuadEncoder.h>
#include <Encoder.h>
#include "driver_ack_retry.h"
#include <cmath>
#include "stall_detector.h"

// Teensy 4.0 pin functionalities
// Serial RX: 0, 7, 15, 16, 21, 25, 28
// Serial TX: 1, 8, 14, 17, 20, 24, 29
// Hardware quadrature decoder: 0, 1, 2, 3, 4, 5, 7, 8, 30, 31 and 33; 0 and 5 share the same XBAR

// Pin layouts:
// Serial RX: 7, 15, 16, 21, 25, 28
// Serial TX: 8, 14, 17, 20, 24, 29
// Transmitter enable: 5, 6, 12, 18, 19, 22
// Hardware quadrature decoder: 0, 1, 2, 3, 4, 30, 31, 33
// Software quadrature decoder: 23, 26, 27, 32
// Limit switch: 9, 10, 11
// debug/LED_BUILTIN: 13

constexpr uint8_t LimitSwitch[] = { 9, 10, 11 };

// Serial communication
HardwareSerialIMXRT *HWSerials[] = { &Serial2, &Serial3, &Serial4, &Serial5, &Serial6, &Serial7 };
// constexpr uint8_t TE[] = { 5, 6, 12, 18, 19, 22 };
constexpr uint8_t numHWSerials = 6;

constexpr size_t BUFFER_LEN = 256;
uint8_t receivedHWBytes[numHWSerials][BUFFER_LEN];
uint8_t receivedPCBytes[BUFFER_LEN];
size_t PCBytes_len = 0;
size_t HWBytes_len[numHWSerials];
uint8_t sendingPCBytes[BUFFER_LEN];
char sendingHWChars[numHWSerials][BUFFER_LEN];
// size_t numsendingHWChars[numHWSerials];
bool newHWMsg[numHWSerials];
bool newPCMsg = false;
bool newJointVelCmd = false;
bool newJointPosCmd = false;
bool newTargetVelCmd = false;

// Protocals
constexpr size_t REQ_ID_LEN    = 16;

constexpr uint8_t startMarker = '$';
constexpr uint8_t endMarker = '\n'; // <LF>
constexpr uint8_t respMarker = '!';
constexpr uint8_t PCStartMarker = '<';
constexpr uint8_t PCEndMarker = '>';

constexpr uint8_t VEL = 'V';
constexpr uint8_t POS = 'P';
constexpr uint8_t ENC = 'E';
constexpr uint8_t START = 'I';
constexpr uint8_t STOP = 'S';
constexpr uint8_t ZERO = 'Z';
constexpr uint8_t RESET_FAULT = 'R';
constexpr uint8_t FAULT_STATUS = 'Q';
constexpr uint8_t SET_VEL = 'F';
constexpr uint8_t DEBUG = 'D';
constexpr uint8_t CONNECT = 'C';
constexpr uint8_t CALIBRATION = 'K';
constexpr uint8_t LIMIT = 'L';
constexpr uint8_t STALL = 'T';
constexpr uint8_t TRIGGER_P = 'c';
constexpr uint8_t TRIGGER_C = 'b';
constexpr uint8_t TRIGGER_M = 'a';
constexpr uint8_t TRIGGER_N = 'n';

// constexpr uint8_t MAX_PENDING_ACKS = 8;
constexpr uint32_t ACK_TIMEOUT_MS = 1000;
struct PendingAck {
  bool valid = false;
  uint8_t prefix;
  uint8_t uuid[REQ_ID_LEN];
  // uint32_t t_start_ms;
};

PendingAck pendingAck; // task acknowledgement

// Quadrature decoder
QuadEncoder Enc1(1, 1, 0); // A, B
QuadEncoder Enc2(2, 3, 2);
QuadEncoder Enc4(3, 30, 4);
QuadEncoder Enc5(4, 33, 31);
Encoder Enc0(23, 26);  // B, A
Encoder Enc3(27, 32);
QuadEncoder *HWEncoders[] = {&Enc1, &Enc2, &Enc4, &Enc5};
Encoder *SWEncoders[] = {&Enc0, &Enc3};

constexpr float EncRes = 0.045f; // deg
constexpr uint16_t maxRPM = 250;
long EncCounts[numHWSerials];
// float EncPos[numHWSerials] = { 0.0f };
// float jointPos[numHWSerials] = { 0.0f };
// float jointVel[numHWSerials] = { 0.0f };

// Joint transmission, joint/motor, positive direction: right-hand or forward
constexpr float linRate = 67.319841f/20.0f; // mm/rev
constexpr float rotRate = 0.375f/5.0f; // rev/rev
constexpr float catheterBendRate = -1.190625f/5.0f; // mm/rev
constexpr float sheathBendRate = 0.375f/5.0f; // rev/rev
constexpr float jointPRate[numHWSerials] = { linRate/360.0f, rotRate, catheterBendRate/360.0f, 
                                            linRate/360.0f, rotRate, sheathBendRate }; // mm/deg, deg/deg
constexpr float jointVRate[numHWSerials] = { linRate/60.0f, rotRate/60.0f*360.0f, catheterBendRate/60.0f, 
                                            linRate/60.0f, rotRate/60.0f*360.0f, sheathBendRate/60.0f*360.0f }; // mm*min/rev*s, deg*min/rev*s
constexpr float catheterBendUB = 20.8f;
constexpr float catheterBendLB = 0.0f;
constexpr float sheathBendUB = 360.0f;
constexpr float sheathBendLB = -360.0f;

// State Variables
int limitState[5] = { 0, 0, 0, 0, 0 };
// long targetCounts[numHWSerials];
// long targetHz[numHWSerials];
float targetPos[numHWSerials];
uint16_t targetDeg[numHWSerials];
float currentPos[numHWSerials]; // decoupled
float previousPos[numHWSerials]; // decoupled
float currentCatheterLMPos = 0.0f; // coupled
float currentSheathBendPos = 0.0f; // coupled
float targetVel[numHWSerials];
float currentVel[numHWSerials];
uint16_t targetRPM[numHWSerials];
uint16_t currentRPM[numHWSerials];
bool speedValid[numHWSerials] = { false };
uint8_t targetDir[numHWSerials]; // '0': right-hand, '1': left-hand
uint8_t currentDir[numHWSerials];
bool directionValid[numHWSerials] = { false };
bool motorEnabled[numHWSerials] = { false };

// Confirmed faults are latched: the affected physical motor and its coupled
// partner are stopped and ignore the 100 Hz command stream until RESET_FAULT.
constexpr bool FAULT_STOP_ENABLED = true;
StallDetector motionMonitors[numHWSerials];
uint16_t motionFaultSequence = 0;

// Timing
constexpr uint8_t controlCycle = 10; // ms
elapsedMillis sinceLastCycle;
elapsedMillis sinceLastPCMsg;
constexpr uint32_t PC_SILENCE_MS = 10000;
bool watchdog_engaged = true;
constexpr uint16_t DRIVER_DELAY_us = 1000;  // delay serial write for the next command to register
constexpr uint32_t MOTOR_ACK_TIMEOUT_MS = 50;
constexpr uint8_t MOTOR_ACK_ATTEMPTS = 3;
constexpr uint16_t MOTOR_ACK_RETRY_DELAY_MS = 10;
constexpr uint16_t MOTOR_DIRECTION_SETTLE_MS = 5;
bool pc_connected = false;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  memset(HWBytes_len, 0, sizeof(HWBytes_len));
  memset(newHWMsg, 0, sizeof(newHWMsg));
  memset(EncCounts, 0, sizeof(EncCounts));
  memset(targetDeg, 0, sizeof(targetDeg));
  memset(currentPos, 0.0f, sizeof(currentPos));
  memset(previousPos, 0.0f, sizeof(currentPos));
  memset(targetRPM, 0, sizeof(targetRPM));
  memset(currentRPM, 0, sizeof(currentRPM));
  memset(targetDir, '1', sizeof(targetDir));
  memset(currentDir, '1', sizeof(currentDir));

  // Initialize fault state before any motor-driver I/O. Startup deliberately
  // does not wait for or classify driver acknowledgements: drivers may still
  // be powering up (or motor power may intentionally be off), and USB may not
  // have a host yet. The first runtime motion command reasserts direction and
  // requires an acknowledgement before enabling each motor.
  for (uint8_t axis = 0; axis < numHWSerials; axis++) {
    motionMonitors[axis].begin(millis(), EncCounts[axis]);
  }

  // set up motor communications
  for (uint8_t i = 0; i < numHWSerials; i++) {
    // HWSerials[i]->transmitterEnable(TE[i]);
    HWSerials[i]->begin(115200);

    // Always establish a known disabled state before configuring a driver.
    stopMotorAxis(i, true);
    writeMotorCommand(i, "$C0\n", 4);
    writeMotorCommand(i, "$L1\n", 4);
    directionValid[i] = false;
  }

  // set up encoders
  for (uint8_t i = 0; i < 4; i++) {
    HWEncoders[i]->setInitConfig();
    // HWEncoders[i]->EncConfig.positionInitialValue = EncCounts[i];
    HWEncoders[i]->init();
  }

  for (uint8_t i = 0; i < 3; i++) {
    pinMode(LimitSwitch[i], INPUT_PULLUP);
  }
  // digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
    // Asychronous I/O, atomic message processing
    recvPCSerial();
    procPCBytes();
    recvHWSerials();
    // procHWBytes();
    
    // control Cycle 100Hz
    if (sinceLastCycle > controlCycle) {
      sinceLastCycle = 0;
      readEncoders();
      updateMotionMonitors();
      checkLimitSwitches();
      sendMotorCmds();
    }

    // watchdog
    if (sinceLastPCMsg > PC_SILENCE_MS && watchdog_engaged) {
      for (uint8_t i = 0; i < numHWSerials; i++) {
        stopMotorAxis(i, true);
        targetRPM[i] = 0;
        targetVel[i] = 0.0f;
        motionMonitors[i].setCommand(
            millis(), 0, targetDir[i], 0.0f);
      }
      newJointVelCmd = false;
      newJointPosCmd = false;
      watchdog_engaged = false;
    }
}

void recvHWSerials() {
    static bool recvInProgress[numHWSerials] = {false};
    static uint8_t ndx[numHWSerials] = {0};
    uint8_t rb;

    for (uint8_t i = 0; i < numHWSerials; i++){
      // Drain every response. Each valid Tekceleo command produces an
      // acknowledgement; consuming only one frame per control loop can overflow
      // the UART receive buffer at a 100 Hz command rate.
      while (HWSerials[i]->available() > 0) {
          rb = HWSerials[i]->read();

          if (recvInProgress[i] == true) {
              if (rb != endMarker) {
                  receivedHWBytes[i][ndx[i]] = rb;
                  ndx[i]++;
                  if (ndx[i] >= BUFFER_LEN) {
                      ndx[i] = BUFFER_LEN - 1;
                  }
              }
              else {
                  // receivedHWBytes[i][ndx[i]] = '\0'; // terminate the string
                  HWBytes_len[i] = ndx[i];
                  recvInProgress[i] = false;
                  ndx[i] = 0;
                  newHWMsg[i] = true;
              }
          }

          else if (rb == respMarker) {
              recvInProgress[i] = true;
              receivedHWBytes[i][0] = rb; // including start marker for hardware serial
              ndx[i]++;
          }
      }
    }
}

void recvPCSerial() {
    static bool recvInProgress = false;
    static bool expectLenByte = false;
    static uint8_t ndx = 0;
    uint8_t rb;

    while (Serial.available() > 0 && newPCMsg == false) {
      // digitalWrite(LED_BUILTIN,HIGH);
      rb = Serial.read();

      if (recvInProgress) {
        if (ndx < PCBytes_len) {
          receivedPCBytes[ndx++] = rb;
        } else {
          // We already consumed the declared payload; this byte must be the end marker.
          recvInProgress = false;
          ndx = 0;

          if (rb == PCEndMarker) {
            newPCMsg = true;
            sinceLastPCMsg = 0;
            watchdog_engaged = true;
          } else if (rb == PCStartMarker) {
            // Malformed frame; immediately resync on this new start marker.
            expectLenByte = true;
          } else {
            expectLenByte = false;
          }
        }
      } else if (rb == PCStartMarker) {
        expectLenByte = true;
      } else if (expectLenByte) {
        // Repeated start marker: previous one was stale/noisy.
        if (rb == PCStartMarker) {
          expectLenByte = true;
        } else if (rb > 0 && rb <= BUFFER_LEN) {
          PCBytes_len = rb;
          ndx = 0;
          recvInProgress = true;
          expectLenByte = false;
        } else {
          // Invalid length, drop frame and wait for a fresh start marker.
          expectLenByte = false;
        }
      }
    }
}


// static inline void debugHexByte(uint8_t b) {
//   const char hex[] = "0123456789ABCDEF";
//   char out[3];
//   out[0] = hex[(b >> 4) & 0x0F];
//   out[1] = hex[b & 0x0F];
//   out[2] = '\0';
//   Serial.print(out);
// }

// void recvPCSerial() { //old version for serial monitor debug
//     static bool recvInProgress = false;
//     static uint8_t ndx = 0;
//     uint8_t rb;

//     while (Serial.available() > 0 && newPCMsg == false) {
//       rb = Serial.read();

//       if (recvInProgress) {
//         if (rb != PCEndMarker) {
//           receivedPCBytes[ndx] = rb;
//           ndx++;
//           if (ndx >= BUFFER_LEN) {
//             ndx = BUFFER_LEN - 1;
//           }
//         } else {
//           // receivedPCBytes[ndx] = '\0'; // terminate the string
//           PCBytes_len = ndx;
//           newPCMsg = true;
//           sinceLastPCMsg = 0;
//           watchdog_engaged = true;
//           recvInProgress = false;
//           ndx = 0;
//         }
//       }

//       else if (rb == PCStartMarker) {
//         recvInProgress = true;
//       }
//     }
// }

static inline bool hasUUID(uint8_t cmd) {
  return (cmd != VEL && cmd != POS && cmd != CONNECT && cmd != DEBUG);
}

// bool registerPendingAck(uint8_t prefix, const uint8_t* uuid) {
//   for (uint8_t i = 0; i < MAX_PENDING_ACKS; i++) {
//     if (!pendingAcks[i].valid) {
//       pendingAcks[i].valid = true;
//       pendingAcks[i].prefix = prefix;
//       memcpy(pendingAcks[i].uuid, uuid, REQ_ID_LEN);
//       pendingAcks[i].t_start_ms = millis();
//       return true;
//     }
//   }
//   return false;  // table full → drop or NACK
// }

void sendAckIfPending(const char* response = nullptr) {
  if (!pendingAck.valid) return;

  const size_t responseLength = response == nullptr ? 0 : strlen(response);
  Serial.write(&PCStartMarker, 1);
  Serial.write(1 + REQ_ID_LEN + responseLength);
  Serial.write(&pendingAck.prefix, 1);
  Serial.write(pendingAck.uuid, REQ_ID_LEN);
  if (responseLength > 0) {
    Serial.write(response, responseLength);
  }
  Serial.write(&PCEndMarker, 1);

  pendingAck.valid = false;
}

void procPCBytes() {
  // static uint8_t axis = 0;
  // uint8_t pos = 0;
  static float temp;
  // static float abs;
  // static uint8_t req_id[16]

  if (newPCMsg) {
      newPCMsg = false;

    // for (uint8_t i = 0; i < PCBytes_len; i++){ // only iterates through predicates
      // digitalWrite(LED_BUILTIN,HIGH);
      uint8_t cmd = receivedPCBytes[0];

      if (hasUUID(cmd)) {
        // Capture ACK info immediately
        pendingAck.valid = true;
        pendingAck.prefix = cmd;
        memcpy(pendingAck.uuid, receivedPCBytes + 1, REQ_ID_LEN);
        // if (!registerPendingAck(cmd, receivedPCBytes + 1)) {
        //   // optional: send immediate NACK or error
        //   newPCMsg = false;
        //   return;
        // }   
      }

      switch (cmd) {

        case VEL: {// 'V'
          // digitalWrite(LED_BUILTIN, HIGH);
        
          newJointPosCmd = false;
          memcpy(targetVel, receivedPCBytes + 1, 24); // float size 4
          targetVel[0] -= targetVel[2]; // decouple lm and bend for catheter
          targetVel[5] += targetVel[4]; // decouple rot and bend for sheath
          newJointVelCmd = true;
          for (uint8_t axis = 0; axis < numHWSerials; axis++) {
            if (!isVelCmdValid(axis)) {
              newJointVelCmd = false;
              break;
            }
          }
          if (newJointVelCmd) {
            for (uint8_t axis = 0; axis < numHWSerials; axis++) {
              temp = targetVel[axis]/jointVRate[axis];
              targetDir[axis] = (temp >= 0.0f) ? '1' : '0';
              targetRPM[axis] = static_cast<uint16_t>(std::roundf(std::fabs(temp)));
              if (targetRPM[axis] > maxRPM) { targetRPM[axis] = maxRPM; }
            }
          }
          if (!newJointVelCmd) {
            // Never leave an older valid command running after rejecting a
            // newer vector.
            for (uint8_t axis = 0; axis < numHWSerials; axis++) {
              stopMotorAxis(axis, true);
            }
          }
          break;
        }

        case POS: {// 'P'
          newJointVelCmd = false;
          memcpy(targetPos, receivedPCBytes + 1, 24); // float size 4
          targetPos[0] -= targetPos[2]; // decouple lm and bend for catheter
          targetPos[5] += targetPos[4]; // decouple rot and bend for sheath
          
          for (uint8_t axis = 0; axis < numHWSerials; axis++) {
            // memcpy(&targetPos[axis], receivedPCBytes + 1 + axis*4, 4); // float size 4
            newJointPosCmd = isPosCmdValid(axis);
            if (!newJointPosCmd) { break; }

            temp = (targetPos[axis] - currentPos[axis])/jointPRate[axis];

            targetDir[axis] = (temp >= 0.0f) ? '1' : '0';
            targetDeg[axis] = static_cast<uint16_t>(std::roundf(std::fabs(temp)));
          }
          break;
        }

        case SET_VEL: {// 'F', set target velocity for pos control
          memcpy(targetVel, receivedPCBytes + 1 + REQ_ID_LEN, 24); // float size 4
          targetVel[0] -= targetVel[2]; // decouple lm and bend for catheter
          targetVel[5] += targetVel[4]; // decouple rot and bend for sheath

          for (uint8_t axis = 0; axis < numHWSerials; axis++) {
            temp = targetVel[axis]/jointVRate[axis];
            targetRPM[axis] = static_cast<uint16_t>(std::roundf(std::fabs(temp)));
            if (targetRPM[axis] > maxRPM) { targetRPM[axis] = maxRPM; }
            targetDir[axis] = (temp >= 0.0f) ? '1' : '0';
          }
          newTargetVelCmd = true;
          newJointVelCmd = false;
          sendAckIfPending();
          break;
        }

        case ZERO: {// 'Z', set all encoders to 0
          for (uint8_t i = 0; i < 4; i++) {
            HWEncoders[i]->write(0);
          }
          for (uint8_t i = 0; i < 2; i++) {
            SWEncoders[i]->write(0);
          }
          for (uint8_t axis = 0; axis < numHWSerials; axis++) {
            EncCounts[axis] = 0;
            currentPos[axis] = 0.0f;
          }
          currentCatheterLMPos = 0.0f;
          currentSheathBendPos = 0.0f;
          sendAckIfPending();
          break;
        }

        case STOP: {// 'S', stop all motors
          for (uint8_t axis = 0; axis < numHWSerials; axis++) {
            stopMotorAxis(axis, true);
            targetRPM[axis] = 0;
            targetVel[axis] = 0.0f;
            motionMonitors[axis].setCommand(
                millis(), 0, targetDir[axis], 0.0f);
          }
          newJointVelCmd = false;
          newJointPosCmd = false;
          sendAckIfPending();
          break;
        }

        case RESET_FAULT: {// 'R', clear a latched fault only while stopped
          bool safeToReset = true;
          for (uint8_t axis = 0; axis < numHWSerials; axis++) {
            if (targetRPM[axis] != 0 || motorEnabled[axis]) {
              safeToReset = false;
              break;
            }
          }
          if (!safeToReset) {
            sendAckIfPending("ERR_NONZERO");
            break;
          }
          for (uint8_t axis = 0; axis < numHWSerials; axis++) {
            StallDetector::Event event =
                motionMonitors[axis].clearFault(millis(), EncCounts[axis]);
            if (event.transition == StallDetector::RESET) {
              reportMotionEvent(axis, 255, event);
            }
          }
          sendAckIfPending("OK");
          break;
        }

        case FAULT_STATUS: {// 'Q', query latched faults and motor state
          sendFaultStatusAck();
          break;
        }

        // case START: {// 'I', initialize all motors
        //   for (uint8_t axis = 0; axis < numHWSerials; axis++) {
        //     // snprintf(sendingHWChars[axis], sizeof(sendingHWChars[axis]), "$O1\n");
        //     // HWSerials[axis]->print(sendingHWChars[axis]);
        //     HWSerials[axis]->print("$O1\n");
        //   }
        //   break;
        // }

        case DEBUG: {// 'D', send and read raw command
          // digitalWrite(LED_BUILTIN, HIGH);
          // if (PCBytes_len < 1 + REQ_ID_LEN) return;

          // uint8_t axis = receivedPCBytes[1 + REQ_ID_LEN] - '0';
          // const uint8_t* payload = receivedPCBytes + 2 + REQ_ID_LEN;
          // size_t payload_len = PCBytes_len - (2 + REQ_ID_LEN);
          // HWSerials[axis]->write(payload, payload_len);

          uint8_t axis = receivedPCBytes[1 + 0] - '0';
          const uint8_t* payload = receivedPCBytes + 2 + 0;
          size_t payload_len = PCBytes_len - 2 - 0;
          HWSerials[axis]->write(&startMarker, 1);
          HWSerials[axis]->write(payload, payload_len);
          HWSerials[axis]->write(&endMarker, 1);
          
          break;
        }

        case CONNECT: { // 'C', idempotently enable PC telemetry
          // CONNECT used to toggle telemetry. A client that disconnected
          // without sending a second CONNECT left the next client able to
          // accidentally turn telemetry off. Repeated CONNECT frames are now
          // safe and always (re-)announce the current safety state.
          pc_connected = true;
          digitalWrite(LED_BUILTIN, HIGH);
          sendAckIfPending();
          reportLimitStates();
          reportFaultStatusEvents();
          break;
        }

        case CALIBRATION: { // 'K', start motor calibration
          // digitalWrite(LED_BUILTIN, HIGH);
          for (uint8_t axis = 0; axis < numHWSerials; axis++) {
            HWSerials[axis]->print("$K1\n");
          }
          break;
        }

        default:
          break;
      }
    // }
  }
  // PCBytes_len = 0;
}

static inline bool isVelCmdValid(uint8_t axis) {
  switch (axis) {
    case 0: {
      return !((limitState[2] && (limitState[1] || targetVel[0] < 0.0f)) ||
              (limitState[1] && targetVel[3] > 0.0f));
    }
    case 2: {
      return !((limitState[3] < 0 && targetVel[2] < 0.0f) || 
              (limitState[3] > 0 && targetVel[2] > 0.0f));
    }
    case 3: {
      return !((limitState[0] && (limitState[1] || targetPos[3] > 0.0f)) ||
              (limitState[1] && targetVel[3] < 0.0f));
    }
    case 5: {
      return !((limitState[4] < 0 && targetVel[5] < 0.0f) || 
              (limitState[4] > 0 && targetVel[5] > 0.0f));
    }
    default: { return true;}
  }
}

static inline bool isPosCmdValid(uint8_t axis) {
  switch (axis) {
    case 0: {
      return !((limitState[2] && (limitState[1] || targetPos[0] < currentPos[0])) ||
              (limitState[1] && targetPos[0] > currentPos[0]));
    }
    case 2: {
      return !((limitState[3] < 0 && targetPos[2] < currentPos[2]) || 
              (limitState[3] > 0 && targetPos[2] > currentPos[2]));
    }
    case 3: {
      return !((limitState[0] && (limitState[1] || targetPos[3] > currentPos[3])) ||
              (limitState[1] && targetPos[3] < currentPos[3]));
    }
    case 5: {
      return !((limitState[4] < 0 && targetPos[5] < currentSheathBendPos) || 
              (limitState[4] > 0 && targetVel[5] > currentSheathBendPos));
    }
    default: { return true;}
  }
}

void procHWBytes() {
  for (uint8_t i = 0; i < numHWSerials; i++) {
    if (newHWMsg[i]) {
      // snprintf(sendingPCChars, sizeof(sendingPCChars), "Serial%d received: ", i+1);
      Serial.write(&PCStartMarker, 1);
      Serial.write(18+HWBytes_len[i]);
      Serial.printf("serial%d received: ", i+1);
      Serial.write(receivedHWBytes[i], HWBytes_len[i]);
      Serial.write(&PCEndMarker, 1);
      // Serial.print("\r\n");
      newHWMsg[i] = false;
    }
  }
   // TODO

  // switch (pendingAck.prefix) {
  //   case STOP: {
  //     for (uint8_t axis = 0; axis < numHWSerials; axis++) {
  //       if (newHWMsg[axis]) {
  //         // parse hardware response
  //         // if response indicates completion:
  //         sendAckIfPending();
  //         newHWMsg[axis] = false;
  //       }
  //     }
  //     break;
  //   }
  // }
}

void readEncoders() {
  memcpy(previousPos, currentPos, sizeof(currentPos));
  EncCounts[0] = Enc0.read();
  EncCounts[1] = Enc1.read();
  EncCounts[2] = Enc2.read();
  EncCounts[3] = Enc3.read();
  EncCounts[4] = Enc4.read();
  EncCounts[5] = Enc5.read();
  for (uint8_t axis = 0; axis < numHWSerials; axis++) {
    // EncPos[axis] = EncCounts[axis]*EncRes;
    currentPos[axis] = EncCounts[axis]*EncRes*jointPRate[axis];
    currentVel[axis] = (currentPos[axis] - previousPos[axis])/controlCycle*1000.0f;
  }
  // coupled lm and bend for catheter
  currentCatheterLMPos = currentPos[0] + currentPos[2];
  // coupled rot and bend for sheath
  currentSheathBendPos = currentPos[5] - currentPos[4];

  if (pc_connected && Serial.availableForWrite() >= 56 && Serial.available() == 0) {
    // POS frame: physical joint positions (coupling + transmission applied)
    sendingPCBytes[0] = PCStartMarker;
    sendingPCBytes[1] = 25;
    sendingPCBytes[2] = POS;
    memcpy(sendingPCBytes + 3, &currentCatheterLMPos, 4); // 1*float
    memcpy(sendingPCBytes + 7, &currentPos[1], 16); // 4*float
    memcpy(sendingPCBytes + 23, &currentSheathBendPos, 4); // 1*float
    sendingPCBytes[27] = PCEndMarker;
    Serial.write(sendingPCBytes, 28);

    // ENC frame: raw encoder counts, uncoupled, int32 (little-endian). No
    // transmission/coupling applied, so downstream models learn the mapping.
    sendingPCBytes[0] = PCStartMarker;
    sendingPCBytes[1] = 25;
    sendingPCBytes[2] = ENC;
    for (uint8_t i = 0; i < numHWSerials; i++) {
      int32_t c = static_cast<int32_t>(EncCounts[i]);
      memcpy(sendingPCBytes + 3 + i*4, &c, 4); // 6*int32
    }
    sendingPCBytes[27] = PCEndMarker;
    Serial.write(sendingPCBytes, 28);
  }
}

void checkLimitSwitches() {
  static bool trigger_event;
  static int temp = 0;
  static uint8_t *p;

  trigger_event = false;
  p = sendingPCBytes;
  *p++ = LIMIT;
  *p++ = TRIGGER_N;
  *p++ = TRIGGER_N;
  *p++ = TRIGGER_N;

  // linear physical switches
  // for (uint8_t i = 0; i < 3; i++) {
  //   temp = digitalRead(LimitSwitch[i]);
  //   if (temp != limitState[i]) {
  //     limitState[i] = temp;
  //     trigger_event = true;
  //     *p++ = TRIGGER_C + limitState[i];
  //   } else {
  //     *p++ = TRIGGER_N;
  //   }
  // }

  // catheter bending
  if (currentPos[2] >= catheterBendUB) {
      temp = 1;
  } else if (currentPos[2] <= catheterBendLB) {
      temp = -1;
  } else {
      temp = 0;
  }
  if (temp != limitState[3]) {
    limitState[3] = temp;
    trigger_event = true;
    *p++ = TRIGGER_C + limitState[3];
  } else {
    *p++ = TRIGGER_N;
  }

  // sheath bending
  if (currentPos[5] >= sheathBendUB) {
      temp = 1;
  } else if (currentPos[5] <= sheathBendLB) {
      temp = -1;
  } else {
      temp = 0;
  }
  if (temp != limitState[4]) {
    limitState[4] = temp;
    trigger_event = true;
    *p++ = limitState[4] + TRIGGER_C;
  } else {
    *p++ = TRIGGER_N;
  }

  // digitalWrite(LED_BUILTIN, limitState[0]);
  if (trigger_event) {
    for (uint8_t axis = 0; axis < numHWSerials; axis++) {
      stopMotorAxis(axis, true);
    }
    Serial.write(&PCStartMarker, 1);
    Serial.write(6);
    Serial.write(sendingPCBytes, 6);
    Serial.write(&PCEndMarker, 1);
  }
}

void reportLimitStates() {
  sendingPCBytes[0] = LIMIT;

  // linear physical switches
  // for (uint8_t i = 0; i < 3; i++) {
  //   limitState[i] = digitalRead(LimitSwitch[i]);
  //   if (limitState[i]) {
  //     sendingPCBytes[i+1] = TRIGGER_P;
  //   } else {
  //     sendingPCBytes[i+1] = TRIGGER_N;
  //   }
  // }
  sendingPCBytes[1] = TRIGGER_N;
  sendingPCBytes[2] = TRIGGER_N;
  sendingPCBytes[3] = TRIGGER_N;

  // catheter bending
  if (currentPos[2] >= catheterBendUB) {
      limitState[3] = 1;
      sendingPCBytes[4] = TRIGGER_P;
  } else if (currentPos[2] <= catheterBendLB) {
      limitState[3] = -1;
      sendingPCBytes[4] = TRIGGER_M;
  } else {
      limitState[3] = 0;
      sendingPCBytes[4] = TRIGGER_N;
  }

  // sheath bending
  if (currentPos[5] >= sheathBendUB) {
      limitState[4] = 1;
      sendingPCBytes[5] = TRIGGER_P;
  } else if (currentPos[5] <= sheathBendLB) {
      limitState[4] = -1;
      sendingPCBytes[5] = TRIGGER_M;
  } else {
      limitState[4] = 0;
      sendingPCBytes[5] = TRIGGER_N;
  }

  Serial.write(&PCStartMarker, 1);
  Serial.write(6);
  Serial.write(sendingPCBytes, 6);
  Serial.write(&PCEndMarker, 1);
}

uint8_t coupledAxis(uint8_t axis) {
  if (axis == 0) return 2;
  if (axis == 2) return 0;
  if (axis == 4) return 5;
  if (axis == 5) return 4;
  return 255;
}

static inline uint8_t* writeU16LE(uint8_t* p, uint16_t value) {
  *p++ = static_cast<uint8_t>(value & 0xff);
  *p++ = static_cast<uint8_t>((value >> 8) & 0xff);
  return p;
}

static inline uint8_t* writeFloatLE(uint8_t* p, float value) {
  memcpy(p, &value, sizeof(value));
  return p + sizeof(value);
}

void reportMotionEvent(uint8_t axis, uint8_t coupled,
                       const StallDetector::Event& event) {
  uint8_t* p = sendingPCBytes;
  *p++ = STALL;
  for (uint8_t i = 0; i < numHWSerials; i++) {
    *p++ = TRIGGER_N;
  }
  if (axis < numHWSerials &&
      event.transition != StallDetector::RECOVERED &&
      event.transition != StallDetector::RESET) {
    sendingPCBytes[axis + 1] =
        std::signbit(event.commanded_velocity) ? TRIGGER_M : TRIGGER_P;
  }

  // Structured extension after the six legacy axis-state bytes.
  *p++ = 2;  // protocol version
  *p++ = static_cast<uint8_t>(event.transition);
  *p++ = static_cast<uint8_t>(event.fault);
  *p++ = axis;
  *p++ = coupled;
  p = writeU16LE(p, ++motionFaultSequence);
  p = writeFloatLE(p, event.commanded_velocity);
  p = writeFloatLE(p, event.measured_velocity);
  p = writeFloatLE(p, event.window_displacement);
  p = writeU16LE(p, event.target_rpm);
  p = writeU16LE(p, event.window_ms);
  *p++ = event.detail;

  const uint8_t length = static_cast<uint8_t>(p - sendingPCBytes);
  Serial.write(&PCStartMarker, 1);
  Serial.write(length);
  Serial.write(sendingPCBytes, length);
  Serial.write(&PCEndMarker, 1);
}

void sendFaultStatusAck() {
  uint8_t latchedMask = 0;
  uint8_t enabledMask = 0;
  for (uint8_t axis = 0; axis < numHWSerials; axis++) {
    if (motionMonitors[axis].latched()) latchedMask |= (1U << axis);
    if (motorEnabled[axis]) enabledMask |= (1U << axis);
  }
  char response[96];
  snprintf(response, sizeof(response),
           "V1,L=%02X,E=%02X,Q=%u,F=%u,%u,%u,%u,%u,%u",
           latchedMask, enabledMask, motionFaultSequence,
           static_cast<uint8_t>(motionMonitors[0].fault()),
           static_cast<uint8_t>(motionMonitors[1].fault()),
           static_cast<uint8_t>(motionMonitors[2].fault()),
           static_cast<uint8_t>(motionMonitors[3].fault()),
           static_cast<uint8_t>(motionMonitors[4].fault()),
           static_cast<uint8_t>(motionMonitors[5].fault()));
  sendAckIfPending(response);
}

void reportFaultStatusEvents() {
  for (uint8_t axis = 0; axis < numHWSerials; axis++) {
    if (!motionMonitors[axis].latched()) continue;
    StallDetector::Event event;
    event.transition = StallDetector::STATUS;
    event.fault = motionMonitors[axis].fault();
    event.commanded_velocity = targetVel[axis];
    event.measured_velocity = currentVel[axis];
    event.target_rpm = targetRPM[axis];
    reportMotionEvent(axis, coupledAxis(axis), event);
  }
}

void confirmDriverCommunicationFault(uint8_t axis, uint8_t stage) {
  if (motionMonitors[axis].latched()) return;
  StallDetector::Event event;
  event.transition = StallDetector::CONFIRMED;
  event.fault = StallDetector::DRIVER_COMMUNICATION;
  event.commanded_velocity = targetVel[axis];
  event.measured_velocity = currentVel[axis];
  event.target_rpm = targetRPM[axis];
  event.detail = stage;
  motionMonitors[axis].forceLatch(event.fault, millis());
  stopMotorAxis(axis, true);
  const uint8_t coupled = coupledAxis(axis);
  if (coupled < numHWSerials) {
    stopMotorAxis(coupled, true);
    motionMonitors[coupled].forceLatch(event.fault, millis());
  }
  reportMotionEvent(axis, coupled, event);
}

void updateMotionMonitors() {
  const uint32_t now = millis();
  for (uint8_t axis = 0; axis < numHWSerials; axis++) {
    const float unitsPerCount = EncRes * jointPRate[axis];
    StallDetector::Event event =
        motionMonitors[axis].update(now, EncCounts[axis], unitsPerCount);
    if (event.transition == StallDetector::NO_EVENT) {
      continue;
    }

    const uint8_t coupled = coupledAxis(axis);
    if (event.transition == StallDetector::CONFIRMED &&
        FAULT_STOP_ENABLED) {
      stopMotorAxis(axis, true);
      if (coupled < numHWSerials) {
        stopMotorAxis(coupled, true);
        motionMonitors[coupled].forceLatch(event.fault, now);
      }
    }
    reportMotionEvent(axis, coupled, event);
  }
}

static inline void writeMotorCommand(
    uint8_t axis, const char* command, size_t length) {
  HWSerials[axis]->write(command, length);
  HWSerials[axis]->flush();
  delayMicroseconds(DRIVER_DELAY_us);
}

static inline void writeMotorCommandNoDelay(
    uint8_t axis, const char* command, size_t length) {
  HWSerials[axis]->write(command, length);
  HWSerials[axis]->flush();
}

static inline void drainMotorResponses(uint8_t axis) {
  while (HWSerials[axis]->available() > 0) {
    HWSerials[axis]->read();
  }
  newHWMsg[axis] = false;
}

static bool waitForMotorAck(uint8_t axis) {
  const uint32_t start = millis();
  bool inFrame = false;
  while (millis() - start < MOTOR_ACK_TIMEOUT_MS) {
    while (HWSerials[axis]->available() > 0) {
      const uint8_t value = HWSerials[axis]->read();
      if (!inFrame) {
        inFrame = value == respMarker;
      } else if (value == endMarker) {
        return true;
      }
    }
  }
  return false;
}

static bool writeMotorCommandWithAck(
    uint8_t axis, const char* command, size_t length) {
  return DriverAckRetry::run(
      MOTOR_ACK_ATTEMPTS,
      [&]() {
        drainMotorResponses(axis);
        writeMotorCommand(axis, command, length);
        return waitForMotorAck(axis);
      },
      [&](uint8_t) {
        // Discard a response that completed just after the prior timeout, then
        // give the driver a bounded recovery interval before retransmission.
        drainMotorResponses(axis);
        delay(MOTOR_ACK_RETRY_DELAY_MS);
      });
}

static inline void stopMotorAxis(uint8_t axis, bool force) {
  if (force || motorEnabled[axis]) {
    writeMotorCommand(axis, "$O0\n", 4);
  }
  motorEnabled[axis] = false;
  speedValid[axis] = false;
  currentRPM[axis] = 0;
  // Reassert direction before this motor is enabled again. This prevents the
  // firmware cache from surviving a stall, watchdog stop, or driver power cycle.
  directionValid[axis] = false;
}

static bool ensureMotorDirection(uint8_t axis) {
  const bool changed = !directionValid[axis] ||
                       targetDir[axis] != currentDir[axis];

  if (changed) {
    stopMotorAxis(axis, true);
    // The stop command is intentionally unacknowledged. Let the driver finish
    // processing it before draining its response and issuing direction.
    delay(MOTOR_DIRECTION_SETTLE_MS);
  }
  if (changed) {
    // Remove acknowledgements for earlier C/O commands so the next complete
    // response belongs to this direction command.
    char *p = sendingHWChars[axis];
    *p++ = startMarker;
    *p++ = 'S';
    *p++ = targetDir[axis];
    *p++ = endMarker;
    if (!writeMotorCommandWithAck(
            axis, sendingHWChars[axis], p - sendingHWChars[axis])) {
      directionValid[axis] = false;
      stopMotorAxis(axis, true);
      confirmDriverCommunicationFault(axis, 'S');
      return false;
    }
    currentDir[axis] = targetDir[axis];
    directionValid[axis] = true;
  }
  return directionValid[axis];
}

void sendMotorCmds() {
  static char *p;
  if (pc_connected && newJointVelCmd) {
    for (uint8_t axis = 0; axis < numHWSerials; axis++) {
      if (FAULT_STOP_ENABLED && motionMonitors[axis].latched()) {
        continue;
      }
      if (targetRPM[axis] == 0) {
        // Stop only on the non-zero -> zero transition. Repeated command
        // heartbeats must not flood every driver with redundant O0 frames.
        stopMotorAxis(axis, false);
        motionMonitors[axis].setCommand(
            millis(), 0, targetDir[axis], 0.0f);
        continue;
      }

      if (!ensureMotorDirection(axis)) {
        continue;
      }

      if (!speedValid[axis] || currentRPM[axis] != targetRPM[axis]) {
        p = sendingHWChars[axis];
        *p++ = startMarker;
        *p++ = 'C';
        p = write_uint16(p, targetRPM[axis]);
        *p++ = endMarker;
        if (!writeMotorCommandWithAck(
                axis, sendingHWChars[axis], p - sendingHWChars[axis])) {
          confirmDriverCommunicationFault(axis, 'C');
          continue;
        }
        currentRPM[axis] = targetRPM[axis];
        speedValid[axis] = true;
      }

      if (!motorEnabled[axis]) {
        if (!writeMotorCommandWithAck(axis, "$O1\n", 4)) {
          confirmDriverCommunicationFault(axis, 'O');
          continue;
        }
        motorEnabled[axis] = true;
      }
      motionMonitors[axis].setCommand(
          millis(), targetRPM[axis], targetDir[axis], targetVel[axis]);
    }
    newJointVelCmd = false;

  } else if (newJointPosCmd) {
    for (uint8_t axis = 0; axis < numHWSerials; axis++) {
      if (FAULT_STOP_ENABLED && motionMonitors[axis].latched()) {
        continue;
      }
      if (!ensureMotorDirection(axis)) {
        continue;
      }

      if (newTargetVelCmd || !speedValid[axis] ||
          currentRPM[axis] != targetRPM[axis]) {
        p = sendingHWChars[axis];
        *p++ = startMarker;
        *p++ = 'C';
        p = write_uint16(p, targetRPM[axis]);
        *p++ = endMarker;
        if (!writeMotorCommandWithAck(
                axis, sendingHWChars[axis], p - sendingHWChars[axis])) {
          confirmDriverCommunicationFault(axis, 'C');
          continue;
        }
        currentRPM[axis] = targetRPM[axis];
        speedValid[axis] = true;
      }

      p = sendingHWChars[axis];
      *p++ = startMarker;
      *p++ = 'A';
      p = write_uint16(p, targetDeg[axis]);
      *p++ = endMarker;
      if (!writeMotorCommandWithAck(
              axis, sendingHWChars[axis], p - sendingHWChars[axis])) {
        confirmDriverCommunicationFault(axis, 'A');
        continue;
      }

      if (!motorEnabled[axis]) {
        if (!writeMotorCommandWithAck(axis, "$O1\n", 4)) {
          confirmDriverCommunicationFault(axis, 'O');
          continue;
        }
        motorEnabled[axis] = true;
      }
      motionMonitors[axis].setCommand(
          millis(), targetRPM[axis], targetDir[axis], targetVel[axis]);
    }
    newJointPosCmd = false;
    newTargetVelCmd = false;
  }
}

static inline char* write_uint16(char *p, uint16_t v) {
    char tmp[5];
    uint8_t i = 0;

    do {
        tmp[i++] = '0' + (v % 10);
        v /= 10;
    } while (v);

    while (i--)
        *p++ = tmp[i];

    return p;
}

// =======================================================================

// static inline void set_zero_vel()
// {
//   for (uint8_t axis = 0; axis < numHWSerials; axis++){
//     HWSerials[axis]->write("$C0\n", 4);
//   }
// }

// void float_sign_abs(float x, uint8_t &sign, float &abs_x) {
//     static uint32_t bits = 1;

//     // copy float bits to uint32_t
//     memcpy(&bits, &x, sizeof(bits));

//     sign = (bits >> 31) & 1;

//     // clear sign bit for absolute value
//     bits &= 0x7FFFFFFF;

//     // copy back to float
//     memcpy(&abs_x, &bits, sizeof(abs_x));
// }
