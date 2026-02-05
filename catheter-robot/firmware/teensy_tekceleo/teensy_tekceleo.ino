#include <QuadEncoder.h>
#include <Encoder.h>
#include <cmath>

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

constexpr size_t BUFFER_LEN = 32;
uint8_t receivedHWBytes[numHWSerials][BUFFER_LEN];
uint8_t receivedPCBytes[BUFFER_LEN];
size_t PCBytes_len = 0;
size_t HWBytes_len[numHWSerials] = { 0 };
uint8_t sendingPCBytes[BUFFER_LEN];
char sendingHWChars[numHWSerials][BUFFER_LEN];
size_t numsendingHWChars[numHWSerials] = { 0 };
bool newHWMsg[numHWSerials] = { false };
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
constexpr uint8_t SET_VEL = 'F';
constexpr uint8_t DEBUG = 'D';
constexpr uint8_t CONNECT = 'C';
constexpr uint8_t LIMIT = 'L';
constexpr uint8_t LIMIT_P = 'c';
constexpr uint8_t LIMIT_C = 'b';
constexpr uint8_t LIMIT_M = 'a';
constexpr uint8_t LIMIT_N = 'n';

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
QuadEncoder Enc1(1, 0, 1); // A, B
QuadEncoder Enc2(2, 2, 3);
QuadEncoder Enc4(3, 4, 30);
QuadEncoder Enc5(4, 31, 33);
Encoder Enc0(26, 23);  // B, A
Encoder Enc3(32, 27);
QuadEncoder *HWEncoders[] = {&Enc1, &Enc2, &Enc4, &Enc5};
Encoder *SWEncoders[] = {&Enc0, &Enc3};

constexpr float EncRes = 0.045f; // deg
constexpr uint16_t maxRPM = 250;
long EncCounts[numHWSerials] = { 0 };
float EncPos[numHWSerials] = { 0.0f };
// float jointPos[numHWSerials] = { 0.0f };
// float jointVel[numHWSerials] = { 0.0f };

// Joint transmission, joint/motor, positive direction: right-hand or forward
constexpr float linRate = 67.319841f; // mm/rev
constexpr float rotRate = 0.375f; // rev/rev
constexpr float catheterBendRate = -1.190625f; // mm/rev
constexpr float sheathBendRate = 0.375f; // rev/rev
constexpr float jointPRate[numHWSerials] = { linRate/360.0f, rotRate, catheterBendRate/360.0f, 
                                            linRate/360.0f, rotRate, sheathBendRate }; // mm/deg, deg/deg
constexpr float jointVRate[numHWSerials] = { linRate/60.0f, rotRate/60.0f, catheterBendRate/60.0f, 
                                            linRate/60.0f, rotRate/60.0f, sheathBendRate/60.0f }; // mm*min/rev*s, rev*min/rev*s
constexpr float catheterBendUB = 20.8f;
constexpr float catheterBendLB = 0.0f;
constexpr float sheathBendUB = 360.0f;
constexpr float sheathBendLB = -360.0f;

// State Variables
int limitState[5] = { 0 };
// long targetCounts[numHWSerials];
// long targetHz[numHWSerials];
float targetPos[numHWSerials] = { 0.0f };
uint16_t targetDeg[numHWSerials] = { 0 };
float currentPos[numHWSerials] = { 0.0f }; // decoupled
float currentSheathBendPos = 0.0f; // coupled
float targetVel[numHWSerials] = { 0.0f };
uint16_t targetRPM[numHWSerials] = { 0 };
uint8_t targetDir[numHWSerials] = { '1' }; // '0': right-hand, '1': left-hand
uint8_t currentDir[numHWSerials] = { '1' };

// Timing
elapsedMillis sinceLastCycle;
elapsedMillis sinceLastPCMsg;
constexpr uint32_t PC_SILENCE_MS = 10000;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  // set up motor communications
  for (uint8_t i = 0; i < numHWSerials; i++) {
    // HWSerials[i]->transmitterEnable(TE[i]);
    HWSerials[i]->begin(115200);

    // set motors in closed-loop mode
    HWSerials[i]->write("$L1\n", 4);
    snprintf(sendingHWChars[i], sizeof(sendingHWChars[i]), "$S%u\n", currentDir[i]);
    HWSerials[i]->write(sendingHWChars[i]);
    sendingHWChars[i][0] = '\0';
  }

  // set up encoders
  for (uint8_t i = 0; i < 2; i++) {
    HWEncoders[i]->setInitConfig();
    // HWEncoders[i]->EncConfig.positionInitialValue = EncCounts[i];
    HWEncoders[i]->init();
  }

  for (uint8_t i = 0; i < 3; i++) {
    pinMode(LimitSwitch[i], INPUT_PULLUP);
  }
}

void loop() {
    // Asychronous I/O 
    recvPCSerial();
    procPCBytes();
    recvHWSerials();
    procHWBytes();
    
    // control Cycle 100Hz
    if (sinceLastCycle > 10) {
      sinceLastCycle = 0;
      readEncoders();
      CheckLimitSwitches();
      sendMotorCmds();
    }

    // watchdog
    if (sinceLastPCMsg > PC_SILENCE_MS) {
      for (uint8_t i = 0; i < numHWSerials; i++)
        HWSerials[i]->print("$O0\n");
    }
}

void recvHWSerials() {
    static bool recvInProgress[numHWSerials] = {false};
    static uint8_t ndx[numHWSerials] = {0};
    uint8_t rb;

    for (uint8_t i = 0; i < numHWSerials; i++){
      while (HWSerials[i]->available() > 0 && newHWMsg[i] == false) {
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
    static uint8_t ndx = 0;
    uint8_t rb;

    while (Serial.available() > 0 && newPCMsg == false) {
      rb = Serial.read();

      if (recvInProgress == true) {
        if (rb != PCEndMarker) {
          receivedPCBytes[ndx] = rb;
          ndx++;
          if (ndx >= BUFFER_LEN) {
            ndx = BUFFER_LEN - 1;
          }
        }
        else {
          // receivedPCBytes[ndx] = '\0'; // terminate the string
          PCBytes_len = ndx;
          recvInProgress = false;
          ndx = 0;
          newPCMsg = true;
          sinceLastPCMsg = 0;
        }
      }

      else if (rb == PCStartMarker) {
        recvInProgress = true;
      }
    }
}

static inline bool hasUUID(uint8_t cmd) {
  return (cmd != VEL && cmd != POS && cmd != CONNECT);
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

void sendAckIfPending() {
  if (!pendingAck.valid) return;

  Serial.write(&PCStartMarker, 1);
  Serial.write(&pendingAck.prefix, 1);
  Serial.write(pendingAck.uuid, REQ_ID_LEN);
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
          newJointPosCmd = false;
          for (uint8_t axis = 0; axis < numHWSerials; axis++) {
            memcpy(&targetVel[axis], receivedPCBytes + 1 + axis*4, 4); // float size 4
            newJointVelCmd = isVelCmdValid(axis);
            if (!newJointVelCmd) { break; }

            if (axis == 5) { targetVel[5] += targetVel[4]; } // decouple rot and bend for sheath

            temp = targetVel[axis]/jointVRate[axis];

            targetDir[axis] = (temp >= 0.0f) ? '1' : '0';
            targetRPM[axis] = static_cast<uint16_t>(std::roundf(std::fabs(temp)));

            if (targetRPM[axis] > maxRPM) { targetRPM[axis] = maxRPM; }
          }
          break;
        }

        case POS: {// 'P'
          newJointVelCmd = false;
          for (uint8_t axis = 0; axis < numHWSerials; axis++) {
            memcpy(&targetPos[axis], receivedPCBytes + 1 + axis*4, 4); // float size 4
            newJointPosCmd = isPosCmdValid(axis);
            if (!newJointPosCmd) { break; }

            if (axis == 5) { targetPos[5] += targetPos[4]; } // decouple rot and bend for sheath

            temp = (targetPos[axis] - currentPos[axis])/jointPRate[axis];

            targetDir[axis] = (temp >= 0.0f) ? '1' : '0';
            targetDeg[axis] = static_cast<uint16_t>(std::roundf(std::fabs(temp)));
          }
          break;
        }

        case SET_VEL: {// 'F', set target velocity for pos control
          for (uint8_t axis = 0; axis < numHWSerials; axis++) {
            memcpy(&targetVel[axis], receivedPCBytes + 1 + REQ_ID_LEN + axis*4, 4); // float size 4

            if (axis == 5) { targetVel[5] += targetVel[4]; } // decouple rot and bend for sheath

            temp = targetVel[axis]/jointVRate[axis];
            targetRPM[axis] = static_cast<uint16_t>(std::roundf(std::fabs(temp)));
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
            currentSheathBendPos = 0.0f;
          }
          sendAckIfPending();
          break;
        }

        case STOP: {// 'S', stop all motors
          for (uint8_t axis = 0; axis < numHWSerials; axis++) {
            // snprintf(sendingHWChars[axis], sizeof(sendingHWChars[axis]), "$O0\n");
            // HWSerials[axis]->print(sendingHWChars[axis]);
            HWSerials[axis]->print("$O0\n");
          }
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
          // if (PCBytes_len < 1 + REQ_ID_LEN) return;

          // uint8_t axis = receivedPCBytes[1 + REQ_ID_LEN] - '0';
          // const uint8_t* payload = receivedPCBytes + 2 + REQ_ID_LEN;
          // size_t payload_len = PCBytes_len - (2 + REQ_ID_LEN);
          // HWSerials[axis]->write(payload, payload_len);

          uint8_t axis = receivedPCBytes[1 + REQ_ID_LEN] - '0';
          const uint8_t* payload = receivedPCBytes + 2 + REQ_ID_LEN;
          size_t payload_len = PCBytes_len - 2 - REQ_ID_LEN;
          HWSerials[axis]->write(&startMarker, 1);
          HWSerials[axis]->write(payload, payload_len);
          HWSerials[axis]->write(&endMarker, 1);
          
          break;
        }

        case CONNECT: { // 'C', confirm PC connection
          digitalWrite(LED_BUILTIN, HIGH);
          sendAckIfPending();
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
  ; // TODO

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
  EncCounts[0] = Enc0.read();
  EncCounts[1] = Enc1.read();
  EncCounts[2] = Enc2.read();
  EncCounts[3] = Enc3.read();
  EncCounts[4] = Enc4.read();
  EncCounts[5] = Enc5.read();
  for (uint8_t axis = 0; axis < numHWSerials; axis++) {
    // EncPos[axis] = EncCounts[axis]*EncRes;
    currentPos[axis] = EncCounts[axis]*EncRes*jointPRate[axis];
  }
  // coupled rot and bend for sheath
  currentSheathBendPos = currentPos[5] - currentPos[4];

  sendingPCBytes[0] = PCStartMarker;
  sendingPCBytes[1] = POS;
  memcpy(sendingPCBytes + 2, currentPos, 20); // 5*float
  memcpy(sendingPCBytes + 22, &currentSheathBendPos, 4); // 1*float
  sendingPCBytes[26] = PCEndMarker;
  Serial.write(sendingPCBytes, 27);
}

void CheckLimitSwitches() {
  static bool trigger_event = false;
  static int temp = 0;
  static uint8_t *p;

  p = sendingPCBytes;
  *p++ = LIMIT;

  // linear physical switches
  for (uint8_t i = 0; i < 3; i++) {
    temp = digitalRead(LimitSwitch[i]);
    if (temp != limitState[i]) {
      limitState[i] = temp;
      trigger_event = true;
      *p++ = LIMIT_C + limitState[i];
    } else {
      *p++ = LIMIT_N;
    }
  }

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
    *p++ = LIMIT_C + limitState[3];
  } else {
    *p++ = LIMIT_N;
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
    *p++ = limitState[4] + LIMIT_C;
  } else {
    *p++ = LIMIT_N;
  }

  // digitalWrite(LED_BUILTIN, limitState[0]);
  if (trigger_event) {
    for (uint8_t axis = 0; axis < numHWSerials; axis++) {
      HWSerials[axis]->write("$O0\n", 4);
    }
    Serial.write(&PCStartMarker, 1);
    Serial.write(sendingPCBytes, 6);
    Serial.write(&PCEndMarker, 1);
  }
}

void sendMotorCmds() {
  static char *p;
  if (newJointVelCmd) {
    for (uint8_t axis = 0; axis < numHWSerials; axis++) {
      p = sendingHWChars[axis];

      if (targetDir[axis] != currentDir[axis]) {
        // ("$S%u\n", targetDir[axis])
        *p++ = startMarker;
        *p++ = 'S';
        *p++ = targetDir[axis];
        *p++ = endMarker;
        currentDir[axis] = targetDir[axis];
      }

      *p++ = startMarker;
      *p++ = 'C';
      p = write_uint16(p, targetRPM[axis]);
      *p++ = endMarker;

      HWSerials[axis]->write(sendingHWChars[axis], p - sendingHWChars[axis]);
    }
    newJointVelCmd = false;

  } else if (newJointPosCmd) {
    for (uint8_t axis = 0; axis < numHWSerials; axis++) {
      p = sendingHWChars[axis];
      

      if (targetDir[axis] != currentDir[axis]) {
        // ("$S%u\n", targetDir[axis])
        *p++ = startMarker;
        *p++ = 'S';
        *p++ = targetDir[axis];
        *p++ = endMarker;
        currentDir[axis] = targetDir[axis];
      }

      if (newTargetVelCmd) {
        *p++ = startMarker;
        *p++ = 'C';
        p = write_uint16(p, targetVel[axis]);
        *p++ = endMarker;
      }

      *p++ = startMarker;
      *p++ = 'A';
      p = write_uint16(p, targetDeg[axis]);
      *p++ = endMarker;

      HWSerials[axis]->write(sendingHWChars[axis], p - sendingHWChars[axis]);
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

// void printCharMsg() {
//   for (uint8_t i = 0; i < numHWSerials; i++) {
//     if (newHWMsg[i]) {
//       snprintf(sendingPCChars, sizeof(sendingPCChars), "Serial%d received: ", i+1);
//       Serial.print(sendingPCChars);
//       Serial.write(receivedHWBytes[i], HWBytes_len[i]);
//       Serial.print("\r\n");
//       newHWMsg[i] = false;
//     }
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