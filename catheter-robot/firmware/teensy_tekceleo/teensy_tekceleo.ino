#include <QuadEncoder.h>
#include <Encoder.h>

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
constexpr uint8_t TE[] = { 5, 6, 12, 18, 19, 22 };
constexpr uint8_t numHWSerials = 6;
constexpr char startMarker = '$';
constexpr char endMarker = '\x0A'; // <LF>, '\n'
constexpr char PCCharStartMarker = '<';
constexpr char PCCharEndMarker = '>';
constexpr uint8_t PCByteStartMarker = 0x3C;  // '<'
constexpr uint8_t PCByteEndMarker = 0x3E;    // '>'
constexpr uint8_t numChars = 32;
constexpr uint8_t numBytes = 32;
char receivedHWChars[numHWSerials][numChars];
char receivedPCChars[numChars];
uint8_t receivedPCBytes[numBytes];
uint8_t numRecvdBytes = 0;
char sendingPCChars[numChars];
uint8_t sendingPCBytes[numBytes];
char sendingHWChars[numHWSerials][numChars];
uint8_t numSendingHWChars[numHWSerials] = { 0 };
bool newHWMsg[numHWSerials] = { false };
bool newPCMsg = false;

// Quadrature decoder
QuadEncoder Enc1(1, 0, 1); // A, B
QuadEncoder Enc2(2, 2, 3);
QuadEncoder Enc4(3, 4, 30);
QuadEncoder Enc5(4, 31, 33);
Encoder Enc3(26, 23);  // B, A
Encoder Enc6(32, 27);
QuadEncoder *HWEncoders[] = {&Enc1, &Enc2, &Enc4, &Enc5};
Encoder *SWEncoders[] = {&Enc3, &Enc6};

// Encoder Enc1(1, 0); // B, A
// Encoder Enc2(3, 2);
// QuadEncoder *HWEncoders[] = {&Enc4, &Enc5};
// Encoder *SWEncoders[] = {&Enc1, &Enc2, &Enc3, &Enc6};
long EncCounts[numHWSerials] = { 10,20,30,40,50,60 };
// long PMD401EncCounts[numHWSerials];
double EncPositions[numHWSerials];

// Variables
int isLimitSwitchBlocked[3] = { 0 };
// double targetPositions[numHWSerials];
long targetCounts[numHWSerials];
long targetHz[numHWSerials];
uint16_t targetRPM[numHWSerials] = { 0 };
uint8_t rotationDirection[numHWSerials] = { 1 };


// Timing
elapsedMillis sincePrintEnc;
// elapsedMicros sinceRecvPCMsg;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  // set up motor communications
  for (uint8_t i = 0; i < numHWSerials; i++) {
    HWSerials[i]->transmitterEnable(TE[i]);
    HWSerials[i]->begin(115200);

    // set motors in closed-loop mode
    snprintf(sendingHWChars[i], sizeof(sendingHWChars[i]), "$L1\n");
    HWSerials[i]->print(sendingHWChars[i]);
    snprintf(sendingHWChars[i], sizeof(sendingHWChars[i]), "$S%u\n", rotationDirection[i]);
    HWSerials[i]->print(sendingHWChars[i]);
    sendingHWChars[i][0] = '\0';
    // Serial.println(i);
  }

  // set up encoders
  for (uint8_t i = 0; i < 2; i++) {
    HWEncoders[i]->setInitConfig();
    // HWEncoders[i]->EncConfig.positionInitialValue = EncCounts[i];
    HWEncoders[i]->init();
  }

  // Sync encoder count with PMD401
  // alignEncoders();

  for (uint8_t i = 0; i < 3; i++) {
    pinMode(LimitSwitch[i], INPUT_PULLUP);
  }
}

void loop() {
    // put your main code here, to run repeatedly:
    CheckLimitSwitches();
    // digitalWrite(LED_BUILTIN, HIGH);

    
    recvSerialBytes();
    procPCBytes();
    
    if (sincePrintEnc > 100) {
      readEncoders();
      sincePrintEnc = 0;
    }

    // debug
    // recvSerialChars();
    // procPCChars();
    // recvHWSerials();
    // printCharMsg();
}

void recvHWSerials() {
    static bool recvInProgress[numHWSerials] = {false};
    static uint8_t ndx[numHWSerials] = {0};
    char rc;

    for (uint8_t i = 0; i < numHWSerials; i++){
      while (HWSerials[i]->available() > 0 && newHWMsg[i] == false) {
          rc = HWSerials[i]->read();

          if (recvInProgress[i] == true) {
              if (rc != endMarker) {
                  receivedHWChars[i][ndx[i]] = rc;
                  ndx[i]++;
                  if (ndx[i] >= numChars) {
                      ndx[i] = numChars - 1;
                  }
              }
              else {
                  receivedHWChars[i][ndx[i]] = '\0'; // terminate the string
                  recvInProgress[i] = false;
                  ndx[i] = 0;
                  newHWMsg[i] = true;
              }
          }

          else if (rc == startMarker) {
              recvInProgress[i] = true;
              receivedHWChars[i][0] = rc; // including start marker for hardware serial
              ndx[i]++;
          }
      }
    }
}

void recvSerialChars() {
    static bool recvInProgress = false;
    static uint8_t ndx = 0;
    char rc;

    while (Serial.available() > 0 && newPCMsg == false) {
      rc = Serial.read();

      if (recvInProgress == true) {
        if (rc != PCCharEndMarker) {
          receivedPCChars[ndx] = rc;
          ndx++;
          if (ndx >= numChars) {
            ndx = numChars - 1;
          }
        }
        else {
          receivedPCChars[ndx] = '\0'; // terminate the string
          // numRecvdBytes = ndx;
          recvInProgress = false;
          ndx = 0;
          newPCMsg = true;
          // sinceRecvPCMsg = 0;
        }
      }

      else if (rc == PCCharStartMarker) {
        recvInProgress = true;
      }
    }
}

void recvSerialBytes() {
    static bool recvInProgress = false;
    static uint8_t ndx = 0;
    uint8_t rb;

    while (Serial.available() > 0 && newPCMsg == false) {
      rb = Serial.read();

      if (recvInProgress == true) {
        if (rb != PCByteEndMarker) {
          receivedPCBytes[ndx] = rb;
          ndx++;
          if (ndx >= numBytes) {
            ndx = numBytes - 1;
          }
        }
        else {
          receivedPCBytes[ndx] = '\0'; // terminate the string
          numRecvdBytes = ndx;
          recvInProgress = false;
          ndx = 0;
          newPCMsg = true;
          // sinceRecvPCMsg = 0;
        }
      }

      else if (rb == PCByteStartMarker) {
        recvInProgress = true;
      }
    }
}

bool alignEncoders() {
  // uint8_t failedAxis = 0;
  uint32_t startTime;
  for (uint8_t i = 0; i < numHWSerials; i++) {
    snprintf(sendingHWChars[i], sizeof(sendingHWChars[i]), "X%dE\r", i+1);
    HWSerials[i]->print(sendingHWChars[i]);
    startTime = millis();
    while (newHWMsg[i] == false) {
      recvHWSerials();
      if (newHWMsg[i] == false && millis() - startTime > 500) {
        // failedAxis = i+1;
        return false;
      }
    }
    sscanf(receivedHWChars[i], "X%*dE:%ld\r", &EncCounts[i]); // should return 1
    newHWMsg[i] = false;
  }
  // for (uint8_t i = 0; i < 3; i++) {
  //   SWEncoders[i]->write(EncCounts[i]);
  // }
  // for (uint8_t i = 0; i < 2; i++) {
  //   HWEncoders[i]->write(EncCounts[i+3]);
  // }
  // for (uint8_t i = 0; i < 1; i++) {
  //   SWEncoders[i+3]->write(EncCounts[i+5]);
  // }
  return true;
}

void readEncoders() {
  for (uint8_t i = 0; i < 3; i++) {
    EncCounts[i] = SWEncoders[i]->read();
    // snprintf(sendingPCChars, sizeof(sendingPCChars), "Enc%d: ", i+1);
    // Serial.print(sendingPCChars);
    // Serial.println(EncCounts[i]);
  }
  for (uint8_t i = 0; i < 2; i++) {
    EncCounts[i+3] = HWEncoders[i]->read();
    // snprintf(sendingPCChars, sizeof(sendingPCChars), "Enc%d: ", i+4);
    // Serial.print(sendingPCChars);
    // Serial.println(EncCounts[i+3]);
  }
  for (uint8_t i = 0; i < 1; i++) {
    EncCounts[i+5] = SWEncoders[i+3]->read();
    // snprintf(sendingPCChars, sizeof(sendingPCChars), "Enc%d: ", i+6);
    // Serial.print(sendingPCChars);
    // Serial.println(EncCounts[i+5]);
  }

  // uint32_t startTime;
  // for (uint8_t i = 0; i < numHWSerials; i++) {
  //   snprintf(sendingHWChars[i], sizeof(sendingHWChars[i]), "X%dE\r", i+1); //i+1
  //   HWSerials[i]->print(sendingHWChars[i]);
  //   startTime = millis();
  //   while (newHWMsg[i] == false) {
  //     recvHWSerials();
  //     if (newHWMsg[i] == false && millis() - startTime > 100) {
  //       // failedAxis = i+1;
  //       // return false;
  //       break;
  //     }
  //   }
  //   sscanf(receivedHWChars[i], "X%*dE:%ld\r", &EncCounts[i]); // should return 1
  //   newHWMsg[i] = false;
  // }

  sendingPCBytes[0] = PCByteStartMarker;
  sendingPCBytes[1] = 0x45; // 'E'
  memcpy(sendingPCBytes + 2, EncCounts, sizeof(EncCounts));
  sendingPCBytes[2 + sizeof(EncCounts)] = PCByteEndMarker;
  Serial.write(sendingPCBytes, 3 + sizeof(EncCounts));
}

void procPMD401Msg() {
  ;
}

void procPCChars() {
  static uint8_t axis = 0;
  uint8_t pos = 0;
  if (newPCMsg) {
    // Serial.println(receivedPCChars);
    // Serial1.print("X1E\r");
    switch (receivedPCChars[0]) {

      case startMarker: // Directly sending command to PMD401
        pos = 0;
        for (uint8_t i = 0; receivedPCChars[i] != '\0'; i++) {
          if (receivedPCChars[i] == startMarker && receivedPCChars[i + 1] != '\0') { // check next char exists
            // TODO: check axis validity, including the scenario that 'X' is the last byte
            if (i != 0) {
              sendingHWChars[axis][pos++] = endMarker;
              numSendingHWChars[axis] = pos;
              sendingHWChars[axis][pos] = '\0';
            }
            axis = receivedPCChars[i + 1] - '0' - 1;
            sendingHWChars[axis][0] = startMarker;
            pos = 1;
          }
          else {
            sendingHWChars[axis][pos++] = receivedPCChars[i];
            if (receivedPCChars[i + 1] == '\0') {
              sendingHWChars[axis][pos++] = endMarker;
              numSendingHWChars[axis] = pos;
              sendingHWChars[axis][pos] = '\0';
            }
          }
        }
        // for (uint8_t i = 0; i < numHWSerials; i++) {
          // Serial.write((const uint8_t*)sendingHWChars[i], numSendingHWChars[i]);
          // Serial.write('\n');
          // Serial.write(0);
          // Serial.println(sendingHWChars[i]);
        // }
        sendCmd();
        // Serial1.print("X1EX2E\r");
        break;

      default:
        break;
    }
  }
  newPCMsg = false;
}

void procPCBytes() {
  // static uint8_t axis = 0;
  // uint8_t pos = 0;
  static uint8_t targetDirection = 1;

  if (newPCMsg) {

    for (uint8_t i = 0; i < numRecvdBytes; i++){ // only iterates through predicates
      // digitalWrite(LED_BUILTIN,HIGH);
      switch (receivedPCBytes[i]) {

        case 0x43: // 'C'
          for (uint8_t axis = 0; axis < numHWSerials; axis++) {
            // targetCounts[axis] = (long)receivedPCBytes[++i];
            // targetCounts[axis] |= (long)receivedPCBytes[++i] << 8;
            // targetCounts[axis] |= (long)receivedPCBytes[++i] << 16;
            // targetCounts[axis] |= (long)receivedPCBytes[++i] << 24;
            targetDirection = receivedPCBytes[++i];
            if (targetDirection != rotationDirection[axis]) {
              snprintf(sendingHWChars[axis], sizeof(sendingHWChars[axis]), "$S%u\n", targetDirection);
              rotationDirection[axis] = targetDirection;
            }
            targetRPM[axis] = (uint16_t)receivedPCBytes[++i];
            targetRPM[axis] |= (uint16_t)receivedPCBytes[++i] << 8;
            snprintf(sendingHWChars[axis], sizeof(sendingHWChars[axis]), "$C%hu\n", targetPRM[axis]);
            HWSerials[axis]->print(sendingHWChars[axis]);
          }
          break;

        case 0x4A: // 'F'
          digitalWrite(LED_BUILTIN,HIGH);
          for (uint8_t axis = 0; axis < numHWSerials; axis++) {
            targetHz[axis] = (long)receivedPCBytes[++i];
            targetHz[axis] |= (long)receivedPCBytes[++i] << 8;
            targetHz[axis] |= (long)receivedPCBytes[++i] << 16;
            targetHz[axis] |= (long)receivedPCBytes[++i] << 24;
            snprintf(sendingHWChars[axis], sizeof(sendingHWChars[axis]), "$F%ld\n", targetHz[axis]);
            HWSerials[axis]->print(sendingHWChars[axis]);
          }
          break;

        case 0x41: // 'A' check motor serial connection and align encoder readings
          bool aligned = false;
          while (!aligned) {
            aligned = alignEncoders();
          }
          sendingPCBytes[0] = PCByteStartMarker;
          sendingPCBytes[1] = 0x41; // 'A'
          sendingPCBytes[2] = PCByteEndMarker;
          Serial.write(sendingPCBytes, 3);
          // Unpark motors
          for (uint8_t axis = 0; axis < numHWSerials; axis++) {
            snprintf(sendingHWChars[axis], sizeof(sendingHWChars[axis]), "X%dS\r", axis+1);
            HWSerials[axis]->print(sendingHWChars[axis]);
          }
          break;

        case 0x5A: // 'Z', set all encoders to 0
          // for (uint8_t axis = 0; axis < numHWSerials; axis++) {
          //   snprintf(sendingHWChars[axis], sizeof(sendingHWChars[axis]), "X%dE0\r", axis+1);
          //   HWSerials[axis]->print(sendingHWChars[axis]);
          // }

          for (uint8_t i = 0; i < 4; i++) {
            HWEncoders[i]->write(0);
          }
          for (uint8_t i = 0; i < 2; i++) {
            SWEncoders[i+4]->write(0);
          }
          break;

        case 0x53: // 'S', stop all motors
          for (uint8_t axis = 0; axis < numHWSerials; axis++) {
            snprintf(sendingHWChars[axis], sizeof(sendingHWChars[axis]), "$O0\n");
            HWSerials[axis]->print(sendingHWChars[axis]);
          }
          break;

        case 0x49: // 'I', initialize all motors
          for (uint8_t axis = 0; axis < numHWSerials; axis++) {
            snprintf(sendingHWChars[axis], sizeof(sendingHWChars[axis]), "$O1\n");
            HWSerials[axis]->print(sendingHWChars[axis]);
          }
          break;

        default:
          break;
      }
    }
  }
  newPCMsg = false;
  // numRecvdBytes = 0;
}

void CheckLimitSwitches() {
  bool trigger_event = false;
  int temp = 0;
  for (uint8_t i = 0; i < 3; i++) {
    temp = digitalRead(LimitSwitch[i]);
    if (temp != isLimitSwitchBlocked[i]) {
      isLimitSwitchBlocked[i] = temp;
      trigger_event = true;
    }
  }
  // digitalWrite(LED_BUILTIN, isLimitSwitchBlocked[0]);
  if (trigger_event) {
    sendingPCBytes[0] = PCByteStartMarker;
    sendingPCBytes[1] = 0x4C; // 'L'
    for (uint8_t i = 0; i < 3; i++) {
      sendingPCBytes[i+2] = isLimitSwitchBlocked[i]? 0x31 : 0x30; // 0:1
    }
    sendingPCBytes[5] = PCByteEndMarker;
    Serial.write(sendingPCBytes, 6);
  }
}

void sendCmd() {
  for (uint8_t i = 0; i < numHWSerials; i++) {
    sendingHWChars[i][numSendingHWChars[i]] = '\0';
    HWSerials[i]->print(sendingHWChars[i]);
    // Serial.println(sendingHWChars[i]);
    // HWSerials[i]->write((const uint8_t*)sendingHWChars[i], numSendingHWChars[i]);
    numSendingHWChars[i] = 0;
    // delayMicroseconds(1000); // for single RS485 bus
  }
}

void printCharMsg() {
  for (uint8_t i = 0; i < numHWSerials; i++) {
    if (newHWMsg[i]) {
      snprintf(sendingPCChars, sizeof(sendingPCChars), "Serial%d received: ", i+1);
      Serial.print(sendingPCChars);
      Serial.println(receivedHWChars[i]);
      newHWMsg[i] = false;
      // if (i == 4) {
      //   Serial.print("Time elasped since msg received: ");
      //   Serial.print((unsigned long)sinceRecvPCMsg);
      //   Serial.println("us");
      // }
    }
  }
}
