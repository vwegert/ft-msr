/******************************************************************************
*
* This file belongs to the Model Storage Robot project. For more details and
* the original source file, please check https://github.com/vwegert/ft-msr.
*
* This is free and unencumbered software released into the public domain.
*
* Anyone is free to copy, modify, publish, use, compile, sell, or
* distribute this software, either in source code form or as a compiled
* binary, for any purpose, commercial or non-commercial, and by any
* means.
*
* In jurisdictions that recognize copyright laws, the author or authors
* of this software dedicate any and all copyright interest in the
* software to the public domain. We make this dedication for the benefit
* of the public at large and to the detriment of our heirs and
* successors. We intend this dedication to be an overt act of
* relinquishment in perpetuity of all present and future rights to this
* software under copyright law.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
* OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* For more information, please refer to <http://unlicense.org>
*
*******************************************************************************/

// ===== symbolic names for the pins ==========================================

//   P_ are the Photon output pins (the ROBO Interface input)
const int P_READY = D0;
const int P_CMD0  = D1;
const int P_CMD1  = D2;
const int P_CMD2  = D3;
const int P_CMD3  = D4;
const int P_CMD4  = D5;

//   R_ are the ROBO Interface output pins (our input)
const int R_BUSY  = A0;
const int R_ACK   = A1;
const int R_COL0  = A3;
const int R_COL1  = A4;

// ===== constants for the commands sent to the ROBO Interface ================

const int CMD_EMPTY       = -1; // dummy that is never actually sent
const int CMD_INIT        =  0;
const int CMD_NAV_A1      =  1;
const int CMD_NAV_A2      =  2;
const int CMD_NAV_A3      =  3;
const int CMD_NAV_A4      =  4;
const int CMD_NAV_B1      =  5;
const int CMD_NAV_B2      =  6;
const int CMD_NAV_B3      =  7;
const int CMD_NAV_B4      =  8;
const int CMD_NAV_C1      =  9;
const int CMD_NAV_C2      = 10;
const int CMD_NAV_C3      = 11;
const int CMD_NAV_C4      = 12;
const int CMD_NAV_D1      = 13;
const int CMD_NAV_D2      = 14;
const int CMD_NAV_D3      = 15;
const int CMD_NAV_D4      = 16;
const int CMD_NAV_E1      = 17;
const int CMD_NAV_E2      = 18;
const int CMD_NAV_E3      = 19;
const int CMD_NAV_E4      = 20;
const int CMD_PICKUP      = 21;
const int CMD_STORE       = 22;
const int CMD_RETRIEVE    = 23;
const int CMD_CHECK_COLOR = 24;
const int CMD_DROP_A      = 25;
const int CMD_DROP_B      = 26;
const int CMD_DROP_C      = 27;
const int CMD_DROP_D      = 28;
const int CMD_DROP_E      = 29;
const int CMD_DROP_F      = 30;
const int CMD_CHECK_INPUT = 31;

// ===== constants for command timing =========================================

const int TIMEOUT_ACK      = 500;   // max ms to wait for an ACK
const int TIMEOUT_CMD      = 60000; // max ms to wait for command execution
const int DELAY_MAIN_LOOP  = 100; // ms to sleep during a main loop iteration
const int DELAY_WAIT_ACK   = 100; // ms to sleep waiting for an ACK
const int DELAY_ACK_BUSY   = 300; // ms to sleep after receiving the BUSY signal
const int DELAY_WAIT_BUSY  = 100; // ms to sleep waiting for command completion
const int DELAY_WAIT_COLOR = 100; // ms to sleep waiting for command completion

// ===== input check ==========================================================

const int INIT_INTERVAL_CHECK_INPUT = 5000; // ms to wait between input checks
unsigned long lastInputCheck = 0;

// ===== internal state =======================================================

const int STATE_READY     = 0; // ready to accept new commands
const int STATE_RECEIVING = 1; // currently receiving commands to be executed
const int STATE_RECEIVED  = 2; // queue is filled, waiting for execution start
const int STATE_BUSY      = 3; // currently executing queued commands

int state = STATE_READY;

// ===== cloud function signatures and return values ==========================

int cfSetCheckInterval(String intv); // change the check interval
int cfEnumerateBins(String dummy);   // sends a list of all storage bins
int cfEnumerateOuts(String dummy);   // sends a list of all output slots
int cfInitialize(String dummy);      // re-initialize the robot
int cfPickup(String dummy);          // take a piece from the input slot
int cfDrop(String location);         // drop a piece into an output slot
int cfStore(String location);        // store a piece in a bin
int cfRetrieve(String location);     // retrieve a piece from a bin
int cfCheckColor(String dummy);      // determine the color of the current piece

const int RET_OK               = 0;   // anything from here on up will be OK
const int RET_BUSY             = -1;  // the device is currently busy
const int RET_INVALID_LOCATION = -2;  // an invalid location was specified
const int RET_NO_ACTION        = -3;  // the device did not acknowledge
const int RET_TIMEOUT_ACK      = -4;  // timeout waiting for an acknowledgement
const int RET_TIMEOUT_CMD      = -5;  // timeout waiting for command execution

// ===== cloud variables and values ===========================================

// busy flag: 0 or 1, depending on the state of robot
int cvBusy;

// the ID of the action that is currently being executed
int cvActionID;

// input occupied flag: 1 if there's a piece in the input slot
int cvInputOccupied;
const int INPUT_EMPTY = 0;
const int INPUT_OCCUPIED = 1;

// pincers occupied flag: 1 if there's a piece in the pincers
int cvPincersOccupied;
const int PINCERS_EMPTY = 0;
const int PINCERS_OCCUPIED = 1;

// last identified color: "" (empty), "white", "red", "black"
const int CV_COLOR_LENGTH = 10;
char cvColor[CV_COLOR_LENGTH];

String COLOR_EMPTY   = "";
String COLOR_WHITE   = "white";
String COLOR_RED     = "red";
String COLOR_BLACK   = "black";
String COLOR_UNKNOWN = "unknown";

// input check interval (0 = no checks)
int cvCheckInterval;

// ===== analog-digital input =================================================

// if the value of an input is above this threshold, it's considered a 1
const int ANALOG_THRESHOLD = 1800;

bool myDigitalRead(int pin) {
  return (analogRead(pin) > ANALOG_THRESHOLD);
}

// ===== action ID management =================================================

const int ACTION_ID_MIN = 10000;
const int ACTION_ID_MAX = 99999;
int currentAction = 0;

const int HISTORY_SIZE = 100;
int history[HISTORY_SIZE];
int historyPtr = 0;

int generateActionID() {
  while (true) {
    int newID = random(ACTION_ID_MIN, ACTION_ID_MAX);
    bool duplicate = false;
    for (int i = 0; i < HISTORY_SIZE; i++) {
      if (history[i] == newID) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {
      history[historyPtr] = newID;
      historyPtr = (historyPtr >= HISTORY_SIZE - 1) ? 0 : historyPtr + 1;
      return newID;
    }
  }
}

// ===== command queue ========================================================

const int QUEUE_SIZE = 2; // at the moment, we don't need any more than this
int queue[QUEUE_SIZE];

void clearQueue() {
  for(int queuePos = 0; queuePos < QUEUE_SIZE; queuePos++) {
    queue[queuePos] = CMD_EMPTY;
  }
}

int enqueueCommands(int cmd1, int cmd2 = CMD_EMPTY) {
  if (state != STATE_READY) {
    return RET_BUSY;
  }
  state = STATE_RECEIVING;
  queue[0] = cmd1;
  queue[1] = cmd2;
  currentAction = generateActionID();
  state = STATE_RECEIVED;
  return currentAction;
}

int processCommands() {
  int result = RET_OK;
  for(int queuePos = 0; queuePos < QUEUE_SIZE; queuePos++) {
    if (queue[queuePos] != CMD_EMPTY) {
      result = issueCommand(queue[queuePos]);
      if (result < 0) {
        break;
      }
    }
  }
  return result;
}

int getLastCommand() {
  int lastCommand = CMD_EMPTY;
  for(int queuePos = 0; queuePos < QUEUE_SIZE; queuePos++) {
    if (queue[queuePos] != CMD_EMPTY) {
      lastCommand = queue[queuePos];
    }
  }
  return lastCommand;
}

// ===== initial setup ========================================================

void setup() {
  // configure the hardware pins
  pinMode(P_READY, OUTPUT);
  pinMode(P_CMD0,  OUTPUT);
  pinMode(P_CMD1,  OUTPUT);
  pinMode(P_CMD2,  OUTPUT);
  pinMode(P_CMD3,  OUTPUT);
  pinMode(P_CMD4,  OUTPUT);
  pinMode(R_BUSY,  INPUT);
  pinMode(R_ACK,   INPUT);
  pinMode(R_COL0,  INPUT);
  pinMode(R_COL1,  INPUT);

  // register the cloud variables
  // max name len:   123456789012
  Particle.variable("busy",        cvBusy);
  Particle.variable("actionID",    cvActionID);
  Particle.variable("lastColor",   cvColor);
  Particle.variable("inputOcc",    cvInputOccupied);
  Particle.variable("pincersOcc",  cvPincersOccupied);
  Particle.variable("checkInt",    cvCheckInterval);

  // register the cloud functions
  // max name len:   123456789012
  Particle.function("setCheckInt", cfSetCheckInterval);
  Particle.function("enumBins",    cfEnumerateBins);
  Particle.function("enumOuts",    cfEnumerateOuts);
  Particle.function("initialize",  cfInitialize);
  Particle.function("pickup",      cfPickup);
  Particle.function("drop",        cfDrop);
  Particle.function("store",       cfStore);
  Particle.function("retrieve",    cfRetrieve);
  Particle.function("checkColor",  cfCheckColor);

  // initialize the history
  for (int i = 0; i < HISTORY_SIZE; i++) {
    history[i] = 0;
  }

  // set the default check interval
  cvCheckInterval = INIT_INTERVAL_CHECK_INPUT;

  // workaround for buggy RNG initialization
  // see https://community.particle.io/t/random-number-generator-not-initialized/20006
  // see https://community.particle.io/t/random-seed-from-cloud-not-working/17343/11
  uint32_t seed = HAL_RNG_GetRandomNumber();
  srand(seed);
}

// ===== main loop ============================================================

void loop() {

  // if we have received an action through a cloud function, execute it
  if (state == STATE_RECEIVED) {
    int result;

    // update the state to reflect we're busy now
    state = STATE_BUSY;
    cvBusy = 1;
    cvActionID = currentAction;

    // send an event to show we're working on something
    Particle.publish(String::format("robo/action/%d", currentAction),
      "started", 3600, PRIVATE);

    // process the command queue and execute each command
    result = processCommands();

    // evaluate the result and send an appropriate event
    if (result < 0) {
      Particle.publish(String::format("robo/action/%d", currentAction),
        String::format("failed: %d", result), 3600, PRIVATE);
    } else {
      Particle.publish(String::format("robo/action/%d", currentAction),
        "completed", 3600, PRIVATE);

      // depending on the last command, set the cloud variables
      updateStatusVariables();

    }

    // clear the command queue
    clearQueue();

    // reset the state to show we're no longer busy
    cvBusy = 0;
    cvActionID = 0;
    currentAction = 0;
    state = STATE_READY;
  }

  // Once in a while, check whether there's a piece in the input slot.
  if (cvCheckInterval > 0) {
    updateInputStatus();
  }

  delay(DELAY_MAIN_LOOP);
}

// ===== cloud function implementations =======================================

int cfSetCheckInterval(String intv) {
  cvCheckInterval = intv.toInt();
}

int cfEnumerateBins(String dummy) {
  Particle.publish("robo/bins",
    "A1, A2, A3, A4, B1, B2, B3, B4, C1, C2, C3, C4, D1, D2, D3, D4, E1, E2, E3, E4",
    3600, PRIVATE);
}

int cfEnumerateOuts(String dummy) {
  Particle.publish("robo/outputs", "A, B, C, D, E, F", 3600, PRIVATE);
}

int cfInitialize(String dummy) {
  return enqueueCommands(CMD_INIT);
}

int cfPickup(String dummy) {
  return enqueueCommands(CMD_PICKUP);
}

int cfDrop(String location) {
  if (location.equals("A")) return enqueueCommands(CMD_DROP_A);
  if (location.equals("B")) return enqueueCommands(CMD_DROP_B);
  if (location.equals("C")) return enqueueCommands(CMD_DROP_C);
  if (location.equals("D")) return enqueueCommands(CMD_DROP_D);
  if (location.equals("E")) return enqueueCommands(CMD_DROP_E);
  if (location.equals("F")) return enqueueCommands(CMD_DROP_F);
  Particle.publish("robo/diagnostics",
    String::format("invalid location: %s", location.c_str()), 3600, PRIVATE);
  return RET_INVALID_LOCATION;
}

int cfStore(String location) {
  return enqueueLocalizedCommand(location, CMD_STORE);
}

int cfRetrieve(String location) {
  return enqueueLocalizedCommand(location, CMD_RETRIEVE);
}

int cfCheckColor(String dummy) {
  return enqueueCommands(CMD_CHECK_COLOR);
}

int enqueueLocalizedCommand(String location, int cmd2) {
  if (location.equals("A1")) return enqueueCommands(CMD_NAV_A1, cmd2);
  if (location.equals("A2")) return enqueueCommands(CMD_NAV_A2, cmd2);
  if (location.equals("A3")) return enqueueCommands(CMD_NAV_A3, cmd2);
  if (location.equals("A4")) return enqueueCommands(CMD_NAV_A4, cmd2);
  if (location.equals("B1")) return enqueueCommands(CMD_NAV_B1, cmd2);
  if (location.equals("B2")) return enqueueCommands(CMD_NAV_B2, cmd2);
  if (location.equals("B3")) return enqueueCommands(CMD_NAV_B3, cmd2);
  if (location.equals("B4")) return enqueueCommands(CMD_NAV_B4, cmd2);
  if (location.equals("C1")) return enqueueCommands(CMD_NAV_C1, cmd2);
  if (location.equals("C2")) return enqueueCommands(CMD_NAV_C2, cmd2);
  if (location.equals("C3")) return enqueueCommands(CMD_NAV_C3, cmd2);
  if (location.equals("C4")) return enqueueCommands(CMD_NAV_C4, cmd2);
  if (location.equals("D1")) return enqueueCommands(CMD_NAV_D1, cmd2);
  if (location.equals("D2")) return enqueueCommands(CMD_NAV_D2, cmd2);
  if (location.equals("D3")) return enqueueCommands(CMD_NAV_D3, cmd2);
  if (location.equals("D4")) return enqueueCommands(CMD_NAV_D4, cmd2);
  if (location.equals("E1")) return enqueueCommands(CMD_NAV_E1, cmd2);
  if (location.equals("E2")) return enqueueCommands(CMD_NAV_E2, cmd2);
  if (location.equals("E3")) return enqueueCommands(CMD_NAV_E3, cmd2);
  if (location.equals("E4")) return enqueueCommands(CMD_NAV_E4, cmd2);
  Particle.publish("robo/diagnostics",
    String::format("invalid location: %s", location.c_str()), 3600, PRIVATE);
  return RET_INVALID_LOCATION;
}

// ===== command interface ====================================================

int issueCommand(int command) {
  unsigned long startTime;
  unsigned long duration;

  // set the command bits
  digitalWrite(P_CMD0, ((command &  1) > 0) ? HIGH : LOW);
  digitalWrite(P_CMD1, ((command &  2) > 0) ? HIGH : LOW);
  digitalWrite(P_CMD2, ((command &  4) > 0) ? HIGH : LOW);
  digitalWrite(P_CMD3, ((command &  8) > 0) ? HIGH : LOW);
  digitalWrite(P_CMD4, ((command & 16) > 0) ? HIGH : LOW);

  // put up the READY signal and wait for the ROBO Interface to ACK
  startTime = millis();
  digitalWrite(P_READY, HIGH);
  while (!(myDigitalRead(R_ACK))) {
    Particle.process();
    delay(DELAY_WAIT_ACK);
    unsigned long duration = millis() - startTime;
    if (duration > TIMEOUT_ACK) {
      digitalWrite(P_READY, LOW);
      return RET_TIMEOUT_ACK;
    }
  }

  // once the command has been acknowledged, pull down the READY signal
  digitalWrite(P_READY, LOW);
  delay(DELAY_ACK_BUSY);

  // the BUSY signal should be active (LOW!) now, signaling that the robot is
  // working
  if (myDigitalRead(R_BUSY) != false) {
    return RET_NO_ACTION;
  }

  // wait for the command to be executed (until BUSY is pulled up again)
  startTime = millis();
  while (!(myDigitalRead(R_BUSY))) {
    Particle.process();
    delay(DELAY_WAIT_BUSY);
    duration = millis() - startTime;
    if (duration > TIMEOUT_CMD) {
      return RET_TIMEOUT_CMD;
    }
  }
  return RET_OK;
}

// ===== return value interpretation ==========================================

void updateStatusVariables() {

  delay(DELAY_WAIT_COLOR);
  int colorCode = (myDigitalRead(R_COL0) ? 1 : 0 ) +
                  (myDigitalRead(R_COL1) ? 2 : 0) ;
  switch(getLastCommand()) {
    case CMD_INIT:
      cvPincersOccupied = PINCERS_EMPTY;
      COLOR_EMPTY.toCharArray(cvColor, CV_COLOR_LENGTH);
      break;

    case CMD_PICKUP:
      if (colorCode > 0) {
        cvPincersOccupied = PINCERS_OCCUPIED;
        COLOR_UNKNOWN.toCharArray(cvColor, CV_COLOR_LENGTH);
      } else {
        cvPincersOccupied = PINCERS_EMPTY;
        COLOR_EMPTY.toCharArray(cvColor, CV_COLOR_LENGTH);
      }
      break;

    case CMD_STORE:
      cvPincersOccupied = PINCERS_EMPTY;
      COLOR_EMPTY.toCharArray(cvColor, CV_COLOR_LENGTH);
      break;

    case CMD_RETRIEVE:
      if (colorCode > 0) {
        cvPincersOccupied = PINCERS_OCCUPIED;
        COLOR_UNKNOWN.toCharArray(cvColor, CV_COLOR_LENGTH);
      } else {
        cvPincersOccupied = PINCERS_EMPTY;
        COLOR_EMPTY.toCharArray(cvColor, CV_COLOR_LENGTH);
      }
      break;

    case CMD_CHECK_COLOR:
      switch(colorCode) {
        case 0:
          cvPincersOccupied = PINCERS_EMPTY;
          COLOR_EMPTY.toCharArray(cvColor, CV_COLOR_LENGTH);
          break;
        case 1:
          cvPincersOccupied = PINCERS_OCCUPIED;
          COLOR_WHITE.toCharArray(cvColor, CV_COLOR_LENGTH);
          break;
        case 2:
          cvPincersOccupied = PINCERS_OCCUPIED;
          COLOR_RED.toCharArray(cvColor, CV_COLOR_LENGTH);
          break;
        case 3:
          cvPincersOccupied = PINCERS_OCCUPIED;
          COLOR_BLACK.toCharArray(cvColor, CV_COLOR_LENGTH);
          break;
      }
      Particle.publish("robo/diagnostics",
        String::format("detected color: %d (%s)", colorCode,
                       String(cvColor).c_str()), 3600, PRIVATE);
      break;

    case CMD_DROP_A:
    case CMD_DROP_B:
    case CMD_DROP_C:
    case CMD_DROP_D:
    case CMD_DROP_E:
    case CMD_DROP_F:
      cvPincersOccupied = PINCERS_EMPTY;
      COLOR_EMPTY.toCharArray(cvColor, CV_COLOR_LENGTH);
      break;
  }

}

void updateInputStatus() {

  unsigned long timestamp = millis();
  if ((lastInputCheck > timestamp) ||
      ((timestamp - lastInputCheck) > cvCheckInterval)) {
    if (issueCommand(CMD_CHECK_INPUT) == RET_OK) {
      if (myDigitalRead(R_COL0) || myDigitalRead(R_COL1)) {
        cvInputOccupied = INPUT_OCCUPIED;
        Particle.publish("robo/input", "occupied", 3600, PRIVATE);
      } else {
        cvInputOccupied = INPUT_EMPTY;
      }
    }
    lastInputCheck = timestamp;
  }

}
