/*
 * Prior ProScan Controller Emulator
 *
 * This Arduino sketch emulates the serial communication interface of a
 * Prior ProScan microscope stage controller.
 *
 * Serial Configuration: 9600 baud (default), can be changed to 19200 or 38400
 * Command Termination: Carriage Return (CR)
 * Delimiters: COMMA, SPACE, TAB, EQUALS, SEMICOLON, COLON
 */

// Configuration Constants
#define DEFAULT_BAUD 9600
#define MAX_COMMAND_LENGTH 128
#define VERSION_NUMBER 100  // Version 1.00
#define SERIAL_NUMBER "12345"

// Motion timing constants
#define BASE_SPEED_XY 10000.0  // microns/sec at 100% speed (10 mm/s)
#define BASE_SPEED_Z 1000.0    // microns/sec at 100% speed (1 mm/s)

// Controller State
struct ControllerState {
  // Position tracking (in microsteps or microns depending on scale)
  long posX;
  long posY;
  long posZ;

  // Step sizes
  long stepU;  // X step size
  long stepV;  // Y step size
  long stepW;  // Z step size

  // Speed settings (percentage 1-100)
  int stageSpeedXY;    // SMS
  int stageAccel;      // SAS
  int stageSCurve;     // SCS
  int zSpeed;          // SMZ
  int zAccel;          // SAZ
  int zSCurve;         // SCZ
  int joystickSpeed;   // O
  int focusSpeed;      // OF

  // Direction settings
  int xDirection;      // XD (1 or -1)
  int yDirection;      // YD (1 or -1)
  int joyXDirection;   // JXD
  int joyYDirection;   // JYD
  int joyZDirection;   // JZD

  // Control flags
  bool compatibilityMode;  // COMP (0=Standard, 1=Compatibility)
  bool joystickEnabled;    // J/H
  bool errorHumanMode;     // ERROR (0=codes, 1=human readable)

  // Motion status (bits for X, Y, Z, A, F1, F2)
  byte motionStatus;

  // Limit switch status
  byte limitHitStatus;
  byte limitCurrentStatus;

  // Backlash settings
  bool backlashStageSerial;
  long backlashStageSerialValue;
  bool backlashStageJoy;
  long backlashStageJoyValue;
  bool backlashZSerial;
  long backlashZSerialValue;
  bool backlashZJoy;
  long backlashZJoyValue;

  // Resolution settings (microns)
  float resolutionX;
  float resolutionY;
  float resolutionZ;

  // UPR - microns per revolution
  float micronsPerRevZ;

  // Skew settings
  float skewAngle;

  long baudRate;

  // XY Motion state
  bool movingXY;
  unsigned long xyMotionStartTime;
  long xyMotionStartX, xyMotionStartY;
  long xyMotionTargetX, xyMotionTargetY;
  unsigned long xyMotionDuration;

  // Z Motion state
  bool movingZ;
  unsigned long zMotionStartTime;
  long zMotionStartZ;
  long zMotionTargetZ;
  unsigned long zMotionDuration;

  // Filter wheel state (3 wheels max)
  bool filterWheelFitted[3];       // Which wheels are fitted
  int filterPosition[3];            // Current position (1-based)
  int filterPositionsPerWheel[3];  // Number of positions per wheel
  int filterSpeed[3];               // Speed setting (1-100%)
  int filterAccel[3];               // Acceleration setting (1-100%)
  int filterSCurve[3];              // S-Curve setting (1-100%)
  bool filterAutoHome[3];           // Auto home on startup
  bool filterShutterClose;          // Auto close shutters during filter move

  // Shutter state (3 shutters max)
  bool shutterFitted[3];            // Which shutters are fitted
  bool shutterState[3];             // Current state (false=open, true=closed)
  bool shutterDefaultState[3];      // Default startup state (false=open, true=closed)
  unsigned long shutterTimerEnd[3]; // Timer end time for timed shutter operations

  // Lumen Pro state
  bool lumenProFitted;              // Is Lumen Pro fitted
  bool lumenProPowerOn;             // Power state (on/off)
  int lumenProCurrentOutput;        // Current light output (0-100%)
  int lumenProPositionOutput[10];   // Light output per position (1-10)
} state;

// Command buffer
char commandBuffer[MAX_COMMAND_LENGTH];
int bufferIndex = 0;

// Function prototypes
void initializeState();
void processCommand(char* cmd);
void parseAndExecute(char* cmd);
bool isDelimiter(char c);
void skipDelimiters(char** ptr);
long parseNumber(char** ptr);
float parseFloat(char** ptr);
void extractCommand(char** ptr, char* cmdOut, int maxLen);

// Command handlers
void cmd_Question();
void cmd_Date();
void cmd_Version();
void cmd_Serial();
void cmd_Comp(char* args);
void cmd_Error(char* args);
void cmd_Dollar(char* args);
void cmd_Equals();
void cmd_LMT();
void cmd_I();
void cmd_K();
void cmd_Baud(char* args);
void cmd_P(char* args);
void cmd_PS(char* args);
void cmd_PX(char* args);
void cmd_PY(char* args);
void cmd_PZ(char* args);
void cmd_G(char* args);
void cmd_GR(char* args);
void cmd_GX(char* args);
void cmd_GY(char* args);
void cmd_GZ(char* args);
void cmd_V(char* args);
void cmd_F(char* args);
void cmd_B(char* args);
void cmd_L(char* args);
void cmd_R(char* args);
void cmd_U(char* args);
void cmd_D(char* args);
void cmd_M();
void cmd_Z();
void cmd_X(char* args);
void cmd_C(char* args);
void cmd_J();
void cmd_H();
void cmd_O(char* args);
void cmd_OF(char* args);
void cmd_SMS(char* args);
void cmd_SAS(char* args);
void cmd_SCS(char* args);
void cmd_SMZ(char* args);
void cmd_SAZ(char* args);
void cmd_SCZ(char* args);
void cmd_XD(char* args);
void cmd_YD(char* args);
void cmd_JXD(char* args);
void cmd_JYD(char* args);
void cmd_JZD(char* args);
void cmd_Stage();
void cmd_Focus();
void cmd_RES(char* args);
void cmd_UPR(char* args);
void cmd_7(char* args);
void cmd_Filter(char* args);
void cmd_FPW(char* args);
void cmd_SAF(char* args);
void cmd_SCF(char* args);
void cmd_SMF(char* args);
void cmd_8(char* args);
void cmd_Shutter(char* args);
void cmd_Light(char* args);

// Motion helper functions
void startXYMotion(long targetX, long targetY);
void startZMotion(long targetZ);
void startXYZMotion(long targetX, long targetY, long targetZ);
void updateXYMotion();
void updateZMotion();
void stopAllMotion();
byte calculateMotionStatus();

// Shutter helper functions
void updateShutterTimers();

void setup() {
  Serial.begin(DEFAULT_BAUD);
  initializeState();
  bufferIndex = 0;
}

void loop() {
  // Update motion (interpolate position)
  updateXYMotion();
  updateZMotion();

  // Update shutter timers
  updateShutterTimers();

  // Read incoming serial data
  while (Serial.available() > 0) {
    char c = Serial.read();

    // Handle Carriage Return - execute command
    if (c == '\r' || c == '\n') {
      if (bufferIndex > 0) {
        commandBuffer[bufferIndex] = '\0';
        processCommand(commandBuffer);
        bufferIndex = 0;
      }
    }
    // Special case: I, K, # can be executed immediately in compatibility mode
    else if (state.compatibilityMode && (c == 'I' || c == 'K' || c == '#')) {
      commandBuffer[0] = c;
      commandBuffer[1] = '\0';
      processCommand(commandBuffer);
      bufferIndex = 0;
    }
    // Buffer the character
    else if (bufferIndex < MAX_COMMAND_LENGTH - 1) {
      commandBuffer[bufferIndex++] = c;
    }
  }
}

void initializeState() {
  // Initialize position to 0,0,0
  state.posX = 0;
  state.posY = 0;
  state.posZ = 0;

  // Default step sizes
  state.stepU = 100;
  state.stepV = 100;
  state.stepW = 100;

  // Default speed settings (50%)
  state.stageSpeedXY = 50;
  state.stageAccel = 50;
  state.stageSCurve = 50;
  state.zSpeed = 50;
  state.zAccel = 50;
  state.zSCurve = 50;
  state.joystickSpeed = 50;
  state.focusSpeed = 50;

  // Default directions
  state.xDirection = 1;
  state.yDirection = 1;
  state.joyXDirection = 1;
  state.joyYDirection = 1;
  state.joyZDirection = 1;

  // Control flags
  state.compatibilityMode = false;  // Standard mode
  state.joystickEnabled = true;     // Joystick on by default
  state.errorHumanMode = false;     // Error codes by default

  // Status
  state.motionStatus = 0;
  state.limitHitStatus = 0;
  state.limitCurrentStatus = 0;

  // Backlash disabled by default
  state.backlashStageSerial = false;
  state.backlashStageSerialValue = 0;
  state.backlashStageJoy = false;
  state.backlashStageJoyValue = 0;
  state.backlashZSerial = false;
  state.backlashZSerialValue = 0;
  state.backlashZJoy = false;
  state.backlashZJoyValue = 0;

  // Resolution (default 1 micron)
  state.resolutionX = 1.0;
  state.resolutionY = 1.0;
  state.resolutionZ = 1.0;

  // Default Z microns per revolution
  state.micronsPerRevZ = 100.0;

  // No skew
  state.skewAngle = 0.0;

  state.baudRate = DEFAULT_BAUD;

  // Motion state (not moving initially)
  state.movingXY = false;
  state.xyMotionStartTime = 0;
  state.xyMotionStartX = 0;
  state.xyMotionStartY = 0;
  state.xyMotionTargetX = 0;
  state.xyMotionTargetY = 0;
  state.xyMotionDuration = 0;

  state.movingZ = false;
  state.zMotionStartTime = 0;
  state.zMotionStartZ = 0;
  state.zMotionTargetZ = 0;
  state.zMotionDuration = 0;

  // Filter wheel initialization
  for (int i = 0; i < 3; i++) {
    state.filterWheelFitted[i] = true;  // All wheels fitted by default
    state.filterPosition[i] = 1;          // Start at position 1
    state.filterPositionsPerWheel[i] = 10; // Default 10 positions
    state.filterSpeed[i] = 50;            // Default 50% speed
    state.filterAccel[i] = 50;            // Default 50% acceleration
    state.filterSCurve[i] = 50;           // Default 50% S-Curve
    state.filterAutoHome[i] = false;      // No auto home by default
  }
  state.filterShutterClose = false;       // Don't auto close shutters

  state.filterWheelFitted[1] = false;  // Take wheel 2 away.

  // Shutter initialization
  for (int i = 0; i < 3; i++) {
    state.shutterFitted[i] = true;        // All shutters fitted by default
    state.shutterState[i] = true;         // Closed by default
    state.shutterDefaultState[i] = true;  // Default to closed on startup
    state.shutterTimerEnd[i] = 0;         // No timer active
  }

  // Lumen Pro initialization
  state.lumenProFitted = true;            // Lumen Pro fitted by default
  state.lumenProPowerOn = true;           // Default to on
  state.lumenProCurrentOutput = 0;        // Start at 0% output
  // Standard 10-position shutter light output settings
  state.lumenProPositionOutput[0] = 0;    // Position 1: 0%
  state.lumenProPositionOutput[1] = 11;   // Position 2: 11%
  state.lumenProPositionOutput[2] = 22;   // Position 3: 22%
  state.lumenProPositionOutput[3] = 33;   // Position 4: 33%
  state.lumenProPositionOutput[4] = 44;   // Position 5: 44%
  state.lumenProPositionOutput[5] = 55;   // Position 6: 55%
  state.lumenProPositionOutput[6] = 66;   // Position 7: 66%
  state.lumenProPositionOutput[7] = 77;   // Position 8: 77%
  state.lumenProPositionOutput[8] = 88;   // Position 9: 88%
  state.lumenProPositionOutput[9] = 100;  // Position 10: 100%
}


void processCommand(char* cmd) {
  // Trim leading whitespace
  while (*cmd && (*cmd == ' ' || *cmd == '\t')) {
    cmd++;
  }

  // Empty command - return position (P command default)
  if (*cmd == '\0') {
    cmd_P(NULL);
    return;
  }

  parseAndExecute(cmd);
}

void parseAndExecute(char* cmd) {
  char command[32];
  char* ptr = cmd;

  // Extract the command keyword
  extractCommand(&ptr, command, sizeof(command));

  // Skip delimiters after command
  skipDelimiters(&ptr);

  // Route to appropriate handler
  if (strcmp(command, "?") == 0) {
    cmd_Question();
  }
  else if (strcmp(command, "DATE") == 0) {
    cmd_Date();
  }
  else if (strcmp(command, "VERSION") == 0) {
    cmd_Version();
  }
  else if (strcmp(command, "SERIAL") == 0) {
    cmd_Serial();
  }
  else if (strcmp(command, "COMP") == 0) {
    cmd_Comp(ptr);
  }
  else if (strcmp(command, "ERROR") == 0) {
    cmd_Error(ptr);
  }
  else if (strcmp(command, "$") == 0) {
    cmd_Dollar(ptr);
  }
  else if (strcmp(command, "=") == 0) {
    cmd_Equals();
  }
  else if (strcmp(command, "LMT") == 0) {
    cmd_LMT();
  }
  else if (strcmp(command, "I") == 0) {
    cmd_I();
  }
  else if (strcmp(command, "K") == 0) {
    cmd_K();
  }
  else if (strcmp(command, "BAUD") == 0) {
    cmd_Baud(ptr);
  }
  else if (strcmp(command, "P") == 0) {
    cmd_P(ptr);
  }
  else if (strcmp(command, "PS") == 0) {
    cmd_PS(ptr);
  }
  else if (strcmp(command, "PX") == 0) {
    cmd_PX(ptr);
  }
  else if (strcmp(command, "PY") == 0) {
    cmd_PY(ptr);
  }
  else if (strcmp(command, "PZ") == 0) {
    cmd_PZ(ptr);
  }
  else if (strcmp(command, "G") == 0) {
    cmd_G(ptr);
  }
  else if (strcmp(command, "GR") == 0) {
    cmd_GR(ptr);
  }
  else if (strcmp(command, "GX") == 0) {
    cmd_GX(ptr);
  }
  else if (strcmp(command, "GY") == 0) {
    cmd_GY(ptr);
  }
  else if (strcmp(command, "GZ") == 0) {
    cmd_GZ(ptr);
  }
  else if (strcmp(command, "V") == 0) {
    cmd_V(ptr);
  }
  else if (strcmp(command, "F") == 0) {
    cmd_F(ptr);
  }
  else if (strcmp(command, "B") == 0) {
    cmd_B(ptr);
  }
  else if (strcmp(command, "L") == 0) {
    cmd_L(ptr);
  }
  else if (strcmp(command, "R") == 0) {
    cmd_R(ptr);
  }
  else if (strcmp(command, "U") == 0) {
    cmd_U(ptr);
  }
  else if (strcmp(command, "D") == 0) {
    cmd_D(ptr);
  }
  else if (strcmp(command, "M") == 0) {
    cmd_M();
  }
  else if (strcmp(command, "Z") == 0) {
    cmd_Z();
  }
  else if (strcmp(command, "X") == 0) {
    cmd_X(ptr);
  }
  else if (strcmp(command, "C") == 0) {
    cmd_C(ptr);
  }
  else if (strcmp(command, "J") == 0) {
    cmd_J();
  }
  else if (strcmp(command, "H") == 0) {
    cmd_H();
  }
  else if (strcmp(command, "O") == 0) {
    cmd_O(ptr);
  }
  else if (strcmp(command, "OF") == 0) {
    cmd_OF(ptr);
  }
  else if (strcmp(command, "SMS") == 0) {
    cmd_SMS(ptr);
  }
  else if (strcmp(command, "SAS") == 0) {
    cmd_SAS(ptr);
  }
  else if (strcmp(command, "SCS") == 0) {
    cmd_SCS(ptr);
  }
  else if (strcmp(command, "SMZ") == 0) {
    cmd_SMZ(ptr);
  }
  else if (strcmp(command, "SAZ") == 0) {
    cmd_SAZ(ptr);
  }
  else if (strcmp(command, "SCZ") == 0) {
    cmd_SCZ(ptr);
  }
  else if (strcmp(command, "XD") == 0) {
    cmd_XD(ptr);
  }
  else if (strcmp(command, "YD") == 0) {
    cmd_YD(ptr);
  }
  else if (strcmp(command, "JXD") == 0) {
    cmd_JXD(ptr);
  }
  else if (strcmp(command, "JYD") == 0) {
    cmd_JYD(ptr);
  }
  else if (strcmp(command, "JZD") == 0) {
    cmd_JZD(ptr);
  }
  else if (strcmp(command, "STAGE") == 0) {
    cmd_Stage();
  }
  else if (strcmp(command, "FOCUS") == 0) {
    cmd_Focus();
  }
  else if (strcmp(command, "RES") == 0) {
    cmd_RES(ptr);
  }
  else if (strcmp(command, "UPR") == 0) {
    cmd_UPR(ptr);
  }
  else if (strcmp(command, "7") == 0) {
    cmd_7(ptr);
  }
  else if (strcmp(command, "FILTER") == 0) {
    cmd_Filter(ptr);
  }
  else if (strcmp(command, "FPW") == 0) {
    cmd_FPW(ptr);
  }
  else if (strcmp(command, "SAF") == 0) {
    cmd_SAF(ptr);
  }
  else if (strcmp(command, "SCF") == 0) {
    cmd_SCF(ptr);
  }
  else if (strcmp(command, "SMF") == 0) {
    cmd_SMF(ptr);
  }
  else if (strcmp(command, "8") == 0) {
    cmd_8(ptr);
  }
  else if (strcmp(command, "SHUTTER") == 0) {
    cmd_Shutter(ptr);
  }
  else if (strcmp(command, "LIGHT") == 0) {
    cmd_Light(ptr);
  }
  else {
    // Unknown command - send error
    Serial.print("E,1\r");  // Unknown command error
  }
}

// Helper functions for parsing
bool isDelimiter(char c) {
  return (c == ',' || c == ' ' || c == '\t' || c == '=' || c == ';' || c == ':');
}

void skipDelimiters(char** ptr) {
  while (**ptr && isDelimiter(**ptr)) {
    (*ptr)++;
  }
}

long parseNumber(char** ptr) {
  skipDelimiters(ptr);

  if (**ptr == '\0') {
    return 0;
  }

  long result = 0;
  int sign = 1;

  if (**ptr == '-') {
    sign = -1;
    (*ptr)++;
  } else if (**ptr == '+') {
    (*ptr)++;
  }

  while (**ptr >= '0' && **ptr <= '9') {
    result = result * 10 + (**ptr - '0');
    (*ptr)++;
  }

  return result * sign;
}

float parseFloat(char** ptr) {
  skipDelimiters(ptr);

  if (**ptr == '\0') {
    return 0.0;
  }

  float result = 0.0;
  float sign = 1.0;

  if (**ptr == '-') {
    sign = -1.0;
    (*ptr)++;
  } else if (**ptr == '+') {
    (*ptr)++;
  }

  // Parse integer part
  while (**ptr >= '0' && **ptr <= '9') {
    result = result * 10.0 + (**ptr - '0');
    (*ptr)++;
  }

  // Parse decimal part
  if (**ptr == '.') {
    (*ptr)++;
    float decimal = 0.0;
    float divisor = 10.0;
    while (**ptr >= '0' && **ptr <= '9') {
      decimal += (**ptr - '0') / divisor;
      divisor *= 10.0;
      (*ptr)++;
    }
    result += decimal;
  }

  return result * sign;
}

void extractCommand(char** ptr, char* cmdOut, int maxLen) {
  int i = 0;

  // Extract command until delimiter or end
  while (**ptr && !isDelimiter(**ptr) && i < maxLen - 1) {
    cmdOut[i++] = **ptr;
    (*ptr)++;
  }
  cmdOut[i] = '\0';
}

// ============================================================================
// MOTION HELPER FUNCTIONS
// ============================================================================

void startXYMotion(long targetX, long targetY) {
  // Calculate distance
  long deltaX = targetX - state.posX;
  long deltaY = targetY - state.posY;
  float distance = sqrt((float)(deltaX * deltaX) + (float)(deltaY * deltaY));

  if (distance < 1.0) {
    // Zero distance, just set position
    state.posX = targetX;
    state.posY = targetY;
    state.movingXY = false;
    return;
  }

  // Calculate duration based on speed
  float speed = BASE_SPEED_XY * (state.stageSpeedXY / 100.0);  // microns/sec
  state.xyMotionDuration = (unsigned long)((distance / speed) * 1000.0);  // milliseconds

  // Store start and target positions
  state.xyMotionStartX = state.posX;
  state.xyMotionStartY = state.posY;
  state.xyMotionTargetX = targetX;
  state.xyMotionTargetY = targetY;
  state.xyMotionStartTime = millis();
  state.movingXY = true;
}

void startZMotion(long targetZ) {
  // Calculate distance
  long deltaZ = targetZ - state.posZ;
  float distance = abs(deltaZ);

  if (distance < 1.0) {
    // Zero distance, just set position
    state.posZ = targetZ;
    state.movingZ = false;
    return;
  }

  // Calculate duration based on speed
  float speed = BASE_SPEED_Z * (state.zSpeed / 100.0);  // microns/sec
  state.zMotionDuration = (unsigned long)((distance / speed) * 1000.0);  // milliseconds

  // Store start and target positions
  state.zMotionStartZ = state.posZ;
  state.zMotionTargetZ = targetZ;
  state.zMotionStartTime = millis();
  state.movingZ = true;
}

void startXYZMotion(long targetX, long targetY, long targetZ) {
  // Start both XY and Z motions independently
  startXYMotion(targetX, targetY);
  startZMotion(targetZ);
}

void updateXYMotion() {
  if (!state.movingXY) return;

  unsigned long elapsed = millis() - state.xyMotionStartTime;

  if (elapsed >= state.xyMotionDuration) {
    // XY motion complete
    state.posX = state.xyMotionTargetX;
    state.posY = state.xyMotionTargetY;
    state.movingXY = false;
  } else {
    // Interpolate XY position (linear)
    float progress = (float)elapsed / (float)state.xyMotionDuration;
    state.posX = state.xyMotionStartX + (long)((state.xyMotionTargetX - state.xyMotionStartX) * progress);
    state.posY = state.xyMotionStartY + (long)((state.xyMotionTargetY - state.xyMotionStartY) * progress);
  }
}

void updateZMotion() {
  if (!state.movingZ) return;

  unsigned long elapsed = millis() - state.zMotionStartTime;

  if (elapsed >= state.zMotionDuration) {
    // Z motion complete
    state.posZ = state.zMotionTargetZ;
    state.movingZ = false;
  } else {
    // Interpolate Z position (linear)
    float progress = (float)elapsed / (float)state.zMotionDuration;
    state.posZ = state.zMotionStartZ + (long)((state.zMotionTargetZ - state.zMotionStartZ) * progress);
  }
}

void stopAllMotion() {
  // Stop both XY and Z motion immediately at current position
  if (state.movingXY) {
    state.xyMotionTargetX = state.posX;
    state.xyMotionTargetY = state.posY;
    state.movingXY = false;
  }
  if (state.movingZ) {
    state.zMotionTargetZ = state.posZ;
    state.movingZ = false;
  }
}

byte calculateMotionStatus() {
  byte status = 0;
  if (state.movingXY) {
    status |= 0x03;  // X and Y moving (bits 0 and 1)
  }
  if (state.movingZ) {
    status |= 0x04;  // Z moving (bit 2)
  }
  return status;
}

void updateShutterTimers() {
  // Check each shutter for active timers
  unsigned long now = millis();
  for (int i = 0; i < 3; i++) {
    if (state.shutterTimerEnd[i] > 0 && now >= state.shutterTimerEnd[i]) {
      // Timer expired - toggle shutter back to opposite state
      state.shutterState[i] = !state.shutterState[i];
      state.shutterTimerEnd[i] = 0;  // Clear timer
    }
  }
}

// ============================================================================
// COMMAND IMPLEMENTATIONS
// ============================================================================

void cmd_Question() {
  Serial.print("PROSCAN INFORMATION\r");
  Serial.print("DSP_1 IS 4-AXIS STEPPER VERSION 2.7\r");
  Serial.print("DSP_2 IS 2-AXIS STEPPER VERSION 2.7\r");
  Serial.print("DRIVE CHIPS 010111 (F2 F1 A Z Y X) 0 = Not Fitted\r");
  Serial.print("JOYSTICK ");
  Serial.print(state.joystickEnabled ? "ACTIVE\r" : "INACTIVE\r");
  Serial.print("STAGE = H101/2\r");
  Serial.print("FOCUS = NORMAL\r");
  Serial.print("FILTER_1 = NONE\r");
  Serial.print("FILTER_2 = NONE\r");
  Serial.print("SHUTTERS = 000 (S3 S2 S1) 0 = Not Fitted\r");
  Serial.print("AUTOFOCUS = NONE\r");
  Serial.print("VIDEO = NONE\r");
  Serial.print("END\r");
}

void cmd_Date() {
  Serial.print("ProScan Emulator v1.00 Compiled Dec 2024\r");
}

void cmd_Version() {
  Serial.print(VERSION_NUMBER);
  Serial.print("\r");
}

void cmd_Serial() {
  Serial.print(SERIAL_NUMBER);
  Serial.print("\r");
}

void cmd_Comp(char* args) {
  if (*args == '\0') {
    // Report current mode
    Serial.print(state.compatibilityMode ? "1\r" : "0\r");
  } else {
    // Set mode
    long mode = parseNumber(&args);
    state.compatibilityMode = (mode == 1);
    Serial.print("0\r");
  }
}

void cmd_Error(char* args) {
  if (*args == '\0') {
    // Report current error mode
    Serial.print(state.errorHumanMode ? "1\r" : "0\r");
  } else {
    // Set error mode
    long mode = parseNumber(&args);
    state.errorHumanMode = (mode == 1);
    Serial.print("0\r");
  }
}

void cmd_Dollar(char* args) {
  // Motion status command
  // Returns decimal number representing motion status
  byte status = calculateMotionStatus();
  Serial.print(status);
  Serial.print("\r");
}

void cmd_Equals() {
  // Limit switch hit status
  // Returns hex value, reading clears it
  Serial.print("00\r");
  state.limitHitStatus = 0;
}

void cmd_LMT() {
  // Current limit switch status
  Serial.print("00\r");
}

void cmd_I() {
  // Stop movement (controlled)
  stopAllMotion();
  Serial.print("R\r");
}

void cmd_K() {
  // Emergency stop
  stopAllMotion();
  Serial.print("R\r");
}

void cmd_Baud(char* args) {
  long baudArg = parseNumber(&args);
  long newBaud = DEFAULT_BAUD;

  if (baudArg == 96) {
    newBaud = 9600;
  } else if (baudArg == 19) {
    newBaud = 19200;
  } else if (baudArg == 38) {
    newBaud = 38400;
  }

  Serial.print("0\r");
  delay(100);  // Allow response to be sent
  Serial.end();
  Serial.begin(newBaud);
  state.baudRate = newBaud;
}

// Position commands
void cmd_P(char* args) {
  if (*args == '\0') {
    // Report position
    Serial.print(state.posX);
    Serial.print(",");
    Serial.print(state.posY);
    Serial.print(",");
    Serial.print(state.posZ);
    Serial.print("\r");
  } else {
    // Set position
    state.posX = parseNumber(&args);
    state.posY = parseNumber(&args);
    state.posZ = parseNumber(&args);
    Serial.print("0\r");
  }
}

void cmd_PS(char* args) {
  if (*args == '\0') {
    // Report stage position
    Serial.print(state.posX);
    Serial.print(",");
    Serial.print(state.posY);
    Serial.print("\r");
  } else {
    // Set stage position
    state.posX = parseNumber(&args);
    state.posY = parseNumber(&args);
    Serial.print("0\r");
  }
}

void cmd_PX(char* args) {
  if (*args == '\0') {
    Serial.print(state.posX);
    Serial.print("\r");
  } else {
    state.posX = parseNumber(&args);
    Serial.print("0\r");
  }
}

void cmd_PY(char* args) {
  if (*args == '\0') {
    Serial.print(state.posY);
    Serial.print("\r");
  } else {
    state.posY = parseNumber(&args);
    Serial.print("0\r");
  }
}

void cmd_PZ(char* args) {
  if (*args == '\0') {
    Serial.print(state.posZ);
    Serial.print("\r");
  } else {
    state.posZ = parseNumber(&args);
    Serial.print("0\r");
  }
}

// Movement commands - Go to absolute position
void cmd_G(char* args) {
  long x = parseNumber(&args);
  long y = parseNumber(&args);
  long z = parseNumber(&args);

  startXYZMotion(x, y, z);
  Serial.print("R\r");
}

void cmd_GR(char* args) {
  // Go Relative
  long x = parseNumber(&args);
  long y = parseNumber(&args);
  long z = parseNumber(&args);

  long targetX = state.posX + x;
  long targetY = state.posY + y;
  long targetZ = state.posZ + z;
  startXYZMotion(targetX, targetY, targetZ);
  Serial.print("R\r");
}

void cmd_GX(char* args) {
  long x = parseNumber(&args);
  startXYMotion(x, state.posY);
  Serial.print("R\r");
}

void cmd_GY(char* args) {
  long y = parseNumber(&args);
  startXYMotion(state.posX, y);
  Serial.print("R\r");
}

void cmd_GZ(char* args) {
  long z = parseNumber(&args);
  startZMotion(z);
  Serial.print("R\r");
}

void cmd_V(char* args) {
  // Move to absolute Z position
  long z = parseNumber(&args);
  startZMotion(z);
  Serial.print("R\r");
}

// Relative movement commands
void cmd_F(char* args) {
  // Forward (Y+)
  long steps;
  if (*args == '\0') {
    steps = state.stepV * state.yDirection;
  } else {
    steps = parseNumber(&args) * state.yDirection;
  }
  startXYMotion(state.posX, state.posY + steps);
  Serial.print("R\r");
}

void cmd_B(char* args) {
  // Back (Y-)
  long steps;
  if (*args == '\0') {
    steps = state.stepV * state.yDirection;
  } else {
    steps = parseNumber(&args) * state.yDirection;
  }
  startXYMotion(state.posX, state.posY - steps);
  Serial.print("R\r");
}

void cmd_L(char* args) {
  // Left (X-)
  long steps;
  if (*args == '\0') {
    steps = state.stepU * state.xDirection;
  } else {
    steps = parseNumber(&args) * state.xDirection;
  }
  startXYMotion(state.posX - steps, state.posY);
  Serial.print("R\r");
}

void cmd_R(char* args) {
  // Right (X+)
  long steps;
  if (*args == '\0') {
    steps = state.stepU * state.xDirection;
  } else {
    steps = parseNumber(&args) * state.xDirection;
  }
  startXYMotion(state.posX + steps, state.posY);
  Serial.print("R\r");
}

void cmd_U(char* args) {
  // Up (Z+)
  long steps;
  if (*args == '\0') {
    steps = state.stepW;
  } else {
    steps = parseNumber(&args);
  }
  startZMotion(state.posZ + steps);
  Serial.print("R\r");
}

void cmd_D(char* args) {
  // Down (Z-)
  long steps;
  if (*args == '\0') {
    steps = state.stepW;
  } else {
    steps = parseNumber(&args);
  }
  startZMotion(state.posZ - steps);
  Serial.print("R\r");
}

void cmd_M() {
  // Move to 0,0,0
  startXYZMotion(0, 0, 0);
  Serial.print("R\r");
}

void cmd_Z() {
  // Set position to 0,0,0
  state.posX = 0;
  state.posY = 0;
  state.posZ = 0;
  Serial.print("0\r");
}

// Step size commands
void cmd_X(char* args) {
  if (*args == '\0') {
    // Report step size
    Serial.print(state.stepU);
    Serial.print(",");
    Serial.print(state.stepV);
    Serial.print("\r");
  } else {
    // Set step size
    state.stepU = parseNumber(&args);
    state.stepV = parseNumber(&args);
    Serial.print("0\r");
  }
}

void cmd_C(char* args) {
  if (*args == '\0') {
    // Report Z step size
    Serial.print(state.stepW);
    Serial.print("\r");
  } else {
    // Set Z step size
    state.stepW = parseNumber(&args);
    Serial.print("0\r");
  }
}

// Joystick control
void cmd_J() {
  state.joystickEnabled = true;
  Serial.print("0\r");
}

void cmd_H() {
  state.joystickEnabled = false;
  Serial.print("0\r");
}

void cmd_O(char* args) {
  if (*args == '\0') {
    Serial.print(state.joystickSpeed);
    Serial.print("\r");
  } else {
    state.joystickSpeed = parseNumber(&args);
    // Constrain to 1-100
    if (state.joystickSpeed < 1) state.joystickSpeed = 1;
    if (state.joystickSpeed > 100) state.joystickSpeed = 100;
    Serial.print("0\r");
  }
}

void cmd_OF(char* args) {
  if (*args == '\0') {
    Serial.print(state.focusSpeed);
    Serial.print("\r");
  } else {
    state.focusSpeed = parseNumber(&args);
    if (state.focusSpeed < 1) state.focusSpeed = 1;
    if (state.focusSpeed > 100) state.focusSpeed = 100;
    Serial.print("0\r");
  }
}

// Speed/Acceleration settings
void cmd_SMS(char* args) {
  if (*args == '\0') {
    Serial.print(state.stageSpeedXY);
    Serial.print("\r");
  } else {
    state.stageSpeedXY = parseNumber(&args);
    if (state.stageSpeedXY < 1) state.stageSpeedXY = 1;
    if (state.stageSpeedXY > 100) state.stageSpeedXY = 100;
    Serial.print("0\r");
  }
}

void cmd_SAS(char* args) {
  if (*args == '\0') {
    Serial.print(state.stageAccel);
    Serial.print("\r");
  } else {
    state.stageAccel = parseNumber(&args);
    if (state.stageAccel < 1) state.stageAccel = 1;
    if (state.stageAccel > 100) state.stageAccel = 100;
    Serial.print("0\r");
  }
}

void cmd_SCS(char* args) {
  if (*args == '\0') {
    Serial.print(state.stageSCurve);
    Serial.print("\r");
  } else {
    state.stageSCurve = parseNumber(&args);
    if (state.stageSCurve < 1) state.stageSCurve = 1;
    if (state.stageSCurve > 100) state.stageSCurve = 100;
    Serial.print("0\r");
  }
}

void cmd_SMZ(char* args) {
  if (*args == '\0') {
    Serial.print(state.zSpeed);
    Serial.print("\r");
  } else {
    state.zSpeed = parseNumber(&args);
    if (state.zSpeed < 1) state.zSpeed = 1;
    if (state.zSpeed > 100) state.zSpeed = 100;
    Serial.print("0\r");
  }
}

void cmd_SAZ(char* args) {
  if (*args == '\0') {
    Serial.print(state.zAccel);
    Serial.print("\r");
  } else {
    state.zAccel = parseNumber(&args);
    if (state.zAccel < 1) state.zAccel = 1;
    if (state.zAccel > 100) state.zAccel = 100;
    Serial.print("0\r");
  }
}

void cmd_SCZ(char* args) {
  if (*args == '\0') {
    Serial.print(state.zSCurve);
    Serial.print("\r");
  } else {
    state.zSCurve = parseNumber(&args);
    if (state.zSCurve < 1) state.zSCurve = 1;
    if (state.zSCurve > 100) state.zSCurve = 100;
    Serial.print("0\r");
  }
}

// Direction settings
void cmd_XD(char* args) {
  long dir = parseNumber(&args);
  if (dir == 1 || dir == -1) {
    state.xDirection = dir;
  }
  Serial.print("0\r");
}

void cmd_YD(char* args) {
  long dir = parseNumber(&args);
  if (dir == 1 || dir == -1) {
    state.yDirection = dir;
  }
  Serial.print("0\r");
}

void cmd_JXD(char* args) {
  if (*args == '\0') {
    Serial.print(state.joyXDirection);
    Serial.print("\r");
  } else {
    long dir = parseNumber(&args);
    if (dir == 1 || dir == -1) {
      state.joyXDirection = dir;
    }
    Serial.print("0\r");
  }
}

void cmd_JYD(char* args) {
  if (*args == '\0') {
    Serial.print(state.joyYDirection);
    Serial.print("\r");
  } else {
    long dir = parseNumber(&args);
    if (dir == 1 || dir == -1) {
      state.joyYDirection = dir;
    }
    Serial.print("0\r");
  }
}

void cmd_JZD(char* args) {
  if (*args == '\0') {
    Serial.print(state.joyZDirection);
    Serial.print("\r");
  } else {
    long dir = parseNumber(&args);
    if (dir == 1 || dir == -1) {
      state.joyZDirection = dir;
    }
    Serial.print("0\r");
  }
}

// Information commands
void cmd_Stage() {
  Serial.print("STAGE = H101/2\r");
  Serial.print("TYPE = 1\r");
  Serial.print("SIZE_X = 108 MM\r");
  Serial.print("SIZE_Y = 71 MM\r");
  Serial.print("MICROSTEPS/MICRON = 25\r");
  Serial.print("LIMITS = NORMALLY CLOSED\r");
  Serial.print("END\r");
}

void cmd_Focus() {
  Serial.print("FOCUS = NORMAL\r");
  Serial.print("TYPE = 0\r");
  Serial.print("MICRONS/REV = ");
  Serial.print((int)state.micronsPerRevZ);
  Serial.print("\r");
  Serial.print("END\r");
}

void cmd_RES(char* args) {
  // Extract axis parameter
  char axis[8];
  char* ptr = args;
  extractCommand(&ptr, axis, sizeof(axis));

  if (*ptr == '\0' && axis[0] != '\0') {
    // Report resolution for axis
    if (axis[0] == 's' || axis[0] == 'S' || axis[0] == 'x' || axis[0] == 'X') {
      Serial.print(state.resolutionX, 3);
      Serial.print("\r");
    } else if (axis[0] == 'y' || axis[0] == 'Y') {
      Serial.print(state.resolutionY, 3);
      Serial.print("\r");
    } else if (axis[0] == 'z' || axis[0] == 'Z') {
      Serial.print(state.resolutionZ, 3);
      Serial.print("\r");
    }
  } else {
    // Set resolution
    float res = parseFloat(&ptr);
    if (axis[0] == 's' || axis[0] == 'S') {
      state.resolutionX = res;
      state.resolutionY = res;
    } else if (axis[0] == 'x' || axis[0] == 'X') {
      state.resolutionX = res;
    } else if (axis[0] == 'y' || axis[0] == 'Y') {
      state.resolutionY = res;
    } else if (axis[0] == 'z' || axis[0] == 'Z') {
      state.resolutionZ = res;
    }
    Serial.print("0\r");
  }
}

void cmd_UPR(char* args) {
  // UPR - Units Per Revolution
  char axis[8];
  char* ptr = args;
  extractCommand(&ptr, axis, sizeof(axis));

  if (*ptr == '\0' && axis[0] != '\0') {
    // Report UPR for axis
    if (axis[0] == 'z' || axis[0] == 'Z') {
      Serial.print((int)state.micronsPerRevZ);
      Serial.print("\r");
    }
  } else {
    // Set UPR
    float upr = parseFloat(&ptr);
    if (axis[0] == 'z' || axis[0] == 'Z') {
      state.micronsPerRevZ = upr;
    }
    Serial.print("0\r");
  }
}

// ============================================================================
// FILTER WHEEL COMMANDS
// ============================================================================

void cmd_7(char* args) {
  // Filter wheel control command
  char* ptr = args;

  // Get first parameter
  skipDelimiters(&ptr);

  // Check for single letter commands (C or D)
  if ((*ptr == 'C' || *ptr == 'c') && (*(ptr+1) == '\0' || isDelimiter(*(ptr+1)))) {
    // Enable automatic shutter closure
    state.filterShutterClose = true;
    Serial.print("0\r");
    return;
  }
  if ((*ptr == 'D' || *ptr == 'd') && (*(ptr+1) == '\0' || isDelimiter(*(ptr+1)))) {
    // Disable automatic shutter closure
    state.filterShutterClose = false;
    Serial.print("0\r");
    return;
  }

  // Parse wheel number
  long wheelNum = parseNumber(&ptr);

  // Check for all-wheels command (7 0,f1,f2,f3)
  if (wheelNum == 0 && !state.compatibilityMode) {
    // COMP 0 mode - move all wheels
    long f1 = parseNumber(&ptr);
    long f2 = parseNumber(&ptr);
    long f3 = parseNumber(&ptr);

    // Move each fitted wheel
    if (state.filterWheelFitted[0] && f1 > 0 && f1 <= state.filterPositionsPerWheel[0]) {
      state.filterPosition[0] = f1;
    }
    if (state.filterWheelFitted[1] && f2 > 0 && f2 <= state.filterPositionsPerWheel[1]) {
      state.filterPosition[1] = f2;
    }
    if (state.filterWheelFitted[2] && f3 > 0 && f3 <= state.filterPositionsPerWheel[2]) {
      state.filterPosition[2] = f3;
    }
    Serial.print("R\r");
    return;
  }

  // Validate wheel number (1-3)
  if (wheelNum < 1 || wheelNum > 3) {
    Serial.print("E,1\r");  // Invalid parameter
    return;
  }

  int wheelIdx = wheelNum - 1;

  // Check if wheel is fitted
  if (!state.filterWheelFitted[wheelIdx]) {
    Serial.print("E,17\r");  // No wheel fitted
    return;
  }

  // Get the filter parameter (can be number or letter)
  skipDelimiters(&ptr);

  // Check for letter commands
  if (*ptr == 'N' || *ptr == 'n') {
    // Next filter
    state.filterPosition[wheelIdx]++;
    if (state.filterPosition[wheelIdx] > state.filterPositionsPerWheel[wheelIdx]) {
      state.filterPosition[wheelIdx] = 1;  // Wrap around
    }
    Serial.print("R\r");
    return;
  }
  else if (*ptr == 'P' || *ptr == 'p') {
    // Previous filter
    state.filterPosition[wheelIdx]--;
    if (state.filterPosition[wheelIdx] < 1) {
      state.filterPosition[wheelIdx] = state.filterPositionsPerWheel[wheelIdx];  // Wrap around
    }
    Serial.print("R\r");
    return;
  }
  else if (*ptr == 'F' || *ptr == 'f') {
    // Report current filter position
    Serial.print(state.filterPosition[wheelIdx]);
    Serial.print("\r");
    return;
  }
  else if (*ptr == 'H' || *ptr == 'h') {
    // Home routine
    state.filterPosition[wheelIdx] = 1;  // Return to position 1
    Serial.print("R\r");
    return;
  }
  else if (*ptr == 'A' || *ptr == 'a') {
    // Auto home on startup
    state.filterAutoHome[wheelIdx] = true;
    Serial.print("R\r");
    return;
  }
  else if (*ptr == 'D' || *ptr == 'd') {
    // Disable auto home on startup
    state.filterAutoHome[wheelIdx] = false;
    Serial.print("R\r");
    return;
  }
  else {
    // Numeric position
    long position = parseNumber(&ptr);
    if (position < 1 || position > state.filterPositionsPerWheel[wheelIdx]) {
      Serial.print("E,1\r");  // Invalid position
      return;
    }
    state.filterPosition[wheelIdx] = position;
    Serial.print("R\r");
  }
}

void cmd_Filter(char* args) {
  // FILTER command - print information about filter wheel
  long wheelNum = parseNumber(&args);

  if (wheelNum < 1 || wheelNum > 3) {
    Serial.print("E,1\r");
    return;
  }

  int wheelIdx = wheelNum - 1;

  // Print filter wheel information
  Serial.print("FILTER_");
  Serial.print(wheelNum);
  Serial.print(" = ");

  if (state.filterWheelFitted[wheelIdx]) {
    Serial.print("HF110-10\r");  // Example model
  } else {
    Serial.print("NONE\r");
  }

  Serial.print("TYPE = 3\r");
  Serial.print("PULSES PER REV = 67200\r");
  Serial.print("FILTERS PER WHEEL = ");
  Serial.print(state.filterPositionsPerWheel[wheelIdx]);
  Serial.print("\r");
  Serial.print("OFFSET = 10080\r");
  Serial.print("HOME AT STARTUP = ");
  Serial.print(state.filterAutoHome[wheelIdx] ? "TRUE\r" : "FALSE\r");
  Serial.print("SHUTTERS CLOSED = ");
  Serial.print(state.filterShutterClose ? "TRUE\r" : "FALSE\r");
  Serial.print("END\r");
}

void cmd_FPW(char* args) {
  // FPW - Filter Positions per Wheel
  char* ptr = args;
  long wheelNum = parseNumber(&ptr);

  if (wheelNum < 1 || wheelNum > 3) {
    Serial.print("E,1\r");
    return;
  }

  int wheelIdx = wheelNum - 1;

  skipDelimiters(&ptr);

  if (*ptr == '\0') {
    // Report number of filter positions
    Serial.print(state.filterPositionsPerWheel[wheelIdx]);
    Serial.print("\r");
  } else {
    // Set number of filter positions
    long positions = parseNumber(&ptr);
    if (positions > 0 && positions <= 20) {  // Reasonable limit
      state.filterPositionsPerWheel[wheelIdx] = positions;
      // Make sure current position is valid
      if (state.filterPosition[wheelIdx] > positions) {
        state.filterPosition[wheelIdx] = positions;
      }
    }
    Serial.print("0\r");
  }
}

void cmd_SAF(char* args) {
  // SAF - Set/report filter wheel Acceleration
  char* ptr = args;
  long wheelNum = parseNumber(&ptr);

  if (wheelNum < 1 || wheelNum > 3) {
    Serial.print("E,1\r");
    return;
  }

  int wheelIdx = wheelNum - 1;

  skipDelimiters(&ptr);

  if (*ptr == '\0') {
    // Report acceleration
    Serial.print(state.filterAccel[wheelIdx]);
    Serial.print("\r");
  } else {
    // Set acceleration
    long accel = parseNumber(&ptr);
    if (accel >= 1 && accel <= 100) {
      state.filterAccel[wheelIdx] = accel;
    }
    Serial.print("0\r");
  }
}

void cmd_SCF(char* args) {
  // SCF - Set/report filter wheel S-Curve
  char* ptr = args;
  long wheelNum = parseNumber(&ptr);

  if (wheelNum < 1 || wheelNum > 3) {
    Serial.print("E,1\r");
    return;
  }

  int wheelIdx = wheelNum - 1;

  skipDelimiters(&ptr);

  if (*ptr == '\0') {
    // Report S-Curve
    Serial.print(state.filterSCurve[wheelIdx]);
    Serial.print("\r");
  } else {
    // Set S-Curve
    long scurve = parseNumber(&ptr);
    if (scurve >= 1 && scurve <= 100) {
      state.filterSCurve[wheelIdx] = scurve;
    }
    Serial.print("0\r");
  }
}

void cmd_SMF(char* args) {
  // SMF - Set/report filter wheel Maximum speed
  char* ptr = args;
  long wheelNum = parseNumber(&ptr);

  if (wheelNum < 1 || wheelNum > 3) {
    Serial.print("E,1\r");
    return;
  }

  int wheelIdx = wheelNum - 1;

  skipDelimiters(&ptr);

  if (*ptr == '\0') {
    // Report speed
    Serial.print(state.filterSpeed[wheelIdx]);
    Serial.print("\r");
  } else {
    // Set speed
    long speed = parseNumber(&ptr);
    if (speed >= 1 && speed <= 100) {
      state.filterSpeed[wheelIdx] = speed;
    }
    Serial.print("0\r");
  }
}

// ============================================================================
// SHUTTER COMMANDS
// ============================================================================

void cmd_8(char* args) {
  // Shutter control command
  char* ptr = args;

  // Parse shutter number
  long shutterNum = parseNumber(&ptr);

  // Check for set-default-state command (8 0,s1,s2,s3)
  if (shutterNum == 0) {
    long s1 = parseNumber(&ptr);
    long s2 = parseNumber(&ptr);
    long s3 = parseNumber(&ptr);

    // Set default states
    if (s1 >= 0 && s1 <= 1) {
      state.shutterDefaultState[0] = (s1 == 1);  // 0=open, 1=closed
    }
    if (s2 >= 0 && s2 <= 1) {
      state.shutterDefaultState[1] = (s2 == 1);
    }
    if (s3 >= 0 && s3 <= 1) {
      state.shutterDefaultState[2] = (s3 == 1);
    }
    Serial.print("0\r");
    return;
  }

  // Validate shutter number (1-3)
  if (shutterNum < 1 || shutterNum > 3) {
    Serial.print("E,1\r");  // Invalid parameter
    return;
  }

  int shutterIdx = shutterNum - 1;

  // Check if shutter is fitted
  if (!state.shutterFitted[shutterIdx]) {
    Serial.print("E,20\r");  // Shutter not fitted
    return;
  }

  skipDelimiters(&ptr);

  // Check if this is a status query (only shutter number provided)
  if (*ptr == '\0') {
    // Return shutter status (0=open, 1=closed)
    Serial.print(state.shutterState[shutterIdx] ? "1\r" : "0\r");
    return;
  }

  // Parse command parameter (0=open, 1=close)
  long command = parseNumber(&ptr);

  if (command != 0 && command != 1) {
    Serial.print("E,1\r");  // Invalid parameter
    return;
  }

  // Check for optional timer parameter
  skipDelimiters(&ptr);
  long timerMs = 0;
  if (*ptr != '\0') {
    timerMs = parseNumber(&ptr);
  }

  // Set shutter state
  state.shutterState[shutterIdx] = (command == 1);  // 0=open, 1=closed

  // Set timer if specified
  if (timerMs > 0) {
    state.shutterTimerEnd[shutterIdx] = millis() + timerMs;
  } else {
    state.shutterTimerEnd[shutterIdx] = 0;  // No timer
  }

  Serial.print("R\r");
}

void cmd_Shutter(char* args) {
  // SHUTTER command - print information about shutter
  long shutterNum = parseNumber(&args);

  if (shutterNum < 1 || shutterNum > 3) {
    Serial.print("E,1\r");
    return;
  }

  int shutterIdx = shutterNum - 1;

  // Print shutter information
  Serial.print("SHUTTER_");
  Serial.print(shutterNum);
  Serial.print(" = ");

  if (state.shutterFitted[shutterIdx]) {
    Serial.print("NORMAL\r");
  } else {
    Serial.print("NONE\r");
  }

  Serial.print("DEFAULT_STATE=");
  Serial.print(state.shutterDefaultState[shutterIdx] ? "CLOSED\r" : "OPEN\r");
  Serial.print("END\r");
}

// ============================================================================
// LUMEN PRO COMMANDS
// ============================================================================

void cmd_Light(char* args) {
  // LIGHT command - Lumen Pro control
  char* ptr = args;

  // Check if Lumen Pro is fitted
  if (!state.lumenProFitted) {
    Serial.print("E,20\r");  // Shutter/device not fitted
    return;
  }

  skipDelimiters(&ptr);

  // If no arguments, report current light output
  if (*ptr == '\0') {
    Serial.print(state.lumenProCurrentOutput);
    Serial.print("\r");
    return;
  }

  // Check for POWER sub-command
  char testCmd[16];
  char* tmpPtr = ptr;
  extractCommand(&tmpPtr, testCmd, sizeof(testCmd));

  if (strcmp(testCmd, "POWER") == 0) {
    // LIGHT POWER command
    ptr = tmpPtr;
    skipDelimiters(&ptr);

    if (*ptr == '\0') {
      // Report power status
      Serial.print(state.lumenProPowerOn ? "1\r" : "0\r");
    } else {
      // Set power status
      long powerState = parseNumber(&ptr);
      if (powerState == 0 || powerState == 1) {
        state.lumenProPowerOn = (powerState == 1);
        Serial.print("R\r");
      } else {
        Serial.print("E,1\r");
      }
    }
    return;
  }

  // Check if first parameter is a number or letter
  if (*ptr == 'h' || *ptr == 'H') {
    // Home routine
    state.lumenProCurrentOutput = 0;
    Serial.print("0\r");
    return;
  }

  // Parse first parameter
  long param1 = parseNumber(&ptr);

  skipDelimiters(&ptr);

  // Check if there's a second parameter
  if (*ptr == '\0') {
    // Single number parameter: LIGHT n - Set light output to n%
    if (param1 >= 0 && param1 <= 100) {
      state.lumenProCurrentOutput = param1;
      Serial.print("0\r");
    } else {
      Serial.print("E,1\r");  // Invalid parameter
    }
    return;
  }

  // Two parameters: Either LIGHT P,n or LIGHT P,?
  // param1 is position P
  if (param1 < 1 || param1 > 10) {
    Serial.print("E,1\r");  // Invalid position
    return;
  }

  int posIdx = param1 - 1;

  // Check if second parameter is '?'
  if (*ptr == '?') {
    // LIGHT P,? - Report % output of position P
    Serial.print(state.lumenProPositionOutput[posIdx]);
    Serial.print("\r");
    return;
  }

  // LIGHT P,n - Set position P to n% light output
  long lightOutput = parseNumber(&ptr);
  if (lightOutput >= 0 && lightOutput <= 100) {
    state.lumenProPositionOutput[posIdx] = lightOutput;
    Serial.print("0\r");
  } else {
    Serial.print("E,1\r");  // Invalid parameter
  }
}
