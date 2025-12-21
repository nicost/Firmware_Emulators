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

// Motion helper functions
void startXYMotion(long targetX, long targetY);
void startZMotion(long targetZ);
void startXYZMotion(long targetX, long targetY, long targetZ);
void updateXYMotion();
void updateZMotion();
void stopAllMotion();
byte calculateMotionStatus();

void setup() {
  Serial.begin(DEFAULT_BAUD);
  initializeState();
  bufferIndex = 0;
}

void loop() {
  // Update motion (interpolate position)
  updateXYMotion();
  updateZMotion();

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
