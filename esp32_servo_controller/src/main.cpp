/*
 * Quadruped Robot - ESP32 Servo Controller
 * 
 * Receives joint angle commands from Raspberry Pi via UART
 * Drives 12 servos via PCA9685 I2C PWM driver
 * Reads BNO055 IMU for orientation feedback
 * 
 * Hardware Connections:
 *   ESP32 GPIO21 (SDA) -> PCA9685 SDA, BNO055 SDA
 *   ESP32 GPIO22 (SCL) -> PCA9685 SCL, BNO055 SCL
 *   ESP32 GPIO16 (RX2) -> Raspberry Pi GPIO14 (TX)
 *   ESP32 GPIO17 (TX2) -> Raspberry Pi GPIO15 (RX)
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

// ============== CONFIGURATION ==============

// PCA9685 Settings
#define PCA9685_ADDRESS 0x40
#define SERVO_FREQ 50  // 50Hz for standard servos

// Servo pulse width limits (in microseconds)
#define SERVO_MIN_US 500   // Minimum pulse width
#define SERVO_MAX_US 2500  // Maximum pulse width

// Servo angle limits (degrees)
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

// Number of servos (4 legs × 3 joints)
#define NUM_SERVOS 12

// Servo channel mapping
// Format: {Front-Left, Front-Right, Rear-Left, Rear-Right} × {Hip, Thigh, Knee}
const uint8_t SERVO_CHANNELS[NUM_SERVOS] = {
  0, 1, 2,    // Front-Left: Hip, Thigh, Knee
  3, 4, 5,    // Front-Right: Hip, Thigh, Knee
  6, 7, 8,    // Rear-Left: Hip, Thigh, Knee
  9, 10, 11   // Rear-Right: Hip, Thigh, Knee
};

// Servo direction (1 = normal, -1 = reversed)
// Adjust based on how your servos are mounted
int8_t SERVO_DIRECTION[NUM_SERVOS] = {
  1, 1, 1,    // Front-Left
  -1, -1, -1, // Front-Right (mirrored)
  1, 1, 1,    // Rear-Left
  -1, -1, -1  // Rear-Right (mirrored)
};

// Servo center offsets (degrees) - calibrate these!
float SERVO_OFFSETS[NUM_SERVOS] = {
  90, 90, 90,  // Front-Left
  90, 90, 90,  // Front-Right
  90, 90, 90,  // Rear-Left
  90, 90, 90   // Rear-Right
};

// UART Settings
#define SERIAL_BAUD 115200
#define PI_SERIAL Serial  // Use USB Serial instead of GPIO

// IMU Settings
#define BNO055_ADDRESS 0x28
#define IMU_UPDATE_INTERVAL_MS 10  // 100Hz IMU updates

// ============== GLOBAL OBJECTS ==============

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS);

// Current servo positions (degrees)
float currentAngles[NUM_SERVOS];
float targetAngles[NUM_SERVOS];

// IMU data
float roll, pitch, yaw;
bool imuConnected = false;

// Timing
unsigned long lastImuUpdate = 0;
unsigned long lastServoUpdate = 0;

// Serial command buffer
#define CMD_BUFFER_SIZE 256
char cmdBuffer[CMD_BUFFER_SIZE];
int cmdIndex = 0;

// ============== HELPER FUNCTIONS ==============

/**
 * Convert angle (degrees) to PCA9685 PWM value
 */
uint16_t angleToPWM(float angle) {
  // Constrain angle to valid range
  angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  
  // Convert angle to pulse width in microseconds
  float pulseUs = map(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_US, SERVO_MAX_US);
  
  // Convert pulse width to PCA9685 tick value (4096 ticks per 20ms period)
  uint16_t pwmValue = (uint16_t)((pulseUs / 20000.0) * 4096.0);
  
  return pwmValue;
}

/**
 * Set a single servo to a specific angle
 */
void setServoAngle(uint8_t servoIndex, float angle) {
  if (servoIndex >= NUM_SERVOS) return;
  
  // Apply direction and offset
  float adjustedAngle = SERVO_OFFSETS[servoIndex] + (angle * SERVO_DIRECTION[servoIndex]);
  
  // Constrain to valid range
  adjustedAngle = constrain(adjustedAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  
  // Set PWM
  uint8_t channel = SERVO_CHANNELS[servoIndex];
  uint16_t pwmValue = angleToPWM(adjustedAngle);
  pwm.setPWM(channel, 0, pwmValue);
  
  // Update current position
  currentAngles[servoIndex] = angle;
}

/**
 * Set all servos to their target angles (with optional smoothing)
 */
void updateServos() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoAngle(i, targetAngles[i]);
  }
}

/**
 * Read IMU orientation data
 */
void updateIMU() {
  if (!imuConnected) return;
  
  sensors_event_t event;
  bno.getEvent(&event);
  
  // Get Euler angles (in degrees)
  roll = event.orientation.z;
  pitch = event.orientation.y;
  yaw = event.orientation.x;
}

/**
 * Send IMU data to Raspberry Pi
 */
void sendIMUData() {
  if (!imuConnected) return;
  
  // Format: IMU:roll,pitch,yaw
  PI_SERIAL.print("IMU:");
  PI_SERIAL.print(roll, 2);
  PI_SERIAL.print(",");
  PI_SERIAL.print(pitch, 2);
  PI_SERIAL.print(",");
  PI_SERIAL.println(yaw, 2);
}

/**
 * Parse and execute a command from the Raspberry Pi
 * 
 * Command formats:
 *   ANGLES:a0,a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11  - Set all 12 joint angles
 *   SERVO:index,angle                              - Set single servo
 *   LEG:legIndex,shoulder,hip,knee                 - Set one leg (0-3)
 *   CENTER                                          - Center all servos
 *   GETIMU                                          - Request IMU data
 *   CALIBRATE:index,offset                          - Set servo offset
 *   STATUS                                          - Get system status
 */
void processCommand(const char* cmd) {
  // Parse command type
  if (strncmp(cmd, "ANGLES:", 7) == 0) {
    // Parse 12 comma-separated angles
    const char* ptr = cmd + 7;
    for (int i = 0; i < NUM_SERVOS; i++) {
      targetAngles[i] = atof(ptr);
      ptr = strchr(ptr, ',');
      if (ptr) ptr++;
      else if (i < NUM_SERVOS - 1) {
        PI_SERIAL.println("ERR:ANGLES_PARSE");
        return;
      }
    }
    updateServos();
    PI_SERIAL.println("OK:ANGLES");
  }
  else if (strncmp(cmd, "SERVO:", 6) == 0) {
    // Parse single servo command
    int index;
    float angle;
    if (sscanf(cmd + 6, "%d,%f", &index, &angle) == 2) {
      if (index >= 0 && index < NUM_SERVOS) {
        targetAngles[index] = angle;
        setServoAngle(index, angle);
        PI_SERIAL.println("OK:SERVO");
      } else {
        PI_SERIAL.println("ERR:SERVO_INDEX");
      }
    } else {
      PI_SERIAL.println("ERR:SERVO_PARSE");
    }
  }
  else if (strncmp(cmd, "LEG:", 4) == 0) {
    // Parse leg command (legIndex, shoulder, hip, knee)
    int legIndex;
    float shoulder, hip, knee;
    if (sscanf(cmd + 4, "%d,%f,%f,%f", &legIndex, &shoulder, &hip, &knee) == 4) {
      if (legIndex >= 0 && legIndex < 4) {
        int baseIdx = legIndex * 3;
        targetAngles[baseIdx] = shoulder;      // Shoulder on base plate
        targetAngles[baseIdx + 1] = hip;       // Hip on leg
        targetAngles[baseIdx + 2] = knee;      // Knee on leg
        setServoAngle(baseIdx, shoulder);
        setServoAngle(baseIdx + 1, hip);
        setServoAngle(baseIdx + 2, knee);
        PI_SERIAL.println("OK:LEG");
      } else {
        PI_SERIAL.println("ERR:LEG_INDEX");
      }
    } else {
      PI_SERIAL.println("ERR:LEG_PARSE");
    }
  }
  else if (strcmp(cmd, "CENTER") == 0) {
    // Center all servos (set all angles to 0)
    for (int i = 0; i < NUM_SERVOS; i++) {
      targetAngles[i] = 0;
    }
    updateServos();
    PI_SERIAL.println("OK:CENTER");
  }
  else if (strcmp(cmd, "GETIMU") == 0) {
    sendIMUData();
  }
  else if (strncmp(cmd, "CALIBRATE:", 10) == 0) {
    // Set servo offset
    int index;
    float offset;
    if (sscanf(cmd + 10, "%d,%f", &index, &offset) == 2) {
      if (index >= 0 && index < NUM_SERVOS) {
        SERVO_OFFSETS[index] = offset;
        PI_SERIAL.println("OK:CALIBRATE");
      } else {
        PI_SERIAL.println("ERR:CAL_INDEX");
      }
    } else {
      PI_SERIAL.println("ERR:CAL_PARSE");
    }
  }
  else if (strcmp(cmd, "STATUS") == 0) {
    PI_SERIAL.print("STATUS:IMU=");
    PI_SERIAL.print(imuConnected ? "OK" : "FAIL");
    PI_SERIAL.print(",SERVOS=");
    PI_SERIAL.println(NUM_SERVOS);
  }
  else {
    PI_SERIAL.println("ERR:UNKNOWN_CMD");
  }
}

/**
 * Read and process serial commands from Raspberry Pi
 */
void processSerial() {
  while (PI_SERIAL.available()) {
    char c = PI_SERIAL.read();
    
    if (c == '\n' || c == '\r') {
      if (cmdIndex > 0) {
        cmdBuffer[cmdIndex] = '\0';
        processCommand(cmdBuffer);
        cmdIndex = 0;
      }
    } else if (cmdIndex < CMD_BUFFER_SIZE - 1) {
      cmdBuffer[cmdIndex++] = c;
    }
  }
}

// ============== SETUP ==============

void setup() {
  // Initialize USB serial (used for both debug AND Pi communication)
  Serial.begin(115200);
  Serial.println("Quadruped ESP32 Servo Controller");
  Serial.println("================================");
  Serial.println("Using USB Serial for Pi communication");
  
  // Initialize I2C
  Wire.begin(21, 22);  // SDA=GPIO21, SCL=GPIO22
  Wire.setClock(400000);  // 400kHz I2C
  Serial.println("I2C initialized on GPIO21/22");
  
  // Initialize PCA9685
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  Serial.println("PCA9685 initialized");
  
  // Initialize BNO055 IMU
  if (bno.begin()) {
    imuConnected = true;
    bno.setExtCrystalUse(true);
    Serial.println("BNO055 IMU initialized");
  } else {
    imuConnected = false;
    Serial.println("WARNING: BNO055 not found!");
  }
  
  // Initialize servo positions to center
  for (int i = 0; i < NUM_SERVOS; i++) {
    currentAngles[i] = 0;
    targetAngles[i] = 0;
  }
  updateServos();
  Serial.println("Servos centered");
  
  // Ready
  Serial.println("Ready!");
  PI_SERIAL.println("READY");
}

// ============== MAIN LOOP ==============

void loop() {
  unsigned long now = millis();
  
  // Process incoming serial commands
  processSerial();
  
  // Update IMU at fixed interval
  if (imuConnected && (now - lastImuUpdate >= IMU_UPDATE_INTERVAL_MS)) {
    updateIMU();
    lastImuUpdate = now;
  }
  
  // Small delay to prevent overwhelming the system
  delay(1);
}
