/***********************************************************************
 * FSD_Main_Control.ino (No GSM)
 * - Reads vision data from ESP32-CAM via Serial1.
 * - Reads IMU (MPU6050) via I2C.
 * - Reads wheel encoders via Interrupts.
 * - Runs a State Machine (LANE_FOLLOWING, AVOID_OBSTACLE).
 * - Controls two ESCs via ESP32Servo (PWM).
 ***********************************************************************/

// --- Includes ---
#include <ESP32Servo.h>     // For ESC control
#include <Wire.h>           // For I2C
#include <Adafruit_MPU6050.h> // For IMU
#include <Adafruit_Sensor.h>
#include <HardwareSerial.h> // For multiple serial ports

// --- Pin Definitions (from schematic) ---
// Note: Schematic uses ESP32-S3. Pins may vary slightly.
// Adjust these pins to match your board's physical layout.

// Serial Ports
#define SERIAL_DEBUG Serial    // PC Debug (USB)
#define SERIAL_CAM   Serial1   // To ESP32-CAM (GPIO 9, 10?)

// I2C (Schematic: MPU6050 & HMC5883L share I2C bus)
#define I2C_SDA_PIN 8  //
#define I2C_SCL_PIN 9  //

// Motor Control (ESC PWM)
// The schematic shows "M1 ESC" and "M2 ESC" headers.
// We'll assign them to ESP32 PWM-capable pins.
const int ESC_LEFT_PIN = 2;   // Example pin, must be PWM capable
const int ESC_RIGHT_PIN = 4;  // Example pin, must be PWM capable

// Wheel Encoders (TCRT5000)
// Schematic shows "MOT SPEED" and "CAR SPEED"
const int ENCODER_LEFT_PIN = 34;  // "MOT SPEED"
const int ENCODER_RIGHT_PIN = 35; // "CAR SPEED"

// --- Global Constants ---
const long LOOP_INTERVAL_MS = 100; // 100ms loop time
const float OBSTACLE_AVOID_DISTANCE = 1.5; // (m) Start avoidance
const float WHEEL_RADIUS_METERS = 0.02; // 2cm
const int ENCODER_PULSES_PER_REV = 20;  // Assumed, TUNE THIS

// --- Global Variables ---
// State Machine
enum FsdState { LANE_FOLLOWING, AVOID_OBSTACLE };
FsdState currentState = LANE_FOLLOWING;

// Sensor Data
float g_laneOffsetPx = 0.0;
bool  g_obstacleDetected = false;
float g_obstacleDistanceM = 99.0;
float g_imuRoll = 0.0;
float g_imuPitch = 0.0;

// Odometry
volatile long g_encoderCountLeft = 0;
volatile long g_encoderCountRight = 0;
float g_totalDistance = 0.0;
long g_lastLoopTime = 0;

// Motor Control
int g_baseSpeed = 60; // 0-180 servo scale
int g_leftMotorSpeed = 90;
int g_rightMotorSpeed = 90;

// --- Hardware Objects ---
Servo escLeft;
Servo escRight;
Adafruit_MPU6050 mpu;

HardwareSerial SerialCam(1); // Use UART 1

// --- Interrupt Service Routines (ISRs) ---
void IRAM_ATTR isr_left_encoder() {
  // Use += 1 to avoid C++ volatile increment warning
  g_encoderCountLeft += 1;
}
void IRAM_ATTR isr_right_encoder() {
  // Use += 1 to avoid C++ volatile increment warning
  g_encoderCountRight += 1;
}

void setup() {
  SERIAL_DEBUG.begin(115200);
  SERIAL_DEBUG.println("FSD Main Control Booting...");

  // Init Serial Ports
  //SERIAL_CAM.begin(115200, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);
  // For now, let's use the defaults on Serial1
  SERIAL_CAM.begin(115200); // RX=GPIO9, TX=GPIO10 on standard ESP32

  // Init I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!mpu.begin()) {
    SERIAL_DEBUG.println("Failed to find MPU6050 chip");
  } else {
    SERIAL_DEBUG.println("MPU6050 Found!");
  }

  // Init Encoders
  pinMode(ENCODER_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), isr_left_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), isr_right_encoder, RISING);

  // Init ESCs
  escLeft.attach(ESC_LEFT_PIN, 1000, 2000); // 1000us min, 2000us max
  escRight.attach(ESC_RIGHT_PIN, 1000, 2000);
  
  SERIAL_DEBUG.println("Arming ESCs... send neutral pulse.");
  escLeft.write(90);  // 90 = 1500us (neutral)
  escRight.write(90);
  delay(3000); // Wait for ESCs to arm
  SERIAL_DEBUG.println("ESCs Armed.");

  g_lastLoopTime = millis();
}

void loop() {
  // --- 1. 100ms Timed Loop ---
  if (millis() - g_lastLoopTime >= LOOP_INTERVAL_MS) {
    g_lastLoopTime = millis();
    float dt = LOOP_INTERVAL_MS / 1000.0; // Time delta in seconds

    // --- 2. Read Sensors ---
    readCameraData();
    readImuData();
    updateOdometry(dt);

    // --- 3. Run State Machine ---
    runStateMachine();

    // --- 4. Execute Control Logic ---
    executeControl();

    // --- 5. Update Actuators ---
    updateMotors();

    // --- 6. Report Status ---
    // (Example: send status to debug serial every 10 loops = 1 sec)
    static int loopCount = 0;
    if (loopCount++ > 10) {
      loopCount = 0;
      char statusMsg[50];
      snprintf(statusMsg, sizeof(statusMsg), "State: %d, Dist: %.1f", currentState, g_obstacleDistanceM);
      SERIAL_DEBUG.println(statusMsg);
    }
  }
}

/**
 * @brief Checks for and parses data from the ESP32-CAM.
 */
void readCameraData() {
  if (SERIAL_CAM.available()) {
    String data = SERIAL_CAM.readStringUntil('\n');
    SERIAL_DEBUG.print("CAM Data: ");
    SERIAL_DEBUG.println(data);
    
    // Parse the string: "L:offset,W:width,O:detected,D:distance"
    // This is a simple parser. A robust one would be better.
    float L, W, D;
    int O;
    int parsed = sscanf(data.c_str(), "L:%f,W:%f,O:%d,D:%f", &L, &W, &O, &D);

    if (parsed == 4) {
      g_laneOffsetPx = L;
      g_obstacleDetected = (O == 1);
      g_obstacleDistanceM = D;
    }
  }
}

/**
 * @brief Reads Roll and Pitch from the MPU6050.
 */
void readImuData() {
  // *** FIX: Removed the .available() check ***
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Simple Roll/Pitch calculation from accelerometer
  // This is noisy and inaccurate when moving. A real FSD
  // would use a Kalman filter or sensor fusion.
  g_imuRoll = atan2(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;
  g_imuPitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * RAD_TO_DEG;
}

/**
 * @brief Calculates RPM and distance from encoder ticks. (Appendix A2)
 */
void updateOdometry(float dt) {
  // Read and reset encoder counts in a "critical section"
  // to prevent interrupts from changing them during the read.
  noInterrupts();
  long countL = g_encoderCountLeft;
  long countR = g_encoderCountRight;
  g_encoderCountLeft = 0;
  g_encoderCountRight = 0;
  interrupts();

  // Calculate RPM (A2, Step 1)
  float rpsL = (countL / (float)ENCODER_PULSES_PER_REV) / dt;
  float rpmL = rpsL * 60;
  float rpsR = (countR / (float)ENCODER_PULSES_PER_REV) / dt;
  float rpmR = rpsR * 60;

  // Calculate linear velocity (v)
  float vL = 2 * PI * WHEEL_RADIUS_METERS * rpsL;
  float vR = 2 * PI * WHEEL_RADIUS_METERS * rpsR;
  float v_avg = (vL + vR) / 2.0;

  // Estimate distance (A2, Step 2)
  float distance_travelled = v_avg * dt;
  
  // Refine with Pitch (A2, Step 4)
  // If car is pitched up/down, true forward distance is reduced
  g_totalDistance += distance_travelled * cos(radians(g_imuPitch));

  SERIAL_DEBUG.printf("RPM L: %.1f, R: %.1f. Dist: %.3f\n", rpmL, rpmR, g_totalDistance);
}

/**
 * @brief Main FSD state machine.
 */
void runStateMachine() {
  switch (currentState) {
    case LANE_FOLLOWING:
      if (g_obstacleDetected && g_obstacleDistanceM < OBSTACLE_AVOID_DISTANCE) {
        SERIAL_DEBUG.println("STATE CHANGE -> AVOID_OBSTACLE");
        currentState = AVOID_OBSTACLE;
        // Here you would trigger your planner
      }
      break;

    case AVOID_OBSTACLE:
      if (!g_obstacleDetected || g_obstacleDistanceM >= OBSTACLE_AVOID_DISTANCE) {
        SERIAL_DEBUG.println("STATE CHANGE -> LANE_FOLLOWING");
        currentState = LANE_FOLLOWING;
      }
      // Obstacle avoidance logic would be running here
      break;
  }
}

/**
 * @brief Calculates motor speeds based on state.
 */
void executeControl() {
  if (currentState == LANE_FOLLOWING) {
    // --- Stanley-style Controller (Simplified) ---
    // 1. Heading correction (from IMU roll)
    // We'll use a simple P-controller for roll.
    float rollError = 0.0 - g_imuRoll; // Target roll is 0
    float kP_roll = 0.5; // TUNE THIS
    float steering_roll = kP_roll * rollError;

    // 2. Lateral offset correction (from camera)
    float offsetError = g_laneOffsetPx; // Target offset is 0
    float kP_offset = 0.3; // TUNE THIS
    float steering_offset = kP_offset * offsetError;
    
    // Total steering command (clamped)
    float steering = constrain(steering_roll + steering_offset, -90, 90);

    // Map steering to differential motor speeds
    // 90 = neutral, 0=reverse, 180=forward
    g_leftMotorSpeed = 90 + g_baseSpeed + steering;
    g_rightMotorSpeed = 90 + g_baseSpeed - steering; // Opposite steering
    
    // Constrain to valid servo range
    g_leftMotorSpeed = constrain(g_leftMotorSpeed, 90, 180);
    g_rightMotorSpeed = constrain(g_rightMotorSpeed, 90, 180);

  } else if (currentState == AVOID_OBSTACLE) {
    // This is the "rule-based planner" from the lab
    // Example: Turn left to avoid
    SERIAL_DEBUG.println("Executing Avoidance: Turning Left");
    g_leftMotorSpeed = 90 + g_baseSpeed - 30; // Slow down left motor
    g_rightMotorSpeed = 90 + g_baseSpeed + 30; // Speed up right motor
  }
}

/**
 * @brief Sends the final PWM commands to the ESCs.
 */
void updateMotors() {
  escLeft.write(g_leftMotorSpeed);
  escRight.write(g_rightMotorSpeed);
  SERIAL_DEBUG.printf("Motors L: %d, R: %d\n", g_leftMotorSpeed, g_rightMotorSpeed);
}