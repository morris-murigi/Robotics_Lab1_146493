/***********************************************************************
 * FSD_Vision_Processor.ino
 * - Captures low-resolution grayscale images.
 * - Processes the image to find lane offset, obstacles, and distance.
 * - Sends a data string over Serial2 to the main ESP32.
 *
 * Connections:
 * - Main ESP32 TX  -> ESP32-CAM UOR (GPIO 14)
 * - Main ESP32 RX  -> ESP32-CAM UOT (GPIO 15)
 ***********************************************************************/

#include "esp_camera.h"
#include "Arduino.h"

// Define the Serial port for communication with the main ESP32
// We'll use Serial2 (GPIO 14 as RX, GPIO 15 as TX by default on CAM)
// NOTE: The pin numbers are different on the AI-THINKER module
// UOT = GPIO 15
// UOR = GPIO 14
// We will use GPIO 1 (U0TXD) and GPIO 3 (U0RXD) to talk to the Main ESP32
// And Serial (GPIO 1, 3) for USB debugging
#define SERIAL_TO_MAINBOARD Serial

// AI-THINKER CAMERA PINS (from datasheet/lecture)
#define CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// --- Vision Processing Constants (from Lab 1 Manual & MATLAB) ---
// Pixel Labelling Thresholds
const int TW_THRESHOLD = 205; // White lane
const int TY_THRESHOLD = 175; // Yellow lane
const int TO_THRESHOLD = 100; // Obstacle

// Camera & World Parameters
const int CAM_IMG_WIDTH = 160;  // QQVGA width
const int CAM_IMG_HEIGHT = 120; // QQVGA height
const float CAM_FOV_DEGREES = 60.0;
const float OBSTACLE_REAL_WIDTH_METERS = 0.2; // 20cm cube

// Struct to hold all the data we find
struct VisionData {
  float laneOffsetPx = 0.0;
  float laneWidthPx = 0.0;
  bool obstacleDetected = false;
  float obstacleDistanceM = 99.0;
  int obstacleWidthPx = 0;
};

void setup() {
  // Start the serial for debugging
  Serial.begin(115200);
  Serial.println("ESP32-CAM Vision Processor Booting...");

  // Start the serial for communication with the main ESP32
  SERIAL_TO_MAINBOARD.begin(115200, SERIAL_8N1, 3, 1); // Use GPIO 3 (RX) and 1 (TX)

  // Camera Config
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  
  // --- CRITICAL ---
  // We set pixel format to GRAYSCALE and a low resolution
  // to make processing fast enough for the ESP32.
  config.pixel_format = PIXFORMAT_GRAYSCALE; // [cite: 11031]
  config.frame_size = FRAMESIZE_QQVGA;     // 160x120
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  Serial.println("Camera Init OK.");
}

void loop() {
  // 1. Capture a frame
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Frame capture failed");
    return;
  }

  // 2. Process the frame
  VisionData data = processFrame(fb->buf, fb->width, fb->height);

  // 3. Return the frame buffer so it can be reused
  esp_camera_fb_return(fb);

  // 4. Send the results to the main ESP32
  // Format: L:[offset],W:[width],O:[detected],D:[distance]
  char buffer[100];
  snprintf(buffer, sizeof(buffer), "L:%.1f,W:%.1f,O:%d,D:%.2f\n",
           data.laneOffsetPx,
           data.laneWidthPx,
           data.obstacleDetected ? 1 : 0,
           data.obstacleDistanceM);
           
  SERIAL_TO_MAINBOARD.print(buffer);
  
  // Optional: print to debug serial
  Serial.print("Sent: ");
  Serial.print(buffer);

  // Run at ~10Hz, matching the main control loop
  delay(100); 
}

/**
 * @brief Processes a grayscale frame buffer to find lanes and obstacles.
 * This function contains all the logic from the MATLAB script.
 */
VisionData processFrame(uint8_t *buf, int width, int height) {
  VisionData data;

  // --- A1.3: Lane Following ---
  // Sample one row, 60% down the image
  int sampleRow = height * 0.6;
  int xl = 0;       // Left-most white pixel
  int xr = width;   // Right-most yellow pixel
  
  for (int c = 0; c < width; c++) {
    uint8_t pixel = buf[sampleRow * width + c]; // Get pixel brightness
    
    // This is the A1.2 labelling logic
    if (pixel > TW_THRESHOLD) { // White
      xl = max(xl, c);
    } else if (pixel > TY_THRESHOLD) { // Yellow
      xr = min(xr, c);
    }
  }

  if (xl > 0 && xr < width) { // If we found both lanes
    data.laneWidthPx = xr - xl;
    float laneCenterPx = (xl + xr) / 2.0;
    data.laneOffsetPx = laneCenterPx - (width / 2.0);
  } else {
    data.laneOffsetPx = 0.0; // No lane found, assume center
    data.laneWidthPx = 0.0;
  }

  // --- A1.4: Obstacle Detection ---
  int r_start = height * 0.5;
  int r_end = height * 0.7;
  int c_start = width * 0.4;
  int c_end = width * 0.6;
  int obstaclePixelCount = 0;
  int c_min = width;
  int c_max = 0;
  
  for (int r = r_start; r < r_end; r++) {
    for (int c = c_start; c < c_end; c++) {
      uint8_t pixel = buf[r * width + c];
      // A1.2 logic for obstacle
      if (pixel > TO_THRESHOLD && pixel <= TY_THRESHOLD) {
        obstaclePixelCount++;
        c_min = min(c_min, c);
        c_max = max(c_max, c);
      }
    }
  }
  
  if (obstaclePixelCount > 5) { // Require a few pixels to trigger
    data.obstacleDetected = true;
    
    // --- A1.5: Object Size ---
    data.obstacleWidthPx = c_max - c_min + 1;
    
    // --- A1.6: Object Distance ---
    float theta_o_deg = ((float)data.obstacleWidthPx / (float)width) * CAM_FOV_DEGREES;
    float theta_o_rad = radians(theta_o_deg);
    if (theta_o_rad > 0) {
      data.obstacleDistanceM = OBSTACLE_REAL_WIDTH_METERS / theta_o_rad;
    } else {
      data.obstacleDistanceM = 99.0;
    }
  } else {
    data.obstacleDetected = false;
    data.obstacleWidthPx = 0;
    data.obstacleDistanceM = 99.0;
  }
  
  return data;
}