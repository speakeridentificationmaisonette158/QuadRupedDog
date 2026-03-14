/*
 * XIAO ESP32-S3 Sense — Camera Streaming Firmware
 *
 * Captures frames from the onboard OV2640 camera and serves them
 * as an MJPEG stream over WiFi.  The Raspberry Pi connects to
 * http://<xiao-ip>:81/stream to receive the video feed.
 *
 * WiFi credentials are set via serial command or hardcoded below
 * (move to a secrets header for production use).
 *
 * Pin mapping (XIAO ESP32-S3 Sense built-in camera):
 *   Directly connected via the camera FPC — no user wiring required.
 */

#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"

// ============== WIFI CONFIGURATION ==============
// Set via environment or change here for your network
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// ============== CAMERA PIN DEFINITIONS ==============
// XIAO ESP32-S3 Sense onboard camera (OV2640)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39

#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13

// ============== STREAM SERVER ==============
WebServer server(81);

// MJPEG stream boundary
#define BOUNDARY "frame"

/**
 * Initialize the OV2640 camera with JPEG output.
 */
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  // Use PSRAM for higher resolution
  if (psramFound()) {
    config.frame_size   = FRAMESIZE_VGA;   // 640x480
    config.jpeg_quality = 12;
    config.fb_count     = 2;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
  } else {
    config.frame_size   = FRAMESIZE_QVGA;  // 320x240
    config.jpeg_quality = 15;
    config.fb_count     = 1;
    config.fb_location  = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  Serial.println("Camera initialized");
  return true;
}

/**
 * Handle MJPEG stream requests.
 *
 * Sends a multipart response where each part is a JPEG frame.
 * The client (Pi's OpenCV VideoCapture) reconnects automatically.
 */
void handleStream() {
  WiFiClient client = server.client();

  String header = "HTTP/1.1 200 OK\r\n"
                  "Content-Type: multipart/x-mixed-replace; boundary=" BOUNDARY "\r\n"
                  "\r\n";
  client.print(header);

  while (client.connected()) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Frame capture failed");
      break;
    }

    String part = "--" BOUNDARY "\r\n"
                  "Content-Type: image/jpeg\r\n"
                  "Content-Length: " + String(fb->len) + "\r\n"
                  "\r\n";
    client.print(part);
    client.write(fb->buf, fb->len);
    client.print("\r\n");

    esp_camera_fb_return(fb);

    // ~15 fps cap to avoid saturating WiFi
    delay(66);
  }
}

/**
 * Handle single-frame snapshot requests at /capture.
 */
void handleCapture() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "Capture failed");
    return;
  }

  server.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
  server.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

/**
 * Simple status page at /.
 */
void handleRoot() {
  String html = "<html><body>"
                "<h1>XIAO Camera</h1>"
                "<p><a href='/stream'>MJPEG Stream</a></p>"
                "<p><a href='/capture'>Snapshot</a></p>"
                "</body></html>";
  server.send(200, "text/html", html);
}

// ============== SETUP ==============

void setup() {
  Serial.begin(115200);
  Serial.println("XIAO ESP32-S3 Camera Module");
  Serial.println("===========================");

  // Initialize camera
  if (!initCamera()) {
    Serial.println("FATAL: Camera init failed. Halting.");
    while (true) { delay(1000); }
  }

  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("Connected! IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("WARNING: WiFi connection failed. Stream unavailable.");
    Serial.println("Camera will still capture locally.");
  }

  // Set up HTTP routes
  server.on("/", handleRoot);
  server.on("/stream", HTTP_GET, handleStream);
  server.on("/capture", HTTP_GET, handleCapture);

  server.begin();
  Serial.println("Stream server started on port 81");
  Serial.println("Ready!");
}

// ============== MAIN LOOP ==============

void loop() {
  server.handleClient();
}
