/*
  RTK GNSS System - Rover Firmware

  PURPOSE:
  This firmware connects an ESP32 to South Africa's TrigNet NTRIP service to 
  receive RTK corrections for high-precision GPS positioning using a u-blox 
  ZED-F9P GNSS module. It logs both ellipsoidal (WGS84) and orthometric (MSL) 
  heights with geoid separation values.
  
  KEY FEATURES:
  - Dual height logging (Ellipsoidal + MSL/Orthometric + Geoid Separation)
  - Point averaging for improved accuracy
  - OLED display for real-time status
  - SD card logging with daily file rotation
  - Web-based configuration interface
  - Button controls for display toggle and logging
  
  HARDWARE REQUIREMENTS:
  - ESP32 development board
  - u-blox ZED-F9P GNSS module
  - SH1106 OLED display (128x64)
  - SD card module
  - Two push buttons
*/

// ==================== LIBRARY INCLUDES ====================
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <base64.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

SFE_UBLOX_GNSS myGNSS;
Preferences preferences;
WebServer server(80);

//====================OLED CONFIG==========================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_I2C_ADDRESS 0x3C
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//ZED F9P configuration pins
#define RXD2 16
#define TXD2 17
HardwareSerial gpsSerial(1);
#define LED_BUILTIN 2

//Push buttons configuration
#define BUTTON_PIN 4          // Display toggle button
#define BUTTON_LOG_PIN 15     // SD logging control button
#define DEBOUNCE_DELAY 200

//SD card CS configuration pin
#define SD_CS 5

//I2C for OLED configuration pins
#define I2C_SDA 21
#define I2C_SCL 22

//====================WIFI & NTRIP CONFIG (from NVS)=================
String stored_ssid = "";
String stored_password = "";
String stored_casterHost = "";
int stored_casterPort = 2101;
String stored_mountPoint = "";
String stored_casterUser = "";
String stored_casterUserPW = "";

// Setup Mode AP credentials
const char* setup_ap_ssid = "GPS_Logger_Setup";
const char* setup_ap_password = "configure123";

//====================POINT AVERAGING STRUCTURES=================
struct PointAccumulator {
  int pointNumber = -1;
  double sumLat = 0.0;
  double sumLon = 0.0;
  double sumAltMSL = 0.0;           // MSL/Orthometric height
  double sumAltEllipsoid = 0.0;     // Ellipsoidal height
  double sumHDOP = 0.0;
  double sumPDOP = 0.0;
  uint64_t sumHAcc = 0;
  uint64_t sumVAcc = 0;
  int count = 0;
  
  int fixedCount = 0;
  int floatCount = 0;
  int standardCount = 0;
  
  uint32_t bestHAcc = 4294967295;
  uint32_t bestVAcc = 4294967295;
  uint16_t bestHDOP = 9999;
  
  String firstTimestamp = "";
  String lastTimestamp = "";
  
  // Resets all rover GNSS data variables and statistics to their initial state for a new measurement session.
  void reset() {
    pointNumber = -1;
    sumLat = sumLon = sumAltMSL = sumAltEllipsoid = sumHDOP = sumPDOP = 0.0;
    sumHAcc = sumVAcc = 0;
    count = 0;
    fixedCount = floatCount = standardCount = 0;
    bestHAcc = 4294967295;
    bestVAcc = 4294967295;
    bestHDOP = 9999;
    firstTimestamp = "";
    lastTimestamp = "";
  }
  
  // Adds a new GNSS measurement to the rover’s dataset, updating cumulative sums, accuracy metrics, and timestamps.
  void addMeasurement(double lat, double lon, double altMSL, double altEllipsoid,
                     uint16_t hDOP, uint16_t pDOP, uint32_t hAcc, uint32_t vAcc, 
                     uint8_t carrSoln, String timestamp) {
    sumLat += lat;
    sumLon += lon;
    sumAltMSL += altMSL;
    sumAltEllipsoid += altEllipsoid;
    sumHDOP += (hDOP / 100.0);
    sumPDOP += (pDOP / 100.0);
    sumHAcc += hAcc;
    sumVAcc += vAcc;
    count++;
    
    if (carrSoln == 2) fixedCount++;
    else if (carrSoln == 1) floatCount++;
    else standardCount++;
    
    if (hAcc < bestHAcc) bestHAcc = hAcc;
    if (vAcc < bestVAcc) bestVAcc = vAcc;
    if (hDOP < bestHDOP) bestHDOP = hDOP;
    
    if (firstTimestamp == "") firstTimestamp = timestamp;
    lastTimestamp = timestamp;
  }
  

  // Returns average or percentage values of the collected GNSS measurements.
  // If no measurements have been added, returns 0 to avoid division by zero.
  double getAvgLat() { return (count > 0) ? sumLat / count : 0.0; }
  double getAvgLon() { return (count > 0) ? sumLon / count : 0.0; }
  double getAvgAltMSL() { return (count > 0) ? sumAltMSL / count : 0.0; }
  double getAvgAltEllipsoid() { return (count > 0) ? sumAltEllipsoid / count : 0.0; }
  double getAvgHDOP() { return (count > 0) ? sumHDOP / count : 0.0; }
  double getAvgPDOP() { return (count > 0) ? sumPDOP / count : 0.0; }
  double getAvgHAcc() { return (count > 0) ? (double)sumHAcc / count : 0.0; }
  double getAvgVAcc() { return (count > 0) ? (double)sumVAcc / count : 0.0; }
  float getFixedPercentage() { return (count > 0) ? (fixedCount * 100.0 / count) : 0.0; }
  
  // Calculate geoid separation from the data
  double getGeoidSeparation() { 
    return (count > 0) ? (sumAltEllipsoid - sumAltMSL) / count : 0.0; 
  }
};

//====================GLOBAL VARIABLES=================
WiFiClient ntripClient;
bool transmitLocation = true;
bool sdCardAvailable = false;
bool oledAvailable = false;
bool trignetConnected = false;
bool setupMode = false;

unsigned long bothButtonsPressedStart = 0;
bool bothButtonsPressed = false;
const unsigned long SETUP_MODE_HOLD_TIME = 10000;

bool showGPSData = false;
bool lastButton1State = HIGH;
bool button1State = HIGH;
unsigned long lastDebounce1Time = 0;

bool isLogging = false;
bool lastButton2State = HIGH;
bool button2State = HIGH;
unsigned long lastDebounce2Time = 0;

String currentCsvFileName = "";
int lastLoggedDay = -1;
unsigned long lastLogTime = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long loggingStartTime = 0;
unsigned long byteCount = 0;
unsigned long logCount = 0;
unsigned long logFailCount = 0;
unsigned long lastSDCheck = 0;

int currentPointNumber = -1;
int dailyPointCounter = 0;

PointAccumulator currentPoint;

const unsigned long LOG_INTERVAL = 1000;
const unsigned long DISPLAY_UPDATE_INTERVAL = 500;
const unsigned long SD_CHECK_INTERVAL = 10000;

//position data structure
struct PositionData {
  double latitude  = 0.0;
  double longitude = 0.0;
  double altitude  = 0.0;              // MSL/Orthometric height
  double altitudeEllipsoid = 0.0;      // NEW: Ellipsoidal height
  uint32_t hAcc = 4294967295;
  uint32_t vAcc = 4294967295;
  uint16_t pDOP = 9999;
  uint16_t hDOP = 9999;
  int satellites   = 0;
  uint8_t fixType = 0;
  uint8_t carrSoln = 0;
  unsigned long lastUpdate = 0;
};
PositionData currentPos;
PositionData displayedPos;

//GPS time retrieval
struct GPSDateTime {
  uint16_t year = 0;
  uint8_t month = 0;
  uint8_t day = 0;
  uint8_t hour = 0;
  uint8_t minute = 0;
  uint8_t second = 0;
  bool valid = false;
};
GPSDateTime gpsTime;

//====================NVS STORAGE FUNCTIONS=================
// Loads stored Wi-Fi and NTRIP credentials from non-volatile storage (Preferences) and prints them to the serial monitor.
// Uses default values if no stored credentials are found.
void loadStoredCredentials() {
  preferences.begin("gps-config", true);
  
  stored_ssid = preferences.getString("wifi_ssid", "");
  stored_password = preferences.getString("wifi_pass", "");
  stored_casterHost = preferences.getString("ntrip_host", "196.15.132.2");
  stored_casterPort = preferences.getInt("ntrip_port", 2101);
  stored_mountPoint = preferences.getString("mount_point", "RTKNetGauteng");
  stored_casterUser = preferences.getString("ntrip_user", "");
  stored_casterUserPW = preferences.getString("ntrip_pass", "");
  
  preferences.end();
  
  Serial.println(F("\n--- Loaded Credentials ---"));
  Serial.print(F("WiFi SSID: ")); Serial.println(stored_ssid.isEmpty() ? "(not set)" : stored_ssid);
  Serial.print(F("NTRIP Host: ")); Serial.println(stored_casterHost);
  Serial.print(F("NTRIP Port: ")); Serial.println(stored_casterPort);
  Serial.print(F("Mount Point: ")); Serial.println(stored_mountPoint);
  Serial.print(F("NTRIP User: ")); Serial.println(stored_casterUser.isEmpty() ? "(not set)" : stored_casterUser);
  Serial.println(F("-------------------------\n"));
}

// Saves the provided Wi-Fi and NTRIP credentials to non-volatile storage (Preferences) for future use.
void saveCredentials(String ssid, String pass, String host, int port, String mount, String user, String userPW) {
  preferences.begin("gps-config", false);
  
  preferences.putString("wifi_ssid", ssid);
  preferences.putString("wifi_pass", pass);
  preferences.putString("ntrip_host", host);
  preferences.putInt("ntrip_port", port);
  preferences.putString("mount_point", mount);
  preferences.putString("ntrip_user", user);
  preferences.putString("ntrip_pass", userPW);
  
  preferences.end();
  
  Serial.println(F("✓ Credentials saved to NVS"));
}

//====================WEB SERVER HANDLERS=================
// HTML content for the web-based configuration page served by the rover.
// Allows users to enter and submit Wi-Fi and NTRIP settings, which are then saved to the device.
const char* configPageHTML = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>GPS Logger Setup</title>
  <style>
    body { font-family: Arial, sans-serif; max-width: 500px; margin: 50px auto; padding: 20px; background: #f0f0f0; }
    h1 { color: #333; }
    .form-group { margin-bottom: 15px; }
    label { display: block; margin-bottom: 5px; font-weight: bold; }
    input { width: 100%; padding: 8px; box-sizing: border-box; border: 1px solid #ccc; border-radius: 4px; }
    button { background: #4CAF50; color: white; padding: 10px 20px; border: none; border-radius: 4px; cursor: pointer; font-size: 16px; }
    button:hover { background: #45a049; }
    .section { background: white; padding: 20px; margin-bottom: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
    .success { color: green; font-weight: bold; }
    .note { font-size: 12px; color: #666; margin-top: 5px; }
  </style>
</head>
<body>
  <h1>🛰️ GPS Logger Setup</h1>
  
  <div class="section">
    <h2>WiFi Configuration</h2>
    <form action="/save" method="POST">
      <div class="form-group">
        <label>WiFi SSID:</label>
        <input type="text" name="wifi_ssid" placeholder="Your WiFi network name" required>
      </div>
      <div class="form-group">
        <label>WiFi Password:</label>
        <input type="password" name="wifi_pass" placeholder="Your WiFi password" required>
      </div>
      
      <h2>TrigNet NTRIP Configuration</h2>
      <div class="form-group">
        <label>NTRIP Server Host:</label>
        <input type="text" name="ntrip_host" value="196.15.132.2" required>
        <div class="note">Default: 196.15.132.2 (TrigNet South Africa)</div>
      </div>
      <div class="form-group">
        <label>NTRIP Server Port:</label>
        <input type="number" name="ntrip_port" value="2101" required>
      </div>
      <div class="form-group">
        <label>Mount Point:</label>
        <input type="text" name="mount_point" value="RTKNetGauteng" required>
        <div class="note">e.g., RTKNetGauteng, RTKNetWesternCape</div>
      </div>
      <div class="form-group">
        <label>NTRIP Username:</label>
        <input type="text" name="ntrip_user" placeholder="Your TrigNet username" required>
      </div>
      <div class="form-group">
        <label>NTRIP Password:</label>
        <input type="password" name="ntrip_pass" placeholder="Your TrigNet password" required>
      </div>
      
      <button type="submit">💾 Save & Restart</button>
    </form>
  </div>
  
  <div class="note" style="text-align: center;">
    After saving, the device will restart and connect to your WiFi network.
  </div>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send(200, "text/html", configPageHTML);
}

// Handles HTTP requests to the root URL ("/") by sending the configuration HTML page to the client.
void handleSave() {
  if (server.method() == HTTP_POST) {
    String ssid = server.arg("wifi_ssid");
    String pass = server.arg("wifi_pass");
    String host = server.arg("ntrip_host");
    int port = server.arg("ntrip_port").toInt();
    String mount = server.arg("mount_point");
    String user = server.arg("ntrip_user");
    String userPW = server.arg("ntrip_pass");
    
    saveCredentials(ssid, pass, host, port, mount, user, userPW);
    
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><style>";
    html += "body { font-family: Arial; text-align: center; margin-top: 50px; }";
    html += ".success { color: green; font-size: 24px; }";
    html += "</style></head><body>";
    html += "<div class='success'>✓ Configuration Saved!</div>";
    html += "<p>Device will restart in 3 seconds...</p>";
    html += "</body></html>";
    
    server.send(200, "text/html", html);
    
    delay(3000);
    ESP.restart();
  }
}

// Puts the rover into setup mode:
// - Enables access point with predefined SSID and password
// - Starts the web server to serve the configuration page
// - Updates the OLED display (if available) with setup instructions
// - Prints all relevant information to the Serial Monitor for debugging and user guidance
// - Continuously handles web client requests until setupMode is exited
void startSetupMode() {
  setupMode = true;
  
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║   ENTERING SETUP MODE              ║"));
  Serial.println(F("╚════════════════════════════════════╝"));
  
  if (oledAvailable) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(F("SETUP MODE"));
    display.println();
    display.println(F("Connect to WiFi:"));
    display.println(setup_ap_ssid);
    display.println();
    display.print(F("Pass: "));
    display.println(setup_ap_password);
    display.display();
  }
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(setup_ap_ssid, setup_ap_password);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print(F("AP IP address: "));
  Serial.println(IP);
  Serial.println(F("Connect to WiFi: ")); Serial.println(setup_ap_ssid);
  Serial.println(F("Password: ")); Serial.println(setup_ap_password);
  Serial.println(F("Then open: http://192.168.4.1"));
  
  server.on("/", handleRoot);
  server.on("/save", handleSave);
  server.begin();
  
  Serial.println(F("Web server started - waiting for configuration..."));
  
  while (setupMode) {
    server.handleClient();
    delay(10);
  }
}

//====================DATE FORMATTING=================
// Converts a time value in milliseconds to a formatted string "HH:MM:SS".
// Useful for displaying elapsed time or uptime in a human-readable format.
String millisToTime(unsigned long ms) {
  unsigned long totalSeconds = ms / 1000;
  int hours = (totalSeconds / 3600) % 24;
  int minutes = (totalSeconds / 60) % 60;
  int seconds = totalSeconds % 60;
  
  char timeStr[9];
  sprintf(timeStr, "%02d:%02d:%02d", hours, minutes, seconds);
  return String(timeStr);
}

// Returns the current GPS date and time as a formatted string "YYYY-MM-DD HH:MM:SS".
// If GPS time is invalid, returns "NO_GPS_TIME".
String getGPSTimeString() {
  if (!gpsTime.valid) {
    return "NO_GPS_TIME";
  } 
  
  char timeStr[20];
  sprintf(timeStr, "%04d-%02d-%02d %02d:%02d:%02d", 
          gpsTime.year, gpsTime.month, gpsTime.day,
          gpsTime.hour, gpsTime.minute, gpsTime.second);
  return String(timeStr);
}
// Generates a daily CSV filename for logging GPS data.
// Uses the GPS date if valid; otherwise, estimates the day based on system uptime (millis).
String getDailyFileName() {
  if (!gpsTime.valid) {
    unsigned long days = millis() / (1000UL * 60 * 60 * 24);
    char filename[30];
    sprintf(filename, "/gps_log_day%lu.csv", days);
    return String(filename);
  }
  
  char filename[30];
  sprintf(filename, "/gps_%04d%02d%02d.csv", 
          gpsTime.year, gpsTime.month, gpsTime.day);
  return String(filename);
}

// Generates a daily CSV filename for storing averaged GPS measurements.
// Uses the GPS date if valid; otherwise, estimates the day based on system uptime (millis).
String getAveragedFileName() {
  if (!gpsTime.valid) {
    unsigned long days = millis() / (1000UL * 60 * 60 * 24);
    char filename[40];
    sprintf(filename, "/gps_averaged_day%lu.csv", days);
    return String(filename);
  }
  
  char filename[40];
  sprintf(filename, "/gps_averaged_%04d%02d%02d.csv", 
          gpsTime.year, gpsTime.month, gpsTime.day);
  return String(filename);
}

//=================SD health====================
// Checks if the SD card is initialized and accessible.
// Attempts re-initialization if the first check fails and prints status to the Serial Monitor for debugging.
bool checkSDCard() {
  if (!SD.begin(SD_CS)) {
    Serial.println(F("SD Card check failed - attempting re-initialising"));
    sdCardAvailable = false;
    delay(100);
    
    if (SD.begin(SD_CS)) {
      Serial.println(F("✓ SD Card re-initialized successfully"));
      sdCardAvailable = true;
      return true;
    } else {
      Serial.println(F("✗ SD Card re-init failed"));
      return false;
    }
  }
  
  sdCardAvailable = true;
  return true;
}

//==================OLED INITIALISATION===================
// Initializes the OLED display over I2C, clears the screen, and shows a startup message.
// Prints status messages to the Serial Monitor for debugging and confirmation.
bool initOLED() {
  Serial.println(F("\n--- Initializing OLED Display ---"));
  
  Wire.begin(I2C_SDA, I2C_SCL);
  if(!display.begin(OLED_I2C_ADDRESS)) {
    Serial.println(F("✗ SH1106 OLED allocation failed"));
    return false;
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println(F("GPS Logger"));
  display.println(F("Initializing..."));
  display.display();
  
  Serial.println(F("✓ OLED Display Ready"));
  Serial.println(F("--- OLED Initialized ---\n"));
  return true;
}

//===================BUTTON ONE - Display Screen==================
// Monitors Button 1 with debounce logic to toggle between displaying GPS data and the status screen.
// Prints messages to the Serial Monitor for debugging and user feedback.
void checkButton1() {
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButton1State) {
    lastDebounce1Time = millis();
  }
  if ((millis() - lastDebounce1Time) > DEBOUNCE_DELAY) {
    if (reading != button1State) {
      button1State = reading;
      if (button1State == LOW) {
        showGPSData = !showGPSData;
        if (showGPSData) {
          displayedPos = currentPos;
          Serial.println(F("GPS data captured and displayed"));
        } else {
          Serial.println(F("Returned to status screen"));
        }
      }
    }
  }
  
  lastButton1State = reading;
}

//===================CHECK FOR SETUP MODE (BOTH BUTTONS)==================
// Checks if both buttons are pressed simultaneously to trigger setup mode.
// Requires holding both buttons for 10 seconds.
// Provides countdown feedback on the OLED (if available) and prints status/debug messages to the Serial Monitor.
void checkSetupModeActivation() {
  bool btn1Pressed = (digitalRead(BUTTON_PIN) == LOW);
  bool btn2Pressed = (digitalRead(BUTTON_LOG_PIN) == LOW);
  
  if (btn1Pressed && btn2Pressed) {
    if (!bothButtonsPressed) {
      bothButtonsPressedStart = millis();
      bothButtonsPressed = true;
      Serial.println(F("Hold both buttons for 10 seconds to enter Setup Mode..."));
      
      if (oledAvailable) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println(F("Setup Mode?"));
        display.println();
        display.println(F("Keep holding..."));
        display.display();
      }
    } else {
      unsigned long holdTime = millis() - bothButtonsPressedStart;
      
      if (oledAvailable && holdTime % 1000 < 100) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println(F("Setup Mode in:"));
        display.println();
        display.setTextSize(2);
        display.print((SETUP_MODE_HOLD_TIME - holdTime) / 1000 + 1);
        display.println(F(" sec"));
        display.display();
      }
      
      if (holdTime >= SETUP_MODE_HOLD_TIME) {
        Serial.println(F("\n🔧 10 SECONDS REACHED - ENTERING SETUP MODE!"));
        bothButtonsPressed = false;
        
        lastButton1State = LOW;
        lastButton2State = LOW;
        button1State = LOW;
        button2State = LOW;
        
        startSetupMode();
      }
    }
  } else {
    if (bothButtonsPressed) {
      unsigned long holdTime = millis() - bothButtonsPressedStart;
      if (holdTime < SETUP_MODE_HOLD_TIME) {
        Serial.print(F("Released too early ("));
        Serial.print(holdTime / 1000.0, 1);
        Serial.println(F(" sec). Need 10 seconds for Setup Mode."));
        
        if (oledAvailable) {
          display.clearDisplay();
          display.setTextSize(1);
          display.setCursor(0, 0);
          display.println(F("Cancelled"));
          display.println();
          display.println(F("Hold 10 sec"));
          display.println(F("for Setup Mode"));
          display.display();
          delay(2000);
          updateDisplay();
        }
      }
      bothButtonsPressed = false;
    }
  }
}

//===================BUTTON TWO - Logging Control==================
// Monitors Button 2 with debounce logic to start or stop SD card logging.
// When pressed, toggles logging state, initializes or finalizes data collection, and prints detailed status/debug messages to the Serial Monitor.
void checkButton2() {
  int reading = digitalRead(BUTTON_LOG_PIN);
  if (reading != lastButton2State) {
    lastDebounce2Time = millis();
  }
  if ((millis() - lastDebounce2Time) > DEBOUNCE_DELAY) {
    if (reading != button2State) {
      button2State = reading;
      
      if (button2State == LOW) {
        isLogging = !isLogging;
        
        if (isLogging) {
          if (!checkSDCard()) {
            Serial.println(F("Cannot start logging - SD card unavailable"));
            isLogging = false;
            return;
          }
          
          currentPointNumber++;
          dailyPointCounter = currentPointNumber;
          
          currentPoint.reset();
          currentPoint.pointNumber = currentPointNumber;
          
          loggingStartTime = millis();
          lastLogTime = millis();
          logCount = 0;
          logFailCount = 0;
          
          Serial.print(F("SD Card logging STARTED - Point #"));
          Serial.println(currentPointNumber);
          Serial.println(F("Collecting samples for averaging..."));
          
        } else {
          Serial.print(F("SD Card logging STOPPED - Point #"));
          Serial.println(currentPointNumber);
          Serial.print(F("Total raw logs: "));
          Serial.print(logCount);
          Serial.print(F(" | Failed: "));
          Serial.println(logFailCount);
          
          if (currentPoint.count > 0) {
            Serial.print(F("Calculating average from "));
            Serial.print(currentPoint.count);
            Serial.println(F(" samples..."));
            
            if (saveAveragedPoint(currentPoint)) {
              Serial.println(F("✓ Averaged point saved successfully!"));
            } else {
              Serial.println(F("✗ Failed to save averaged point"));
            }
          } else {
            Serial.println(F("No samples collected for this point"));
          }
        }
      }
    }
  }
  
  lastButton2State = reading;
}

//=================OLED DISPLAY - MODIFIED for dual heights====================
// Updates the OLED display with either detailed GPS data or a general status screen.
// Shows connection status (WiFi, NTRIP, SD card), logging info, satellite and height data, and vertical accuracy.
// Supports dual-height display (MSL and ellipsoidal) and prints relevant measurements in human-readable format.
void updateDisplay() {
  if (!oledAvailable) return;
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  
  if (showGPSData) {    
    //line 1: Connection Status + Logging indicator
    display.print(F("W:"));
    if (WiFi.status() == WL_CONNECTED) {
      display.print(F("OK"));
    } else {
      display.print(F("X"));
    }
    display.print(F(" N:"));
    if (trignetConnected) {
      display.print(F("OK"));
    } else {
      display.print(F("X"));
    }
    display.print(F(" SD:"));
    if (sdCardAvailable) {
      display.print(F("OK"));
    } else {
      display.print(F("X"));
    }
    display.println();
    
    display.print(F("Bytes:"));
    display.print(byteCount);
    if (isLogging) {
      display.print(F(" [P"));
      display.print(currentPointNumber);
      display.print(F("]"));
    }
    display.println();
    
    display.println(F("--------------------"));
        
    display.print(F("HDOP:"));
    if (displayedPos.hDOP < 9999) {
      display.print(displayedPos.hDOP / 100.0, 2);
    } else {
      display.print(F("--"));
    }
    display.print(F(" PDOP:"));
    if (displayedPos.pDOP < 9999) {
      display.println(displayedPos.pDOP / 100.0, 2);
    } else {
      display.println(F("--"));
    }
    
    // MSL Height
    display.print(F("MSL:"));
    if (displayedPos.altitude != -17.0) {
      display.print(displayedPos.altitude, 1);
      display.print(F("m"));
    } else {
      display.print(F("--"));
    }
    
    // Satellites
    display.print(F(" Sat:"));
    display.println(displayedPos.satellites);
    
    // Ellipsoidal Height
    display.print(F("ELL:"));
    if (displayedPos.altitudeEllipsoid != -17.0) {
      display.print(displayedPos.altitudeEllipsoid, 1);
      display.println(F("m"));
    } else {
      display.println(F("--"));
    }
        
    //vertical Accuracy (in cm)
    display.print(F("vAcc:"));
    if (displayedPos.vAcc < 100000) {
      display.print(displayedPos.vAcc / 10.0, 1);
      display.print(F("cm"));
    } else {
      display.print(F("--"));
    }
    
    //log count and samples
    if (isLogging) {
      display.print(F(" L:"));
      display.print(logCount);
      display.print(F("/"));
      display.print(currentPoint.count);
    }
    display.println();
    
  } else {
    //show status screen 
    display.setTextSize(1);
    display.println(F("GPS Logger"));
    display.println(F(""));
    
    //WiFi Status
    display.print(F("WiFi: "));
    if (WiFi.status() == WL_CONNECTED) {
      display.println(F("Connected"));
    } else {
      display.println(F("Disconnected"));
    }
    
    //NTRIP Status
    display.print(F("TrigNet: "));
    if (trignetConnected) {
      display.println(F("Connected"));
    } else {
      display.println(F("Disconnected"));
    }
    
    //SD Card Status
    display.print(F("SD Card: "));
    if (sdCardAvailable) {
      display.println(F("Ready"));
    } else {
      display.println(F("ERROR"));
    }
    
    //RTCM Bytes
    display.print(F("RTCM: "));
    display.println(byteCount);
    
    //logging Status
    display.println(F(""));
    display.print(F("Log: "));
    if (isLogging) {
      display.print(F("ON P"));
      display.println(currentPointNumber);
      display.print(F("Samples: "));
      display.println(currentPoint.count);
    } else {
      display.print(F("OFF"));
      if (currentPointNumber >= 0) {
        display.print(F(" (Last:P"));
        display.print(currentPointNumber);
        display.print(F(")"));
      }
      display.println();
    }
  }
  
  display.display();
}

//=================GPGGA MESSAGES====================
// Sends the current GPGGA NMEA message to the NTRIP caster if connected and location transmission is enabled.
void pushGPGGA(NMEA_GGA_data_t *nmeaData) {
  if (ntripClient.connected() && transmitLocation) {
    ntripClient.print((const char *)nmeaData->nmea);
  }
}

//===================OBTAINING POSITION DATA - MODIFIED==================
// Processes incoming UBX PVT data to update the rover’s current position, GPS time, fix status, and satellite information.
// Updates both MSL and ellipsoidal heights, calculates geoid separation, and prints detailed position info to the Serial Monitor for debugging.
// Handles daily point counter reset when a new day is detected and logs data to the SD card at defined intervals if logging is active.

void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct) {
  double latitude  = ubxDataStruct->lat / 10000000.0;
  double longitude = ubxDataStruct->lon / 10000000.0;
  double altitudeMSL = ubxDataStruct->hMSL / 1000.0;           // Orthometric/MSL
  double altitudeEllipsoid = ubxDataStruct->height / 1000.0;   // Ellipsoidal
  uint32_t hAcc = ubxDataStruct->hAcc;
  uint32_t vAcc = ubxDataStruct->vAcc;
  uint16_t pDOP = ubxDataStruct->pDOP;
  uint8_t fixType = ubxDataStruct->fixType;
  uint8_t carrSoln = ubxDataStruct->flags.bits.carrSoln;
  uint8_t numSats = ubxDataStruct->numSV;

  gpsTime.year = ubxDataStruct->year;
  gpsTime.month = ubxDataStruct->month;
  gpsTime.day = ubxDataStruct->day;
  gpsTime.hour = ubxDataStruct->hour;
  gpsTime.minute = ubxDataStruct->min;
  gpsTime.second = ubxDataStruct->sec;
  gpsTime.valid = ubxDataStruct->flags.bits.gnssFixOK;

  if (gpsTime.valid && gpsTime.day != lastLoggedDay) {
    if (lastLoggedDay != -1) {
      Serial.println(F("New day detected - resetting point counter"));
      currentPointNumber = -1;
      dailyPointCounter = 0;
    }
    lastLoggedDay = gpsTime.day;
  }

  // Update global position with BOTH heights
  currentPos.latitude = latitude;
  currentPos.longitude = longitude;
  currentPos.altitude = altitudeMSL;
  currentPos.altitudeEllipsoid = altitudeEllipsoid;
  currentPos.hAcc = hAcc;
  currentPos.vAcc = vAcc;
  currentPos.pDOP = pDOP;
  currentPos.satellites = numSats;
  currentPos.fixType = fixType;
  currentPos.carrSoln = carrSoln;
  currentPos.lastUpdate = millis();

  // Calculate geoid separation
  double geoidSep = altitudeEllipsoid - altitudeMSL;

  Serial.print(F("Lat: "));
  Serial.print(latitude, 7);
  Serial.print(F("  Long: "));
  Serial.print(longitude, 7);
  Serial.print(F("  MSL: "));
  Serial.print(altitudeMSL, 3);
  Serial.print(F("m  Ellipsoid: "));
  Serial.print(altitudeEllipsoid, 3);
  Serial.print(F("m  N: "));
  Serial.print(geoidSep, 3);
  Serial.print(F("m"));
  Serial.print(F("  PDOP: "));
  Serial.print(pDOP / 100.0, 2);
  Serial.print(F("  Fix: "));
  Serial.print(fixType);
  if (fixType == 0) Serial.print(F(" (None)"));
  else if (fixType == 1) Serial.print(F(" (Dead Reckoning)"));
  else if (fixType == 2) Serial.print(F(" (2D)"));
  else if (fixType == 3) Serial.print(F(" (3D)"));
  else if (fixType == 4) Serial.print(F(" (GNSS + Dead Reckoning)"));
  else if (fixType == 5) Serial.print(F(" (Time Only)"));
  else Serial.print(F(" (UNKNOWN)"));
  Serial.print(F("  Carrier: "));
  Serial.print(carrSoln);
  if (carrSoln == 0) Serial.print(F(" (None)"));
  else if (carrSoln == 1) Serial.print(F(" (Float)"));
  else if (carrSoln == 2) Serial.print(F(" (Fixed)"));
  else Serial.print(F(" (UNKNOWN)"));
  Serial.print(F("  hAcc: "));
  Serial.print(hAcc / 10.0, 1);
  Serial.print(F("cm  vAcc: "));
  Serial.print(vAcc / 10.0, 1);
  Serial.print(F("cm  Sats: "));
  Serial.print(numSats);

  if (isLogging) {
    Serial.print(F("  [LOG P"));
    Serial.print(currentPointNumber);
    Serial.print(F(": "));
    Serial.print(currentPoint.count);
    Serial.print(F("]"));
  }
  Serial.println();

  if (isLogging && (millis() - lastLogTime >= LOG_INTERVAL)) {
    logToSD(latitude, longitude, altitudeMSL, altitudeEllipsoid, hAcc, vAcc, pDOP, fixType, carrSoln, numSats);
    lastLogTime = millis();
  }
}

//====================GET DOP DATA===================
// Retrieves the latest Dilution of Precision (DOP) values from the GNSS module and updates the current position’s HDOP and PDOP metrics.
void updateDOPData() {
  if (myGNSS.getDOP()) {
    currentPos.pDOP = myGNSS.packetUBXNAVDOP->data.pDOP;
    currentPos.hDOP = myGNSS.packetUBXNAVDOP->data.hDOP;
  }
}

//====================Initialise SD Card=================
// Initializes the SD card with up to 3 attempts, detects its type and size, and prints status messages to the Serial Monitor.
// Confirms readiness for logging both MSL and ellipsoidal heights and instructs user to start logging via Button 2.
bool initSDCard() {
  Serial.println(F("\n--- Initializing SD Card ---"));
  
  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.print(F("Attempt "));
    Serial.print(attempt);
    Serial.println(F("/3..."));
    if (SD.begin(SD_CS)) {
      uint8_t cardType = SD.cardType();
      if (cardType == CARD_NONE) {
        Serial.println(F("✗ No SD card attached"));
        delay(500);
        continue;
      }
      Serial.print(F("✓ SD Card Type: "));
      if (cardType == CARD_MMC) Serial.println(F("MMC"));
      else if (cardType == CARD_SD) Serial.println(F("SDSC"));
      else if (cardType == CARD_SDHC) Serial.println(F("SDHC"));
      else Serial.println(F("UNKNOWN"));

      uint64_t cardSize = SD.cardSize() / (1024 * 1024);
      Serial.printf("✓ SD Card Size: %llu MB\n", cardSize);

      Serial.println(F("--- SD Card Ready ---\n"));
      Serial.println(F("Logging both Ellipsoidal and MSL heights"));
      Serial.println(F("Press Button 2 to start logging"));
      return true;
    }
    
    delay(500);
  }
  
  Serial.println(F("✗ SD Card initialization failed after 3 attempts"));
  return false;
}

//====================SD CARD FILE CREATION - MODIFIED HEADER=================
// Ensures that the daily CSV file exists on the SD card.
// Creates a new file with a header including both MSL and ellipsoidal heights if it doesn’t exist, and prints status/debug messages to the Serial Monitor.
bool ensureDailyFile() {
  String newFileName = getDailyFileName();
  
  if (newFileName != currentCsvFileName) {
    Serial.print(F("Switching to new file: "));
    Serial.println(newFileName);
    currentCsvFileName = newFileName;
  }
  
  if (!SD.exists(currentCsvFileName.c_str())) {
    Serial.print(F("Creating new file: "));
    Serial.println(currentCsvFileName);
    
    File file = SD.open(currentCsvFileName.c_str(), FILE_WRITE);
    if (!file) {
      Serial.println(F("✗ Failed to create file"));
      return false;
    }
    
    // Header with BOTH height columns
    size_t written = file.println("PointNumber,DateTime,Latitude,Longitude,AltitudeMSL_m,AltitudeEllipsoid_m,GeoidSep_m,HDOP,PDOP,hAcc_mm,vAcc_mm,FixType,CarrierSolution,Satellites");
    file.close();
    
    if (written == 0) {
      Serial.println(F("✗ Failed to write header"));
      return false;
    }
    
    Serial.println(F("✓ Raw data file created with dual height header"));
    
    delay(100);
    if (!SD.exists(currentCsvFileName.c_str())) {
      Serial.println(F("✗ File creation verification failed"));
      return false;
    }
  }
  
  return true;
}

//====================CREATE AVERAGED FILE - MODIFIED HEADER=================
// Ensures that the daily averaged CSV file exists on the SD card.
// Creates a new file with a header including both MSL and ellipsoidal heights, sample count, RTK fixed percentage, and duration if it doesn’t exist, and prints status/debug messages to the Serial Monitor.
bool ensureAveragedFile() {
  String avgFileName = getAveragedFileName();
  
  if (!SD.exists(avgFileName.c_str())) {
    Serial.print(F("Creating averaged file: "));
    Serial.println(avgFileName);
    
    File file = SD.open(avgFileName.c_str(), FILE_WRITE);
    if (!file) {
      Serial.println(F("✗ Failed to create averaged file"));
      return false;
    }
    
    // Header with BOTH height columns
    file.println("PointNumber,DateTime,Latitude,Longitude,AltitudeMSL_m,AltitudeEllipsoid_m,GeoidSep_m,HDOP,PDOP,hAcc_mm,vAcc_mm,FixType,CarrierSolution,Satellites,SampleCount,RTKFixed_%,Duration_sec");
    file.close();
    
    Serial.println(F("✓ Averaged file created with dual height header"));
    return true;
  }
  
  return true;
}

//====================SAVE AVERAGED POINT - MODIFIED=================
// Saves the averaged GPS measurements from a PointAccumulator to the daily averaged CSV file.
// Calculates averages for latitude, longitude, MSL and ellipsoidal heights, geoid separation, DOP, accuracy, and RTK fix percentage.
// Writes a CSV line with all values, flushes to SD card, and prints detailed summary to the Serial Monitor for debugging and verification.
bool saveAveragedPoint(PointAccumulator &point) {
  if (!sdCardAvailable) {
    Serial.println(F("SD card not available for saving average"));
    return false;
  }
  
  if (point.count == 0) {
    Serial.println(F("No data to average"));
    return false;
  }
  
  if (!ensureAveragedFile()) {
    Serial.println(F("✗ Failed to ensure averaged file exists"));
    return false;
  }
  
  String avgFileName = getAveragedFileName();
  File file = SD.open(avgFileName.c_str(), FILE_APPEND);
  
  if (!file) {
    Serial.println(F("✗ Failed to open averaged file"));
    return false;
  }
  
  int durationSec = point.count * 1;
  double avgGeoidSep = point.getGeoidSeparation();
  
  uint8_t avgFixType = 3;
  uint8_t avgCarrSoln = 0;
  
  if (point.fixedCount > point.floatCount && point.fixedCount > point.standardCount) {
    avgCarrSoln = 2;
  } else if (point.floatCount > point.standardCount) {
    avgCarrSoln = 1;
  }
  
  int avgSatellites = 15;
  
  // CSV line with BOTH heights
  String csvLine = String(point.pointNumber) + "," +
                   point.firstTimestamp + "," +
                   String(point.getAvgLat(), 8) + "," +
                   String(point.getAvgLon(), 8) + "," +
                   String(point.getAvgAltMSL(), 3) + "," +
                   String(point.getAvgAltEllipsoid(), 3) + "," +
                   String(avgGeoidSep, 3) + "," +
                   String(point.getAvgHDOP(), 2) + "," +
                   String(point.getAvgPDOP(), 2) + "," +
                   String((int)point.getAvgHAcc()) + "," +
                   String((int)point.getAvgVAcc()) + "," +
                   String(avgFixType) + "," +
                   String(avgCarrSoln) + "," +
                   String(avgSatellites) + "," +
                   String(point.count) + "," +
                   String(point.getFixedPercentage(), 1) + "," +
                   String(durationSec);
  
  size_t written = file.println(csvLine);
  file.flush();
  file.close();
  
  if (written == 0) {
    Serial.println(F("✗ Failed to write averaged point"));
    return false;
  }
  
  Serial.println(F("\n========== AVERAGED POINT SAVED =========="));
  Serial.print(F("Point #: "));
  Serial.println(point.pointNumber);
  Serial.print(F("Samples: "));
  Serial.print(point.count);
  Serial.print(F(" over "));
  Serial.print(durationSec);
  Serial.println(F(" seconds"));
  Serial.print(F("Avg Position: "));
  Serial.print(point.getAvgLat(), 8);
  Serial.print(F(", "));
  Serial.println(point.getAvgLon(), 8);
  Serial.print(F("Avg MSL Height: "));
  Serial.print(point.getAvgAltMSL(), 3);
  Serial.println(F(" m"));
  Serial.print(F("Avg Ellipsoid Height: "));
  Serial.print(point.getAvgAltEllipsoid(), 3);
  Serial.println(F(" m"));
  Serial.print(F("Avg Geoid Separation: "));
  Serial.print(avgGeoidSep, 3);
  Serial.println(F(" m"));
  Serial.print(F("Avg HDOP: "));
  Serial.print(point.getAvgHDOP(), 2);
  Serial.print(F(" | Avg PDOP: "));
  Serial.println(point.getAvgPDOP(), 2);
  Serial.print(F("Avg hAcc: "));
  Serial.print(point.getAvgHAcc(), 1);
  Serial.print(F(" mm | Avg vAcc: "));
  Serial.print(point.getAvgVAcc(), 1);
  Serial.println(F(" mm"));
  Serial.print(F("RTK Fixed: "));
  Serial.print(point.getFixedPercentage(), 1);
  Serial.print(F("% ("));
  Serial.print(point.fixedCount);
  Serial.print(F("/"));
  Serial.print(point.count);
  Serial.println(F(")"));
  Serial.print(F("Saved to: "));
  Serial.println(avgFileName);
  Serial.println(F("==========================================\n"));
  
  return true;
}

//====================LOGS GPS DATA TO SD CARD - MODIFIED=================
// Logs individual GPS measurements to the daily CSV file on the SD card.
// Handles both MSL and ellipsoidal heights, geoid separation, DOP, fix type, carrier solution, and satellite count.
// Adds measurements to the PointAccumulator for averaging, ensures the daily file exists, retries file opening up to 3 times, and prints detailed status/debug messages to the Serial Monitor.
// Tracks successful logs and failed attempts for monitoring.
void logToSD(double lat, double lon, double altMSL, double altEllipsoid, 
             uint32_t hAcc, uint32_t vAcc, uint16_t pDOP,
             uint8_t fixType, uint8_t carrSoln, uint8_t numSats) {
  
  if (!sdCardAvailable) {
    Serial.println(F("SD card not available - checking..."));
    if (!checkSDCard()) {
      logFailCount++;
      return;
    }
  }
  
  String timeStamp;
  if (gpsTime.valid) {
    timeStamp = getGPSTimeString();
  } else {
    unsigned long ms = millis();
    char fallbackTime[20];
    sprintf(fallbackTime, "MILLIS_%lu", ms);
    timeStamp = String(fallbackTime);
    Serial.println(F("Using millis timestamp - GPS time not valid"));
  }
  
  // Add to point accumulator with BOTH heights
  if (currentPoint.pointNumber == -1) {
    currentPoint.pointNumber = currentPointNumber;
  }
  currentPoint.addMeasurement(lat, lon, altMSL, altEllipsoid, 
                              currentPos.hDOP, pDOP, 
                              hAcc, vAcc, carrSoln, timeStamp);
  
  if (!ensureDailyFile()) {
    Serial.println(F("✗ Failed to ensure daily file exists"));
    logFailCount++;
    sdCardAvailable = false;
    return;
  }
  
  File file;
  for (int attempt = 1; attempt <= 3; attempt++) {
    file = SD.open(currentCsvFileName.c_str(), FILE_APPEND);
    if (file) break;
    Serial.print(F("File open attempt "));
    Serial.print(attempt);
    Serial.println(F(" failed"));
    delay(50);
  }
  
  if (!file) {
    Serial.println(F("✗ Failed to open CSV file after 3 attempts"));
    logFailCount++;
    sdCardAvailable = false;
    return;
  }

  uint16_t hDOP = currentPos.hDOP;
  double geoidSep = altEllipsoid - altMSL;

  // CSV line with BOTH heights
  String csvLine = String(currentPointNumber) + "," +
                   timeStamp + "," + 
                   String(lat, 7) + "," + 
                   String(lon, 7) + "," + 
                   String(altMSL, 3) + "," + 
                   String(altEllipsoid, 3) + "," +
                   String(geoidSep, 3) + "," +
                   String(hDOP / 100.0, 2) + "," +
                   String(pDOP / 100.0, 2) + "," +
                   String(hAcc) + "," + 
                   String(vAcc) + "," + 
                   String(fixType) + "," + 
                   String(carrSoln) + "," + 
                   String(numSats);
  
  size_t bytesWritten = file.println(csvLine);
  file.flush();
  file.close();
  
  if (bytesWritten == 0) {
    Serial.println(F("✗ Write failed - 0 bytes written"));
    logFailCount++;
    return;
  }
  
  logCount++;
  
  if (logCount % 10 == 0) {
    Serial.print(F("✓ Logged #"));
    Serial.print(logCount);
    Serial.print(F(" [P"));
    Serial.print(currentPointNumber);
    Serial.print(F("] Samples: "));
    Serial.print(currentPoint.count);
    if (logFailCount > 0) {
      Serial.print(F(" | Fails: "));
      Serial.print(logFailCount);
    }
    Serial.println();
  }
}

//==================SET UP===================
// Initializes the GPS logger system, including Serial communication, pins, OLED display, SD card, WiFi, and ZED-F9P GNSS module.
// Loads stored credentials, warns if missing, and offers Setup Mode if needed.
// Configures GNSS for UBX/NMEA/RTCM3, sets callbacks for GPGGA and PVT data, and starts WiFi connection to NTRIP server.
// Updates OLED and prints detailed startup and status information to the Serial Monitor for debugging and verification.
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("================================="));
  Serial.println(F("TrigNet NTRIP Client"));
  Serial.println(F("DUAL HEIGHT LOGGING"));
  Serial.println(F("Ellipsoidal + MSL/Orthometric"));
  Serial.println(F("================================="));

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUTTON_LOG_PIN, INPUT_PULLUP);
  
  oledAvailable = initOLED();
  if (!oledAvailable) {
    Serial.println(F("Continuing without OLED display"));
  }
  
  loadStoredCredentials();
  
  if (stored_ssid.isEmpty() || stored_casterUser.isEmpty()) {
    Serial.println(F("\n WARNING: No credentials configured!"));
    Serial.println(F("Hold BOTH buttons for 10 sec to enter Setup Mode"));
    
    if (oledAvailable) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println(F("NO CONFIG!"));
      display.println();
      display.println(F("Hold both buttons"));
      display.println(F("for 10 sec"));
      display.println(F("for Setup Mode"));
      display.display();
      delay(5000);
    }
  }

  sdCardAvailable = initSDCard();
  if (!sdCardAvailable) {
    Serial.println(F("Continuing without SD card logging"));
  }

  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  if (oledAvailable) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("Connecting to"));
    display.println(F("ZED-F9P GPS..."));
    display.display();
  }
  while (!myGNSS.begin(Serial2)) {
    Serial.println(F("u-blox ZED-F9P not detected. Check wiring."));
    delay(2000);
  }
  Serial.println(F("✓ ZED-F9P connected!"));

  myGNSS.setUART1Output(COM_TYPE_UBX | COM_TYPE_NMEA);
  myGNSS.setPortInput(COM_PORT_UART1, COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3);
  myGNSS.setDGNSSConfiguration(SFE_UBLOX_DGNSS_MODE_FIXED);
  myGNSS.setNavigationFrequency(1);
  myGNSS.setMainTalkerID(SFE_UBLOX_MAIN_TALKER_ID_GP);
  myGNSS.setNMEAGPGGAcallbackPtr(&pushGPGGA);
  myGNSS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1, 10);
  myGNSS.setAutoPVTcallbackPtr(&printPVTdata);
  myGNSS.setAutoPVT(true);

  Serial.println(F("✓ ZED-F9P configured"));

  if (!stored_ssid.isEmpty()) {
    Serial.println(F("\nConnecting to WiFi..."));
    
    if (oledAvailable) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println(F("Connecting to"));
      display.println(F("WiFi..."));
      display.display();
    }

    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    WiFi.begin(stored_ssid.c_str(), stored_password.c_str());

    int wifiAttempts = 0;
    while (WiFi.status() != WL_CONNECTED && wifiAttempts < 20) {
      delay(500);
      Serial.print(F("."));
      wifiAttempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println(F("\n✓ Wi-Fi connected!"));
      Serial.print(F("IP: "));
      Serial.println(WiFi.localIP());

      if (oledAvailable) {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(F("WiFi: Connected"));
        display.print(F("IP: "));
        display.println(WiFi.localIP());
        display.display();
        delay(2000);
      }
      
      connectToTrigNet();
    } else {
      Serial.println(F("\n✗ Wi-Fi connection failed!"));
      if (oledAvailable) {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(F("WiFi: FAILED"));
        display.display();
        delay(2000);
      }
    }
  }
  
  if (oledAvailable) {
    updateDisplay();
  }
  
  Serial.println(F("\n================================="));
  Serial.println(F("System Ready!"));
  Serial.println(F("Logging BOTH heights:"));
  Serial.println(F("- Ellipsoidal (RTK native)"));
  Serial.println(F("- MSL/Orthometric (converted)"));
  Serial.println(F("- Geoid Separation (N)"));
  Serial.println(F("=================================\n"));
}

//==================LOOP FUNCTIONS===================
// Main program loop handling all ongoing tasks:
// 1. Checks GNSS module for new UBX/NMEA/RTCM data and triggers callbacks.
// 2. Monitors both buttons to detect Setup Mode activation or logging toggles.
// 3. Updates DOP values every second.
// 4. Periodically checks SD card health during logging.
// 5. Updates OLED display at regular intervals.
// 6. Manages Wi-Fi connection, attempting reconnection if disconnected.
// 7. Monitors NTRIP (TrigNet) client connection, reconnecting if needed.
// 8. Processes NTRIP data when connected.
// 9. Contains a small delay to avoid blocking and allow timely handling of all tasks.
void loop() {
  myGNSS.checkUblox();
  myGNSS.checkCallbacks();
  
  checkSetupModeActivation();
  
  static unsigned long lastDOPUpdate = 0;
  if (millis() - lastDOPUpdate > 1000) {
    updateDOPData();
    lastDOPUpdate = millis();
  }

  checkButton1();
  checkButton2();

  if (isLogging && (millis() - lastSDCheck >= SD_CHECK_INTERVAL)) {
    if (!checkSDCard()) {
      Serial.println(F("SD card health check failed during logging"));
    }
    lastSDCheck = millis();
  }

  if (oledAvailable && (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL)) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }

  if (WiFi.status() != WL_CONNECTED && !stored_ssid.isEmpty()) {
    Serial.println(F("Wi-Fi disconnected! Reconnecting..."));
    WiFi.disconnect();
    WiFi.begin(stored_ssid.c_str(), stored_password.c_str());
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(F("."));
      attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println(F("\n✓ Wi-Fi reconnected!"));
    } else {
      Serial.println(F("\n✗ Wi-Fi reconnection failed"));
    }
  }

  if (!ntripClient.connected()) {
    if (trignetConnected) {
      Serial.println(F("TrigNet disconnected! Reconnecting..."));
      trignetConnected = false;
    }
    if (!stored_ssid.isEmpty() && WiFi.status() == WL_CONNECTED) {
      connectToTrigNet();
    }
  }

  if (ntripClient.connected()) {
    processConnection();
  }

  delay(10);
}

//====================CONNECTION TO TRIGNET=================
// Connects to TrigNet NTRIP caster using stored credentials:
// 1. Checks if TrigNet credentials are configured; skips connection if missing.
// 2. Updates OLED display (if available) to show connection progress.
// 3. Opens a TCP connection to the NTRIP caster host/port.
// 4. Encodes username:password in Base64 and sends the HTTP GET request for the mount point.
// 5. Waits up to 5 seconds to read the server response header.
// 6. Prints the server response header to Serial for debugging.
// 7. Checks if HTTP response contains "200" → sets trignetConnected flag accordingly.
// 8. Stops the client and flags failure if connection was unsuccessful.
// Serial prints are used extensively for debugging and monitoring connection progress.
void connectToTrigNet() {
  if (stored_casterHost.isEmpty() || stored_casterUser.isEmpty()) {
    Serial.println(F("TrigNet credentials not configured - skipping connection"));
    return;
  }
  
  Serial.println(F("Connecting to TrigNet..."));
  
  if (oledAvailable) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("WiFi: Connected"));
    display.println(F("TrigNet: Connecting..."));
    display.display();
  }
  
  if (ntripClient.connect(stored_casterHost.c_str(), stored_casterPort)) {
    String auth = stored_casterUser + ":" + stored_casterUserPW;
    base64 b;
    String authBase64 = b.encode(auth);

    ntripClient.print("GET /" + stored_mountPoint + " HTTP/1.0\r\n");
    ntripClient.print("User-Agent: NTRIP ESP32_TrigNet_Client v2.0\r\n");
    ntripClient.print("Authorization: Basic " + authBase64 + "\r\n");
    ntripClient.print("\r\n");

    unsigned long startTime = millis();
    char response[512] = {0};
    size_t responseSpot = 0;
    bool headerComplete = false;
    while (!headerComplete && (millis() - startTime < 5000)) {
      while (ntripClient.available()) {
        if (responseSpot < sizeof(response) - 1) response[responseSpot++] = ntripClient.read();
        if (strstr(response, "\r\n\r\n") != NULL) {
          headerComplete = true;
          break;
        }
      }
      if (!headerComplete) delay(10);
    }

    Serial.println(F("\n--- Server Response Header ---"));
    Serial.print(response);
    Serial.println(F("--- End Response ---\n"));

    if (strstr(response, "200") != NULL) {
      Serial.println(F("✓ HTTP 200 OK - TrigNet connection successful!"));
      trignetConnected = true;
    } else {
      Serial.println(F("✗ TrigNet connection failed!"));
      ntripClient.stop();
      trignetConnected = false;
    }
  } else {
    Serial.println(F("✗ Failed to connect to TrigNet!"));
    trignetConnected = false;
  }
}

//==================RTCM BYTES RETRIEVAL AND PROCESSING===================
// Retrieves RTCM correction bytes from the NTRIP (TrigNet) connection and pushes them to the ZED-F9P:
// 1. Checks if the NTRIP client is connected; returns false if not.
// 2. Reads available bytes from the NTRIP client into a local buffer rtcmData.
// 3. Increments a global byteCount for monitoring total bytes received.
// 4. Briefly flashes the built-in LED for each byte read (visual debugging).
// 5. Stops reading if the buffer is full (prevents overflow).
// 6. If any bytes were read, pushes them to the GNSS module via myGNSS.pushRawData().
// 7. Prints to Serial how many RTCM bytes were successfully pushed — useful for debugging RTCM reception and GNSS corrections.
bool processConnection() {
  if (!ntripClient.connected()) return false;

  uint8_t rtcmData[512 * 4];
  size_t rtcmCount = 0;
  while (ntripClient.available()) {
    byte b = ntripClient.read();
    byteCount++;
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1);
    digitalWrite(LED_BUILTIN, LOW);
    rtcmData[rtcmCount++] = b;
    if (rtcmCount == sizeof(rtcmData)) break;
  }

  if (rtcmCount > 0) {
    myGNSS.pushRawData(rtcmData, rtcmCount);
    Serial.print(F("→ Pushed "));
    Serial.print(rtcmCount);
    Serial.println(F(" RTCM bytes to ZED-F9P"));
  }

  return true;
}