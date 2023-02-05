/*
  This software, the ideas and concepts is Copyright (c) David Bird 2020
  All rights to this software are reserved.
  It is prohibited to redistribute or reproduce of any part or all of the software contents in any form other than the following:
  1. You may print or download to a local hard disk extracts for your personal and non-commercial use only.
  2. You may copy the content to individual third parties for their personal use, but only if you acknowledge the author David Bird as the source of the material.
  3. You may not, except with my express written permission, distribute or commercially exploit the content.
  4. You may not transmit it or store it in any other website or other form of electronic retrieval system for commercial purposes.
  5. You MUST include all of this copyright and permission notice ('as annotated') and this shall be included in all copies or substantial portions of the software
     and where the software use is visible to an end-user.
  THE SOFTWARE IS PROVIDED "AS IS" FOR PRIVATE USE ONLY, IT IS NOT FOR COMMERCIAL USE IN WHOLE OR PART OR CONCEPT.
  FOR PERSONAL USE IT IS SUPPLIED WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR
  A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHOR OR COPYRIGHT HOLDER BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OR
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  See more at http://dsbird.org.uk
*/

//################# LIBRARIES ################
#include <Arduino.h>                      // Built-in
#include <WiFi.h>                      // Built-in
#include <ESPmDNS.h>                   // Built-in
#include <SPIFFS.h>                    // Built-in
#include "ESPAsyncWebServer.h"         // https://github.com/me-no-dev/ESPAsyncWebServer/tree/63b5303880023f17e1bca517ac593d8a33955e94
#include "AsyncTCP.h"                  // https://github.com/me-no-dev/AsyncTCP
#include <Wire.h>
#include "SHTSensor.h"
#include "config.hpp"

//################ CONSTANTS ################
const int MAX_SENSOR_READINGS=144;         // maximum number of sensor readings, typically 144/day at 6-per-hour
const int NUM_OF_SENSORS=2;              // number of sensors (+1), set by the graphing section
const bool NO_REFRESH=false;          // Set auto refresh OFF
const bool REFRESH=true;           // Set auto refresh ON
const bool ON=true;           // Set the Relay ON
const bool OFF=false;          // Set the Relay OFF
const bool RELAY_REVERSE=true;          // Set to true for Relay that requires a signal LOW for ON
const bool SIMULATING=THERMOSTAT_SIMULATING;    // Switch OFF for actual sensor readings, ON for simulated random values
const int RELAY_PIN=THERMOSTAT_RELAY_PIN;
const int SENSOR_PIN=THERMOSTAT_SENSOR_PIN;
const String VERSION = "2.0";      // Programme version, see change log at end
const String SITE_TITLE = "Smart Thermostat";
const String YEAR = "2022";     // For the footer line
const String LEGEND_COLOR = "black";           // Only use HTML colour names
const String TITLE_COLOR = "purple";
const String BACKGROUND_COLOR = "gainsboro";
const String SETTINGS_FILENAME = "params.txt";  // Storage file name on flash
const char* SERVER_NAME = "thermostat";                     // Connect to the server with http://hpserver.local/ e.g. if name = "myserver" use http://myserver.local/
const char* WIFI_SSID = THERMOSTAT_WIFI_SSID;             // WiFi SSID     replace with details for your local network
const char* WIFI_PASSWORD = THERMOSTAT_WIFI_PASSWORD;         // WiFi Password replace with details for your local network
const char* TIMEZONE = THERMOSTAT_TIMEZONE;
const int NUM_OF_EVENTS=4;              // Number of events per-day, 4 is a practical limit

typedef struct {
  float Temp = 0;
  byte  Humi = 0;
} SensorDataType;

struct Settings {
  String DoW;                // Day of Week for the programmed event
  String Start[NUM_OF_EVENTS]; // Start time
  String Stop[NUM_OF_EVENTS];  // End time
  String Temp[NUM_OF_EVENTS];  // Required temperature during the Start-End times
};

//################ VARIABLES ################
SHTSensor sht;
SensorDataType _sensorData[NUM_OF_SENSORS][MAX_SENSOR_READINGS];
String _sensorReading[NUM_OF_SENSORS][6];    // 254 Sensors max. and 6 Parameters per sensor T, H, Relay-state. Maximum LoRa adress range is 255 - 1 for Server so 0 - 253
String _time_str, _doW_str;                 // For Date and Time
Settings _timer[7];                        // Timer settings, 7-days of the week
int _sensorReadingPointer[NUM_OF_SENSORS];   // Used for sensor data storage
float  _hysteresis           = 0.2;        // Heating Hysteresis default value
float  _temperature          = 0;          // Variable for the current temperature
float  _humidity             = 0;          // Variable for the current temperature
float  _targetTemp           = 20;         // Default thermostat value for set temperature
int    _frostTemp            = 5;          // Default thermostat value for frost protection temperature
float  _manOverrideTemp      = 21;         // Manual override temperature
float  _maxTemperature       = 28;         // Maximum temperature detection, switches off thermostat when reached
bool   _manualOverride       = false;      // Manual override
int    _earlyStart           = 0;          // Default thermostat value for early start of heating
String _relayState           = "OFF";      // Current setting of the control/thermostat relay
String _timerState           = "OFF";      // Current setting of the timer
String _units                = "M";        // or Units = "I" for °F and 12:12pm time format
String _webpage              = "";         // General purpose variable to hold HTML code for display
int    _timerCheckDuration   = 5000;       // Check for timer event every 5-seconds
int    _lastReadingDuration  = 1;          // Add sensor reading every n-mins
int    _lastTimerSwitchCheck = 0;          // Counter for last timer check
int    _lastReadingCheck     = 0;          // Counter for last reading saved check
float  _lastTemperature      = 0;          // Last temperature used for rogue reading detection
int    _unixTime             = 0;          // Time now (when updated) of the current time

// To access server from outside of a WiFi (LAN) network e.g. on port 8080 add a rule on your Router that forwards a connection request
// to http://your_WAN_address:8080/ to http://your_LAN_address:8080 and then you can view your ESP server from anywhere.
// Example http://yourhome.ip:8080 and your ESP Server is at 192.168.0.40, then the request will be directed to http://192.168.0.40:8080
AsyncWebServer server(THERMOSTAT_SERVER_PORT); // Server on IP address port 80 (web-browser default, change to your requirements, e.g. 8080

//#########################################
//################ SENSORS ################
//#########################################
void startSensor() {
  if (!SIMULATING) {                               // If not sensor simulating, then start the real one
      Wire.begin();
      delay(1000); // let serial console settle

       if (sht.init()) {
            Serial.print("Sensor started...\n");
        } else {
            Serial.print("Unable to init sensors\n");
        }
        sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM); // only supported by SHT3x
  }
}

void readSensor() {
  if (SIMULATING) {
    _temperature = 20.2 + random(-15, 15) / 10.0; // Generate a random temperature value between 18.7° and 21.7°
    _humidity    = random(45, 55);                // Generate a random humidity value between 45% and 55%
  }
  else
  {
    if (sht.readSample()) {
        _humidity = sht.getHumidity();

        _temperature = sht.getTemperature();
        if (_temperature >= 50 || _temperature < -30){
            _temperature = _lastTemperature; // Check and correct any errorneous readings
         }
        _lastTemperature = _temperature;
    } else {
        Serial.print("Error in readSample()\n");
    }
  }
  Serial.println("Temperature = " + String(_temperature, 1) + ", Humidity = " + String(_humidity, 0));
}

void switchRelay(bool demand) {
  pinMode(RELAY_PIN, OUTPUT);

  if (demand) {
    _relayState = "ON";
    if (RELAY_REVERSE) {
      digitalWrite(RELAY_PIN, LOW);
    }
    else
    {
      digitalWrite(RELAY_PIN, HIGH);
    }
    Serial.println("Thermostat ON");
  }
  else
  {
    _relayState = "OFF";
    if (RELAY_REVERSE) {
      digitalWrite(RELAY_PIN, HIGH);
    }
    else
    {
      digitalWrite(RELAY_PIN, LOW);
    }
    Serial.println("Thermostat OFF");
  }
}

void controlHeating() {
  if (_temperature < (_targetTemp - _hysteresis)) {           // Check if room temeperature is below set-point and hysteresis offset
    switchRelay(ON);                                    // Switch Relay/Heating ON if so
  }
  if (_temperature > (_targetTemp + _hysteresis)) {           // Check if room temeperature is above set-point and hysteresis offset
    switchRelay(OFF);                                   // Switch Relay/Heating OFF if so
  }
  if (_temperature > _maxTemperature) {                      // Check for faults/over-temperature
    switchRelay(OFF);                                   // Switch Relay/Heating OFF if temperature is above maximum temperature
  }
}

void addReadingToSensorData(byte RxdFromID, float Temperature, byte Humidity) {
  byte ptr, p;
  ptr = _sensorReadingPointer[RxdFromID];
  _sensorData[RxdFromID][ptr].Temp = Temperature;
  _sensorData[RxdFromID][ptr].Humi = Humidity;
  ptr++;
  if (ptr >= MAX_SENSOR_READINGS) {
    p = 0;
    do {
      _sensorData[RxdFromID][p].Temp  = _sensorData[RxdFromID][p + 1].Temp;
      _sensorData[RxdFromID][p].Humi  = _sensorData[RxdFromID][p + 1].Humi;
      p++;
    } while (p < MAX_SENSOR_READINGS);
    ptr = MAX_SENSOR_READINGS - 1;
    _sensorData[RxdFromID][MAX_SENSOR_READINGS - 1].Temp = Temperature;
    _sensorData[RxdFromID][MAX_SENSOR_READINGS - 1].Humi = Humidity;
  }
  _sensorReadingPointer[RxdFromID] = ptr;
}

void assignMaxSensorReadingsToArray() {
  _sensorReading[1][0] = 1;
  _sensorReading[1][1] = _temperature;
  _sensorReading[1][2] = _humidity;
  _sensorReading[1][3] = _relayState;
  addReadingToSensorData(1, _temperature, _humidity); // Only sensor-1 is implemented here, could  be more though
}


//#########################################
//################ SYSTEM #################
//#########################################
void setupSystem() {
  Serial.begin(9600);                                           // Initialise serial communications
  delay(200);
  Serial.println(__FILE__);
  Serial.println("Starting...");
}

void setupDeviceName(const char *DeviceName) {
  if (MDNS.begin(DeviceName)) { // The name that will identify your device on the network
    Serial.println("mDNS responder started");
    Serial.print("Device name: ");
    Serial.println(DeviceName);
    MDNS.addService("n8i-mlp", "tcp", 23); // Add service
  }
  else
    Serial.println("Error setting up MDNS responder");
}

void startWiFi() {
  Serial.print("\r\nConnecting to: "); Serial.println(String(WIFI_SSID));
  IPAddress dns(8, 8, 8, 8); // Use Google as DNS
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);       // switch off AP
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(50);
  }
  Serial.println("\nWiFi connected at: " + WiFi.localIP().toString());
}

boolean updateLocalTime() {
  struct tm timeinfo;
  time_t now;
  char  time_output[30];
  while (!getLocalTime(&timeinfo, 15000)) {                        // Wait for up to 15-sec for time to synchronise
    return false;
  }
  time(&now);
  _unixTime = now;
  //See http://www.cplusplus.com/reference/ctime/strftime/
  strftime(time_output, sizeof(time_output), "%H:%M", &timeinfo);  // Creates: '14:05'
  _time_str = time_output;
  strftime(time_output, sizeof(time_output), "%w", &timeinfo);     // Creates: '0' for Sun
  _doW_str  = time_output;
  return true;
}

boolean setupTime() {
  configTime(0, 0, "time.nist.gov");                               // (gmtOffset_sec, daylightOffset_sec, ntpServer)
  setenv("TZ", TIMEZONE, 1);                                       // setenv()adds "TZ" variable to the environment, only used if set to 1, 0 means no change
  tzset();
  delay(200);
  bool TimeStatus = updateLocalTime();
  return TimeStatus;
}

String convertUnixTime(int unix_time) {
  time_t tm = unix_time;
  struct tm *now_tm = localtime(&tm);
  char output[40];
  strftime(output, sizeof(output), "%H:%M", now_tm);               // Returns 21:12
  return output;
}

void startSPIFFS() {
  Serial.println("Starting SPIFFS");
  boolean SPIFFS_Status;
  SPIFFS_Status = SPIFFS.begin();
  if (SPIFFS_Status == false)
  { // Most likely SPIFFS has not yet been formated, so do so
    Serial.println("Formatting SPIFFS (it may take some time)...");
    SPIFFS.begin(true); // Now format SPIFFS
    File datafile = SPIFFS.open("/" + SETTINGS_FILENAME, "r");
    if (!datafile || !datafile.isDirectory()) {
      Serial.println("SPIFFS failed to start..."); // Nothing more can be done, so delete and then create another file
      SPIFFS.remove("/" + SETTINGS_FILENAME); // The file is corrupted!!
      datafile.close();
    }
  }
  else Serial.println("SPIFFS Started successfully...");
}

String getWiFiSignal() {
  float Signal = WiFi.RSSI();
  Signal = 90 / 40.0 * Signal + 212.5; // From Signal = 100% @ -50dBm and Signal = 10% @ -90dBm and y = mx + c
  if (Signal > 100) Signal = 100;
  return " " + String(Signal, 0) + "%";
}

//#########################################
//################ SETTINGS ###############
//#########################################
void initialise_Array() {
  _timer[0].DoW = "Sun"; _timer[1].DoW = "Mon"; _timer[2].DoW = "Tue"; _timer[3].DoW = "Wed"; _timer[4].DoW = "Thu"; _timer[5].DoW = "Fri"; _timer[6].DoW = "Sat";
}

void saveSettingsPage() {
  Serial.println("Getting ready to Save settings...");
  File dataFile = SPIFFS.open("/" + SETTINGS_FILENAME, "w");
  if (dataFile) { // Save settings
    Serial.println("Saving settings...");
    for (byte dow = 0; dow < 7; dow++) {
      Serial.println("Day of week = " + String(dow));
      for (byte p = 0; p < NUM_OF_EVENTS; p++) {
        dataFile.println(_timer[dow].Temp[p]);
        dataFile.println(_timer[dow].Start[p]);
        dataFile.println(_timer[dow].Stop[p]);
        Serial.println("Period: " + String(p) + " " + _timer[dow].Temp[p] + " from: " + _timer[dow].Start[p] + " to: " + _timer[dow].Stop[p]);
      }
    }
    dataFile.println(_hysteresis, 1);
    dataFile.println(_frostTemp, 1);
    dataFile.println(_earlyStart);
    Serial.println("Saved Hysteresis : " + String(_hysteresis));
    Serial.println("Saved Frost Temp : " + String(_frostTemp));
    Serial.println("Saved EarlyStart : " + String(_earlyStart));
    dataFile.close();
    Serial.println("Settings saved...");
  }
}

void recoverSettings() {
  String Entry;
  Serial.println("Reading settings...");
  File dataFile = SPIFFS.open("/" + SETTINGS_FILENAME, "r");
  if (dataFile) { // if the file is available, read it
    Serial.println("Recovering settings...");
    while (dataFile.available()) {
      for (byte dow = 0; dow < 7; dow++) {
        Serial.println("Day of week = " + String(dow));
        for (byte p = 0; p < NUM_OF_EVENTS; p++) {
          _timer[dow].Temp[p]  = dataFile.readStringUntil('\n'); _timer[dow].Temp[p].trim();
          _timer[dow].Start[p] = dataFile.readStringUntil('\n'); _timer[dow].Start[p].trim();
          _timer[dow].Stop[p]  = dataFile.readStringUntil('\n'); _timer[dow].Stop[p].trim();
          Serial.println("Period: " + String(p) + " " + _timer[dow].Temp[p] + " from: " + _timer[dow].Start[p] + " to: " + _timer[dow].Stop[p]);
        }
      }
      Entry = dataFile.readStringUntil('\n'); Entry.trim(); _hysteresis = Entry.toFloat();
      Entry = dataFile.readStringUntil('\n'); Entry.trim(); _frostTemp  = Entry.toInt();
      Entry = dataFile.readStringUntil('\n'); Entry.trim(); _earlyStart = Entry.toInt();
      Serial.println("Recovered Hysteresis : " + String(_hysteresis));
      Serial.println("Recovered Frost Temp : " + String(_frostTemp));
      Serial.println("Recovered EarlyStart : " + String(_earlyStart));
      dataFile.close();
      Serial.println("Settings recovered...");
    }
  }
}

//#########################################
//################ SCHEDULING #############
//#########################################
void updateTargetTemperature() {
  String TimeNow;
  TimeNow = convertUnixTime(_unixTime);
  for (byte dow = 0; dow < 7; dow++) {
    for (byte p = 0; p < NUM_OF_EVENTS; p++) {
      if (String(dow) == _doW_str && (TimeNow >= _timer[dow].Start[p] && TimeNow < _timer[dow].Stop[p]))
      {
        _targetTemp = _timer[dow].Temp[p].toFloat();               // Found the programmed set-point temperature from the scheduled time period
      }
    }
  }
  if (_manualOverride == ON) _targetTemp = _manOverrideTemp;
  Serial.println("Target Temperature = " + String(_targetTemp, 1) + "°");
}

void checkAndSetFrostTemperature() {
  if (_timerState == "OFF" && _manualOverride == OFF) { // Only check for frost protection when heating is off
    if (_temperature < (_frostTemp - _hysteresis)) {     // Check if temperature is below Frost Protection temperature and hysteresis offset
      switchRelay(ON);                             // Switch Relay/Heating ON if so
      Serial.println("Frost protection actuated...");
    }
    if (_temperature > (_frostTemp + _hysteresis)) {     // Check if temerature is above Frost Protection temperature and hysteresis offset
      switchRelay(OFF);                            // Switch Relay/Heating OFF if so
    }
  }
}


void CheckTimerEvent() {
  String TimeNow;
  updateTargetTemperature();
  TimeNow    = convertUnixTime(_unixTime);                  // Get the current time e.g. 15:35
  _timerState = "OFF";                                      // Switch timer off until decided by the schedule
  if (_earlyStart > 0) {                                    // If early start is enabled by a value > 0
    TimeNow = convertUnixTime(_unixTime + _earlyStart * 60); // Advance the clock by the Early Start Duration
  }
  if (_manualOverride == ON) {                              // If manual override is enabled then turn the heating on
    _targetTemp = _manOverrideTemp;                          // Set the target temperature to the manual overide temperature
    controlHeating();                                      // Control the heating as normal
  }
  else
  {
    for (byte dow = 0; dow < 7; dow++) {                     // Look for any valid timer events, if found turn the heating on
      for (byte p = 0; p < NUM_OF_EVENTS; p++) {
        // Now check for a scheduled ON time, if so Switch the Timer ON and check the temperature against target temperature
        if (String(dow) == _doW_str && (TimeNow >= _timer[dow].Start[p] && TimeNow <= _timer[dow].Stop[p] && _timer[dow].Start[p] != ""))
        {
          _timerState = "ON";
          controlHeating();
          _manualOverride = OFF; // If it was ON turn it OFF when the timer starts a controlled period
        }
      }
    }
  }
  checkAndSetFrostTemperature();
}


//#########################################
//################ PAGES ##################
//#########################################
String preLoadChartData(byte Channel, String Type) {
  byte r = 0;
  String Data = "";
  do {
    if (Type == "Temperature") {
      Data += "[" + String(r) + "," + String(_sensorData[Channel][r].Temp, 1) + "," + String(_targetTemp, 1) + "],";
    }
    else
    {
      Data += "[" + String(r) + "," + String(_sensorData[Channel][r].Humi) + "],";
    }
    r++;
  } while (r < MAX_SENSOR_READINGS);
  Data += "]";
  return Data;
}

void append_HTML_header(bool refreshMode) {
  _webpage  = "<!DOCTYPE html><html lang='en'>";
  _webpage += "<head>";
  _webpage += "<title>" + SITE_TITLE + "</title>";
  _webpage += "<meta charset='UTF-8'>";
  if (refreshMode) _webpage += "<meta http-equiv='refresh' content='5'>"; // 5-secs refresh time, test needed to prevent auto updates repeating some commands
  _webpage += "<script src=\"https://code.jquery.com/jquery-3.2.1.min.js\"></script>";
  _webpage += "<style>";
  _webpage += "body             {width:68em;margin-left:auto;margin-right:auto;font-family:Arial,Helvetica,sans-serif;font-size:14px;color:blue;background-color:#e1e1ff;text-align:center;}";
  _webpage += ".centre          {margin-left:auto;margin-right:auto;}";
  _webpage += "h2               {margin-top:0.3em;margin-bottom:0.3em;font-size:1.4em;}";
  _webpage += "h3               {margin-top:0.3em;margin-bottom:0.3em;font-size:1.2em;}";
  _webpage += "h4               {margin-top:0.3em;margin-bottom:0.3em;font-size:0.8em;}";
  _webpage += ".on              {color: red;}";
  _webpage += ".off             {color: limegreen;}";
  _webpage += ".topnav          {overflow: hidden;background-color:lightcyan;}";
  _webpage += ".topnav a        {float:left;color:blue;text-align:center;padding:1em 1.14em;text-decoration:none;font-size:1.3em;}";
  _webpage += ".topnav a:hover  {background-color:deepskyblue;color:white;}";
  _webpage += ".topnav a.active {background-color:lightblue;color:blue;}";
  _webpage += "table tr, td     {padding:0.2em 0.5em 0.2em 0.5em;font-size:1.0em;font-family:Arial,Helvetica,sans-serif;}";
  _webpage += "col:first-child  {background:lightcyan}col:nth-child(2){background:#CCC}col:nth-child(8){background:#CCC}";
  _webpage += "tr:first-child   {background:lightcyan}";
  _webpage += ".large           {font-size:1.8em;padding:0;margin:0}";
  _webpage += ".medium          {font-size:1.4em;padding:0;margin:0}";
  _webpage += ".ps              {font-size:0.7em;padding:0;margin:0}";
  _webpage += "#outer           {width:100%;display:flex;justify-content:center;}";
  _webpage += "footer           {padding:0.08em;background-color:cyan;font-size:1.1em;}";
  _webpage += ".numberCircle    {border-radius:50%;width:2.7em;height:2.7em;border:0.11em solid blue;padding:0.2em;color:blue;text-align:center;font-size:3em;";
  _webpage += "                  display:inline-flex;justify-content:center;align-items:center;}";
  _webpage += ".wifi            {padding:3px;position:relative;top:1em;left:0.36em;}";
  _webpage += ".wifi, .wifi:before {display:inline-block;border:9px double transparent;border-top-color:currentColor;border-radius:50%;}";
  _webpage += "<p class='ps'><i>Copyright &copy;&nbsp;D L Bird " + String(YEAR) + " V" + VERSION + "</i></p>";
  _webpage += ".wifi:before     {content:'';width:0;height:0;}";
  _webpage += "</style></head>";
  _webpage += "<body>";
  _webpage += "<div class='topnav'>";
  _webpage += "<a href='/'>Status</a>";
  _webpage += "<a href='graphs'>Graph</a>";
  _webpage += "<a href='timer'>Schedule</a>";
  _webpage += "<a href='setup'>Setup</a>";
  _webpage += "<a href='help'>Help</a>";
  _webpage += "<a href=''></a>";
  _webpage += "<a href=''></a>";
  _webpage += "<a href=''></a>";
  _webpage += "<a href=''></a>";
  _webpage += "<div class='wifi'/></div><span>" + getWiFiSignal() + "</span>";
  _webpage += "</div><br>";
}

void append_HTML_footer() {
  _webpage += "<footer>";
  _webpage += "<p class='medium'>ESP Smart Thermostat</p>";
  _webpage += "</footer>";
  _webpage += "</body></html>";
}

void add_Graph(byte Channel, String Type, String Title, String GraphType, String Units, String Colour, String Div) {
  String Data = preLoadChartData(Channel, Title);
  _webpage += "function draw" + Type + String(Channel) + "() {";
  if (Type == "GraphT") {
    _webpage += " var data = google.visualization.arrayToDataTable(" + String("[['Hour', 'Rm T°', 'Tgt T°'],") + Data + ");";
  }
  else
    _webpage += " var data = google.visualization.arrayToDataTable(" + String("[['Hour', 'RH %'],") + Data + ");";
  _webpage += " var options = {";
  _webpage += "  title: '" + Title + "',";
  _webpage += "  titleFontSize: 14,";
  _webpage += "  backgroundColor: '" + BACKGROUND_COLOR + "',";
  _webpage += "  legendTextStyle: { color: '" + LEGEND_COLOR + "' },";
  _webpage += "  titleTextStyle:  { color: '" + TITLE_COLOR  + "' },";
  _webpage += "  hAxis: {color: '#FFF'},";
  _webpage += "  vAxis: {color: '#FFF', title: '" + Units + "'},";
  _webpage += "  curveType: 'function',";
  _webpage += "  pointSize: 1,";
  _webpage += "  lineWidth: 1,";
  _webpage += "  width:  450,";
  _webpage += "  height: 280,";
  _webpage += "  colors:['" + Colour + (Type == "GraphT" ? "', 'orange" : "") + "'],";
  _webpage += "  legend: { position: 'right' }";
  _webpage += " };";
  _webpage += " var chart = new google.visualization.LineChart(document.getElementById('" + Div + GraphType + String(Channel) + "'));";
  _webpage += "  chart.draw(data, options);";
  _webpage += " };";
}


void HomePage() {
  readSensor();
  append_HTML_header(REFRESH);
  _webpage += "<h2>Smart Thermostat Status</h2><br>";
  _webpage += "<div class='numberCircle'><span class=" + String((_relayState == "ON" ? "'on'>" : "'off'>")) + String(_temperature, 1) + "&deg;</span></div><br><br><br>";
  _webpage += "<table class='centre'>";
  _webpage += "<tr>";
  _webpage += "<td>Temperature</td>";
  _webpage += "<td>Humidity</td>";
  _webpage += "<td>Target Temperature</td>";
  _webpage += "<td>Thermostat Status</td>";
  _webpage += "<td>Schedule Status</td>";
  if (_manualOverride) {
    _webpage += "<td>ManualOverride</td>";
  }
  _webpage += "</tr>";
  _webpage += "<tr>";
  _webpage += "<td class='large'>" + String(_temperature, 1)       + "&deg;</td>";
  _webpage += "<td class='large'>" + String(_humidity, 0)          + "%</td>";
  _webpage += "<td class='large'>" + String(_targetTemp, 1) + "&deg;</td>";
  _webpage += "<td class='large'><span class=" + String((_relayState == "ON" ? "'on'>" : "'off'>")) + _relayState + "</span></td>"; // (condition ? that : this) if this then that else this
  _webpage += "<td class='large'><span class=" + String((_timerState == "ON" ? "'on'>" : "'off'>")) + _timerState + "</span></td>";
  if (_manualOverride) {
    _webpage += "<td class='large'>" + String(_manualOverride ? "ON" : "OFF") + "</td>";
  }
  _webpage += "</tr>";
  _webpage += "</table>";
  _webpage += "<br>";
  append_HTML_footer();
}

void GraphsPage() {
  append_HTML_header(REFRESH);
  _webpage += "<h2>Thermostat Readings</h2>";
  _webpage += "<script type='text/javascript' src='https://www.gstatic.com/charts/loader.js'></script>";
  _webpage += "<script type='text/javascript'>";
  _webpage += "google.charts.load('current', {'packages':['corechart']});";
  _webpage += "google.charts.setOnLoadCallback(drawGraphT1);"; // Pre-load function names for Temperature graphs
  _webpage += "google.charts.setOnLoadCallback(drawGraphH1);"; // Pre-load function names for Humidity graphs
  add_Graph(1, "GraphT", "Temperature", "TS", "°C", "red",  "chart_div");
  add_Graph(1, "GraphH", "Humidity",    "HS", "%",  "blue", "chart_div");
  _webpage += "</script>";
  _webpage += "<div id='outer'>";
  _webpage += "<table>";
  _webpage += "<tr>";
  _webpage += "  <td><div id='chart_divTS1' style='width:50%'></div></td>";
  _webpage += "  <td><div id='chart_divHS1' style='width:50%'></div></td>";
  _webpage += "</tr>";
  _webpage += "</table>";
  _webpage += "<br>";
  _webpage += "</div>";
  _webpage += "<p>Heating status : <span class=" + String((_relayState == "ON" ? "'on'>" : "'off'>")) + _relayState + "</span></p>";
  append_HTML_footer();
}

void TimerSetPage() {
  append_HTML_header(NO_REFRESH);
  _webpage += "<h2>Thermostat Schedule Setup</h2><br>";
  _webpage += "<h3>Enter required temperatures and time, use Clock symbol for ease of time entry</h3><br>";
  _webpage += "<FORM action='/handletimer'>";
  _webpage += "<table class='centre'>";
  _webpage += "<col><col><col><col><col><col><col><col>";
  _webpage += "<tr><td>Control</td>";
  _webpage += "<td>" + _timer[0].DoW + "</td>";
  for (byte dow = 1; dow < 6; dow++) { // Heading line showing DoW
    _webpage += "<td>" + _timer[dow].DoW + "</td>";
  }
  _webpage += "<td>" + _timer[6].DoW + "</td>";
  _webpage += "</tr>";
  for (byte p = 0; p < NUM_OF_EVENTS; p++) {
    _webpage += "<tr><td>Temp</td>";
    _webpage += "<td><input type='text' name='" + String(0) + "." + String(p) + ".Temp' value='"  + _timer[0].Temp[p]   + "' maxlength='5' size='6'></td>";
    for (int dow = 1; dow < 6; dow++) {
      _webpage += "<td><input type='text' name='" + String(dow) + "." + String(p) + ".Temp' value='"      + _timer[dow].Temp[p] + "' maxlength='5' size='5'></td>";
    }
    _webpage += "<td><input type='text' name='" + String(6) + "." + String(p) + ".Temp' value='"  + _timer[6].Temp[p]   + "' maxlength='5' size='5'></td>";
    _webpage += "</tr>";
    _webpage += "<tr><td>Start</td>";
    _webpage += "<td><input type='time' name='" + String(0) + "." + String(p) + ".Start' value='" + _timer[0].Start[p] + "'></td>";
    for (int dow = 1; dow < 6; dow++) {
      _webpage += "<td><input type='time' name='" + String(dow) + "." + String(p) + ".Start' value='" + _timer[dow].Start[p] + "'></td>";
    }
    _webpage += "<td><input type='time' name='" + String(6) + "." + String(p) + ".Start' value='" + _timer[6].Start[p] + "'></td>";
    _webpage += "</tr>";
    _webpage += "<tr><td>Stop</td>";
    _webpage += "<td><input type='time' name='" + String(0) + "." + String(p) + ".Stop' value='" + _timer[0].Stop[p] + "'></td>";
    for (int dow = 1; dow < 6; dow++) {
      _webpage += "<td><input type='time' name='" + String(dow) + "." + String(p) + ".Stop' value='" + _timer[dow].Stop[p] + "'></td>";
    }
    _webpage += "<td><input type='time' name='" + String(6) + "." + String(p) + ".Stop' value='" + _timer[6].Stop[p] + "'></td>";
    _webpage += "</tr>";
    if (p < (NUM_OF_EVENTS - 1)) {
      _webpage += "<tr><td></td><td></td>";
      for (int dow = 2; dow < 7; dow++) {
        _webpage += "<td>-</td>";
      }
      _webpage += "<td></td></tr>";
    }
  }
  _webpage += "</table>";
  _webpage += "<div class='centre'>";
  _webpage += "<br><input type='submit' value='Enter'><br><br>";
  _webpage += "</div></form>";
  append_HTML_footer();
}

void SetupPage() {
  append_HTML_header(NO_REFRESH);
  _webpage += "<h2>Thermostat System Setup</h2><br>";
  _webpage += "<h3>Enter required parameter values</h3><br>";
  _webpage += "<FORM action='/handlesetup'>";
  _webpage += "<table class='centre'>";
  _webpage += "<tr>";
  _webpage += "<td>Setting</td><td>Value</td>";
  _webpage += "</tr>";
  _webpage += "<tr>";
  _webpage += "<td><label for='hysteresis'>Hysteresis value (e.g. 0 - 1.0&deg;) [N.N]</label></td>";
  _webpage += "<td><input type='text' size='4' pattern='[0-9][.][0-9]' name='hysteresis' value='" + String(_hysteresis, 1) + "'></td>"; // 0.0 valid input style
  _webpage += "</tr>";
  _webpage += "<tr>";
  _webpage += "<td><label for='frosttemp'>Frost Protection Temperature&deg; [NN]</label></td>";
  _webpage += "<td><input type='text' size='4' pattern='[0-9]*' name='frosttemp' value='" + String(_frostTemp) + "'></td>"; // 00-99 valid input style
  _webpage += "</tr>";
  _webpage += "<tr>";
  _webpage += "<td><label for='earlystart'>Early start duration (mins) [NN]</label></td>";
  _webpage += "<td><input type='text' size='4' pattern='[0-9]*' name='earlystart' value='" + String(_earlyStart) + "'></td>"; // 00-99 valid input style
  _webpage += "</tr>";
  _webpage += "<tr>";
  _webpage += "<td><label for='manualoveride'>Manual heating over-ride </label></td>";
  _webpage += "<td><select name='manualoverride'><option value='ON'>ON</option>";
  _webpage += "<option selected value='OFF'>OFF</option></select></td>"; // ON/OFF
  _webpage += "</tr>";
  _webpage += "<td><label for='manualoverridetemp'>Manual Override Temperature&deg; </label></td>";
  _webpage += "<td><input type='text' size='4' pattern='[0-9]*' name='manualoverridetemp' value='" + String(_manOverrideTemp, 0) + "'></td>"; // 00-99 valid input style
  _webpage += "</tr>";  _webpage += "</table>";
  _webpage += "<br><input type='submit' value='Enter'><br><br>";
  _webpage += "</form>";
  append_HTML_footer();
}

void HelpPage() {
  append_HTML_header(NO_REFRESH);
  _webpage += "<h2>Help</h2><br>";
  _webpage += "<div style='text-align: left;font-size:1.1em;'>";
  _webpage += "<br><u><b>Setup Menu</b></u>";
  _webpage += "<p><i>Hysteresis</i> - this setting is used to prevent unwanted rapid switching on/off of the heating as the room temperature";
  _webpage += " nears or falls towards the set/target-point temperature. A normal setting is 0.5&deg;C, the exact value depends on the environmental characteristics, ";
  _webpage += "for example, where the thermostat is located and how fast a room heats or cools.</p>";
  _webpage += "<p><i>Frost Protection Temperature</i> - this setting is used to protect from low temperatures and pipe freezing in cold conditions. ";
  _webpage += "It helps prevent low temperature damage by turning on the heating until the risk of freezing has been prevented.</p>";
  _webpage += "<p><i>Early Start Duration</i> - if greater than 0, begins heating earlier than scheduled so that the scheduled temperature is reached by the set time.</p>";
  _webpage += "<p><i>Heating Manual Override</i> - switch the heating on and control to the desired temperature, switched-off when the next timed period begins.</p>";
  _webpage += "<p><i>Heating Manual Override Temperature</i> - used to set the desired manual override temperature.</p>";
  _webpage += "<u><b>Schedule Menu</b></u>";
  _webpage += "<p>Determines the heating temperature for each day of the week and up to 4 heating periods in a day. ";
  _webpage += "To set the heating to come on at 06:00 and off at 09:00 with a temperature of 20&deg; enter 20 then the required start/end times. ";
  _webpage += "Repeat for each day of the week and heating period within the day for the required heat profile.</p>";
  _webpage += "<u><b>Graph Menu</b></u>";
  _webpage += "<p>Displays the target temperature set and the current measured temperature and humidity. ";
  _webpage += "Thermostat status is also displayed as temperature varies.</p>";
  _webpage += "<u><b>Status Menu</b></u>";
  _webpage += "<p>Displays the current temperature and humidity. ";
  _webpage += "Displays the temperature the thermostat is controlling towards, the current state of the thermostat (ON/OFF) and ";
  _webpage += "timer status (ON/OFF).</p>";
  _webpage += "</div>";
  append_HTML_footer();
}

//#########################################
//################ SERVER #################
//#########################################
void startServer(){
  // Set handler for '/'
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->redirect("/homepage");       // Go to home page
  });
  // Set handler for '/homepage'
  server.on("/homepage", HTTP_GET, [](AsyncWebServerRequest * request) {
    HomePage();
    request->send(200, "text/html", _webpage);
  });
  // Set handler for '/graphs'
  server.on("/graphs", HTTP_GET, [](AsyncWebServerRequest * request)   {
    GraphsPage();
    request->send(200, "text/html", _webpage);
  });
  // Set handler for '/timer'
  server.on("/timer", HTTP_GET, [](AsyncWebServerRequest * request) {
    TimerSetPage();
    request->send(200, "text/html", _webpage);
  });
  // Set handler for '/setup'
  server.on("/setup", HTTP_GET, [](AsyncWebServerRequest * request) {
    SetupPage();
    request->send(200, "text/html", _webpage);
  });
  // Set handler for '/help'
  server.on("/help", HTTP_GET, [](AsyncWebServerRequest * request) {
    HelpPage();
    request->send(200, "text/html", _webpage);
  });
  // Set handler for '/handletimer' inputs
  server.on("/handletimer", HTTP_GET, [](AsyncWebServerRequest * request) {
    for (byte dow = 0; dow < 7; dow++) {
      for (byte p = 0; p < NUM_OF_EVENTS; p++) {
        _timer[dow].Temp[p]  = request->arg(String(dow) + "." + String(p) + ".Temp");
        _timer[dow].Start[p] = request->arg(String(dow) + "." + String(p) + ".Start");
        _timer[dow].Stop[p]  = request->arg(String(dow) + "." + String(p) + ".Stop");
      }
    }
    saveSettingsPage();
    request->redirect("/homepage");                       // Go back to home page
  });
  // Set handler for '/handlesetup' inputs
  server.on("/handlesetup", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (request->hasArg("hysteresis")) {
      String numArg = request->arg("hysteresis");
      _hysteresis = numArg.toFloat();
    }
    if (request->hasArg("frosttemp")) {
      String numArg = request->arg("frosttemp");
      _frostTemp     = numArg.toFloat();
    }
    if (request->hasArg("earlystart")) {
      String numArg = request->arg("earlystart");
      _earlyStart    = numArg.toInt();
    }
    if (request->hasArg("manualoverride")) {
      String stringArg = request->arg("manualoverride");
      if (stringArg == "ON") _manualOverride = true; else _manualOverride = false;
    }
    if (request->hasArg("manualoverridetemp")) {
      String numArg   = request->arg("manualoverridetemp");
      _manOverrideTemp = numArg.toFloat();
    }
    saveSettingsPage();
    request->redirect("/homepage");                       // Go back to home page
  });

  server.begin();
}

//#########################################
//################ MAIN #################
//#########################################
void setup() {
  setupSystem();                          // General system setup
  startWiFi();                            // Start WiFi services
  setupTime();                            // Start NTP clock services
  startSPIFFS();                          // Start SPIFFS filing system
  initialise_Array();                     // Initialise the array for storage and set some values
  recoverSettings();                      // Recover settings from LittleFS
  setupDeviceName(SERVER_NAME);            // Set logical device name
  startServer();

  startSensor();
  readSensor();                                           // Get current sensor values
  for (int r = 0; r < MAX_SENSOR_READINGS; r++) {
    _sensorData[1][r].Temp = _temperature;
    _sensorData[1][r].Humi = _humidity;
  }
  switchRelay(OFF);                                    // Switch heating OFF
  readSensor();                                           // Get current sensor values
  _lastTimerSwitchCheck = millis() + _timerCheckDuration;   // preload timer value with update duration
}

void loop() {

  if ((millis() - _lastTimerSwitchCheck) > _timerCheckDuration) {
    _lastTimerSwitchCheck = millis();                      // Reset time
    readSensor();                                         // Get sensor readings, or get simulated values if 'simulated' is ON
    updateLocalTime();                                    // Updates Time UnixTime to 'now'
    CheckTimerEvent();                                    // Check for schedules actuated
  }
  if ((millis() - _lastReadingCheck) > (_lastReadingDuration * 60 * 1000)) {
    _lastReadingCheck = millis();                          // Update reading record every ~n-mins e.g. 60,000uS = 1-min
    assignMaxSensorReadingsToArray();
  }
}

