#include "Firebase_Arduino_WiFiNINA.h"
#include "Wifi_Login.h"
#include <Arduino_LSM6DS3.h>
#include <ArduinoJson.h>
#include <ArduinoBLE.h>

/*------------MACROS--------------*/
// Firebase configuration
#define FIREBASE_HOST "smartgloves-e450e-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "dGconvs4SP3IWXB7wohi9KliWVZnIRHp3IgJrrOY"

//  Size of JSON payload is determined as follows:
//    - timestamp -> string = 9 bytes (00:00:00)
//    - coordinates for both sensors (x,y,z) -> 112 bytes
//    - Total = 121 bytes
// Calculated at: https://arduinojson.org/v6/assistant/
#define PAYLOAD_LENGTH 121

/*------------GLOBAL VARIABLES--------------*/
FirebaseData firebaseData;
WiFiUDP Udp;

// Variables for storing previous accelerometer readings
float ax_old = 0;
float ay_old = 0;
float az_old = 0;

// Variables for storing previous gyroscope readings
float gx_old = 0;
float gy_old = 0;
float gz_old = 0;

// Last time the IMU sensors were read, in ms
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;
uint32_t hours = 0;
uint32_t minutes = 0;

unsigned int localPort = 2390; // local port to listen on
IPAddress ip(10, 31, 164, 169);  // Static IP: 10.31.164.169
char  ReplyBuffer[] = "acknowledged";

/*------------FORWARD DECLARATIONS--------------*/
void updateIMUReadings();
void setupClock();

void updateIMUReadings() {
 /*  1. Read sensors every 200ms
  *  2. If any value changed, update values
  *  3. Create JSON packet and send to Firebase DB
  */
  float ax, ay, az, gx, gy, gz;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    // DEBUG INFO: Transmit accelerometer readings to serial monitor for debugging purposes
    //    Serial.print(ax);
    //    Serial.print('\t');
    //    Serial.print(ay);
    //    Serial.print('\t');
    //    Serial.println(az);
  }

  if(IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    // DEBUG INFO: Transmit gyroscope readings to serial monitor for debugging purposes
    //    Serial.print(gx);
    //    Serial.print('\t');
    //    Serial.print(gy);
    //    Serial.print('\t');
    //    Serial.println(gz);
  }

  uint32_t seconds = currentMillis / 1000;
  //uint32_t minutes = seconds / 60;
  
  uint32_t microseconds = TC4->COUNT32.COUNT.reg / 48;
  //Serial.println(String(microseconds));
  if (microseconds >= 60000000) {
    minutes++;
    hours = minutes / 60;
    resetClock();
  }
  currentMillis %= 1000;
  seconds %= 60;
  //minutes += total_minutes;
  minutes %= 60;
    
  String ms = currentMillis >= 10? (currentMillis >= 100? String(currentMillis) : "0" + String(currentMillis)) : "00" + String(currentMillis);
  String secs = seconds >= 10? String(seconds) : "0" + String(seconds);
  String mins = minutes >= 10? String(minutes) : "0" + String(minutes);
  String hrs = hours >= 10? String(hours) : "0" + String(hours);
  String time_string = hrs + ':' + mins + ':' + secs + ':' + ms;
  
  if(ax != ax_old || ay != ay_old || az != az_old ||
     gx != gx_old || gy != gy_old || gz != gz_old) {
    //Serial.println("Reading changed");
    ax_old = ax;
    ay_old = ay;
    az_old = az;
    gx_old = gx;
    gy_old = gy;
    gz_old = gz;

    StaticJsonDocument<PAYLOAD_LENGTH> doc;
    doc["ax"] = ax;
    doc["ay"] = ay;
    doc["az"] = az;
    doc["gx"] = gx;
    doc["gy"] = gy;
    doc["gz"] = gz;
    doc["timestamp"] = time_string;
    // TODO: Add attribute to indicate which glove is sending the data

    String jsonString;
    serializeJson(doc, jsonString);
    //Serial.println("Pushing to firebase: " + jsonString);
    Firebase.pushJSON(firebaseData, time_string + "/leftGlove", jsonString);
  }
}

void setup() {
  // Initialize serial monitor
  // (Used for debugging purposes)
  Serial.begin(9600);
  while (!Serial);
  Serial.println();

  // Set static IP address
  WiFi.config(ip);

  // Connect to Wi-Fi network
  Serial.println("Connecting to Wi-Fi");
  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.println("Waiting to connect...");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  // Connect to Firebase
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH, WIFI_SSID, WIFI_PASSWORD);
  Firebase.reconnectWiFi(true);

  // Open socket on local port for listening to incoming UDP packets
  if(!Udp.begin(localPort)) {
    Serial.println("Error initiliazing UDP module - no sockets available");
  }

  Serial.println("-----------------------------------");
  Serial.println("----------Begin Sampling-----------");
  Serial.println("-----------------------------------");
  Serial.println();

  // Initialize IMU sensors
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // DEBUG INFO: Sample rate
  // Accelerometer sample rate in Hz and G's
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println("Acceleration in G's");
  Serial.println();
  
  // Gyroscope sample rate in Hz and degrees/s
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println("Gyroscope in degrees/second");
  Serial.println();

  // Wait for start signal from right glove
  while(!Udp.parsePacket())
  {
    //Serial.println("Awaiting UDP Packet from right glove");
    delay(500);
  }; 
  Serial.println("Starting to send acknow. pack.");
  // send ack packet
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(ReplyBuffer);
  Udp.endPacket();

  // start the timer
  setupClock();
}

void setupClock() {
  //Serial.println("Starting to setup clock");
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |                // Enable the generic clock...
                      GCLK_CLKCTRL_GEN_GCLK0 |            // ....on GCLK0 at 48MHz
                      GCLK_CLKCTRL_ID_TC4_TC5;            // Feed the GCLK0 to TC4 and TC5
  while (GCLK->STATUS.bit.SYNCBUSY);                      // Wait for synchronization
 
  TC4->COUNT32.CTRLA.reg |= //TC_CTRLA_PRESCALER_DIV1 |     // Set the timer prescaler 
                            //TC_CTRLA_PRESCSYNC_PRESC |    // Reload timer on next prescaler clock
                            TC_CTRLA_MODE_COUNT32;        // Set the TC4 timer to 32-bit mode in conjuction with timer TC5
                   
  TC4->COUNT32.CTRLA.bit.ENABLE = 1;                      // Enable TC4
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);               // Wait for synchronization
  
  TC4->COUNT32.READREQ.reg = TC_READREQ_RCONT |           // Enable a continuous read request
                             TC_READREQ_ADDR(0x10);       // Offset of the 32-bit COUNT register
  while(TC4->COUNT32.STATUS.bit.SYNCBUSY);                // Wait for (read) synchronization
}

void resetClock() {
  TC4->COUNT32.CTRLBSET.reg = TC_CTRLBSET_CMD_RETRIGGER;
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);
}

void loop() {
  int i = 0; 
  // configure for # of iterations you'd like
  //Serial.println("Entering loop:");
  while(i < 250000000) {
    uint32_t microseconds = TC4->COUNT32.COUNT.reg / 48; 
    currentMillis = microseconds / 1000;
    //Serial.println("in loop() on iteration " + String(i));
    //Serial.println(String(currentMillis));
    // Read from sensors every 200ms
    if (currentMillis % 250 == 0) {
      //Serial.println(String(currentMillis));
      previousMillis = currentMillis;
      // push reading from this glove to firebase
      updateIMUReadings();
    }
    i++;
  }
  Serial.println("End of loop");
  while(1);
}
