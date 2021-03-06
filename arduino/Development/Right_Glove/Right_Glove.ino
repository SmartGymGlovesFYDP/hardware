#include "Firebase_Arduino_WiFiNINA.h"
#include "Wifi_Login.h"
#include <Arduino_LSM6DS3.h>
#include <ArduinoJson.h>
#include <ArduinoBLE.h>

/*------------MACROS--------------*/
// Firebase configuration
#define FIREBASE_HOST "smartgloves-e450e-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "nVXQ8jwSV9b2aNOA7NwbB9L2UJCuDCfWQRB901Er"

//  Size of JSON payload is determined as follows:
//    - timestamp -> string = 9 bytes (00:00:00)
//    - coordinates for both sensors (x,y,z) -> 112 bytes
//    - Total = 121 bytes
// Calculated at: https://arduinojson.org/v6/assistant/
#define PAYLOAD_LENGTH 150

/*------------GLOBAL VARIABLES--------------*/
FirebaseData firebaseData;
WiFiUDP Udp;
//IPAddress left_ip(10, 0, 0, 49);  // Static IP of left glove: 10.32.110.77
IPAddress broadcastAddr(255, 255, 255, 255); // broadcast address 
unsigned int left_port = 2390;
unsigned int localPort = 2390;

// Last time the IMU sensors were read, in ms
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;
uint32_t minutes = 0;
uint32_t hours = 0;

// Variables for pressure sensor
const int FSR_PIN = A0; // Pin connected to FSR/resistor divider
int fsrADC = 0;

#define PRESSURE_THRESHOLD 80
#define ACCELERATION_MAX 2
#define LED_A 2

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

  if (sqrt(sq(ax) + sq(ay) + sq(az)) > ACCELERATION_MAX) {
    digitalWrite(LED_A, HIGH);
  }
  else {
    digitalWrite(LED_A, LOW);
  }
  
  uint32_t seconds = currentMillis / 1000;

  // Reset if close to overflow (every 30 mins)
  uint32_t microseconds = TC4->COUNT32.COUNT.reg / 48;
  if(microseconds >= 60000000) {
    minutes++;
    hours = minutes/60;
    resetClock();
  }
  currentMillis %= 1000;
  seconds %= 60;
  minutes %= 60;
  
  String ms = currentMillis >= 10? (currentMillis >= 100? String(currentMillis) : "0" + String(currentMillis)) : "00" + String(currentMillis);
  String secs = seconds >= 10? String(seconds) : "0" + String(seconds);
  String mins = minutes >= 10? String(minutes) : "0" + String(minutes);
  String hrs = hours >= 10? String(hours) : "0" + String(hours);
  String time_string = hrs + ':' + mins + ':' + secs + ':' + ms;

  StaticJsonDocument<PAYLOAD_LENGTH> doc;
  doc["ax"] = ax;
  doc["ay"] = ay;
  doc["az"] = az;
  doc["gx"] = gx;
  doc["gy"] = gy;
  doc["gz"] = gz;
  doc["timestamp"] = time_string;

  String jsonString;
  serializeJson(doc, jsonString);
  //Serial.println("Pushing to firebase: " + jsonString);
  String title = time_string + "/rightGlove";
  if(!Firebase.pushJSON(firebaseData, title, jsonString)) {
    //Serial.println("Failed to push " + time_string + " to firebase");
  }
}

void setup() {
  // Initialize serial monitor
  // (Used for debugging purposes)
  //Serial.begin(9600);
  //while (!Serial);

  // Set A0 and input pin for pressure sensor
  pinMode(FSR_PIN, INPUT);
  pinMode(LED_A, OUTPUT);
  
  // Connect to Wi-Fi network
  //Serial.println("Connecting to Wi-Fi");
  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    // status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    //status = WiFi.beginEnterprise(WIFI_SSID, WIFI_USER, WIFI_PASSWORD);
    status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    //Serial.println("Waiting to connect...");
    //Serial.println(status);
    //Serial.println(WIFI_SSID);
    //Serial.println(WIFI_USER);
    //Serial.println(WIFI_PASSWORD);
    delay(300);
  }
  
  //Serial.println();
  //Serial.print("Connected with IP: ");
  //Serial.println(WiFi.localIP());
  //Serial.println();

  // Connect to Firebase
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH, WIFI_SSID, WIFI_PASSWORD);
  Firebase.reconnectWiFi(true);

  // Open socket on local port for listening to incoming UDP packets
  if(!Udp.begin(localPort)) {
    //Serial.println("Error initiliazing UDP module - no sockets available");
  }

  //Serial.println("-----------------------------------");
  //Serial.println("----------Begin Sampling-----------");
  //Serial.println("-----------------------------------");
  //Serial.println();

  // Initialize IMU sensors
  if (!IMU.begin()) {
    //Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // DEBUG INFO: Sample rate
  // Accelerometer sample rate in Hz and G's
  //Serial.print("Accelerometer sample rate = ");
  //Serial.print(IMU.accelerationSampleRate());
  //Serial.println(" Hz");
  //Serial.println("Acceleration in G's");
  //Serial.println();
  
  // Gyroscope sample rate in Hz and degrees/s
  //Serial.print("Gyroscope sample rate = ");
  //Serial.print(IMU.gyroscopeSampleRate());
  //Serial.println(" Hz");
  //Serial.println("Gyroscope in degrees/second");
  //Serial.println();

  // Send start signal to left glove and wait for ack packet
  while(1) {
    if(!Udp.beginPacket(broadcastAddr, left_port)) {
      //Serial.println("Udp.beginPacket(): Error with supplied IP or port of remote host");
    }
    char buf[] = "start";
    Udp.write(buf);
    if(!Udp.endPacket()) {
      //Serial.println("Error sending UDP Packet to left glove");
    };
    if(Udp.parsePacket())
    {
      setupClock();
      break;
    }; 
    delay(1000);
    //Serial.println("Awaiting acknowledgement from left glove");
  }
}

void setupClock() {   
  //Serial.println("Going to setup clock");
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
  TC4->COUNT32.CTRLBSET.reg = TC_CTRLBSET_CMD_RETRIGGER;   // Retrigger the TC4 timer
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);   // Wait for syncrhonization
}

void loop() {
  // configure for # of iterations you'd like
  while(fsrADC >= PRESSURE_THRESHOLD) {
    //Serial.println("Sufficient force:" + String(fsrADC));
    uint32_t microseconds = TC4->COUNT32.COUNT.reg / 48;
    currentMillis = microseconds / 1000;
    //Serial.println("microseconds:" + String(microseconds));
    //Serial.println(String(currentMillis));
    // Read from sensors every 250ms
    if (currentMillis%250 <= 10) {
      fsrADC = analogRead(FSR_PIN);
      currentMillis -= currentMillis%250;
      //Serial.println(String(currentMillis));
      // push reading from this glove to firebase
      updateIMUReadings();
    }
  }

  // Do not do anything while there is no pressure applied
  while(fsrADC < PRESSURE_THRESHOLD) {
    //Serial.println("Not enough force" + String(fsrADC));
    fsrADC = analogRead(FSR_PIN);
  }
}
