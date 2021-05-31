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
//    - timestamp -> string = 13 bytes (00:00:00.000)
//    - coordinates for both sensors (x,y,z) -> 112 bytes
//    - Total = 125 bytes
// Calculated at: https://arduinojson.org/v6/assistant/
#define PAYLOAD_LENGTH 125

#define SERVICE_UUID "2A5D"
#define CHARACTERISTIC_UUID "190313ee-5f69-489f-a812-b93eeb413329"

/*------------GLOBAL VARIABLES--------------*/
FirebaseData firebaseData;
BLEDevice peripheral;

// Variables for storing previous accelerometer readings
float ax_old = 0;
float ay_old = 0;
float az_old = 0;

// Variables for storing previous gyroscope readings
float gx_old = 0;
float gy_old = 0;
float gz_old = 0;

// Last time the IMU sensors were read, in ms
long previousMillis = 0;

/*------------FORWARD DECLARATIONS--------------*/
void updateIMUReadings();
bool setupBLE();
bool connectPeripheral();
void readBLE();

bool setupBLE() {
  // initialize the BLE hardware
  BLE.begin();
  Serial.println("BLE Initialized as Central");
  
  // start scanning for peripherals
  BLE.scanForUuid(SERVICE_UUID);
  int max_attempts = 20; // i.e. try for 2s (TOD0: Determine if sufficient)
  for(int i = 0; i < max_attempts; i++) {
    delay(100); // delay 100ms (TODO: determine appropriate delay interval)
    peripheral = BLE.available();
    if(peripheral){
      Serial.print("Found ");
      Serial.print(peripheral.address());
      Serial.print(" '");
      Serial.print(peripheral.localName());
      Serial.print("' ");
      Serial.print(peripheral.advertisedServiceUuid());
      Serial.println();
      if (peripheral.localName() != "IMUMonitor") {
        return false;
      }
      // stop scanning
      BLE.stopScan();
      return true;
    }
  }
  return false;
}

bool connectPeripheral() {
  if(peripheral.connected()) {
    Serial.println("Already connnected");
    return true;
  } else if (peripheral.connect()) {
    Serial.println("Connected to peripheral");
    return true;
  }
  Serial.println("Failed to connect to peripheral!");
  return false;
}

void readBLE() {
  // retrieve the IMU characteristic
  BLECharacteristic imuCharacteristic = peripheral.characteristic(CHARACTERISTIC_UUID);

  // Validation
  if (!imuCharacteristic) {
    Serial.println("Peripheral does not have IMU characteristic!");
    peripheral.disconnect();
    return;
  } else if (!imuCharacteristic.canRead()) {
    Serial.println("Peripheral does not have a readable LED characteristic!");
    peripheral.disconnect();
    return;
  }

  // Read data
  char buf[PAYLOAD_LENGTH + 1];
  imuCharacteristic.readValue(buf, PAYLOAD_LENGTH);
  buf[PAYLOAD_LENGTH] = '\0';
  Serial.println(buf);

  // Push to firebase
  Firebase.pushJSON(firebaseData, "Gyroscope/Sample", buf);
}

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

  unsigned long currentMillis = millis();
  unsigned long seconds = currentMillis / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  currentMillis %= 1000;
  seconds %= 60;
  minutes %= 60;
  hours %= 24;

  String ms = currentMillis > 10? (currentMillis > 100? String(currentMillis) : "0" + String(currentMillis)) : "00" + String(currentMillis);
  String secs = seconds > 10? String(seconds) : "0" + String(seconds);
  String mins = minutes > 10? String(minutes) : "0" + String(minutes);
  String hrs = hours > 10? String(hours) : "0" + String(hours);
  String time_string = hrs + ':' + mins + ':' + secs + '.' + ms;
  
  if(ax != ax_old || ay != ay_old || az != az_old ||
     gx != gx_old || gy != gy_old || gz != gz_old) {
    Serial.println("Reading changed");
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
    Serial.println("Pushing to firebase: " + jsonString);
    Firebase.pushJSON(firebaseData, "Gyroscope/Sample", jsonString);
  }
}

void setup() {
  // Initialize serial monitor
  // (Used for debugging purposes)
  Serial.begin(9600);
  while (!Serial);
  Serial.println();

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

  // Connect to other glove over BLE
  //setupBLE();

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
}

void loop() {
  int i = 0; 
  // configure for # of iterations you'd like
  while(i < 1) {
    long currentMillis = millis();
    // Read from sensors every 200ms
    if (currentMillis - previousMillis >= 200) {
      previousMillis = currentMillis;
      // push reading from this glove to firebase
      updateIMUReadings();
      // push reading from other glove to firebase
      if(connectPeripheral()){
        readBLE();
      }
    }
    i++;
  }
}
