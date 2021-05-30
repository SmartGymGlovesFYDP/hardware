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

void setup() {
  // Initialize serial monitor
  // (Used for debugging purposes)
  Serial.begin(9600);
  while (!Serial);
  Serial.println();

  // Connect to Wi-Fi network
  Serial.print("Connecting to Wi-Fi");
  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Waiting to connect...\n");
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
  if(setupBLE()) {
    readBLE();
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
}

bool setupBLE() {
     // initialize the BLE hardware
    BLE.begin();
    Serial.println("BLE Initialized as Central");
  
    // start scanning for peripherals
    while(1){
      BLE.scanForUuid("2A5D");
      peripheral = BLE.available();
      if(peripheral){
        Serial.print("Found ");
        Serial.print(peripheral.address());
        Serial.print(" '");
        Serial.print(peripheral.localName());
        Serial.print("' ");
        Serial.print(peripheral.advertisedServiceUuid());
        Serial.println();
        break;
     }
    }
    //BLE.scan();
    
    // check if a peripheral has been discovered
    peripheral = BLE.available();
  
    if (peripheral) {
      // discovered a peripheral, print out address, local name, and advertised service
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
      
    } else {
      Serial.println("Could not find IMUMonitor service UUID");
    }
    return false;
}

void readBLE() {
  // connect to the peripheral
  Serial.println("Connecting to peripheral ...");

  if (peripheral.connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
  Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    Serial.println("Attributes discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

  // retrieve the LED characteristic
  BLECharacteristic imuCharacteristic = peripheral.characteristic(CHARACTERISTIC_UUID);

  if (!imuCharacteristic) {
    Serial.println("Peripheral does not have IMU characteristic!");
    peripheral.disconnect();
    return;
  } else if (!imuCharacteristic.canRead()) {
    Serial.println("Peripheral does not have a readable LED characteristic!");
    peripheral.disconnect();
    return;
  }

  if (peripheral.connected()) {
    char buf[PAYLOAD_LENGTH];
    imuCharacteristic.readValue(buf, PAYLOAD_LENGTH);
    Serial.println(buf);
  }

  Serial.println("Peripheral disconnected");
}

void loop() {
  int i = 0; // configure for # of iterations you'd like
  while(i < 1) {
    long currentMillis = millis();
    // Read from sensors every 200ms
    if (currentMillis - previousMillis >= 200) {
      previousMillis = currentMillis;
      updateIMUReadings();
    }
    i++;
  }
  while(1);
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

    String jsonString;
    serializeJson(doc, jsonString);
    Firebase.pushJSON(firebaseData, "Gyroscope/Sample", jsonString);
  }
}
