#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

#define CONVERT_G_TO_MS2    9.80665f
#define FREQUENCY_HZ        119
#define INTERVAL_MS         (1000 / (FREQUENCY_HZ + 1))

float accX, accY, accZ, gyrX, gyrY, gyrZ; // max. 5 characters String each (if negative)
const String del = ";"; // 1 character
const int strLen = 10 + 6 * 5 + 6; // + 3 for 3 delimiters
unsigned long last_interval_ms = 0;

// check https://www.uuidgenerator.net/ to generate your own unique UUIDs
BLEService accelService("9a412dc6-cb80-4e8c-a9b9-356e5e867a0b");
BLEStringCharacteristic accelCharacteristic("9a412dc6-cb80-4e8c-a9b9-356e5e867a0b", BLERead | BLENotify, strLen);

void setup() {
  IMU.begin();
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);

  if (!BLE.begin()) {
    Serial.println("BLE failed to Initiate");
    delay(500);
    while (1);
  }

  BLE.setLocalName("Arduino Accelerometer");
  BLE.setAdvertisedService(accelService);
  accelService.addCharacteristic(accelCharacteristic);
  BLE.addService(accelService);
  accelCharacteristic.writeValue("1111111111;-0.00;-0.00;-0.00;-0.00;-0.00;-0.00");
  BLE.advertise();

  Serial.println("Bluetooth device is now active, waiting for connections...");
}

void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);
    while (central.connected()) {
        last_interval_ms = millis();
        read_Acc_Gyr();
  
        String msg = get_message();
        accelCharacteristic.writeValue(msg);
  
        print_to_serial();
      
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
  Serial.print("Disconnected from central: ");
  Serial.println(central.address());
}

void read_Acc_Gyr() {
  if (IMU.accelerationAvailable()&& IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(accX, accY, accZ);
    IMU.readGyroscope(gyrX, gyrY, gyrZ);

    accX *= CONVERT_G_TO_MS2;
    accY *= CONVERT_G_TO_MS2;
    accZ *= CONVERT_G_TO_MS2;
    gyrX *= CONVERT_G_TO_MS2;
    gyrY *= CONVERT_G_TO_MS2;
    gyrZ *= CONVERT_G_TO_MS2;
  }
}

void print_to_serial() {
  Serial.print(last_interval_ms); Serial.print('\t');
  Serial.print(accX); Serial.print('\t');
  Serial.print(accY); Serial.print('\t');
  Serial.print(accZ); Serial.print('\n');
  Serial.print(gyrX); Serial.print('\t');
  Serial.print(gyrY); Serial.print('\t');
  Serial.print(gyrZ); Serial.print('\n');
}

String get_message() {
  return String(last_interval_ms) + del + String(accX) + del + String(accY) + del + String(accZ)+del+String(gyrX) + del + String(gyrY) + del + String(gyrZ); 
}
