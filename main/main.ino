/*
  Active Learning Labs
  Harvard University 
  tinyMLx - OV7675 Camera Test

*/

#include <TinyMLShield.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>


const int ledPin = LED_BUILTIN; // set ledPin to on-board LED
const int buttonPin = 4; // set buttonPin to digital pin 4

// ultrasonic
#define TrigPin 6
#define EchoPin 5

int value_cm;
int oldval;

float x, y, z;
bool orientation = false; // 0 - facing up; 1 - facing down;
bool stable = false; // 0 - not stable; 1 - stable;

bool commandRecv = false; // flag used for indicating receipt of commands from serial port
bool liveFlag = false; // flag as true to live stream raw camera bytes, set as false to take single images on command
bool captureFlag = false;

// Image buffer;
byte image[176 * 144]; // QCIF: 176x144 x 2 bytes per pixel (RGB565)
int8_t image_sent[120 * 100];
//int min_x = (176 - 100) / 2;
//int min_y = (144 - 100) / 2;
int bytesPerFrame;
long previousMillis = 0;  // last time the battery level was checked, in ms

BLEService imgService("19B10010-E8F2-537E-4F6C-D104768A1214"); // create service

// create switch characteristic and allow remote device to read and write
BLEByteCharacteristic startCharacteristic("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEByteCharacteristic stopCharacteristic("19B10012-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
// create button characteristic and allow remote device to get notifications
BLEByteCharacteristic countCharacteristic("19B10013-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEByteCharacteristic widthCharacteristic("19B10014-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEByteCharacteristic heightCharacteristic("19B10015-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEByteCharacteristic resCharacteristic("19B10016-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEByteCharacteristic formatCharacteristic("19B10017-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEByteCharacteristic fpsCharacteristic("19B10018-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);


void setup() {
  Serial.begin(9600);
  while (!Serial);

  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);

  initializeShield();

  // Initialize the OV7675 camera
  if (!Camera.begin(QCIF, GRAYSCALE, 1, OV7675)) {
    Serial.println("Failed to initialize camera");
    while (1);
  }
  bytesPerFrame = Camera.width() * Camera.height() * Camera.bytesPerPixel();

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  // Initialize IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    while (1);
  }

  // set the local name peripheral advertises
  BLE.setLocalName("imgService");
  // set the UUID for the service this peripheral advertises:
  BLE.setAdvertisedService(imgService);

  // add the characteristics to the service
  imgService.addCharacteristic(startCharacteristic);
  imgService.addCharacteristic(stopCharacteristic);
  imgService.addCharacteristic(countCharacteristic);
  imgService.addCharacteristic(widthCharacteristic);
  imgService.addCharacteristic(heightCharacteristic);
  imgService.addCharacteristic(resCharacteristic);
  imgService.addCharacteristic(formatCharacteristic);
  imgService.addCharacteristic(fpsCharacteristic);

  // add the service
  BLE.addService(imgService);

  startCharacteristic.writeValue(0);
  stopCharacteristic.writeValue(0);
  countCharacteristic.writeValue(0);
  widthCharacteristic.writeValue(Camera.width());
  heightCharacteristic.writeValue(Camera.height());
  resCharacteristic.writeValue(QCIF);
  formatCharacteristic.writeValue(RGB565);
  fpsCharacteristic.writeValue(1);

  BLE.advertise();

  Serial.println("Welcome to the Reading Glove application\n");
}

void loop() {
  // poll for Bluetooth® Low Energy events
  BLE.poll();

  long currentMillis = millis();

  bool clicked = readShieldButton();
  if (clicked) {
    if (!captureFlag) {
      captureFlag = true;
    }
  }
  
  value_cm = getdistance(TrigPin,EchoPin);

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    if (z > -1.1 && z < -0.85) {
      orientation = true;
    }
    else {
      orientation = false;
      Serial.println("\nNot facing down");
    }

    if (x > -0.2 && x < 0.2 && y > -0.2 && y < 0.2) {
      stable = true;
    }
    else {
      stable = false;
      Serial.println("\nNot stable");
    }
  }
  
  
  // if 100ms have passed, check the battery level:
  if ((currentMillis - previousMillis >= 100) && value_cm < 26 && orientation && stable && captureFlag) {
    previousMillis = currentMillis;
    Camera.readFrame(image);
    Serial.println("\nImage capture...");
    //delay(3000);
    
    /*
    for (int i = 0; i < bytesPerFrame - 1; i += 2) {
      Serial.print("0x");
      Serial.print(image[i+1], HEX);
      Serial.print(image[i], HEX);
      if (i != bytesPerFrame - 2) {
        Serial.print(", ");
      }
    }
    */

  
    int min_x = (176 - 120) / 2;
    int min_y = (144 - 100) / 2;
    int index = 0;

    // Crop 120x100 image. This lowers FOV, ideally we would downsample but this is simpler. 
    for (int y = min_y; y < min_y + 100; y++) {
      for (int x = min_x; x < min_x + 120; x++) {
        image_sent[index++] = static_cast<int8_t>(image[(y * 176) + x] - 128); // convert TF input image to signed 8-bit
      }
    }
    
    for (int i = 0; i < 120*100; i ++) {
      Serial.print(image_sent[i]);
      Serial.print(", ");
    }
    Serial.print("\n");
  }
}

float getdistance(int Trig,int Echo)
{
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  digitalWrite(Trig, LOW); 
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
    
  return float( pulseIn(Echo, HIGH) * 17 )/1000;  
  
}

