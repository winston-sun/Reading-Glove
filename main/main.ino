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

#define WIDTH 176
#define HEIGHT 100

int value_cm;
int oldval;

float x, y, z;
bool orientation = false; // 0 - facing up; 1 - facing down;
bool stable = false; // 0 - not stable; 1 - stable;

bool commandRecv = false; // flag used for indicating receipt of commands from serial port
bool liveFlag = false; // flag as true to live stream raw camera bytes, set as false to take single images on command
bool captureFlag = false;

// Image buffer;
byte image[176 * 144 * 2]; // QCIF: 176x144 x 2 bytes per pixel (RGB565)
byte image_greyscale[176 * 144]; // QCIF: 176x144 x 1 bytes per pixel (Greyscale)
int8_t image_sent[WIDTH * HEIGHT];
bool grayscale = true;
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
BLECharacteristic imageCharacteristic("19B10019-E8F2-537E-4F6C-D104768A121",  BLERead|BLENotify, 512); 

// FSM
typedef enum
{
    IDLE,
    READING, // Camera capture
    SENDING, // BLE
    MEASURING, // ultrasonic sensor by distance
    FAULT
} state_E;

typedef enum
{
    NONE,
    START, // start the application, button pressed
    CAPTURED, //camera captures the image
    SENT, // BLE sends the image
    TRIGGER, // ultrasonic sensor by distance
    STOP, // stop the application, button pressed
    GENERIC_FAULT
} input_E;

state_E currState = IDLE; //set default current state to be IDLE
volatile input_E input = NONE;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);

  initializeShield();

  // Initialize the OV7675 camera
  if (!grayscale) {
    if (!Camera.begin(QCIF, RGB565, 1, OV7675)) {
      Serial.println("Failed to initialize camera");
      while (1);
    }
  }
  else {
    if (!Camera.begin(QCIF, GRAYSCALE, 1, OV7675)) {
      Serial.println("Failed to initialize camera");
      while (1);
    }
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
      //Serial.println("\nNot facing down");
    }

    if (x > -0.2 && x < 0.2 && y > -0.2 && y < 0.2) {
      stable = true;
    }
    else {
      stable = false;
      //Serial.println("\nNot stable");
    }
  }
  
  
  // if 100ms have passed, check the battery level:
  if ((currentMillis - previousMillis >= 100) && value_cm < 26 && orientation && stable && captureFlag) {
    previousMillis = currentMillis;
    
    if(!grayscale) {
      Camera.readFrame(image);
      Serial.println("\nImage capture...");
      //delay(3000);
      
      
      for (int i = 0; i < bytesPerFrame - 1; i += 2) {
        Serial.print("0x");
        Serial.print(image[i+1], HEX);
        Serial.print(image[i], HEX);
        if (i != bytesPerFrame - 2) {
          Serial.print(", ");
        }
      }
      Serial.println();
    }
    else {
      Camera.readFrame(image_greyscale);
      Serial.println("\nImage capture...");
      
      int min_x = (176 - WIDTH) / 2;
      int min_y = (144 - HEIGHT) / 2;
      int index = 0;

      // Crop 120x100 image. This lowers FOV, ideally we would downsample but this is simpler. 
      for (int y = min_y; y < min_y + HEIGHT; y++) {
        for (int x = min_x; x < min_x + WIDTH; x++) {
          image_sent[index++] = static_cast<int8_t>(image_greyscale[(y * 176) + x] - 128); // convert TF input image to signed 8-bit
        }
      }
      
      for (int i = 0; i < WIDTH*HEIGHT; i ++) {
        Serial.print(image_sent[i]);
        Serial.print(", ");
      }
      Serial.print("\n");
    }
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

void CameraHandler(void)
{
    int min_x = (176 - 90) / 2;
    int min_y = (144 - 90) / 2;
    int index = 0;

    Camera.readFrame(image);
    //Serial.println("\nImage data will be printed out in 3 seconds...");
    Serial.println("\nImage captured");

    //uint8_t bt = 0;
    //uint8_t first = 1;
    
    int i = 0;

    // Crop 120x100 image. This lowers FOV, ideally we would downsample but this is simpler. 
    for (int y = min_y; y < min_y + HEIGHT; y++) {
      for (int x = min_x; x < min_x + WIDTH; x++) {
        image_sent[index++] = static_cast<int8_t>(image[(y * 176) + x] - 128); // convert TF input image to signed 8-bit
      }
    }
    
    for (int i = 0; i < WIDTH*HEIGHT; i ++) {
      Serial.print(image_sent[i]);
      Serial.print(", ");
    }
    Serial.print("\n");
    //int a = (index-1)/8;
    //Serial.print(a);
    //Serial.print("\n");
    //delay(3000);
    for (i = 0; i < 90 * 90; i++) {
      Serial.print("0x");

      Serial.print(image_sent[i] < 16 ? "0" : "");
      Serial.print(image_sent[i], HEX);
      Serial.print(", ");
      //Serial.print(image_sent[i], HEX);
    }
    input = CAPTURED;
    return;
}

void BLEHandler(void) {

    // poll for Bluetooth® Low Energy events
    // BLE.poll();

    // wait for a Bluetooth® Low Energy central
    BLEDevice central = BLE.central();

    // if a central is connected to the peripheral:
    while (!central) {
        central = BLE.central();  
    }

    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());

    int pckt_len = 512;
    /*
    byte image_seg0[pckt_len];
    byte image_seg1[pckt_len];
    byte image_seg2[pckt_len];
    byte image_seg3[pckt_len];
    byte image_seg4[pckt_len];
    byte image_seg5[pckt_len];
    byte image_seg6[pckt_len];
    byte image_seg7[pckt_len];
    byte image_seg8[pckt_len];
    byte image_seg9[pckt_len];
    byte image_seg10[pckt_len];
    byte image_seg11[pckt_len];
    byte image_seg12[pckt_len];
    byte image_seg13[pckt_len];
    byte image_seg14[pckt_len];
    byte image_seg15[420]; // 90*90-512*15 = 420
    */
    /*
    for(int j = 0; j < 512; j++) {
      image_seg1[j] = image_sent[j];
      image_seg2[j] = image_sent[half_len + j];
      image_seg3[j] = image_sent[half_len*2 + j];
      image_seg4[j] = image_sent[half_len*3 + j];
      image_seg5[j] = image_sent[half_len*4 + j];
      image_seg6[j] = image_sent[half_len*5 + j];
    }
    */

    Serial.println("\nBLE Start");

    int error = 0;
    for (int i = 0; i < 16; i++) {
        if (i != 15) {
          if (imageCharacteristic.writeValue(image_sent+i*512, pckt_len) == 0) {
              error += i;
              Serial.print("\n BLE Send Error");
          }
        }
        else if (i == 15) {
          if (imageCharacteristic.writeValue(image_sent+i*512, 420) == 0) {
              error += i;
              Serial.print("\n BLE Send Error");
          }
        }
    }

    Serial.println("\nBLE End");
    /*
    if (imageCharacteristic.writeValue(image_seg1, half_len) == 1) {
        Serial.print("\n1 Success");
    }
    delay(1000);
    if (imageCharacteristic.writeValue(image_seg2, half_len)) {
        Serial.print("\n2 Success");
    }
    delay(1000);
    if (imageCharacteristic.writeValue(image_seg3, half_len)) {
        Serial.print("\n3 Success");
    }
    delay(1000);
    if (imageCharacteristic.writeValue(image_seg4, half_len)) {
        Serial.print("\n4 Success");
    }
    delay(1000);
    if (imageCharacteristic.writeValue(image_seg5, half_len)) {
        Serial.print("\n5 Success");
    }
    delay(1000);
    if (imageCharacteristic.writeValue(image_seg6, half_len)) {
        Serial.print("\n6 Success");
    }
    Serial.println();
    */

    input = SENT;
    return;
}

void UltrasonicHandler(void) {
    // TODO while(1) { }
    value_cm = getdistance(TrigPin,EchoPin);
    while (value_cm < 24) {}
    input = TRIGGER;
    return;
}

void stateMachine() {
    switch(currState) {	
        case IDLE: {
          if (input == START) {
            currState = READING;
          }
        }
        break;
        case READING: {
          CameraHandler(); 
          if (input == STOP) {
            currState = IDLE;
          }
          else if (input == CAPTURED) {
            currState = SENDING;
          }
          else {
            Serial.print("here1");
          }
        }
        break;
        case SENDING: {
          BLEHandler();
          if (input == STOP) {
            currState = IDLE;
          }
          else if (input == SENT) {
            currState = MEASURING;
          }
          else {
            Serial.print("here2");
          }
        }
        break;
        case MEASURING: {
          UltrasonicHandler();
          if (input == STOP) {
            currState = IDLE;
          }
          else if (input == TRIGGER) {
            currState = READING;
          }
          else {
            Serial.print("here3");
          }
        }
        break;
        case FAULT: {
          if (input == STOP) {
            currState = IDLE;
          }
        }
        break;
        default: 
          break;
    }
    input = NONE;
}


