/*
  Active Learning Labs
  Harvard University 
  tinyMLx - OV7675 Camera Test

*/

#include <TinyMLShield.h>
#include <ArduinoBLE.h>


bool commandRecv = false; // flag used for indicating receipt of commands from serial port
bool liveFlag = false; // flag as true to live stream raw camera bytes, set as false to take single images on command
bool captureFlag = false;

// Image buffer;
byte image[176 * 144 * 1]; // QCIF: 176x144 x 2 bytes per pixel (RGB565)
byte image_send[90 * 88 / 8]; // 1 bit per pixel
int bytesPerFrame;

BLEService imageService("19B10010-E8F2-537E-4F6C-D104768A1214"); // create service
BLECharacteristic imageCharacteristic("19B10012-E8F2-537E-4F6C-D104768A1214",  BLERead|BLENotify, 90 * 88 / 8 / 2 / 3);


void setup() {
  Serial.begin(9600);
  while (!Serial);

  initializeShield();

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1);
  }

  // set the local name peripheral advertises
  BLE.setLocalName("ImageCapture");
  // set the UUID for the service this peripheral advertises:
  BLE.setAdvertisedService(imageService);

  // add the characteristics to the service
  imageService.addCharacteristic(imageCharacteristic);
  // add the service
  BLE.addService(imageService);

  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");


  // Initialize the OV7675 camera
  if (!Camera.begin(QCIF, GRAYSCALE, 1, OV7675)) {
    Serial.println("Failed to initialize camera");
    while (1);
  }
  bytesPerFrame = Camera.width() * Camera.height() * Camera.bytesPerPixel();

  Serial.println("Welcome to the OV7675 test\n");
  Serial.println("Available commands:\n");
  Serial.println("single - take a single image and print out the hexadecimal for each pixel (default)");
  Serial.println("live - the raw bytes of images will be streamed live over the serial port");
  Serial.println("capture - when in single mode, initiates an image capture");
}

void loop() {

  // poll for Bluetooth® Low Energy events
  BLE.poll();

  int i = 0;
  String command;

  bool clicked = readShieldButton();
  if (clicked) {
    if (!liveFlag) {
      if (!captureFlag) {
        captureFlag = true;
      }
    }
  }

  // Read incoming commands from serial monitor
  while (Serial.available()) {
    char c = Serial.read();
    if ((c != '\n') && (c != '\r')) {
      command.concat(c);
    } 
    else if (c == '\r') {
      commandRecv = true;
      command.toLowerCase();
    }
  }

  // Command interpretation
  if (commandRecv) {
    commandRecv = false;
    if (command == "live") {
      Serial.println("\nRaw image data will begin streaming in 5 seconds...");
      liveFlag = true;
      delay(5000);
    }
    else if (command == "single") {
      Serial.println("\nCamera in single mode, type \"capture\" to initiate an image capture");
      liveFlag = false;
      delay(200);
    }
    else if (command == "capture") {
      if (!liveFlag) {
        if (!captureFlag) {
          captureFlag = true;
        }
        delay(200);
      }
      else {
        Serial.println("\nCamera is not in single mode, type \"single\" first");
        delay(1000);
      }
    }
  }
  
  if (liveFlag) {
    Camera.readFrame(image);
    Serial.write(image, bytesPerFrame);
  }
  else {
    if (captureFlag) {
      captureFlag = false;

      int min_x = (176 - 90) / 2;
      int min_y = (144 - 88) / 2;
      int index = 0;

      Camera.readFrame(image);
      Serial.println("\nImage data will be printed out in 3 seconds...");

      uint8_t bt = 0;
      uint8_t first = 1;

      // Crop 96x96 image. This lowers FOV, ideally we would downsample but this is simpler. 
      for (int y = min_y; y < min_y + 88; y++) {
        for (int x = min_x; x < min_x + 90; x++) {
          if ((index % 8) == 0 && first) first = 0;
          else if ((index % 8) == 0) {
            image_send[index/8-1] = bt;
            bt = 0;
          }
          
          uint8_t b = (image[(y * 176) + x] < (0xFF / 2)) ? 0 : 1;
          bt = (bt << 1) | (b & 0x1);
          index++;
          //image_send[index++] = static_cast<int8_t>(data[(y * 176) + x] - 128); // convert TF input image to signed 8-bit
        }
      }
      int a = (index-1)/8;
      Serial.print(a);
      Serial.print("\n");
      //delay(3000);
      for (int i = 0; i < 90 * 88 / 8; i++) {
        //Serial.print("0x");
        Serial.print(image_send[i], HEX);

      }

      int half_len = 90 * 88 / 8 / 2 / 3;
      byte image_seg1[half_len];
      byte image_seg2[half_len];
      byte image_seg3[half_len];
      byte image_seg4[half_len];
      byte image_seg5[half_len];
      byte image_seg6[half_len];

      for(int j = 0; j < half_len; j++) {
        image_seg1[j] = image_send[j];
        image_seg2[j] = image_send[half_len + j];
        image_seg3[j] = image_send[half_len*2 + j];
        image_seg4[j] = image_send[half_len*3 + j];
        image_seg5[j] = image_send[half_len*4 + j];
        image_seg6[j] = image_send[half_len*5 + j];
      }



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
    }
  }
}