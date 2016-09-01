/** 
  *Name of file:mesh_gateway.ino
  *Purpose: Research. Baxter robot IMU. Center for Automation Technologies and Systems (CATS) at-
  *         Rensselaer Polytechnic Institute
  *Credit: Andrew Cunningham, Jihoo Park
  *Used: RF24Mesh by TMRh20
  *
  **/
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU9150 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU9150.h"


#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include <SPI.h>
#include <EEPROM.h>
//#include <printf.h>

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9150 accelgyro;

int ax, ay, az;
int gx, gy, gz;
int mx, my, mz;

int IMU1[7] = {0,0,0,0,0,0,0};

/**** Configure the nrf24l01 CE and CS pins ****/
RF24 radio(8,7);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

/**
 * User Configuration: nodeID - A unique identifier for each radio. Allows addressing
 * to change dynamically with physical changes to the mesh.
 *
 * In this example, configuration takes place below, prior to uploading the sketch to the device
 * A unique value from 1-255 must be configured for each node.
 * This will be stored in EEPROM on AVR devices, so remains persistent between further uploads, loss of power, etc.
 *
 **/
#define nodeID 1


uint8_t displayTimer = 0;

void setup() {

  Serial.begin(57600);

// initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    accelgyro.setFullScaleGyroRange(0);
    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU9150 connection successful" : "MPU9150 connection failed");
  
  //printf_begin();
  // Set the nodeID manually
  mesh.setNodeID(nodeID);
  // Connect to the mesh
  Serial.println(F("Connecting to the mesh..."));
  mesh.begin(91);//set chennal to 90
}



void loop() {
  
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  IMU1[0] = ax; IMU1[1] = ay; IMU1[2] = az; IMU1[3] = gx; IMU1[4] = gy; IMU1[5] = gz; IMU1[6] = ax;
  
  mesh.update();

  // Send to the master node every .05 second
 // if (millis() - displayTimer >= 50) {
   // displayTimer = millis();

    // Send an 'L' type message containing the current IMU1()
    if (!mesh.write((byte *)&IMU1, 'L', sizeof((byte *)&IMU1)*7)) {

      // If a write fails, check connectivity to the mesh network
      if ( ! mesh.checkConnection() ) {
        //refresh the network address
        //Serial.println("Renewing Address");
        mesh.renewAddress();
      } else {
        //Serial.println("Send fail, Test OK");
      }
    } else {
      //Serial.println("Send OK: ");
      //Serial.print(IMU1[0]);Serial.print(" ");Serial.print(IMU1[1]);Serial.print(" ");Serial.print(IMU1[2]);Serial.print(" ");Serial.println(IMU1[3]);

      
    }
   delay(2);
  //}

}



