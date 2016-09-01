/** 
  *Name of file:mesh_gateway.ino
  *Purpose: Research. Baxter robot IMU. Center for Automation Technologies and Systems (CATS) at-
  *         Rensselaer Polytechnic Institute
  *Credit: Andrew Cunningham, Jihoo Park
  *Used: RF24Mesh by TMRh20
  *
  **/
  
  
#include "RF24Network.h"
#include "RF24.h"
#include "RF24Mesh.h"
#include <SPI.h>
//Include eeprom.h for AVR (Uno, Nano) etc. except ATTiny
#include <EEPROM.h>

#define DEBUG 0 

int IMU1[7] = {0,0,0,0,0,0,0};
int IMU2[7] = {0,0,0,0,0,0,0};


//used to communicate with Robot Raconteur through serial
byte byteBuffer[14];
byte dat[14];//payload size 14


/***** Configure the chosen CE,CS pins *****/
RF24 radio(8,7);
RF24Network network(radio);
RF24Mesh mesh(radio,network);

uint32_t displayTimer = 0;

void setup() {
  Serial.begin(57600);

  // Set the nodeID to 0 for the master node
  mesh.setNodeID(0);
  Serial.println("Gateway");
  Serial.println(mesh.getNodeID());
  // Connect to the mesh
  mesh.begin(91);//set channel: baxter 1==90/ Baxter2==91 /Baxter3==92

}


void loop() {    

  // Call mesh.update to keep the network updated
  mesh.update();
  
  // In addition, keep the 'DHCP service' running on the master node so addresses will
  // be assigned to the sensor nodes
  mesh.DHCP();
    // Send to the master node every .05 second
//if (millis() - displayTimer >= 500) {
//displayTimer = millis();
  // Check for incoming data from the sensors
  delay(50);
  if(network.available()){
    RF24NetworkHeader header;
    network.peek(header);
    delay(2);
    switch(header.type){
      // Display the incoming IMU data from the sensor nodes
      case 'L': network.read(header,&dat,sizeof(dat));delay(10); byteAToIntA(dat,IMU1); break;
      case 'R': network.read(header,&dat,sizeof(dat));delay(10); byteAToIntA(dat,IMU2); break;
      default: network.read(header,0,0); //Serial.println(header.type);break;
    }
  }
//}
  if(DEBUG){
    Serial.println(" ");
    Serial.println(F("********Assigned Addresses********"));
     for(int i=0; i<mesh.addrListTop; i++){
       Serial.print("IMU NodeID: ");
       Serial.println(mesh.addrList[i].nodeID);
       if((mesh.addrList[i].nodeID==1)){ 
       Serial.print(IMU1[0]);Serial.print(" ");Serial.print(IMU1[1]);Serial.print(" ");Serial.print(IMU1[2]);Serial.print(" ");Serial.println(IMU1[3]);
        }
       else if((mesh.addrList[i].nodeID==2)){ 
       Serial.print(IMU2[0]);Serial.print(" ");Serial.print(IMU2[1]);Serial.print(" ");Serial.print(IMU2[2]);Serial.print(" ");Serial.println(IMU2[3]);
        }

     }
    Serial.println(F("**********************************"));
  }
  
//read serial commands 
  int val = 0;
  int channel=0;
  int static s = -1;
  if (Serial.available() >0) {
    /* whatever is available from the serial is read here    */
    val = Serial.read();
    
    if(DEBUG){
      Serial.print("val is : ");
      Serial.println(val);
    }
    
    if(DEBUG){
      Serial.print("s is : ");
      Serial.println(s);
    }
    
    /* This part basically implements a state machine that 
       reads the serial port and makes just one transition 
       to a new state, depending on both the previous state 
       and the command that is read from the serial port. 
       Some commands need additional inputs from the serial 
       port, so they need 2 or 3 state transitions (each one
       happening as soon as anything new is available from 
       the serial port) to be fully executed. After a command 
       is fully executed the state returns to its initial 
       value s=-1                                            */

    switch (s) {

      /* s=-1 means NOTHING RECEIVED YET ******************* */
      case -1:      

      /* calculate next state                                */
      if (val>47 && val<90) {
    /* the first received value indicates the mode       
           49 is ascii for 1, ... 90 is ascii for Z          
           s=0 is change-pin mode;
           s=10 is DI;  s=20 is DO;  s=30 is AI;  s=40 is AO; 
           s=90 is query script type (1 basic, 2 motor);
           s=340 is change analog reference;
           s=400 example echo returning the input argument;
                                                             */
        s=10*(val-48);
      }
      
      /* the following statements are needed to handle 
         unexpected first values coming from the serial (if 
         the value is unrecognized then it defaults to s=-1) */
      if ((s>40 && s<90) || (s>90 && s!=340 && s!=400)) {
        s=-1;
      }

      /* the break statements gets out of the switch-case, so
      /* we go back and wait for new serial data             */
      break; /* s=-1 (initial state) taken care of           */


     
   
 
      /*s=20 means send IMU data*/
      case 20:
       if(DEBUG){
          Serial.print("val is : ");
          Serial.println(val);
        }
      if (val == 33){
        intToBytes(IMU1,7,byteBuffer);
        Serial.write(byteBuffer,14);
        } 
      else if(val == 34){
        intToBytes(IMU2,7,byteBuffer);
        Serial.write(byteBuffer,14);
        }
      s=-1;
      break; /* s=0 taken care of                            */
     

 

      /* ******* UNRECOGNIZED STATE, go back to s=-1 ******* */
      
      default:
      /* we should never get here but if we do it means we 
         are in an unexpected state so whatever is the second 
         received value we get out of here and back to s=-1  */
      
      s=-1;  /* go back to the initial state, break unneeded */



    } /* end switch on state s                               */
  }  
}
// Fill an int array with the contents of a byte array
// assume that the size of the int array is 7
// and that the size of the byte array is 14
void byteAToIntA(byte bArray[], int iArray[]){
  for(int i =0; i < 7; i++){
   iArray[i] = bArray[2*i] | bArray[(2*i)+1] << 8; 
  }
}

// convert an array of ints to an array of bytes
// specify size (number of ints) in input as sizeOfIntArray
void intToBytes(int input[], int sizeOfIntArray, byte buf[]){
  for(int i = 0; i < sizeOfIntArray;i++){
    buf[i*2] = (byte) input[i];
    buf[i*2+1] = (byte) (input[i] >> 8);
  }
  return;
}
