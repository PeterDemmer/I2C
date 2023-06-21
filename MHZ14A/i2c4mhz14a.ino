/***************************************************************************

   Copyright (c) 2021 Peter Demmer
   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.
  
   i2c4mhz14a.ino, Wire/I2C interface for MH-Z14A CO2-meter
   Raspberry Pi counterpart: i2c4mhz14a.py2
   
   Documentation a.o:
   https://forum.mysensors.org/topic/4355/mh-z14a-co2-sensor
   http://www.doctormonk.com/2018/03/review-and-test-of-mh-z14a-ndir-co2.html
   
   Wiring:
     co2serial RX - A2 - TX MH-Z14A
     co2serial TX - A3 - RX MH-Z14A
     Wire SDA - A4 - 3 RPi
     Wire SCL - A5 - 5 RPi
   
   Board - Arduino Pro Mini
   Port - /dev/ttyUSB0 (CP2102)
   Processor - ATmega328P (5V, 16 MHz)
   Programmer - AVRISP mkII
   
***************************************************************************/


// Include the Wire library for I2C
#include <Wire.h>   // Uses {A4,A5} for {SDA,SCL}
#include <NeoSWSerial.h>
#include "printf.h"

#define NODE 8


// Set up Wire
#define WRREQ 0xFE
#define RDREQ 0xFD

// Set up MH-Z14A CO2-sensor
#define CO2_PIN_RX A2
#define CO2_PIN_TX A3
#define CO2_SER_TIMEOUT 1000   // ms
#define CO2_RCV_RAW_LEN 9
#define CO2_RCV_BUF_LEN 16

#define DEBUG_LEN 160
// maximum Wire message length is 32
#define MSGLEN 64



#define strstart(string, s) (! strncmp(string, s, strlen(s)))


NeoSWSerial co2serial(CO2_PIN_RX, CO2_PIN_TX);


uint8_t deb = 0;
char rxmsg[MSGLEN + 1];
char txmsg[MSGLEN + 1];
char debug[DEBUG_LEN];
uint8_t co2rcvBuf[CO2_RCV_BUF_LEN+1];

 
bool getSerial(char *);
void splitcommands(char*, char*);
void process(char*);
void process_co2(char*);
void process_co2_ser(char *);
void process_debug(char *);


void setup() {
    Serial.begin(9600);
    sprintf(debug, "-id=a%02d init", NODE);
    Serial.println(debug);

    // Join I2C bus as slave with address 8
    Wire.begin(NODE);   
    // Call recvEvent when data received/written                
    Wire.onReceive(recvEvent);
    // Call sendEvent when data requested/read
    Wire.onRequest(sendEvent);   
    
    co2serial.begin(9600);
    
    printf_begin();
}


// Function that executes whenever data is received from Wire master
// This is the follow-up to the master's: SMBus.write_i2c_block_data(11, 0xFE, rxmsg)
void recvEvent(int howMany) {
  uint8_t incount = 0;
  uint8_t rxcount = 0;
  uint8_t msgtype = 0;
  while (Wire.available()) { 
    char c = Wire.read();   // receive byte as a character
    if (incount++ == 0) {   // first character is the message type
      msgtype = c;
    } else {
        rxmsg[rxcount++] = c;
    }
  }
  switch (msgtype) {
    case WRREQ: 
      rxmsg[rxcount] = 0;   // close string
      if (deb > 0) {
        sprintf(debug, "Wire rcvd: 0x%02X '%s' (%d)", msgtype, rxmsg, strlen(rxmsg));
        Serial.println(debug);
      }
      // RDREQ handled by sendEvent()
  }
}


// Function that executes whenever data is requested by Wire master
// This is the reply to the master's: SMBus.read_i2c_block_data(11, 0xFD, strlen(txmsg))
void sendEvent() {
  if (strlen(txmsg)) {
    txmsg[32] = 0;   // truncate to Wire maximum
    uint8_t sent = Wire.write(txmsg);
    if (deb > 0) {
      sprintf(debug, "Wire sent: '%s' (%d)", txmsg, sent);
      Serial.println(debug);
    }
  }
}  


void loop() {
    // Handle messages incoming from Serial
    if (Serial.available()) {
        if (getSerial(rxmsg)) {   
            if (strlen(rxmsg)) {
                splitcommands(rxmsg, txmsg);
                // response is sent to Serial (not requested by Wire)
            }
            rxmsg[0] = 0;
       }
    }
      
    // Handle messages incoming from Wire/I2C:
    if (strlen(rxmsg)) {
        splitcommands(rxmsg, txmsg);
        // response is sent to Serial and requested by Wire
        rxmsg[0] = 0;
    }
}


void id() {
    sprintf(&txmsg[strlen(txmsg)], "id=a%02d, MH-Z14A CO2-sensor;", NODE);
}


bool getSerial(char *rxmsg) {
   char c;
   int myI = 0;
   bool myStop = false, sereceived = false;

   // Read commands string from Serial
   while (Serial.available() && !myStop) {
      c = Serial.read();
      if (c == '\n') {
         myStop = true;
      } else {
         rxmsg[myI++] = c;
      }
      if (myI >= MSGLEN)
         myStop = true;
      sereceived = true;
      delay(10);   // wait for a next character
   }
   rxmsg[myI] = 0;

   return sereceived;
}


void splitcommands(char* rxmsg, char* txmsg) {
    // split rxmsg into commands separated by ';' 
    // and writes the response for each command in txmsg
    txmsg[0] = 0;
    // sprintf(debug, "%ld   rxmsg[%d]=\"%s\"\n", millis(), strlen(rxmsg), rxmsg);
    // Serial.print(debug);

    int myC = 0;
    char myCmd[MSGLEN + 1];
    for (int myI = 0; myI < strlen(rxmsg); myI++) {
        // sprintf(debug, "myI=%d rxmsg[myI]='%c'\n", myI, rxmsg[myI]);
        // Serial.print(debug);
      
        if (rxmsg[myI] == ';' || rxmsg[myI] == 0 || rxmsg[myI] == '\n') {
            myCmd[myC] = 0;
            myC = 0;
            if (myCmd[0] != 0) {
                process(myCmd);   // writes txmsg
            }
        } else {
            myCmd[myC] = rxmsg[myI];
            myC++;
        }
    }
    rxmsg[0] = 0;

    printtx();
}


void printtx(void) {
    if (strlen(txmsg)) {
        // write all responses to the Serial 
        Serial.print('-');
        Serial.println(txmsg);
      
        // Wire response is requested by master and sent by sendEvent()
    }
}


// process command
void process(char *buf) {
    // sprintf(debug, "%6ld   cmd[%d]:\"%s\"\n", millis(), strlen(buf), buf);
    // Serial.print(debug);

    if (strstart(buf, "CO2")) {   /// request CO2 measurment
        process_co2(buf);
    } else if (strstart(buf, "co2")) {  // request CO2 measurement
        process_co2(buf);
    } else if (strstart(buf, "de")) {   // set debug level (0=off)
        process_debug(buf);
    } else if (strstart(buf, "he")) {   // help, for now forwarde to id()
        id();
    } else if (strstart(buf, "id")) {   // what am I 
        id();
    } else if (strstart(buf, "se")) {   // send command to CO2 sensor serial
        process_co2_ser(buf);
    } else  {
        sprintf(&txmsg[strlen(txmsg)], "%s:err;", buf);
    }
}


void process_co2(char* buf) {
    // buf ignored

    process_co2_ser("se=FF0186000000000079;");

    process_co2_ser("se;");
}


void process_debug(char* buf) {
    if (buf[2] != '=') {
        sprintf(&txmsg[strlen(txmsg)], "de=%d;", deb);
    } else if (buf[3] >= '0' && buf[3] <= '9') {
        deb = buf[3] - '0';
        sprintf(&txmsg[strlen(txmsg)], "de=%d;", deb);
    } else {
        sprintf(&txmsg[strlen(txmsg)], "de%c:err;", buf[2]);
    }
}


/* 
  Serial communication example:
  se=FF0186000000000079;se;
  Send: FF 01 86 00 00 00 00 00 79 
  Recv: FF 86 02 D8 3C 00 23 00 41
  Value CO2=728
 */

void process_co2_ser(char* buf) {
    uint32_t end_time = millis() + CO2_SER_TIMEOUT;
    uint8_t rcvCntr = 0;
//  sprintf(&txmsg[strlen(txmsg)], "serial: %s;", buf);

    if (buf[2] == '=') {
        if (deb > 0) {
            sprintf(debug, "-se tx: ");
            Serial.print(debug);
        }
        for (uint8_t i = 3; i < strlen(buf)-1; i += 2) {
            uint8_t high = buf[i]   > '9' ? buf[i]   - 'A' + 10 : buf[i]   - '0';
            uint8_t low  = buf[i+1] > '9' ? buf[i+1] - 'A' + 10 : buf[i+1] - '0';
            uint8_t mybyte = (high & 0xF) << 4;
            mybyte |= (low & 0xF);
            co2serial.write(mybyte);
            if (deb > 0) {
                sprintf(debug, "%02X ", mybyte);
                Serial.print(debug);
            }
        }
        if (deb > 0) {
            Serial.println();
        }
    } else {
        if (deb > 0) {
            sprintf(debug, "-se rx: ");
            Serial.print(debug);
        }
        do {
            if (co2serial.available()) {
                co2rcvBuf[rcvCntr] = co2serial.read();
                co2rcvBuf[rcvCntr] &= 0xFF;
                if (deb > 0) {
                    sprintf(debug, "%02X ", co2rcvBuf[rcvCntr]);
                    Serial.print(debug);
                } else {
                    delay(10);
                }
                rcvCntr++;
            }
        } while ((rcvCntr < CO2_RCV_RAW_LEN) && (millis() < end_time));
        if (deb > 0) {
            sprintf(debug, "\n");
            Serial.print(debug);
        }
        uint16_t co2 = co2rcvBuf[2] * 256 + co2rcvBuf[3];
        uint16_t check = 0;
        for (uint8_t i = 1; i < rcvCntr-1; i++) {
           check += co2rcvBuf[i]; 
        }
        check &= 0xFF;
        check = 0xFF - check + 1;
        if (check != co2rcvBuf[8] || deb > 0) {
            sprintf(&txmsg[strlen(txmsg)], "CO2=%d ppm, chkR=0x%02X chkC=0x%02X", 
                co2, co2rcvBuf[8], check);
        } else {
            sprintf(&txmsg[strlen(txmsg)], "CO2=%d ppm", co2);
            if (millis() < 65000) {
                sprintf(&txmsg[strlen(txmsg)], " (wu);");
            } else {
                sprintf(&txmsg[strlen(txmsg)], " (ok);");
            }
        }
    }
}


// .-.-.
