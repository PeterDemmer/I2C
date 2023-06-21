/**************************************************************************

   Copyright (c) 2021 Peter Demmer
   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.
  
   i2c4sds011.ino, Wire/I2C interface for SDS011 PM-meter
   Raspberry Pi counterpart: i2c4sds011.py2
   
   Wiring:
     sds Enable - D8 - high pulls SDS011 GND to Arduino GND
     sdsSerial TX - D7 - RX SDS011
     sdsSerial RX - D6 - TX SDS011
     Wire SDA - A4 - 3 RPi
     Wire SCL - A5 - 5 RPi

   Board - Arduino Pro Mini
   Port - /dev/ttyUSB0 (CP2102)
   Processor - ATmega328P (5, 16 MHz)
   Programmer - AVRISP mkII
  
*************************************************************************/


// Include the Wire library for I2C
#include <Wire.h>   // Uses {A4,A5} for {SDA,SCL}
#include <NeoSWSerial.h>
#include "printf.h"

#define NODE 9


// Set up Wire
#define WRREQ 0xFE
#define RDREQ 0xFD

// Set up SDS particle matter sensor
#define SDS_ENA 8
#define SDS_TX 7
#define SDS_RX 6
#define SDS_NO_MEAS 10
#define SDS_RAW_LEN 10
#define SDS_ERR 0xFFFF
#define SDS_MEAS_TIMEOUT 2000   // in ms, measurements should repeat every 1 second
uint16_t sdsCntRcvBuf = 0;
NeoSWSerial sdsSerial(SDS_RX, SDS_TX);


#define DEBUG_LEN 64
// maximum Wire message length is 32
#define MSGLEN 64


#define strstart(string, s) (! strncmp(string, s, strlen(s)))


uint8_t deb = 0;
char rxmsg[MSGLEN + 1];
char txmsg[MSGLEN + 1];
char debug[DEBUG_LEN];


bool getSerial(char *);
void splitcommands(char*, char*);
void process(char*);
void process_debug(char *);
void process_pm(char*);
void process_pm_serial(char*);


void setup() {
    Serial.begin(9600);
    sprintf(debug, "-id=a%02d init", NODE);
    Serial.println(debug);

    // Join I2C bus as slave with address 9
    Wire.begin(NODE);   
    // Call recvEvent when data received/written                
    Wire.onReceive(recvEvent);
    // Call sendEvent when data requested/read
    Wire.onRequest(sendEvent);   
  
    pinMode(SDS_ENA, OUTPUT);
    
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
  sprintf(&txmsg[strlen(txmsg)], "id=a%02d, SDS011 PM-sensor;", NODE);
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
    // splits rxmsg into commands separated by ';'
    // and for each command appends the response in txmsg
    txmsg[0] = 0;
    if (deb > 1) {
        sprintf(debug, "6%ld   rxmsg[%d]=\"%s\"\n", millis(), strlen(rxmsg), rxmsg);
        Serial.print(debug);
    }
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
    if (deb > 1) {
        sprintf(debug, "%6ld   cmd[%d]:\"%s\"\n", millis(), strlen(buf), buf);
        Serial.print(debug);
    }

    if (strstart(buf, "de")) {
        process_debug(buf);
    } else if (strstart(buf, "he")) {
        id();
    } else if (strstart(buf, "id")) {
        id();
    } else if (strstart(buf, "pm")) {
        process_pm(buf);
    } else if (strstart(buf, "se")) {
        process_pm_serial(buf);
    } else  {
        sprintf(&txmsg[strlen(txmsg)], "%s:err;", buf);
    }
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


void process_pm(char* buf) {
  uint16_t ignores = SDS_NO_MEAS / 2;
  uint16_t measures = SDS_NO_MEAS / 2;
  int8_t res;
  if (buf[2] == '=') {
    res = sscanf(&buf[3], "%d,%d", &ignores, &measures);
  }
  if (ignores < 1) ignores = 1;
  if (measures < 1) measures = 1;
  if (ignores > 30) ignores = 30;
  if (measures > 30) measures = 30;
  uint8_t rcvBuf[SDS_RAW_LEN];
  uint16_t pm25[ignores+measures];
  uint16_t pm10[ignores+measures];

  if (deb > 0) {
    sprintf(debug, "#ignores=%d,#average=%d", ignores, measures);
    Serial.println(debug);
  }

  digitalWrite(SDS_ENA, HIGH);   // Switches on SDS power, pulls SDS GND to Arduino GND
  delay(1000);
  sdsSerial.begin(9600);
  
  uint8_t iMeas;   // number of measurements received
  uint8_t jMeas = 0;   // number of valid measurements
  uint32_t end_time;

  for (iMeas = 0; iMeas < ignores + measures; iMeas++) {
    end_time = millis() + SDS_MEAS_TIMEOUT;

    for (uint8_t i = 0; i < SDS_RAW_LEN; i++) rcvBuf[i] = 0;

    sdsCntRcvBuf = 0;
    do {
      if (sdsSerial.available()) {
        rcvBuf[sdsCntRcvBuf] = sdsSerial.read();
        rcvBuf[sdsCntRcvBuf] &= 0xFF;

        if ( ((sdsCntRcvBuf == 0) && (rcvBuf[sdsCntRcvBuf] != 0xAA)) || 
             ((sdsCntRcvBuf == 1) && (rcvBuf[sdsCntRcvBuf] != 0xC0)) ) {
            // message should start with 0xAA 0xC0
            sdsCntRcvBuf = 0;   // da capo
            if (deb > 1) {
              sprintf(debug, "%02X\n", rcvBuf[sdsCntRcvBuf]);
              Serial.write(debug);
            }      
        } else {
          if (deb > 1) {
            sprintf(debug, "%02X ", rcvBuf[sdsCntRcvBuf]);
            Serial.write(debug);
          }
          sdsCntRcvBuf++;
        }
      }
    } while ((sdsCntRcvBuf < SDS_RAW_LEN) && (millis() < end_time));

    uint16_t pm25l;
    uint16_t pm10l;
    if (deb > 1) {
      if (! sdsCntRcvBuf) {
        Serial.println("   null");
      } else {
        pm25l = 256 * (rcvBuf[3] & 0xFF) + (rcvBuf[2] & 0xFF);
        pm10l = 256 * (rcvBuf[5] & 0xFF) + (rcvBuf[4] & 0xFF);
        uint16_t checksum = 0;
        for (int i = 2; i < 8; i++) {
          checksum += rcvBuf[i] & 0xFF;
        }
        sprintf(debug, "   %3d.%d ", pm25l / 10, pm25l % 10);
        Serial.print(debug);
        sprintf(debug, "%3d.%d   ", pm10l / 10, pm10l % 10);
        Serial.print(debug);
        if (rcvBuf[0] == 0xAA && rcvBuf[1] == 0xC0 && rcvBuf[8] == checksum % 256 && rcvBuf[9] == 0xAB) {
          Serial.println("OK");
        } else {
          pm25l = SDS_ERR;
          pm10l = SDS_ERR;
          Serial.println("ERR");
        }
      }
    }

    if (! sdsCntRcvBuf || sdsCntRcvBuf != SDS_RAW_LEN) {
      pm25l = SDS_ERR;
      pm10l = SDS_ERR;
    } else {
      pm25l = 256 * (rcvBuf[3] & 0xFF) + (rcvBuf[2] & 0xFF);
      pm10l = 256 * (rcvBuf[5] & 0xFF) + (rcvBuf[4] & 0xFF);
      uint16_t checksum = 0;
      for (int i = 2; i < 8; i++) {
        checksum += rcvBuf[i];
      }
      checksum &= 0xFF;
      if (rcvBuf[0] == 0xAA && rcvBuf[1] == 0xC0 && rcvBuf[8] == checksum % 256 && rcvBuf[9] == 0xAB) {
        pm25[jMeas] = pm25l;
        pm10[jMeas] = pm10l;
        jMeas++;
      } else {
        pm25l = SDS_ERR;
        pm10l = SDS_ERR;
      }
    }
  }   // for iMeas ...

  if (deb > 0) {
    sprintf(debug, "#valid=%d", jMeas);
    Serial.println(debug);
  }

  sdsSerial.end();
  digitalWrite(SDS_ENA, LOW);   // Switches off SDS power


  if (sdsCntRcvBuf && deb > 0) {
    Serial.print("pm25=");
    for (uint8_t iMeas = 0; iMeas < ignores + measures; iMeas++) {
      if (pm25[iMeas] != SDS_ERR) {
        sprintf(debug, "%2d.%d ", pm25[iMeas] / 10, pm25[iMeas] % 10);
        Serial.print(debug);
      }
    }
    Serial.println();
    Serial.print("pm10=");
    for (uint8_t iMeas = 0; iMeas < ignores + measures; iMeas++) {
      if (pm10[iMeas] != SDS_ERR) {
        sprintf(debug, "%2d.%d ", pm10[iMeas] / 10, pm10[iMeas] % 10);
        Serial.print(debug);
      }
    }
    Serial.println();
  }

  // if 10 measurements were taken, calculate the average of the last 5:
  jMeas = 0;
  uint16_t pm25t = (uint16_t) 0;
  uint16_t pm10t = (uint16_t) 0;
  for (iMeas = ignores; iMeas < ignores + measures; iMeas++) {
    if ((pm25[iMeas] != SDS_ERR) && (pm10[iMeas] != SDS_ERR)) {
      pm25t += pm25[iMeas];
      pm10t += pm10[iMeas];
      jMeas++;
    }
    if (deb > 1) {
      sprintf(debug, "iM=%d jM=%d, pm25[]=%d, ", iMeas, jMeas, pm25[iMeas]);
      Serial.print(debug);
      sprintf(debug, "pm25t=%d, pm10[]=%d, pm10t=%d", pm25t, pm10[iMeas], pm10t);
      Serial.println(debug);
    }
  }

  if (jMeas == 0) {
    sprintf(&txmsg[strlen(txmsg)], "pm=err;");
  } else {
    uint16_t pm25a = pm25t / jMeas;
    uint16_t pm10a = pm10t / jMeas;
    if (deb > 0) {
      sprintf(debug, "#average=%d pm25t=%d pm10t=%d pm25a=%d pm10a=%d", jMeas, pm25t, pm10t, pm25a, pm10a);
      Serial.println(debug);
    }
  
    sprintf(&txmsg[strlen(txmsg)], "pm=%d.%d,", pm25a / 10, pm25a % 10);
    sprintf(&txmsg[strlen(txmsg)], "%d.%d;",     pm10a / 10, pm10a % 10);
  }
}



/*
 * 
 * SDS011 serial Communication examples (not used)
 * 
 * Query data reporting mode:
 * AA B4 02 00 00 00 00 00 00 00 00 00 00 00 00 FF FF 00 AB
 * se=AAB402000000000000000000000000FFFF00AB;se;
 * Response:
 * AA C5 ...
 * 
 * Query all sensors measurement data response:
 * AA B4 04 00 00 00 00 00 00 00 00 00 00 00 00 FF FF 02 AB
 * se=AAB404000000000000000000000000FFFF02AB;se;
 * Response:
 * AA C0 PML PMH PML PMH ID1 ID2 CS 0xAB
 * 
 * Set all sensors to sleep:
 * AA B4 06 01 00 00 00 00 00 00 00 00 00 00 00 FF FF 05 AB
 * se=AAB406010000000000000000000000FFFF05AB;se;
 * Response:
 * AA C5 06 01 00 00 ID1 ID2 CS AB
 * 
 * Set all sensors to work:
 * AA B4 06 01 01 00 00 00 00 00 00 00 00 00 00 FF FF 06 AB
 * se=AAB406010100000000000000000000FFFF06AB;se;
 * Sensor response:
 * AA C5 06 01 01 00 ID1 ID2 CS AB
 *
 */

void process_pm_serial(char* buf) {
  // not used
  uint8_t rcvCntr = 0;
  uint8_t rcvBuf[SDS_RAW_LEN + 1];

  //  sprintf(&txmsg[strlen(txmsg)], "serial: %s;", buf);

  if (buf[2] == '=') {
    if (deb > 0) {
      sprintf(debug, "-se tx: ");
      Serial.print(debug);
    }
    for (uint8_t i = 3; i < strlen(buf) - 1; i += 2) {
      uint8_t high = buf[i]   > '9' ? buf[i]   - 'A' + 10 : buf[i]   - '0';
      uint8_t low  = buf[i + 1] > '9' ? buf[i + 1] - 'A' + 10 : buf[i + 1] - '0';
      uint8_t mybyte = (high & 0xF) << 4;
      mybyte |= (low & 0xF);
      sdsSerial.write(mybyte);
      if (deb > 0) {
        sprintf(debug, "%02X ", mybyte);
        Serial.print(debug);
      }
      delay(10);
    }
    if (deb > 0) {
      Serial.println();
    }
  } else {
    if (deb > 0) {
      sprintf(debug, "-se rx: ");
      Serial.print(debug);
    }
    uint32_t end_time = millis() + SDS_MEAS_TIMEOUT;
    do {
      if (sdsSerial.available()) {
        rcvBuf[rcvCntr] = sdsSerial.read();
        rcvBuf[rcvCntr] &= 0xFF;
        if ( ((rcvCntr == 0) && (rcvBuf[rcvCntr] != 0xAA)) || 
             ((rcvCntr == 1) && (rcvBuf[rcvCntr] != 0xC0)) ) {
            // message should start with 0xAA 0xC0
            rcvCntr = 0;   // da capo
            if (deb > 1) {
              sprintf(debug, "%02X\n", rcvBuf[sdsCntRcvBuf]);
              Serial.write(debug);
            }      
        } else {
          if (deb > 0) {
            sprintf(debug, "%02X ", rcvBuf[rcvCntr]);
            Serial.print(debug);
          } else {
            delay(10);
          }
          rcvCntr++;
        }
      }
    } while ((rcvCntr < SDS_RAW_LEN) && (millis() < end_time));
    if (deb > 0) {
      sprintf(debug, " (%d)\n", rcvCntr);
      Serial.print(debug);
    }
    //sprintf(&txmsg[strlen(txmsg)], "rcvCntr=%d ", rcvCntr);
    if (rcvCntr == 0) {
      sprintf(&txmsg[strlen(txmsg)], "(timeout)");
    } else {
      uint16_t pm = rcvBuf[2] * 256 + rcvBuf[3];
      uint16_t check = 0;
      for (uint8_t i = 1; i < rcvCntr - 1; i++) {
        check += rcvBuf[i];
      }
      check &= 0xFF;
      check = 0xFF - check + 1;
      if (check != rcvBuf[8] || deb > 0) {
        sprintf(&txmsg[strlen(txmsg)], "pm=%d ppm, chkR=0x%02X chkC=0x%02X",
                pm, rcvBuf[8], check);
      } else {
        sprintf(&txmsg[strlen(txmsg)], "pm=%d ppm", pm);
        if (millis() < 65000) {
          sprintf(&txmsg[strlen(txmsg)], " (wu);");
        } else {
          sprintf(&txmsg[strlen(txmsg)], " (ok);");
        }
      }
    }
  }   // default / specified
}   // process_pm_serial


// .-.-.
