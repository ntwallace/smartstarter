/***************************************************
  SMS read code based on/taken from Adafruit Open Sesame Sketch
  >>https://github.com/adafruit/Open-Sesame
  
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!
 
 Open Sesame code written by Limor Fried/Ladyada for Adafruit Industries. 
 All changes and additional functions written by by Nick Wallace.
 ****************************************************/

/*
THIS CODE IS STILL IN PROGRESS!
*/

#include <SoftwareSerial.h>
#include "Adafruit_FONA.h"

#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4
#define FONA_RI 5

#define LOCK_PIN 6
#define UNLOCK_PIN 7
#define REMOTE_PIN 8

#define LED 13

#define LOCK true
#define UNLOCK false

#define BUSYWAIT 5000  // milliseconds

// stores senderient SMS #
char sender[25];

// this is a large buffer for replies
char replybuffer[255];

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);

Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

boolean fonainit(void) {
  while (!Serial);

  fonaSS.begin(4800);
  Serial.println(F("Initializing....(May take 3 seconds)"));

  if (! fona.begin(fonaSS)) {
    Serial.println(F("Couldn't find FONA"));
    return false;
  }
  Serial.println(F("FONA is OK"));
  return true;
  
}

void setup() {

  // set LED output for debugging
  pinMode(LED, OUTPUT);
  
  // tri-state key fob pins so they are off
  pinMode(LOCK_PIN, INPUT);
  pinMode(UNLOCK_PIN, INPUT);  
  pinMode(REMOTE_PIN, INPUT);
  
  Serial.begin(115200);
  Serial.println(F("FONA basic test"));

  while (! fonainit()) {
    delay(5000);
  }
  
  // Print SIM card IMEI number.
  char imei[15] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("SIM card IMEI: "); Serial.println(imei);
  }
  
  pinMode(FONA_RI, INPUT);
  digitalWrite(FONA_RI, HIGH); // turn on pullup on RI
  // turn on RI pin change on incoming SMS!
  fona.sendCheckReply(F("AT+CFGRI=1"), F("OK"));
}

int8_t lastsmsnum = 0;

void loop() {
   digitalWrite(LED, HIGH);
   delay(100);
   digitalWrite(LED, LOW);
       
  while (fona.getNetworkStatus() != 1) {
    Serial.println("Waiting for cell connection");
    delay(6000);
  }
  
  // Check if the interrupt pin went low, 
  // and after BUSYWAIT milliseconds break out to check
  // manually for SMS' and connection status
  for (uint16_t i=0; i<BUSYWAIT; i++) {
     if (! digitalRead(FONA_RI)) {
        // RI pin went low, SMS received?
        Serial.println(F("RI went low"));
        break;
     } 
     delay(1);
  }
  
  int8_t smsnum = fona.getNumSMS();
  if (smsnum < 0) {
    Serial.println(F("Could not read # SMS"));
    return;
  } else {
    Serial.print(smsnum); Serial.println(F(" SMS on SIM card!"));
  }
  
  if (smsnum == 0) return;

  // there's an SMS!
  uint8_t n = 0; 
  while (true) {
     uint16_t smslen;
     //char sender[25];
     
     uint8_t len = fona.readSMS(n, replybuffer, 250, &smslen); // pass in buffer and max len!
     // if the length is zero, its a special case where the index number is higher
     // so increase the max we'll look at!
     if (len == 0) {
        Serial.println(F("[empty slot]"));
        n++;
        continue;
     }
     if (! fona.getSMSSender(n, sender, sizeof(sender))) {
       // failed to get the sender?
       sender[0] = 0;
     }

     Serial.print(F("***** SMS #")); Serial.print(n);
     Serial.print(" ("); Serial.print(len); Serial.println(F(") bytes *****"));
     Serial.println(replybuffer);
     Serial.print(F("From: ")); Serial.println(sender);
     Serial.println(F("*****"));
     
     if (strcasecmp(replybuffer, "lock") == 0) {
       // lock the doors
       digitalWrite(LED, HIGH);
       doors(LOCK);
       digitalWrite(LED, LOW);
     }
     if (strcasecmp(replybuffer, "unlock") == 0) {
       // unlock the doors
       digitalWrite(LED, HIGH);
       doors(UNLOCK);
       digitalWrite(LED, LOW);
     }
     if (strcasecmp(replybuffer, "remote") == 0) {
       // start/stop the car
       digitalWrite(LED, HIGH);
       remote();
       digitalWrite(LED, LOW);
     }
     
     delay(3000);
     break;
  }  
  fona.deleteSMS(n);
 
  delay(1000); 
}

void doors(boolean locked){
  if (locked) {
    pinMode(LOCK_PIN, OUTPUT);
    digitalWrite(LOCK_PIN, LOW);
    delay(1000);
    pinMode(LOCK_PIN, INPUT);
    
    Serial.println("Attemped to lock doors");
    Serial.print("Attempting to send SMS to: "); Serial.println(sender);
    sms(sender, "Door lock signal sent!");
  } else {
    pinMode(UNLOCK_PIN, OUTPUT);
    digitalWrite(UNLOCK_PIN, LOW);
    delay(1000);
    pinMode(UNLOCK_PIN, INPUT);

    Serial.println("Attemped to unlock doors");
    Serial.print("Attempting to send SMS to: "); Serial.println(sender);
    sms(sender, "Door unlock signal sent!");
  }
}

void remote(){
  pinMode(REMOTE_PIN, OUTPUT);
  digitalWrite(REMOTE_PIN, LOW);
  delay(1000);
  pinMode(REMOTE_PIN, INPUT);

  Serial.println("Attemped to remote start/stop car");
  Serial.print("Attempting to send SMS to: "); Serial.println(sender);
  sms(sender, "Remote start/stop signal sent!");
}

void sms(char* sender, char* message){
  flushSerial();
  if (!fona.sendSMS(sender, message)) {
    Serial.println(F("Failed"));
  } else {
    Serial.print(F("Sent SMS response to: ")); Serial.println(sender);
  }
}

void flushSerial() {
  while (Serial.available())
    Serial.read();
}

