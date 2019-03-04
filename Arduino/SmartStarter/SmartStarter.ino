/*********************
 * 
 * SmartStarter v3.0
 * January 2019
 * Nick Wallace
 * 
 *********************/

// add 1 ohm resistor and 1000uf cap between car battery -> dc-dc to decouple power (low pass filter)

/* dc-dc max no-load: 20mA
 arduino pro mini 5v load: 23mA (active) / 3.2mA (sleep) -- can cut ~3mA if disable LED
 FONA load: 25mA (idle) / 200mA (receiving data) / 0mA (off)
 viper FOB load: >5mA

 TOTAL: ~23mA (sleep), ~80mA (idle), ~275mA (data)

 25mA for 3 min, 80mA idle for 14 sec, 275mA for 1 sec @ a time
 One cycle is 3.25 minutes, so 18.5 cycles per hour
 Once cycle load is 1.25mA + .333mA + .077mA, so 1.66mA * 18.5 cycles = 30.7mAh draw
*/

// add libs
#include <SoftwareSerial.h>
#include <Adafruit_FONA.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>

// define FONA pins
#define FONA_RST 2
#define FONA_RX 3
#define FONA_TX 4
#define FONA_KEY 5
#define FONA_RI 6

// define keyfob i/o
#define LOCK_PIN 7
#define UNLOCK_PIN 8
#define REMOTE_PIN 9

// define car statuses
#define CAR_BATTERY_CHECK_PIN 10
#define POWER_STATUS_PIN 11
#define LOCK true
#define UNLOCK false

// initialize SMS vars
char sender[25]; // stores sender SMS #
char replybuffer[255]; // buffer for replies
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
int8_t lastsmsnum = 0;

boolean powerOn = false;
boolean remoteStarted = false;
uint8_t period = 8000; // 8 sec for car to start

// initialize FONA software serial
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

void setup() {
  // set keyfob pins off
  pinMode(LOCK_PIN, INPUT);
  pinMode(UNLOCK_PIN, INPUT);
  pinMode(REMOTE_PIN, INPUT);

  // setup car power status pin
  pinMode(POWER_STATUS_PIN, INPUT);
  pinMode(CAR_BATTERY_CHECK_PIN, INPUT);

  // initialize FONA
  Serial.begin(115200);
  while (! fonainit()) {
    delay(5000);
  }

  pinMode(FONA_RI, INPUT);
  digitalWrite(FONA_RI, HIGH); // turn on pullup on RI
  // turn on RI pin change on incoming SMS!
  fona.sendCheckReply(F("AT+CFGRI=1"), F("OK"));
}

void loop() {
  while (fona.getNetworkStatus() != 1) {
    Serial.println("Waiting for cell connection");
    delay(3000);
  }

  // Check if the interrupt pin went low, break out to check manually for SMS, and connection status
  for (uint16_t i = 0; i < 5000; i++) {
    if (! digitalRead(FONA_RI)) {
      // RI pin went low, SMS received?
      Serial.println(F("RI interrupt went low"));
      break;
    }
    delay(1);
  }

  // check if SMS came in
  int8_t smsnum = fona.getNumSMS();
  /*if (smsnum == 0) {
    Serial.println("No text received");
    return;
  }*/
  if (smsnum == 0) {
      digitalWrite(FONA_KEY, LOW); // turn the FONA off for power savings
      delay(3000);
      sleep(180); // sleep for 180 sec
    }

  // SMS was found
  uint8_t n = 0;
  while (true) {
    uint16_t smslen;

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
      doors(LOCK); // lock the doors
    }
    if (strcasecmp(replybuffer, "unlock") == 0) {
      doors(UNLOCK); // unlock the doors
    }
    if (strcasecmp(replybuffer, "remote") == 0) {
      remote(); // start or stop car
    }

    delay(100);
    break;
  }
  fona.deleteSMS(n);

  // check to see if car state changed succesfully after attempted remote start/stop
  if (remoteStarted) {
    if (currentMillis - startMillis >= period) {
      remote_response();
      startMillis = currentMillis
    }
  }

  delay(100);
}

void doors(boolean locked) {
  if (locked) {
    // power to lock button on keyfob
    pinMode(LOCK_PIN, OUTPUT);
    digitalWrite(LOCK_PIN, LOW);
    delay(200);
    pinMode(LOCK_PIN, INPUT);

    Serial.println("Attemped to lock doors");
    Serial.print("Attempting to send SMS to: "); Serial.println(sender);
    sms(sender, "Door lock signal sent!");

  } else {
    //power to unlock button on keyfob
    pinMode(UNLOCK_PIN, OUTPUT);
    digitalWrite(UNLOCK_PIN, LOW);
    delay(200);
    pinMode(UNLOCK_PIN, INPUT);

    Serial.println("Attemped to unlock doors");
    Serial.print("Attempting to send SMS to: "); Serial.println(sender);
    sms(sender, "Door unlock signal sent!");
  }
}

void remote() {
  // if no 5v car power to status pin, car is off
  powerOn = digitalRead(POWER_STATUS_PIN) == HIGH;

  // power remote start/stop button on keyfob, flag state and start timer
  pinMode(REMOTE_PIN, OUTPUT);
  digitalWrite(REMOTE_PIN, LOW);
  delay(200);
  pinMode(REMOTE_PIN, INPUT);
  remoteStarted = true;
  startMillis = millis();

  Serial.println("Attempting to turn car on/off!");
  sms(sender, "Attempting to turn car on/off!");
}

void remote_response() {
  // check if non-constant voltage line status has changed after button was powered
  if (!digitalRead(POWER_STATUS_PIN) && powerOn ) { // car was previously on, now off
    Serial.println("Car was turned off!");
    sms(sender, "Car was turned off!");
    powerOn = false;
  }
  else if (digitalRead(POWER_STATUS_PIN) && !powerOn) { // car was previously off, now on
    Serial.println("Car was turned on!");
    sms(sender, "Car was turned on!");
    powerOn = true;
  }
  else if ((digitalRead(POWER_STATUS_PIN) && powerOn) || (!digitalRead(POWER_STATUS_PIN) && !powerOn)) { // car state didn't change
    Serial.println("Remote start/stop failed!");
    sms(sender, "Remote start/stop failed!");
  }
}

void sms(char sender[10], char message[25]) {
  flushSerial();
  if (!fona.sendSMS(sender, message)) {
    Serial.println(F("Failed"));
  } else {
    Serial.print(F("Sent SMS response to: ")); Serial.println(sender);
  }
}

boolean fonainit(void) {
  // power the FONA on
  pinMode(FONA_KEY, OUTPUT);
  digitalWrite(FONA_KEY, LOW);
  delay(3000);
  pinMode(FONA_KEY, INPUT);

  while (!Serial);

  fonaSS.begin(4800);
  Serial.println(F("Initializing...."));

  if (! fona.begin(fonaSS)) {
    Serial.println(F("Couldn't find FONA"));
    return false;
  }
  Serial.println(F("FONA is online"));
  return true;

}

// setup watchdog function && sleep
int nbrRemaining;
void configure_wdt(void) {
  cli();                           // disable interrupts for changing the registers

  MCUSR = 0;                       // reset status register flags
  WDTCSR |= 0b00011000;            // Set WDCE (5th from left) and WDE (4th from left) to enter config
  WDTCSR =  0b01000000 | 0b100001; // set WDIE/clear WDE | set to 8 sec intervals

  sei();                           // re-enable interrupts
}

// setup watchdog interrupt
ISR(WDT_vect) {
  wdt_disable();  // disable watchdog
}

void sleep(int ncycles) {
  nbrRemaining = (ncycles / 8); // defines how many cycles should sleep

  // Set sleep to full power down
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  // Turn off the ADC while asleep.
  power_adc_disable();

  while (nbrRemaining > 0) { // while some cycles left, sleep!
    // Enable sleep and enter sleep mode.
    sleep_mode();

    // When awake, disable sleep mode
    sleep_disable();

    // we have slept one time more
    nbrRemaining = nbrRemaining - 1;
  }

  // put everything on again
  power_all_enable();
}

void flushSerial() {
  while (Serial.available())
    Serial.read();
}

void softReset() {
  asm volatile ("  jmp 0");
}

