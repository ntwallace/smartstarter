// add libs
#include <SoftwareSerial.h>
#include <Adafruit_FONA.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>

// define FONA pins
#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4
#define FONA_RI 5
#define FONA_KEY 10
#define FONA_BATTERY 11
#define MOSFET 12

// define keyfob i/o
#define LOCK_PIN 6
#define UNLOCK_PIN 7
#define REMOTE_PIN 8
#define POWER_STATUS_PIN 9
#define LOCK true
#define UNLOCK false

// initialize battery vars
int lastBatteryPcnt = 0;
int batteryBasement = 000;  // lowest voltage allowed on numberic scale 0-1023, ex. 500
float batteryConstant = 100.0 / (1023 - batteryBasement);
boolean chargeBattery = false;

// initialize SMS vars
char sender[25]; // stores sender SMS #
char replybuffer[255]; // buffer for replies
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
int8_t lastsmsnum = 0;

boolean powerOn = false;

// initialize FONA software serial
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

boolean checkBattery(int batteryVal) {
  int batteryPcnt = (batteryVal - batteryBasement) * batteryConstant;

  if (lastBatteryPcnt != batteryPcnt) {
    Serial.print("Battery @ ");
    Serial.print(batteryPcnt);
    Serial.print("%");
    Serial.println();
    lastBatteryPcnt = batteryPcnt;
  }

  if (batteryPcnt <= 40) {  //battery at or below 40%
    return true;
  } else {
    return false;
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
  Serial.println(F("FONA is OK"));
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

void setup() {
  // set keyfob pins off
  pinMode(LOCK_PIN, INPUT);
  pinMode(UNLOCK_PIN, INPUT);
  pinMode(REMOTE_PIN, INPUT);

  // setup car power status pin
  pinMode(POWER_STATUS_PIN, INPUT);

  // initialize FONA
  Serial.begin(115200);
  while (! fonainit()) {
    delay(5000);
  }

  // get battery status, charge if =< 40%
  chargeBattery = checkBattery(analogRead(FONA_BATTERY));

  pinMode(FONA_RI, INPUT);
  digitalWrite(FONA_RI, HIGH); // turn on pullup on RI
  // turn on RI pin change on incoming SMS!
  fona.sendCheckReply(F("AT+CFGRI=1"), F("OK"));
}

void loop() {
  while (fona.getNetworkStatus() != 1) {
    Serial.println("Waiting for cell connection");
    delay(6000);
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

  int8_t smsnum = fona.getNumSMS();
  /* if (smsnum < 0) {
    Serial.println(F("Could not read # SMS"));
    return;
    } else {
    Serial.print(smsnum); Serial.println(F(" SMS on SIM card!"));
    } */

  if (smsnum == 0) return;
  /*if (smsnum == 0 && !chargeBattery) {
  digitalWrite(FONA_KEY, LOW);
  delay(3000);
  sleep(72); // sleep for 72 sec */

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

    delay(1000);
    break;
  }
  fona.deleteSMS(n);

  delay(1000);
}

void doors(boolean locked) {
  if (locked) {
    // power to lock button on keyfob
    pinMode(LOCK_PIN, OUTPUT);
    digitalWrite(LOCK_PIN, LOW);
    delay(1000);
    pinMode(LOCK_PIN, INPUT);

    Serial.println("Attemped to lock doors");
    Serial.print("Attempting to send SMS to: "); Serial.println(sender);
    sms(sender, "Door lock signal sent!");

  } else {
    //power to unlock button on keyfob
    pinMode(UNLOCK_PIN, OUTPUT);
    digitalWrite(UNLOCK_PIN, LOW);
    delay(1000);
    pinMode(UNLOCK_PIN, INPUT);

    Serial.println("Attemped to unlock doors");
    Serial.print("Attempting to send SMS to: "); Serial.println(sender);
    sms(sender, "Door unlock signal sent!");
  }
}

void remote() {
  // if no 5v car power to status pin, car is off
  powerOn = digitalRead(POWER_STATUS_PIN) == HIGH;

  // power remote start/stop button on keyfob
  pinMode(REMOTE_PIN, OUTPUT);
  digitalWrite(REMOTE_PIN, LOW);
  delay(1000);
  pinMode(REMOTE_PIN, INPUT);

  Serial.println("Attempted to turn car on/off!");
  sms(sender, "Attempted to turn car on/off!");
  delay(5000);

  // check 5v line status has changed after button was powered
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

void flushSerial() {
  while (Serial.available())
    Serial.read();
}

void softReset() {
  asm volatile ("  jmp 0");
}

