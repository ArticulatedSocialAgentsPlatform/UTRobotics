#include <SPI.h>
#include <MFRC522.h>
//#include <avr/wdt.h>
#define SS_PIN 10
#define RST_PIN 9

MFRC522 mfrc522(SS_PIN, RST_PIN);
bool reedLeft, reedRight;
int poti_cur = 511, poti_prev = 511, poti_state, poti_thresh = 35, poti_center = 530;
String myPins[6];
String output = "";
String prevOutput = "";

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600);  
  Serial.setTimeout(100);
  
  delay(5000);
  
  Serial.println("Starting setup");
  Serial.flush();

  //a watchdog function that checks if the program is still running (and restarts otherwise)
  //wdt_enable(WDTO_1S);

  RFID_init();
  reeds_init();
  
  Serial.println("Finished setup");
  Serial.flush();
}


void loop() {
  
  //tell the watchdog we are still good to go
  //wdt_reset();
  
  if ((((int)(millis()/100))%2) == 0)
  {
    digitalWrite(13, HIGH);
  }
  else
  {
    digitalWrite(13, LOW);
  }
  //debug("still running main loop...");

  // reset output
  output = "";

  reeds_act();
  poti_act();
  pins_act();

  // construct output string
  output += (String)(int)reedLeft + (String)(int)reedRight + (String)poti_state + myPins[0] + myPins[1] + myPins[2] + myPins[3] + myPins[4] + myPins[5];
  // for testing purposes only:
  //output = "002M0000M";
  if(output != prevOutput){
    Serial.println(output);
    Serial.flush();
  }

  prevOutput = output;

  RFID_find();
  Serial.flush();
  
}

void debug(String msg) {
  Serial.println("Debug: " + msg);
}

