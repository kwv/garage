#include <Homie.h>
#define PIN_RELAY D5
#define PIN_SENSOR D4
const int STATUS_INTERVAL = 300;
#define OPENER_EVENT_MS 1000
#define DEBOUNCER_MS    100

// keep track of when the opener is pressed so we can un-set the
// relay after a short time to simulate a button press
unsigned long openerEvent = 0;
// bounce is built into Homie, so you can use it without including it first
//Bounce debouncer = Bounce();
int lastSensorValue = -1;



HomieNode garageNode("status", "sensor");
HomieNode openerNode("garageButton", "trigger");

bool buttonHandler( const HomieRange& range, const String& value ) {

  Homie.getLogger() << "button:" << value << endl;

  openerNode.setProperty("status").send(value);
  if (value == "open" || value == "1" || value == "ON" || value == "true" || value == "OPEN") {
    Homie.getLogger() << "Sending Open!" << value << endl;
    digitalWrite(PIN_RELAY, HIGH);
    openerEvent = millis();
  } else {
    return false;
  }

  return true;
}


void setupHandler() {

    Homie.getLogger() << "setup handler buttons"  << endl;


  // initialise our hardware pins
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_RELAY, LOW);
  pinMode(PIN_SENSOR, INPUT);

  // make sure the relay is off/open
  digitalWrite(PIN_RELAY, LOW);

  // initialise the debouncer
  //debouncer.attach(PIN_SENSOR);
  //debouncer.interval(10000);
}
void loopHandler() {

  int sensorValue = digitalRead(PIN_SENSOR);

  if (sensorValue != lastSensorValue) {
    Homie.getLogger() << "new value:" << sensorValue << " old value:" << lastSensorValue << endl;
    garageNode.setProperty("status").send(sensorValue ? "OPEN" : "CLOSED");
    lastSensorValue = sensorValue;
  }
  // if the opener has been pressed, then un-set after a short time
  // this code has preference to ensure the relay is released even if
  // we lose WIFI or MQTT connectivity and homie has to reconnect
  if (openerEvent && (millis() - openerEvent >= OPENER_EVENT_MS)) {
    digitalWrite(PIN_RELAY, LOW);
    openerEvent = 0;
    openerNode.setProperty("status").send("done");
  }

}

void setup() {
  Serial.begin(115200);

  Homie_setFirmware("garage-magic", "1.0.8"); // The "_" is not a typo! See Magic bytes
  //  Homie.reset();

  Homie.setSetupFunction(setupHandler).setLoopFunction(loopHandler);

  garageNode.advertise("status");
  openerNode.advertise("trigger").settable(buttonHandler);
  openerNode.advertise("status");
  //Homie.getLogger() << "advertising buttons..:"  << endl;


  Homie.setup();
}

void loop() {


  Homie.loop();
}
