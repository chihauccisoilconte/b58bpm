#include <Arduino.h>
#include <BLEMidi.h>

unsigned long previousMillisGetHR = 0; //--> will store the last time Millis (to get Heartbeat) was updated.
unsigned long previousMillisHR = 0; //--> will store the last time Millis (to get BPM) was updated.

const long intervalGetHR = 10; //--> Interval for reading heart rate (Heartbeat) = 10ms.
const long intervalHR = 10000; //--> Interval for obtaining the BPM value based on the sample is 10 seconds.

const int VBMotor = 15;

//potentiometers
const int potPin1 = 12;
const int potPin2 = 14;
const int potPin3 = 27;
const int potPin4 = 26;

// variables for storing the potentiometer value
int potValue1 = 0;
int potValue2 = 0;
int potValue3 = 0;
int potValue4 = 0;

//PulseSensor
const int PulseSensorHRWire = 36;

//LED to detect when the heart is beating
const int LED_D1 = 22;
const int LED_D2 = 19;
const int LED_D3 = 23;
const int LED_D4 = 18;
const int LED_D5 = 5;
const int LED_D6 = 17;

int Threshold = 1950; //--> Determine which Signal to "count as a beat" and which to ignore.

int cntHB = 0; //--> Variable for counting the number of heartbeats.
boolean ThresholdStat = true; //--> Variable for triggers in calculating heartbeats.
int BPMval = 0; //--> Variable to hold the result of heartbeats calculation.




void setup() {


  Serial.begin(115200);
  delay(500);

  //initialize MIDI server
  BLEMidiServer.begin("58BPS device");


  pinMode(VBMotor, OUTPUT);



  pinMode(LED_D1, OUTPUT); //--> Set LED_3 PIN as Output.
  pinMode(LED_D2, OUTPUT); //--> Set LED_3 PIN as Output.
  pinMode(LED_D3, OUTPUT); //--> Set LED_3 PIN as Output.
  pinMode(LED_D4, OUTPUT); //--> Set LED_3 PIN as Output.
  pinMode(LED_D5, OUTPUT); //--> Set LED_3 PIN as Output.
  pinMode(LED_D6, OUTPUT); //--> Set LED_3 PIN as Output.


}





//--------------------------------------------------------------------------------void GetHeartRate()
// This subroutine is for reading the heart rate and calculating it to get the BPM value.
// To get a BPM value based on a heart rate reading for 10 seconds.
void GetHeartRate() {
  //Process of reading heart rate.
  unsigned long currentMillisGetHR = millis();

  if (currentMillisGetHR - previousMillisGetHR >= intervalGetHR) {
    previousMillisGetHR = currentMillisGetHR;

    int PulseSensorHRVal = analogRead(PulseSensorHRWire);

  Serial.println(PulseSensorHRVal); //--> Send the Signal value to Serial Plotter.
  Serial.println(","); //--> Send the Signal value to Serial Plotter.


    if (PulseSensorHRVal > Threshold && ThresholdStat == true) {
      cntHB++;
      ThresholdStat = false;
      digitalWrite(LED_D1, HIGH);
      digitalWrite(LED_D4, HIGH);

      digitalWrite(VBMotor, HIGH);
    }

    if (PulseSensorHRVal < Threshold) {
      ThresholdStat = true;
      digitalWrite(LED_D1, LOW);
      digitalWrite(LED_D4, LOW);

      digitalWrite(VBMotor, LOW);
    }
  }
  //The process for getting the BPM value.
  unsigned long currentMillisHR = millis();

  if (currentMillisHR - previousMillisHR >= intervalHR) {
    previousMillisHR = currentMillisHR;

    BPMval = cntHB * 6; //--> The taken heart rate is for 10 seconds. So to get the BPM value, the total heart rate in 10 seconds x 6.


    cntHB = 0;
  }

}




void loop() {

  digitalWrite(LED_D2, HIGH);
  digitalWrite(LED_D5, HIGH);



  //Calling the GetHeartRate() subroutine
  GetHeartRate();


  // Reading potentiometer value
  potValue1 = analogRead(potPin1);
  potValue2 = analogRead(potPin2);
  potValue3 = analogRead(potPin3);
  potValue4 = analogRead(potPin4);

  //mapping potentiometers values for MIDI messages
  int midicc1 = map(potValue1, 0, 4095, 0, 127);
  int midicc2 = map(potValue2, 0, 4095, 0, 127);
  int midicc3 = map(potValue3, 555, 4095, 0, 127);
  int midicc4 = map(potValue4, 0, 4095, 0, 127);

  if (BLEMidiServer.isConnected()) {
    digitalWrite(LED_D3, HIGH);
    digitalWrite(LED_D6, HIGH);


    BLEMidiServer.controlChange(0, 1, midicc1);
    BLEMidiServer.controlChange(0, 2, midicc2);
    BLEMidiServer.controlChange(0, 3, midicc3);
    BLEMidiServer.controlChange(0, 4, midicc4);


  }
}
