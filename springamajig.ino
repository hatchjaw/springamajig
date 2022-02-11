#include <Audio.h>
#include <ResponsiveAnalogRead.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include "LIS3DHTR.h"

#define WIRE Wire

const bool DEBUG = false;
const bool PLOT = true;

// Sensors
const int FSR_PIN = A2;
const int FLEX_PIN = A3;

// GUItool: begin automatically generated code
AudioInputI2S2           i2s2;           //xy=128,90
AudioFilterStateVariable filter1;        //xy=274,97
AudioAmplifier           amp1;           //xy=412,109
AudioEffectDelay         delay1;         //xy=616,186
AudioMixer4              mixer1;         //xy=795,125
AudioOutputI2S           out;           //xy=1017,110
AudioConnection          patchCord1(i2s2, 0, filter1, 0);
AudioConnection          patchCord2(filter1, 2, amp1, 0);
AudioConnection          patchCord4(amp1, delay1);
AudioConnection          patchCord5(amp1, 0, mixer1, 0);
AudioConnection          patchCord6(delay1, 0, mixer1, 1);
AudioConnection          patchCord7(delay1, 1, mixer1, 2);
AudioConnection          patchCord8(delay1, 2, mixer1, 3);
AudioConnection          patchCord9(mixer1, 0, out, 0);
AudioConnection          patchCord10(mixer1, 0, out, 1);
AudioControlSGTL5000     audioShield;     //xy=1025,48
// GUItool: end automatically generated code

// GUItool: begin automatically generated code
//AudioInputI2S2           i2s2;           //xy=313,80
//AudioFilterStateVariable filter1;        //xy=485,83
//AudioAmplifier           amp1;           //xy=630,75
//AudioOutputI2S           out;           //xy=856,74
//AudioConnection          patchCord1(i2s2, 0, filter1, 0);
//AudioConnection          patchCord2(filter1, 2, amp1, 0);
//AudioConnection          patchCord3(amp1, 0, out, 0);
//AudioConnection          patchCord4(amp1, 0, out, 1);
//AudioControlSGTL5000     audioShield;
// GUItool: end automatically generated code

// Accelerometer
LIS3DHTR<TwoWire> LIS; //IIC

float fsr = 0;
float flex = 0;
float x, y, z = 0;

ResponsiveAnalogRead analog1(FSR_PIN, true);
ResponsiveAnalogRead analog2(FLEX_PIN, true);

void setup() {
  // Set up the accelerometer
  Serial.begin(115200);
  while (!Serial) {};
  LIS.begin(WIRE, 0x19); //IIC init
  delay(100);
  LIS.setFullScaleRange(LIS3DHTR_RANGE_2G);
  LIS.setOutputDataRate(LIS3DHTR_DATARATE_50HZ);
  LIS.setHighSolution(true); //High solution enable

  // Set up the audio shield.
  AudioMemory(1024);
  audioShield.enable();
  audioShield.volume(.6);

  // filter out DC & extremely low frequencies
  filter1.frequency(30);
  // amplify mic signal to useful range
  amp1.gain(8.5);

  // delay
  delay1.delay(0, 250);
  delay1.delay(1, 275);
  delay1.delay(2, 1234);
  for (int ch = 4; ch < 8; ++ch) {
    delay1.disable(ch);
  }

  // mix
  //  mixer1.gain(0, .8);
  //  mixer1.gain(1, .4);
  //  mixer1.gain(2, .25);

  Serial.println("Setup done.");
}

void loop() {
  processSensorData();

  if (DEBUG) {
    debugData();
  }
  
  //Serial.println(AudioMemoryUsage());
}

void processSensorData() {
  LIS.getAcceleration(&x, &y, &z);

  analog1.update();
  analog2.update();

  fsr = analog1.getValue();
  flex = analog2.getValue();

  flex = (flex - 600.) / 200.;
  fsr /= 375.;
}

void debugData() {
  if (PLOT) Serial.print(' '); else Serial.print("fsr: ");
  Serial.print(fsr);
  if (PLOT) Serial.print(' '); else Serial.print(", flex: ");
  Serial.print(flex);
  if (PLOT) Serial.print(' '); else Serial.print(", x: ");
  Serial.print(x);
  if (PLOT) Serial.print(' '); else Serial.print(", y: ");
  Serial.print(y);
  if (PLOT) Serial.print(' '); else Serial.print(", z: ");
  Serial.println(z);
  delay(100);
}
