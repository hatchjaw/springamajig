#include <Audio.h>
#include <ResponsiveAnalogRead.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include "LIS3DHTR.h"
#include "SpringGrain.h"

#define WIRE Wire

// Debug flags
const bool DEBUG_SENSORS = false;
const bool DEBUG_SENSOR_DETAIL = false;
const bool DEBUG_PARAMS = false;
const bool PLOT = false;

// Teensy sensor pins
const int FSR_PIN = A2;
const int FLEX_PIN = A3;

// Faust parameter names
const std::string GRAIN_SIZE = "Grain length (s)";
const std::string GRAIN_DENSITY = "Grain density";
const std::string GRAIN_SPEED = "Grain speed";
const std::string GRAIN_REGULARITY = "Rhythm";
const std::string FREEZE = "Freeze";

// Various parameter constants
const float FREEZE_THRESHOLD = .625;
const float MAX_GRAIN_SIZE = .5;
const float MIN_GRAIN_SIZE = .005;
const float MIN_GRAIN_DENSITY = .001;
const float MAX_GRAIN_DENSITY = 7.5;
const float DENSITY_FLIP_THRESHOLD = .85;
// Debugging introduces delay, which slows down the grain speed change;
// adjust for when not debugging.
const float GRAIN_SPEED_DELTA = DEBUG_SENSORS || DEBUG_PARAMS ? .015 : .001;
const float MAX_GRAIN_SPEED = 2.;
const float MIN_GRAIN_SPEED = .125;

// GUItool: begin automatically generated code
AudioInputI2S2           i2s2;           //xy=128,90
AudioFilterStateVariable filter1;        //xy=274,97
AudioAmplifier           amp1;           //xy=412,109
SpringGrain              sg;
AudioEffectFreeverb      reverb;
//AudioEffectDelay         delay1;         //xy=616,186
AudioMixer4              mixer;         //xy=795,125
AudioAmplifier           amp2; 
AudioOutputI2S           out;           //xy=1017,110
AudioConnection          patchCord1(i2s2, 0, filter1, 0);
AudioConnection          patchCord2(filter1, 2, amp1, 0);
AudioConnection          patchCord3(amp1, sg);
AudioConnection          patchCord4(amp1, 0, mixer, 0);
AudioConnection          patchCord5(sg, reverb);
AudioConnection          patchCord6(sg, 0, mixer, 1);
AudioConnection          patchCord7(reverb, 0, mixer, 2);
AudioConnection          patchCord8(mixer, amp2);
AudioConnection          patchCord9(amp2, 0, out, 0);
AudioConnection          patchCord10(amp2, 0, out, 1);
AudioControlSGTL5000     audioShield;     //xy=1025,48
// GUItool: end automatically generated code

// Accelerometer
LIS3DHTR<TwoWire> LIS; //IIC

float rawFSR = 0;
float fsr = 0;
float flex = 0;
float flexMin = 1000, flexMax = 0;
float rawFlex = 0;
float x, y, z = 0;

ResponsiveAnalogRead analog1(FSR_PIN, true);
ResponsiveAnalogRead analog2(FLEX_PIN, true);

void setup() {
  // Set up the accelerometer
  //  Serial.begin(115200);
  //  while (!Serial) {};
  LIS.begin(WIRE, 0x19); //IIC init
  delay(100);
  LIS.setFullScaleRange(LIS3DHTR_RANGE_2G);
  LIS.setOutputDataRate(LIS3DHTR_DATARATE_50HZ);
  LIS.setHighSolution(true); //High solution enable

  // Set up the audio shield.
  AudioMemory(128);
  audioShield.enable();
  audioShield.volume(.8);

  // filter out DC & extremely low frequencies
  filter1.frequency(30);
  // amplify mic signal to useful range
  amp1.gain(.9);

  // reverb
  reverb.damping(.2);
  reverb.roomsize(.5);

  // mix
  // direct
  mixer.gain(0, .15);
  // granular
  mixer.gain(1, 1.);
  // reverb
  mixer.gain(2, .55);

  // Final level boost/cut
  amp2.gain(1.25);

  sg.setParamValue(GRAIN_SPEED, 1.);
  setParameters();

  //  Serial.println("Setup done.");
}

void loop() {
  processSensorData();

  setParameters();

  if (DEBUG_SENSORS) {
    debugSensors();
  }

  if (DEBUG_PARAMS) {
    debugParams();
  }
}

/**
 * Process and normalise sensor data.
 */
void processSensorData() {
  LIS.getAcceleration(&x, &y, &z);

  analog1.update();
  analog2.update();

  fsr = analog1.getValue();
  flex = analog2.getValue();

  rawFlex = flex;
  flex = (flex - 625.) / 150.;
  flex = clamp(flex, MIN_GRAIN_DENSITY, 1.);

  rawFSR = fsr;
  fsr = (fsr - 25.) / 350.;
  fsr = clamp(fsr, 0., 1.);
}

void setParameters() {
  // Grain-table freeze
  if (flex > FREEZE_THRESHOLD && sg.getParamValue("Freeze") < .1) {
    sg.setParamValue(FREEZE, 1.);
  } else if (flex <= FREEZE_THRESHOLD && sg.getParamValue("Freeze") > .9) {
    sg.setParamValue(FREEZE, 0);
  }

  // Grain density
  float density = flex > DENSITY_FLIP_THRESHOLD ? 1 - flex : flex;
  sg.setParamValue(GRAIN_DENSITY, abs(density) * MAX_GRAIN_DENSITY);

  // Grain size
  float gSize = max(MIN_GRAIN_SIZE, min(flex, abs(x))) * MAX_GRAIN_SIZE;
  sg.setParamValue(GRAIN_SIZE, gSize);

  // Grain regularity
  sg.setParamValue(GRAIN_REGULARITY, fsr);

  // Grain speed
  float nextSpeed = sg.getParamValue(GRAIN_SPEED) - (z * GRAIN_SPEED_DELTA);
  nextSpeed = clamp(nextSpeed, -MAX_GRAIN_SPEED, MAX_GRAIN_SPEED);
  nextSpeed = flip(nextSpeed, MIN_GRAIN_SPEED);
  sg.setParamValue(GRAIN_SPEED, nextSpeed);

  // Reverb amount
  reverb.roomsize(abs(y) * .5);
  mixer.gain(2, abs(y) * .5);
}

void debugSensors() {
  if(rawFlex > flexMax) flexMax = rawFlex;
  if(rawFlex < flexMin) flexMin = rawFlex;
  
  if (PLOT) Serial.print(' '); else Serial.print("fsr: ");
  Serial.print(fsr);
  if (DEBUG_SENSOR_DETAIL) {
    if (PLOT) Serial.print(' '); else Serial.print(", rawFSR: ");
    Serial.print(rawFSR);
    if (PLOT) Serial.print(' '); else Serial.print(", rawflex: ");
    Serial.print(rawFlex);
    if (PLOT) Serial.print(' '); else Serial.print(" (min: ");
    Serial.print(flexMin);
    if (PLOT) Serial.print(' '); else Serial.print("/ max: ");
    Serial.print(flexMax);
    Serial.print(")"); 
  }
  if (PLOT) Serial.print(' '); else Serial.print(", flex: ");
  Serial.print(flex);
  if (PLOT) Serial.print(' '); else Serial.print(", x: ");
  Serial.print(x);
  if (PLOT) Serial.print(' '); else Serial.print(", y: ");
  Serial.print(y);
  if (PLOT) Serial.print(' '); else Serial.print(", z: ");
  Serial.println(z);
  if (!PLOT) {
    Serial.print("audio memory usage: ");
    Serial.println(AudioMemoryUsage());
  }
  delay(100);
}

void debugParams() {
  Serial.print("grain size: ");
  Serial.print(sg.getParamValue(GRAIN_SIZE));
  Serial.print(", grain density: ");
  Serial.print(sg.getParamValue(GRAIN_DENSITY));
  Serial.print(", grain speed: ");
  Serial.print(sg.getParamValue(GRAIN_SPEED));
  Serial.print(", grain regularity: ");
  Serial.print(sg.getParamValue(GRAIN_REGULARITY));
  Serial.print(", freeze: ");
  Serial.print(sg.getParamValue(FREEZE) == 1.0 ? "on" : "off");
  Serial.println(' ');
  delay(100);
}

/**
 * Clamp a value between minimum and maximum values.
 */
float clamp(float param, float min, float max) {
  if (param < min) {
    param = min;
  } else if (param > max) {
    param = max;
  }

  return param;
}

/**
 * Flip a value around zero.
 * If the value exceeds the negative threshold, flip to the positive conjugate.
 * If the value falls below the positive threshold, flip to the negative.
 */
float flip(float param, float threshold) {
  if (param > -threshold && param < 0) {
    param = threshold;
  } else if (param < threshold && param > 0 ) {
    param = -threshold;
  }

   return param;
}
