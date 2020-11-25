/*
bundle config --local path vendor/bundle
bundle install
bundle exec arduino_ci.rb  --skip-examples-compilation
 */
#include "Arduino.h"
#include "ArduinoUnitTests.h"
#include "PID_AutoTune_v0.h"
#include "PID_v1.cpp"

byte ATuneModeRemember = 2;
double input = 80, output = 50, setpoint = 180;
double kp = 2, ki = 0.5, kd = 2;

double kpmodel = 1.5, taup = 100, theta[50];
double outputStart = 5;
double aTuneStep = 50, aTuneNoise = 1, aTuneStartValue = 100;
unsigned int aTuneLookBack = 20;

boolean tuning = false;
unsigned long modelTime;

PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
PID_ATune aTune(&input, &output);

void setup();
void loop();
void changeAutoTune();
void AutoTuneHelper(boolean start);
void report();
void DoModel();

void setup() {
  //   set up simulation data
  for (byte i = 0; i < 50; i++) {
    theta[i] = outputStart;
  }
  modelTime = 0;

  // Setup the pid
  myPID.SetMode(AUTOMATIC);

  // Set the output to the desired starting frequency.
  output = aTuneStartValue;
  aTune.SetNoiseBand(aTuneNoise);
  aTune.SetOutputStep(aTuneStep);
  aTune.SetLookbackSec((int)aTuneLookBack);
  AutoTuneHelper(true);
  tuning = true;
}

void loop() {

  unsigned long now = millis();

  if (tuning) {
    byte val = (aTune.Runtime());
    if (val != 0) {
      tuning = false;
    }
    if (!tuning) { // we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp, ki, kd);
      AutoTuneHelper(false);
    }
  } else
    myPID.Compute();

  theta[30] = output;
  if (now >= modelTime) {
    modelTime += 100;
    DoModel();
  }
}

void AutoTuneHelper(boolean start) {
  if (start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}

void report() {
  std::cout << "setpoint: " << setpoint;
  std::cout << "; input: " << input;
  std::cout << "; output: " << output;
  if (tuning) {
    std::cout << "; tuning mode";
  } else {
    std::cout << "; kp: " << myPID.GetKp();
    std::cout << "; ki: " << myPID.GetKi();
    std::cout << "; kd: " << myPID.GetKd();
  }
  std::cout << std::endl;
}

void DoModel() {
  // cycle the dead time
  for (byte i = 0; i < 49; i++) {
    theta[i] = theta[i + 1];
  }
  // compute the input
  input = (kpmodel / taup) * (theta[0] - outputStart) + input * (1 - 1 / taup) +
          ((float)random(-10, 10)) / 100;
}

unittest(test) {

  // Set Mode Manual
  myPID.SetMode(MANUAL);
  assertEqual(MANUAL, myPID.GetMode());
  setup();

  // After setup() the Mode should be set to Automatic
  assertEqual(AUTOMATIC, myPID.GetMode());

  // Check Kp, Ki, Kd
  assertEqual(2.0, myPID.GetKp());
  assertEqual(0.5, myPID.GetKi());
  assertEqual(2.0, myPID.GetKd());

  // // Set Kp, Ki, Kd and check valuesS
  myPID.SetTunings(kp = 1.5, ki = 0.8, kd = 1.5);
  assertEqual(1.5, myPID.GetKp());
  assertEqual(0.8, myPID.GetKi());
  assertEqual(1.5, myPID.GetKd());

  assertEqual(DIRECT, myPID.GetDirection());

  // change tuning value in loops.
  // This never seems to end
  for (int i = 1; i < 1000; i++) {
    delay(100);
    loop();
    if ((i % 5) == 0) {
      // std::cout<< i << ": ";
      // report();
    }
  }
}

unittest_main()
