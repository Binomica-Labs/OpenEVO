// *******************************************************************************************************************************************
// OpenEvo - Open Source Directed Evolution Engine
// Cycler Version: 1.0.0 (Initial OpenEvo Release)
// Firmware File: 2026-06-04_OpenEvo_Firmware_V1.ino  (Production 1.0)
//   History (pre-1.0 development builds):
//   - Removed legacy quadratic OD calibration. OD model is now
//     inverse (OD = a/IR + b) with a 2-point linear fallback.
//   - Removed redundant legacy irMid/odMid config fields (they
//     duplicated irMidHigh/odMidHigh). Old configs still load:
//     irMid/odMid keys are mapped onto irMidHigh/odMidHigh.
// Copyright 2025 Binomica Labs
// OG Author: Sebastian S. Cocioba
// Mods by Zane Chan, Pin-Che Huang and Phillip Kyriakakis
// License: MIT License (https://opensource.org/licenses/MIT)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files
// (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge,
// publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do
// so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
// FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
// *******************************************************************************************************************************************

// LIBRARIES
#include <PID_v1.h>
#include <math.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
#include <StateMachine.h>
#include "Arduino.h"
#include "Adafruit_TLC5947.h"
// #include <NTC_Thermistor.h> // REMOVED to fix library conflict
// #include <SmoothThermistor.h> // REMOVED to fix library conflict
#include <Adafruit_MotorShield.h>

// ==========================================
// EMBEDDED THERMISTOR CLASS (Fixes Library Issue)
// ==========================================
class Thermistor {
  public:
    virtual double readCelsius() = 0;
};

class NTC_Thermistor : public Thermistor {
  private:
    int _pin;
    double _referenceResistance;
    double _nominalResistance;
    double _nominalTemperature;
    double _bValue;

  public:
    NTC_Thermistor(int pin, double referenceResistance, double nominalResistance, double nominalTemperature, double bValue) {
      _pin = pin;
      _referenceResistance = referenceResistance;
      _nominalResistance = nominalResistance;
      _nominalTemperature = nominalTemperature;
      _bValue = bValue;
    }

    double readCelsius() override {
      int raw = analogRead(_pin);
      if (raw == 0 || raw == 1023) return -273.15; // Avoid divide by zero or infinity
      
      // Calculate resistance
      double resistance = _referenceResistance / (1023.0 / raw - 1.0);
      
      // Steinhart-Hart Beta equation
      double steinhart;
      steinhart = resistance / _nominalResistance;     // (R/Ro)
      steinhart = log(steinhart);                  // ln(R/Ro)
      steinhart /= _bValue;                        // 1/B * ln(R/Ro)
      steinhart += 1.0 / (_nominalTemperature + 273.15); // + (1/To)
      steinhart = 1.0 / steinhart;                 // Invert
      steinhart -= 273.15;                         // Convert to Celsius
      
      return steinhart;
    }
};

class SmoothThermistor : public Thermistor {
  private:
    Thermistor* _thermistor;
    int _factor;
    double _avg = 0;
    bool _first = true;

  public:
    SmoothThermistor(Thermistor* thermistor, int factor) {
      _thermistor = thermistor;
      _factor = factor;
    }

    double readCelsius() override {
      double current = _thermistor->readCelsius();
      // Simple exponential moving average
      if (_first) {
        _avg = current;
        _first = false;
      } else {
        _avg = (current + (_avg * (_factor - 1))) / _factor;
      }
      return _avg;
    }
};
// ==========================================

// DEFAULT CONFIGURATION
const char softwareVersion[] PROGMEM = "v1.9.2.0";
const double EMERGENCY_TEMP_LIMIT = 40.0;  // <----- CHANGE EMERGENCY TEMP LIMIT HERE (°C)
double incubationSetpointTemp = 30.00;     // <----- CHANGE DEFAULT SETPOINT TEMPERATURE HERE (°C)
int motorStirringSpeed = 90;               // <----- CHANGE DEFAULT STIRRING SPEED HERE (0-255)
int OD940LEDBrightness = 500;              // <----- CHANGE DEFAULT IR LED BRIGHTNESS HERE (0-4095)
int pumpOneSpeed = 1161;                   // <----- CHANGE PUMP 1 SPEED HERE
int pumpTwoSpeed = 1162;                   // <----- CHANGE PUMP 2 SPEED HERE
int pumpThreeSpeed = 1162;                 // <----- CHANGE PUMP 3 SPEED HERE
int pumpFourSpeed = 1161;                  // <----- CHANGE PUMP 4 SPEED HERE

// CALIBRATION POINTS (4-point calibration method)
int irLower = 13825;    // <----- Point 1: Low OD IR value
float odLower = 0.0;    // <----- Point 1: Low OD value
// NOTE: "MidLow" = low OD value (high IR), "MidHigh" = high OD value (low IR)
int irMidLow = 5496;    // <----- Point 2: Mid-Low OD (higher IR = less cells)
float odMidLow = 1.0;   // <----- Point 2: Mid-Low OD value
int irMidHigh = 2430;   // <----- Point 3: Mid-High OD (lower IR = more cells)
float odMidHigh = 4.0;  // <----- Point 3: Mid-High OD value
int irUpper = 1224;     // <----- Point 4: High OD IR value
float odUpper = 5.0;    // <----- Point 4: High OD value

// Inverse calibration (OD = a/IR + b) - empirically accurate for light scattering measurements
float inverseA = 0.0;        // Inverse coefficient a (numerator)
float inverseB = 0.0;        // Inverse coefficient b (offset)
bool useInverse = false;     // Flag to use inverse calibration (preferred)

// HARDWARE PINS - FROM WORKING VERSION
const int thermistorMediaPin = A8;           // Media thermistor
const int thermistorHeaterPlatePin = A9;     // Heater thermistor
const int heaterPlateTopPinEnable = 44;
const int heaterPlateTopPinInputOne = 42;
const int heaterPlateTopPinInputTwo = 40;
const int heaterPlateBottomPinEnable = 34;
const int heaterPlateBottomPinInputOne = 38;
const int heaterPlateBottomPinInputTwo = 36;
const int motorStirringPinInputOne = 48;
const int motorStirringPinInputTwo = 46;
const int sdCardCSPin = 53;
const int ledDriverPinData = 25;
const int ledDriverPinClock = 27;
const int ledDriverPinLatch = 23;

// Interface Buttons
const int buttonUP = 26;
const int buttonSEL = 24;
const int buttonDOWN = 22;

// TIMING
const int interval = 1000;  // Live interface update interval (1 second)
const int sdLogInterval = 10000;  // SD card logging interval (10 seconds)
const unsigned long SERIAL_BAUD = 115200;

// STATE FLAGS
bool isPaused = false;
bool isRunning = false;
bool dilutionEventFlag = false;  // Set to true when dilution is triggered, reset after serial send
bool dilutionEventForSD = false; // Separate flag for SD logging (reset after SD write)
float savedPeakOD = 0.0;  // Stores the OD value at the moment dilution was triggered

// BUTTON STATE VARIABLES
bool lastButtonUPState = HIGH;
bool lastButtonSELState = HIGH;
bool lastButtonDOWNState = HIGH;
unsigned long lastButtonTime = 0;
const unsigned long buttonDebounceDelay = 50;

// DATA VARIABLES
unsigned long programStartTime = 0;
float currentOD940 = 0.0;
double mediaTemp = 0.0;
double heaterPlateTemp = 0.0;
double ambientTemp = 0.0;
double outputPWM = 0.0;

// CYCLE TRACKING
int currentCycle = 1;
int currentStep = 1;
int totalSteps = 10;
char currentMediaType[9] = "NEUTRAL";  // Reduced size to match stepProgram

// LED CYCLING CONTROL
int cyclesPerLEDChange = 1;  // <----- CHANGE CYCLES PER LED CHANGE HERE
int currentLEDBrightness = 0;
bool currentLEDState = false;
int currentLEDPin = 1;

// LED CALIBRATION MODE - when true, disables automatic LED restoration after OD measurement
bool ledCalibrationMode = false;
int calibrationLEDChannel = 0;
int calibrationLEDBrightness = 0;

// LED CALIBRATION DATA - user-measured intensity (mW/cm²) and wavelength (nm) for each LED
// These values are used by the interface to display actual irradiance
// Stored on SD card in led_cal.txt
struct LEDCalibration {
    float intensity;      // mW/cm² at 100% PWM
    int wavelength;       // nm
};
LEDCalibration ledCalibration[6] = {
    {0.0, 0},  // LED 1
    {0.0, 0},  // LED 2
    {0.0, 0},  // LED 3
    {0.0, 0},  // LED 4
    {0.0, 0},  // LED 5
    {0.0, 0}   // LED 6
};

// STEP DATA STORAGE
struct StepData {
    char mediaType[9];  // Reduced size - "POSITIVE" is 8 chars + null
    int ledBrightness;
    float temperature;
};
StepData stepProgram[15];  // Increased to 15 steps max
int totalProgramSteps = 3;

// VOLUME CALIBRATION
float volumePerDispensation = 0.1;  // mL per dispensation

// WATCHDOG PROTECTION
bool watchdogActive = false;
double preMediaChangeOD = 0.0;
unsigned long mediaChangeStartTime = 0;
unsigned long watchdogTriggerTime = 0;  // NEW: Track when watchdog was triggered
const float WATCHDOG_OD_DROP_REQUIRED = 0.3;
const unsigned long WATCHDOG_MIN_ALERT_TIME = 5000; // Minimum 5 seconds alert before auto-clear
bool mediaChangeInProgress = false;
unsigned long lastMediaChangeTime = 0;
const unsigned long MEDIA_CHANGE_COOLDOWN = 10000; // 10 seconds

// FAN RAMP-UP SYSTEM
bool stirringRampUpActive = false;
unsigned long stirringRampStartTime = 0;
const unsigned long STIRRING_RAMP_DURATION = 20000; // 20 seconds
const int STIRRING_RAMP_START_SPEED = 10;

// SD CARD DATA LOGGING (comprehensive)
bool sdCardAvailable = false;
bool showingSDError = false;  // Flag to prevent LCD overwriting SD error messages
bool forceLCDUpdate = false;  // Flag to force immediate LCD update (used after JUMP, program changes, etc.)
File dataFile; // Global file handle for OEVO.csv to ensure it can be accessed by the STOP command.
char sdFileName[16] = "OEVO.csv";
bool sdHeaderWritten = false;
unsigned long lastSDSync = 0;
const unsigned long SD_SYNC_INTERVAL = 30000; // Sync every 30 seconds
int max_dispensations = 400;  //<-- CHANGE MAX DISPENSATIONS HERE

// CALIBRATION
float odCalibSlope = -0.00072727; // <-- DEFAULT CALIBRATION SLOPE
float odCalibIntercept = 5.945455;  // <-- DEFAULT CALIBRATION INTERCEPT
bool calibrationSet = false;
float dynamicThreshold = 5.5; // <----- CHANGE DEFAULT THRESHOLD HERE (OD units)

// RAW CALIBRATION POINTS (for config sync)
// Old calibration variables removed - now using irLower, odLower, irUpper, odUpper

// SENSOR VARIABLES - FROM WORKING VERSION
unsigned long IR;
const uint16_t IRblank = 25000; // <-- CHANGE IR BLANK/ZERO DEFAULT HERE

// HARDWARE OBJECTS
Adafruit_MotorShield MotorShield = Adafruit_MotorShield();
Adafruit_DCMotor *pumpOne = MotorShield.getMotor(1);
Adafruit_DCMotor *pumpTwo = MotorShield.getMotor(2);
Adafruit_DCMotor *pumpThree = MotorShield.getMotor(3);
Adafruit_DCMotor *pumpFour = MotorShield.getMotor(4);
Adafruit_TLC5947 ledDriver = Adafruit_TLC5947(1, ledDriverPinClock, ledDriverPinData, ledDriverPinLatch);
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
RTC_DS3231 rtc;
LiquidCrystal_I2C lcd(0x27, 20, 4);
Thermistor* mediaTempThermistorSmooth = NULL;
Thermistor* heaterPlateTempThermistorSmooth = NULL;
// PID Tuning Parameters (from July 31 working version)
const double aggKp = 30, aggKi = 1.5, aggKd = 1;        // Aggressive tuning (very fast response)
const double consKp = 10, consKi = 0.4, consKd = 0.5;   // Conservative tuning (stronger to reach setpoint)
const double gapThreshold = 0.3;                         // Switch to conservative closer to setpoint
const double setpointOffset = 0.25;                      // Offset for better centering
const double pwmSafetyLimit = 160.0;                     // PWM safety limit (increased to 160 for faster heating)

PID mediaTempPID(&mediaTemp, &outputPWM, &incubationSetpointTemp, consKp, consKi, consKd, DIRECT);  // Start conservative

// STATE MACHINE
StateMachine machine = StateMachine();
State* stateMenu;
State* stateStandby;

unsigned long previousMillis = 0;
unsigned long previousSDLogMillis = 0;

// DATA AVERAGING FOR SD LOGGING
float odSum = 0.0;
float tempSum = 0.0;
long irSum = 0;
int dataPointCount = 0;

// FORWARD DECLARATIONS
void setMultiplexerFocus(uint8_t bus);
void initializeHardware();
void calculateOD940();
void heaterCompute();
void stirringON();
void stirringOFF();
void allPumpsStop();
void heaterOFF();
void stateFunctionMenu();
void stateFunctionStandby();
bool transitionMenuToStandby();
void performMediaChange();
void advanceStep();
void applyStepSettings(int stepIndex);
void applyStepSettingsWithLED(int stepIndex);
void updateStirringRamp();
void logDataToSD();
void logAveragedDataToSD();
void initializeSDCard();
void saveConfigToSD();
void saveCyclerToSD();
void saveLEDCalibrationToSD();
void loadConfigFromSD();
void loadCyclerFromSD();
void loadLEDCalibrationFromSD();
void loadLastExperimentState();
void getLastSDLine();
void getRecentSDData(int numLines);
void checkSDCardStatus();
void displaySDError(const char* operation);

void initializeDefaultProgram(); // Function prototype
void loadConfiguration(); // Function prototype

void setup() {
    Serial.begin(SERIAL_BAUD);
    
    Wire.begin();
    
    // Initialize physical buttons
    pinMode(buttonUP, INPUT_PULLUP);
    pinMode(buttonSEL, INPUT_PULLUP);
    pinMode(buttonDOWN, INPUT_PULLUP);
    
    initializeHardware();
    initializeDefaultProgram();
    initializeSDCard();
    loadConfiguration();
    
    // Set up state machine
    stateMenu = machine.addState(&stateFunctionMenu);
    stateStandby = machine.addState(&stateFunctionStandby);
    stateMenu->addTransition(&transitionMenuToStandby, stateStandby);
    
    // Start in menu mode
    // The stateFunctionMenu will display the welcome message
    machine.transitionTo(stateMenu);
}

void handlePhysicalButtons() {
    unsigned long currentTime = millis();
    
    // Debounce buttons
    if (currentTime - lastButtonTime > buttonDebounceDelay) {
        
        // Read current button states
        bool currentUPState = digitalRead(buttonUP);
        bool currentSELState = digitalRead(buttonSEL);
        bool currentDOWNState = digitalRead(buttonDOWN);
        
        // UP button pressed (LOW because INPUT_PULLUP)
        if (currentUPState == LOW && lastButtonUPState == HIGH) {
            // Button pressed
            if (isRunning) {
                // Pause/Resume experiment (equivalent to all buttons)
                if (isPaused) {
                    Serial.print(F("$DEBUG,BUTTON_UNPAUSE_ATTEMPT,sdCardAvailable="));
                    Serial.println(sdCardAvailable ? F("TRUE") : F("FALSE"));
                    
                    // Check if SD card is available before unpausing
                    if (!sdCardAvailable) {
                        Serial.println(F("❌ CANNOT UNPAUSE - No SD card! Insert SD card first."));
                        displaySDError("Unpause Failed");
                    } else {
                        isPaused = false;
                        showingSDError = false;  // Clear error message flag
                        Serial.println(F("$DEBUG,UNPAUSED"));
                        Serial.println(F("$DEBUG,EXPERIMENT_UNPAUSED"));
                        Serial.print(F("$DEBUG,UP_BUTTON_UNPAUSE,isPaused="));
                        Serial.println(isPaused ? F("TRUE") : F("FALSE"));
                        
                        // Re-open data file if SD card is available
                        if (sdCardAvailable && !dataFile) {
                            Serial.println(F("$DEBUG,BUTTON_REOPENING_DATA_FILE"));
                            dataFile = SD.open(sdFileName, FILE_WRITE);
                            if (dataFile) {
                                Serial.println(F("✅ Data file re-opened after SD recovery"));
                            } else {
                                Serial.println(F("❌ Button: Failed to reopen data file!"));
                            }
                        }
                    }
                } else {
                    isPaused = true;
                    stirringOFF();
                    Serial.println(F("$DEBUG,PAUSED"));
                    Serial.println(F("$DEBUG,EXPERIMENT_PAUSED"));
                    Serial.print(F("$DEBUG,UP_BUTTON_PAUSE,isPaused="));
                    Serial.println(isPaused ? F("TRUE") : F("FALSE"));
                    
                    // Force immediate LCD update when pausing via button
                    setMultiplexerFocus(0);
                    lcd.clear();
                    lcd.print(F("PAUSED"));
                    lcd.setCursor(0, 1);
                    lcd.print(F("Cycle:"));
                    lcd.print(currentCycle);
                    lcd.print(F(" Step:"));
                    lcd.print(currentStep);
                    lcd.print(F("/"));
                    lcd.print(totalProgramSteps);
                    lcd.setCursor(0, 2);
                    lcd.print(F("Press any button"));
                    lcd.setCursor(0, 3);
                    lcd.print(F("to resume"));
                    delay(10); // Brief delay for LCD update
                }
            } else {
                // Start experiment - check for SD card first
                if (!sdCardAvailable) {
                    Serial.println(F("❌ Cannot start - No SD card!"));
                    displaySDError("No SD at Start");
                } else {
                    isRunning = true;
                    isPaused = false;
                    Serial.println(F("UP: Experiment started via button"));
                }
            }
            lastButtonTime = currentTime;
        }
        
        // SEL button pressed
        if (currentSELState == LOW && lastButtonSELState == HIGH) {
            // SEL button pressed
            if (isRunning) {
                // Pause/Resume experiment (equivalent to all buttons)
                if (isPaused) {
                    // Check if SD card is available before unpausing
                    if (!sdCardAvailable) {
                        Serial.println(F("❌ CANNOT UNPAUSE - No SD card! Insert SD card first."));
                        displaySDError("Unpause Failed");
                    } else {
                        isPaused = false;
                        showingSDError = false;  // Clear error message flag
                        Serial.println(F("$DEBUG,UNPAUSED"));
                        Serial.println(F("$DEBUG,EXPERIMENT_UNPAUSED"));
                        Serial.print(F("$DEBUG,SEL_BUTTON_UNPAUSE,isPaused="));
                        Serial.println(isPaused ? F("TRUE") : F("FALSE"));
                        
                        // Re-open data file if SD card is available
                        if (sdCardAvailable && !dataFile) {
                            dataFile = SD.open(sdFileName, FILE_WRITE);
                            if (dataFile) {
                                Serial.println(F("✅ Data file re-opened after SD recovery"));
                            }
                        }
                    }
                } else {
                    isPaused = true;
                    stirringOFF();
                    Serial.println(F("$DEBUG,PAUSED"));
                    Serial.println(F("$DEBUG,EXPERIMENT_PAUSED"));
                    Serial.print(F("$DEBUG,SEL_BUTTON_PAUSE,isPaused="));
                    Serial.println(isPaused ? F("TRUE") : F("FALSE"));
                    
                    // Force immediate LCD update when pausing via button
                    setMultiplexerFocus(0);
                    lcd.clear();
                    lcd.print(F("PAUSED"));
                    lcd.setCursor(0, 1);
                    lcd.print(F("Cycle:"));
                    lcd.print(currentCycle);
                    lcd.print(F(" Step:"));
                    lcd.print(currentStep);
                    lcd.print(F("/"));
                    lcd.print(totalProgramSteps);
                    lcd.setCursor(0, 2);
                    lcd.print(F("Press any button"));
                    lcd.setCursor(0, 3);
                    lcd.print(F("to resume"));
                    delay(10); // Brief delay for LCD update
                }
            } else {
                // Start experiment - check for SD card first
                if (!sdCardAvailable) {
                    Serial.println(F("❌ Cannot start - No SD card!"));
                    displaySDError("No SD at Start");
                } else {
                    isRunning = true;
                    isPaused = false;
                    Serial.println(F("SEL: Experiment started via button"));
                }
            }
            lastButtonTime = currentTime;
        }
        
        // DOWN button pressed
        if (currentDOWNState == LOW && lastButtonDOWNState == HIGH) {
            // DOWN button pressed
            if (isRunning) {
                // Pause/Resume experiment (equivalent to all buttons)
                if (isPaused) {
                    // Check if SD card is available before unpausing
                    if (!sdCardAvailable) {
                        Serial.println(F("❌ CANNOT UNPAUSE - No SD card! Insert SD card first."));
                        displaySDError("Unpause Failed");
                    } else {
                        isPaused = false;
                        showingSDError = false;  // Clear error message flag
                        Serial.println(F("$DEBUG,UNPAUSED"));
                        Serial.println(F("$DEBUG,EXPERIMENT_UNPAUSED"));
                        Serial.print(F("$DEBUG,DOWN_BUTTON_UNPAUSE,isPaused="));
                        Serial.println(isPaused ? F("TRUE") : F("FALSE"));
                        
                        // Re-open data file if SD card is available
                        if (sdCardAvailable && !dataFile) {
                            dataFile = SD.open(sdFileName, FILE_WRITE);
                            if (dataFile) {
                                Serial.println(F("✅ Data file re-opened after SD recovery"));
                            }
                        }
                    }
                } else {
                    isPaused = true;
                    stirringOFF();
                    Serial.println(F("$DEBUG,PAUSED"));
                    Serial.println(F("$DEBUG,EXPERIMENT_PAUSED"));
                    Serial.print(F("$DEBUG,DOWN_BUTTON_PAUSE,isPaused="));
                    Serial.println(isPaused ? F("TRUE") : F("FALSE"));
                    
                    // Force immediate LCD update when pausing via button
                    setMultiplexerFocus(0);
                    lcd.clear();
                    lcd.print(F("PAUSED"));
                    lcd.setCursor(0, 1);
                    lcd.print(F("Cycle:"));
                    lcd.print(currentCycle);
                    lcd.print(F(" Step:"));
                    lcd.print(currentStep);
                    lcd.print(F("/"));
                    lcd.print(totalProgramSteps);
                    lcd.setCursor(0, 2);
                    lcd.print(F("Press any button"));
                    lcd.setCursor(0, 3);
                    lcd.print(F("to resume"));
                    delay(10); // Brief delay for LCD update
                }
            } else {
                // Start experiment - check for SD card first
                if (!sdCardAvailable) {
                    Serial.println(F("❌ Cannot start - No SD card!"));
                    displaySDError("No SD at Start");
                } else {
                    isRunning = true;
                    isPaused = false;
                    Serial.println(F("DOWN: Experiment started via button"));
                }
            }
            lastButtonTime = currentTime;
        }
        
        // Update last button states
        lastButtonUPState = currentUPState;
        lastButtonSELState = currentSELState;
        lastButtonDOWNState = currentDOWNState;
    }
}

// ============================================================================
// MAIN LOOP
// ============================================================================
// The Arduino continuously executes this function.
// Handles two main input sources:
//   1. Physical buttons (3 buttons on the device)
//   2. Serial commands (from Python interface via USB)
//
// State machine runs in background via stateFunctionStandby() or stateFunctionMenu()

void loop() {
    
    // ===== PHYSICAL BUTTON HANDLING =====
    // Check for button presses (pause/resume, start experiment)
    handlePhysicalButtons();
    
    // ============================================================================
    // SERIAL COMMAND PROCESSING
    // ============================================================================
    // Receives commands from Python interface over USB serial connection.
    // Commands are newline-terminated strings (e.g., "START_TURBIDOSTAT\n")
    //
    // PROTOCOL:
    //   - Commands are text-based, newline-terminated
    //   - Parameters separated by commas (e.g., "SET_STEP,0,NEUTRAL,2000")
    //   - Responses start with $ (e.g., "$DEBUG,PAUSED")
    //
    // BUFFER SIZE: 300 bytes (handles long commands like program uploads)
    
    if (Serial.available() > 0) {
        char cmd[300];  // Large buffer for commands with many parameters
        int len = Serial.readBytesUntil('\n', cmd, sizeof(cmd) - 1);
        cmd[len] = '\0';  // Null terminate the string
        
        // Remove trailing whitespace and carriage returns
        while (len > 0 && (cmd[len-1] == ' ' || cmd[len-1] == '\r')) {
            cmd[--len] = '\0';
        }

        // OVERFLOW PROTECTION: If buffer filled completely, flush remaining bytes
        if (len >= (int)sizeof(cmd) - 2 && Serial.peek() != -1) {
            while (Serial.available() > 0) {
                char c = Serial.read();
                if (c == '\n') break;
            }
        }
        
        // Echo received command for debugging
        if (len > 0) {
            Serial.print(F("📥 RECEIVED: "));
            Serial.println(cmd);
        }

        
        // ====================================================================
        // START_TURBIDOSTAT - Begin Experiment
        // ====================================================================
        // Initializes and starts a new experiment run.
        // This is the main "go" command from the Python interface.
        //
        // PREREQUISITES:
        //   - SD card must be present (data logging required)
        //   - Cycler program must be uploaded (SET_STEP commands)
        //   - Configuration must be set (temperature, threshold, etc.)
        //
        // INITIALIZATION SEQUENCE:
        //   1. Verify SD card availability (safety check)
        //   2. Open data file (OEVO.csv) for logging
        //   3. Initialize LED cycling from step program
        //   4. Turn off all LEDs, then activate first LED
        //   5. Start stirring motor ramp-up (gradual acceleration)
        //   6. Set experiment flags (isRunning=true, isPaused=false)
        //   7. Transition to standby state (main experiment loop)
        //
        if (strncmp(cmd, "START_TURBIDOSTAT", 17) == 0) {
            Serial.println(F("$DEBUG,START_TURBIDOSTAT_RECEIVED"));
            
            // ===== SAFETY CHECK: SD CARD REQUIRED =====
            // Cannot start experiment without SD card - would lose all data!
            if (!sdCardAvailable) {
                Serial.println(F("$ERROR,CANNOT_START_NO_SD_CARD"));
                Serial.println(F("❌ EXPERIMENT NOT STARTED - Insert SD card first!"));
                displaySDError("No SD at Start");
                return;  // Abort start command
            }

            // ===== HEADER WRITING REMOVED =====
            // Header is now written by the interface via SD_WRITE_HEADER command
            // This prevents duplicate headers when starting a new experiment
            // The interface sends: INIT_SD_CARD -> SD_WRITE_HEADER -> START_TURBIDOSTAT
            
            // ===== DATA FILE INITIALIZATION =====
            // Open the main data file (OEVO.csv) for the entire experiment duration.
            // File stays open (with periodic closes after writes) for performance.
            if (sdCardAvailable && !dataFile) {
                dataFile = SD.open(sdFileName, FILE_WRITE);
                if (dataFile) {
                    Serial.println(F("DEBUG: Main data file opened for logging."));
                } else {
                    Serial.println(F("ERROR: Failed to open main data file for logging!"));
                }
            }

            // ===== EXIT CALIBRATION MODE IF ACTIVE =====
            // Starting an experiment should always exit LED calibration mode
            if (ledCalibrationMode) {
                ledCalibrationMode = false;
                calibrationLEDChannel = 0;
                calibrationLEDBrightness = 0;
                Serial.println(F("$DEBUG,CALIBRATION_MODE_EXITED_ON_START"));
            }
            
            // ===== LED CYCLING INITIALIZATION =====
            // Load settings from CURRENT step (important for resuming experiments)
            int stepIndex = currentStep - 1;  // Convert to 0-based index
            if (stepIndex < 0 || stepIndex >= totalProgramSteps) {
                stepIndex = 0;  // Safety fallback
            }
            
            strcpy(currentMediaType, stepProgram[stepIndex].mediaType);
            currentLEDBrightness = stepProgram[stepIndex].ledBrightness;
            currentLEDState = (currentLEDBrightness > 0);
            
            Serial.print(F("$DEBUG,LOADING_STEP_"));
            Serial.print(currentStep);
            Serial.print(F(",Media:"));
            Serial.print(currentMediaType);
            Serial.print(F(",LED:"));
            Serial.println(currentLEDState ? "ON" : "OFF");
            
            // Calculate which LED pin to use based on current cycle and cyclesPerLEDChange
            int ledCycleIndex = (currentCycle - 1) / cyclesPerLEDChange;
            int ledPin = (ledCycleIndex % 6) + 1;  // LED pins 1-6
            currentLEDPin = ledPin;
            
            // Turn off all 15 LED channels first (clean slate)
            for (int i = 1; i <= 15; i++) {
                ledDriver.setPWM(i, 0);
            }
            
            // Activate correct LED pin at correct brightness
            if (currentLEDState) {
                ledDriver.setPWM(ledPin, currentLEDBrightness);
            }
            ledDriver.write();  // Apply LED settings to hardware
            
            // ===== STIRRING MOTOR RAMP-UP =====
            // Gradual acceleration prevents culture disturbance and motor stress
            stirringRampUpActive = true;
            stirringRampStartTime = millis();
            
            // ===== EXPERIMENT STATE ACTIVATION =====
            // Set flags BEFORE transitioning to ensure proper state
            isPaused = false;
            isRunning = true;
            programStartTime = millis();
            
            Serial.println(F("$DEBUG,EXPERIMENT_STARTED"));
            Serial.print(F("$DEBUG,STATE_FLAGS,isRunning:"));
            Serial.print(isRunning ? F("TRUE") : F("FALSE"));
            Serial.print(F(",isPaused:"));
            Serial.println(isPaused ? F("TRUE") : F("FALSE"));
            
            // ===== STATE MACHINE TRANSITION =====
            // Enter standby state, which runs the main experiment loop
            machine.transitionTo(stateStandby);
            Serial.println(F("$DEBUG,TRANSITIONED_TO_STANDBY"));
        }
        // ====================================================================
        // PAUSE - Temporarily Halt Experiment
        // ====================================================================
        // Pauses the experiment without stopping it completely.
        // Useful for manual intervention (sampling, adjustments, etc.)
        //
        // ACTIONS:
        //   - Set isPaused flag = true
        //   - Turn off stirring motor (prevents splashing during intervention)
        //   - Keep heater running (maintains temperature)
        //   - Keep logging active (still monitors conditions)
        //   - Update LCD to show "PAUSED" status
        //
        // TO RESUME: Send "UNPAUSE" command or press any physical button
        //
        else if (strcmp(cmd, "PAUSE") == 0) {
            isPaused = true;
            stirringOFF();  // Stop stirring for safety during manual intervention
            Serial.println(F("$DEBUG,PAUSED"));
            Serial.println(F("$DEBUG,EXPERIMENT_PAUSED"));
            Serial.print(F("$DEBUG,WATCHDOG_STATUS,"));
            Serial.println(watchdogActive ? F("ACTIVE") : F("INACTIVE"));
            
            // Immediate LCD feedback for user
            setMultiplexerFocus(0);
            lcd.clear();
            lcd.print(F("PAUSED"));
            if (watchdogActive) {
                lcd.print(F(" + WATCHDOG"));
            }
            lcd.setCursor(0, 1);
            lcd.print(F("Cycle:"));
            lcd.print(currentCycle);
            lcd.print(F(" Step:"));
            lcd.print(currentStep);
            lcd.print(F("/"));
            lcd.print(totalProgramSteps);
            lcd.setCursor(0, 2);
            lcd.print(F("Press any button"));
            lcd.setCursor(0, 3);
            lcd.print(F("to resume"));
            delay(10);
        }
        // ====================================================================
        // UNPAUSE - Resume Experiment After Pause
        // ====================================================================
        // Resumes a paused experiment.
        //
        // SAFETY CHECKS:
        //   - SD card must be present (cannot resume without data logging)
        //   - Reopens data file if needed (e.g., after SD card recovery)
        //
        // ACTIONS:
        //   - Set isPaused flag = false
        //   - Stirring motor resumes automatically in main loop
        //   - Data logging resumes
        //   - Clear SD error message flag
        //
        // NOTE: If SD card was removed and reinserted, this command
        //       automatically reopens the data file for continued logging.
        //
        else if (strcmp(cmd, "UNPAUSE") == 0) {
            Serial.print(F("$DEBUG,UNPAUSE_REQUEST,sdCardAvailable="));
            Serial.print(sdCardAvailable ? F("TRUE") : F("FALSE"));
            Serial.print(F(",isRunning="));
            Serial.print(isRunning ? F("TRUE") : F("FALSE"));
            Serial.print(F(",isPaused="));
            Serial.println(isPaused ? F("TRUE") : F("FALSE"));
            
            // ===== SAFETY CHECK: SD CARD REQUIRED =====
            // Cannot unpause without SD card - would lose data!
            if (!sdCardAvailable) {
                Serial.println(F("❌ CANNOT UNPAUSE - No SD card! Insert SD card first."));
                Serial.println(F("$ERROR,UNPAUSE_FAILED_NO_SD"));
                displaySDError("Unpause Failed");
                return;  // Abort unpause
            }
            
            // ===== RESUME EXPERIMENT =====
            isPaused = false;
            showingSDError = false;  // Clear any SD error display
            Serial.println(F("$DEBUG,UNPAUSED"));
            Serial.println(F("$DEBUG,EXPERIMENT_UNPAUSED"));
            Serial.print(F("$DEBUG,UNPAUSE_SUCCESS,isPaused="));
            Serial.println(isPaused ? F("TRUE") : F("FALSE"));
            
            // ===== DATA FILE RECOVERY =====
            // If SD card was removed and reinserted, reopen the data file
            if (sdCardAvailable && !dataFile) {
                Serial.println(F("$DEBUG,ATTEMPTING_TO_REOPEN_DATA_FILE"));
                dataFile = SD.open(sdFileName, FILE_WRITE);
                if (dataFile) {
                    Serial.println(F("✅ Data file re-opened after SD recovery"));
                    Serial.println(F("$DEBUG,DATA_FILE_REOPENED_SUCCESS"));
                } else {
                    Serial.println(F("❌ WARNING: Failed to re-open data file!"));
                    Serial.println(F("$DEBUG,DATA_FILE_REOPEN_FAILED"));
                }
            } else if (dataFile) {
                Serial.println(F("$DEBUG,DATA_FILE_ALREADY_OPEN"));
            }
        }
        else if (strcmp(cmd, "STOP") == 0) {
            Serial.println(F("DEBUG: STOP command received. Executing immediate hardware shutdown."));

            // --- IMMEDIATE HARDWARE SHUTDOWN ---
            // Brute-force stop all active components. This is the most direct way to ensure shutdown.
            heaterOFF();
            stirringOFF();
            allPumpsStop();

            // --- STATE MANAGEMENT ---
            // Now, manage the software state.
            machine.transitionTo(stateMenu); // Transition to the idle state.
            isRunning = false;
            isPaused = false;
            
            Serial.println(F("$DEBUG,EXPERIMENT_STOPPED"));

            // --- FILE SAFETY ---
            // Safely close the main data file to prevent corruption.
            if (dataFile) {
                dataFile.flush();
                dataFile.close();
                Serial.println(F("DEBUG: Main data file flushed and closed."));
            }

            Serial.println(F("DEBUG: STOP command processing complete."));
        }
        // ====================================================================
        // CALIBRATION COMMANDS
        // ====================================================================
        // These commands set up the OD measurement calibration.
        // 3-point calibration (preferred) or 2-point (legacy).
        //
        // TYPICAL WORKFLOW:
        //   1. Measure blank media → SET_CALIBRATION_POINT_1 (low OD)
        //   2. Measure medium density → SET_CALIBRATION_POINT_2 (mid OD)
        //   3. Measure high density → SET_CALIBRATION_POINT_3 (high OD)
        //   4. Python sends SET_INVERSE_CALIBRATION (preferred OD model)
        //   5. Send SAVE_CONFIG_TO_SD to persist calibration
        //
        
        // SET_CALIBRATION_POINT_1,<ir_reading>,<known_od>
        // Sets Point 1: Low OD (typically blank media, OD~0)
        else if (strncmp(cmd, "SET_CALIBRATION_POINT_1", 23) == 0) {
            char* comma1 = strchr(cmd, ',');
            char* comma2 = comma1 ? strchr(comma1 + 1, ',') : NULL;
            
            if (comma1 && comma2) {
                irLower = atoi(comma1 + 1);
                odLower = atof(comma2 + 1);
                Serial.print(F("$DEBUG,CALIBRATION_POINT_1_SET,"));
                Serial.print(irLower);
                Serial.print(F(","));
                Serial.println(odLower, 3);
            }
        }
        // SET_CALIBRATION_POINT_2,<ir_reading>,<known_od>
        // Sets Point 2: Mid-Low OD (typically OD~0.1-0.3)
        else if (strncmp(cmd, "SET_CALIBRATION_POINT_2", 23) == 0) {
            char* comma1 = strchr(cmd, ',');
            char* comma2 = comma1 ? strchr(comma1 + 1, ',') : NULL;
            
            if (comma1 && comma2) {
                irMidLow = atoi(comma1 + 1);
                odMidLow = atof(comma2 + 1);
                Serial.print(F("$DEBUG,CALIBRATION_POINT_2_SET,"));
                Serial.print(irMidLow);
                Serial.print(F(","));
                Serial.println(odMidLow, 3);
            }
        }
        // SET_CALIBRATION_POINT_3,<ir_reading>,<known_od>
        // Sets Point 3: Mid-High OD (typically OD~1.0-2.0)
        else if (strncmp(cmd, "SET_CALIBRATION_POINT_3", 23) == 0) {
            char* comma1 = strchr(cmd, ',');
            char* comma2 = comma1 ? strchr(comma1 + 1, ',') : NULL;
            
            if (comma1 && comma2) {
                irMidHigh = atoi(comma1 + 1);
                odMidHigh = atof(comma2 + 1);
                Serial.print(F("$DEBUG,CALIBRATION_POINT_3_SET,"));
                Serial.print(irMidHigh);
                Serial.print(F(","));
                Serial.println(odMidHigh, 3);
            }
        }
        // SET_CALIBRATION_POINT_4,<ir_reading>,<known_od>
        // Sets Point 4: High OD (typically OD~3-5)
        // Marks calibration as set; Python then sends SET_INVERSE_CALIBRATION.
        else if (strncmp(cmd, "SET_CALIBRATION_POINT_4", 23) == 0) {
            char* comma1 = strchr(cmd, ',');
            char* comma2 = comma1 ? strchr(comma1 + 1, ',') : NULL;
            
            if (comma1 && comma2) {
                irUpper = atoi(comma1 + 1);
                odUpper = atof(comma2 + 1);
                
                calibrationSet = true;  // Enable OD calculation
                
                Serial.print(F("$DEBUG,CALIBRATION_POINT_4_SET,"));
                Serial.print(irUpper);
                Serial.print(F(","));
                Serial.println(odUpper, 3);
            }
        }
        // SET_INVERSE_CALIBRATION,<a>,<b>
        // Sets inverse calibration coefficients (physically correct for light absorption)
        // Formula: OD = a/IR + b (empirical fit for light scattering)
        // This is the preferred method for IR-based OD measurement
        else if (strncmp(cmd, "SET_INVERSE_CALIBRATION", 23) == 0) {
            char* comma1 = strchr(cmd, ',');
            char* comma2 = comma1 ? strchr(comma1 + 1, ',') : NULL;
            
            if (comma1 && comma2) {
                inverseA = atof(comma1 + 1);  // Numerator coefficient
                inverseB = atof(comma2 + 1);  // Offset coefficient
                useInverse = (abs(inverseA) > 1e-10);  // Use inverse if 'a' is significant
                calibrationSet = true;  // Mark calibration as set
                
                Serial.print(F("$DEBUG,INVERSE_CALIBRATION_SET,"));
                Serial.print(inverseA, 6);
                Serial.print(F(","));
                Serial.print(inverseB, 6);
                Serial.print(F(",useInverse="));
                Serial.println(useInverse ? "true" : "false");
            }
        }
        else if (strncmp(cmd, "SET_CALIBRATION_POINTS", 22) == 0) {
            // Backward compatibility: Format: SET_CALIBRATION_POINTS,irLower,odLower,irUpper,odUpper
            char* comma1 = strchr(cmd, ',');
            char* comma2 = comma1 ? strchr(comma1 + 1, ',') : NULL;
            char* comma3 = comma2 ? strchr(comma2 + 1, ',') : NULL;
            char* comma4 = comma3 ? strchr(comma3 + 1, ',') : NULL;
            
            if (comma1 && comma2 && comma3 && comma4) {
                irLower = atoi(comma1 + 1);
                odLower = atof(comma2 + 1);
                irUpper = atoi(comma3 + 1);
                odUpper = atof(comma4 + 1);
                
                // Mark calibration as set so OD calculation will work
                calibrationSet = true;
                
                Serial.print(F("$DEBUG,CALIBRATION_POINTS_SET,"));
                Serial.print(irLower);
                Serial.print(F(","));
                Serial.print(odLower, 1);
                Serial.print(F(","));
                Serial.print(irUpper);
                Serial.print(F(","));
                Serial.println(odUpper, 1);
            }
        }
        else if (strncmp(cmd, "SET_THRESHOLD", 13) == 0) {
            char* commaPos = strchr(cmd, ',');
            if (commaPos) {
                float newThreshold = atof(commaPos + 1);
                dynamicThreshold = newThreshold;

            }
        }
        else if (strncmp(cmd, "SET_TEMPERATURE", 15) == 0) {
            char* commaPos = strchr(cmd, ',');
            if (commaPos) {
                float newTemp = atof(commaPos + 1);
                incubationSetpointTemp = newTemp;

            }
        }
        else if (strncmp(cmd, "SET_STIRRING_SPEED", 18) == 0) {
            char* commaPos = strchr(cmd, ',');
            if (commaPos) {
                int newSpeed = atoi(commaPos + 1);
                motorStirringSpeed = newSpeed;

                if (isRunning) {
                    stirringON();
                }
            }
        }
        else if (strncmp(cmd, "SET_LED_BRIGHTNESS", 18) == 0) {
            char* commaPos = strchr(cmd, ',');
            if (commaPos) {
                int newBrightness = atoi(commaPos + 1);
                if (newBrightness >= 0 && newBrightness <= 4095) {
                    OD940LEDBrightness = newBrightness;
                    // Update the actual LED immediately
                    ledDriver.setPWM(0, OD940LEDBrightness);
                    ledDriver.write();
                    Serial.print(F("$DEBUG,LED_BRIGHTNESS_SET,"));
                    Serial.println(OD940LEDBrightness);
                }
            }
        }
        else if (strncmp(cmd, "TEST_LED", 8) == 0) {
            // TEST_LED command for LED calibration
            // Format: TEST_LED,channel,brightness (channel 1-6, brightness 0-4095)
            // This temporarily overrides the program LED for calibration purposes
            // Setting brightness to 0 exits calibration mode
            char* firstComma = strchr(cmd, ',');
            char* secondComma = firstComma ? strchr(firstComma + 1, ',') : NULL;
            if (firstComma && secondComma) {
                int channel = atoi(firstComma + 1);
                int brightness = atoi(secondComma + 1);
                
                if (channel >= 1 && channel <= 15 && brightness >= 0 && brightness <= 4095) {
                    // Turn off ALL stimulation LEDs first (channels 1-15)
                    for (int i = 1; i <= 15; i++) {
                        ledDriver.setPWM(i, 0);
                    }
                    
                    if (brightness > 0) {
                        // Enter calibration mode - this prevents calculateOD940 from restoring program LED
                        ledCalibrationMode = true;
                        calibrationLEDChannel = channel;
                        calibrationLEDBrightness = brightness;
                        
                        // Turn on the requested LED at specified brightness
                        ledDriver.setPWM(channel, brightness);
                        Serial.print(F("$DEBUG,TEST_LED_ON,"));
                    } else {
                        // Exit calibration mode - resume normal LED operation
                        ledCalibrationMode = false;
                        calibrationLEDChannel = 0;
                        calibrationLEDBrightness = 0;
                        Serial.print(F("$DEBUG,TEST_LED_OFF,"));
                    }
                    ledDriver.write();
                    
                    Serial.print(channel);
                    Serial.print(F(","));
                    Serial.println(brightness);
                }
            }
        }
        else if (strncmp(cmd, "MANUAL_PUMP", 11) == 0) {
            // Format: MANUAL_PUMP,pump_num,num_dispenses
            char* firstComma = strchr(cmd, ',');
            char* secondComma = firstComma ? strchr(firstComma + 1, ',') : NULL;
            if (firstComma && secondComma) {
                int pumpNum = atoi(firstComma + 1);
                int dispenses = atoi(secondComma + 1);
                
                // SAFETY: Force heater OFF during manual pumping
                // The main loop doesn't run during pumping (blocking delays)
                heaterOFF();
                outputPWM = 0;
                Serial.println(F("$DEBUG,SAFETY_HEATER_OFF_DURING_MANUAL_PUMP"));
                
                // Manual pump operation with progress updates
                for (int i = 0; i < dispenses; i++) {
                    switch (pumpNum) {
                        case 1:
                            pumpOne->run(BACKWARD);
                            pumpOne->setSpeed(pumpOneSpeed);
                            delay(100);
                            pumpOne->run(RELEASE);
                            break;
                        case 2:
                            pumpTwo->run(BACKWARD);
                            pumpTwo->setSpeed(pumpTwoSpeed);
                            delay(100);
                            pumpTwo->run(RELEASE);
                            break;
                        case 3:
                            pumpThree->run(BACKWARD);
                            pumpThree->setSpeed(pumpThreeSpeed);
                            delay(100);
                            pumpThree->run(RELEASE);
                            break;
                        case 4:
                            pumpFour->run(BACKWARD);
                            pumpFour->setSpeed(pumpFourSpeed);
                            delay(100);
                            pumpFour->run(RELEASE);
                            break;
                    }
                    delay(10);
                    
                    // Send progress to interface every dispensation (0.1mL increments)
                    Serial.print(F("$PUMP_PROGRESS,MANUAL_PUMP_"));
                    Serial.print(pumpNum);
                    Serial.print(F(","));
                    Serial.print((i + 1) * volumePerDispensation, 1);
                    Serial.print(F(","));
                    Serial.print(dispenses * volumePerDispensation, 1);
                    Serial.println();
                    
                    // Show progress on LCD less frequently to avoid blocking (every 0.2mL)
                    if ((i + 1) % 2 == 0) {
                        setMultiplexerFocus(0);
                        lcd.clear();
                        lcd.print(F("MANUAL PUMP "));
                        lcd.print(pumpNum);
                        lcd.setCursor(0, 1);
                        lcd.print(F("Vol: "));
                        lcd.print((i + 1) * volumePerDispensation, 1);
                        lcd.print(F("mL"));
                        
                        // Send LCD content to interface
                        Serial.print(F("$LCD_CONTENT,"));
                        Serial.print(F("MANUAL PUMP "));
                        Serial.print(pumpNum);
                        Serial.print(F("|Vol: "));
                        Serial.print((i + 1) * volumePerDispensation, 1);
                        Serial.print(F("mL||"));
                        Serial.println();
                    }
                    
                    // SAFETY: Ensure heater stays OFF every 10 pulses (~1mL)
                    if ((i + 1) % 10 == 0) {
                        heaterOFF();
                    }
                }

            }
        }
        else if (strcmp(cmd, "PING") == 0) {
            Serial.println("$PONG");
        }
        else if (strcmp(cmd, "GET_LAST_SD_LINE") == 0) {
            getLastSDLine();
        }
        else if (strcmp(cmd, "GET_CURRENT_STATE") == 0) {
            // Send current experiment state from memory (faster than reading SD card)
            // Format: $STATE,isRunning,isPaused,currentCycle,currentStep,currentLEDPin,currentLEDBrightness,currentLEDState,currentMediaType,dynamicThreshold,incubationSetpointTemp,totalProgramSteps,maxDispensations,stirringSpeed,ledBrightness,irLower,odLower,irMidLow,odMidLow,irMidHigh,odMidHigh,irUpper,odUpper,pumpSpeed1,pumpSpeed2,pumpSpeed3,pumpSpeed4
            Serial.print(F("$STATE,"));
            Serial.print(isRunning ? 1 : 0);
            Serial.print(F(","));
            Serial.print(isPaused ? 1 : 0);
            Serial.print(F(","));
            Serial.print(currentCycle);
            Serial.print(F(","));
            Serial.print(currentStep);
            Serial.print(F(","));
            Serial.print(currentLEDPin);
            Serial.print(F(","));
            Serial.print(currentLEDBrightness);
            Serial.print(F(","));
            Serial.print(currentLEDState ? 1 : 0);
            Serial.print(F(","));
            Serial.print(currentMediaType);
            Serial.print(F(","));
            Serial.print(dynamicThreshold, 4);
            Serial.print(F(","));
            Serial.print(incubationSetpointTemp, 2);
            Serial.print(F(","));
            Serial.print(totalProgramSteps);
            Serial.print(F(","));
            Serial.print(max_dispensations);
            Serial.print(F(","));
            Serial.print(motorStirringSpeed);
            Serial.print(F(","));
            Serial.print(OD940LEDBrightness);
            Serial.print(F(","));
            Serial.print(irLower);
            Serial.print(F(","));
            Serial.print(odLower, 3);
            Serial.print(F(","));
            Serial.print(irMidLow);
            Serial.print(F(","));
            Serial.print(odMidLow, 3);
            Serial.print(F(","));
            Serial.print(irMidHigh);
            Serial.print(F(","));
            Serial.print(odMidHigh, 3);
            Serial.print(F(","));
            Serial.print(irUpper);
            Serial.print(F(","));
            Serial.print(odUpper, 3);
            Serial.print(F(","));
            Serial.print(pumpOneSpeed);
            Serial.print(F(","));
            Serial.print(pumpTwoSpeed);
            Serial.print(F(","));
            Serial.print(pumpThreeSpeed);
            Serial.print(F(","));
            Serial.println(pumpFourSpeed);
        }
        else if (strncmp(cmd, "GET_RECENT_SD_DATA", 18) == 0) {
            // Format: GET_RECENT_SD_DATA,numLines
            char* commaPos = strchr(cmd, ',');
            if (commaPos) {
                int numLines = atoi(commaPos + 1);
                getRecentSDData(numLines);
            } else {
                getRecentSDData(100); // Default to 100 lines
            }
        }
        else if (strcmp(cmd, "SKIP_STEP") == 0) {

            
            // Force skip to work even if not officially "running"
            if (!mediaChangeInProgress) {
                // Set dilution event flags for serial AND SD logging (manual skip also counts as dilution)
                dilutionEventFlag = true;     // For serial output (reset after send)
                dilutionEventForSD = true;    // For SD card logging (reset after write)
                savedPeakOD = currentOD940;   // Save current OD for manual skip
                
                performMediaChange();  // performMediaChange() already calls advanceStep() at the end
                
                // Reset the media change cooldown to prevent automatic double advancement
                lastMediaChangeTime = millis();
            }
        }
        else if (strcmp(cmd, "SKIP_NO_MEDIA") == 0) {
            advanceStep();
            // Reset watchdog on manual intervention
            watchdogActive = false;
            preMediaChangeOD = 0.0;
            Serial.println(F("$DEBUG,WATCHDOG_RESET_MANUAL_SKIP"));
        }
        else if (strncmp(cmd, "JUMP_TO", 7) == 0) {
            // Format: JUMP_TO,cycle,step
            char* firstComma = strchr(cmd, ',');
            char* secondComma = firstComma ? strchr(firstComma + 1, ',') : NULL;
            if (firstComma && secondComma) {
                int newCycle = atoi(firstComma + 1);
                int newStep = atoi(secondComma + 1);
                
                if (newCycle >= 1 && newStep >= 1) {
                    currentCycle = newCycle;
                    currentStep = newStep;
                    // Apply step settings to update LED and media type (force LED update)
                    applyStepSettingsWithLED(currentStep - 1); // -1 because array is 0-indexed
                    
                    // Force immediate LCD update to show new state
                    forceLCDUpdate = true;
                    
                    // Reset watchdog on manual intervention
                    watchdogActive = false;
                    preMediaChangeOD = 0.0;
                    Serial.println(F("$DEBUG,WATCHDOG_RESET_MANUAL_JUMP"));
                    
                    Serial.print(F("Jumped to cycle "));
                    Serial.print(currentCycle);
                    Serial.print(F(", step "));
                    Serial.println(currentStep);
                }
            }
        }
        else if (strncmp(cmd, "SET_CYCLES_PER_LED", 18) == 0) {
            char* commaPos = strchr(cmd, ',');
            if (commaPos) {
                int newCycles = atoi(commaPos + 1);
                if (newCycles >= 1 && newCycles <= 100) {  // Extended range
                    cyclesPerLEDChange = newCycles;

                }
            }
        }
        else if (strncmp(cmd, "SET_STEP", 8) == 0) {
            // Interface sends: SET_STEP,step_num,media_type,led_pin,led_brightness_pwm,temperature

            
            char* commas[6];  // Array to hold comma positions
            commas[0] = strchr(cmd, ',');  // First comma
            for (int i = 1; i < 6 && commas[i-1]; i++) {
                commas[i] = strchr(commas[i-1] + 1, ',');
            }
            
            if (commas[0] && commas[1] && commas[2] && commas[3] && commas[4]) {
                int stepNum = atoi(commas[0] + 1);
                
                // Extract media type (need to copy substring)
                char mediaType[9];
                int mediaLen = commas[2] - commas[1] - 1;
                if (mediaLen > 8) mediaLen = 8;  // Limit to buffer size
                if (mediaLen > 0) {
                    strncpy(mediaType, commas[1] + 1, mediaLen);
                    mediaType[mediaLen] = '\0';
                } else {
                    strcpy(mediaType, "NEUTRAL");  // Default if empty
                }
                
                // Validate media type - ensure it's not empty or invalid
                if (mediaType[0] == '\0' || 
                    (strcmp(mediaType, "NEUTRAL") != 0 && 
                     strcmp(mediaType, "POSITIVE") != 0 && 
                     strcmp(mediaType, "NEGATIVE") != 0)) {
                    strcpy(mediaType, "NEUTRAL");  // Force to NEUTRAL if invalid
                }

                int ledBrightness = atoi(commas[3] + 1);
                float temp = atof(commas[4] + 1);
                

                
                if (stepNum > 0 && stepNum <= totalProgramSteps) {
                                    strcpy(stepProgram[stepNum - 1].mediaType, mediaType);
                stepProgram[stepNum - 1].ledBrightness = ledBrightness;
                stepProgram[stepNum - 1].temperature = temp;
                    


                    // If this is the current step, apply the settings immediately
                    // Use applyStepSettingsWithLED to force LED update regardless of cycle
                    if (stepNum == currentStep && isRunning) {
                        applyStepSettingsWithLED(stepNum - 1);
                        forceLCDUpdate = true;  // Force immediate LCD refresh to show changes
                    }
                } else {

                }
            } else {

            }
        }
        else if (strncmp(cmd, "SET_TOTAL_STEPS", 15) == 0) {
            char* commaPos = strchr(cmd, ',');
            if (commaPos) {
                int newTotal = atoi(commaPos + 1);
                if (newTotal >= 1 && newTotal <= 15) {
                    totalProgramSteps = newTotal;
                    Serial.print(F("Updated totalProgramSteps to: "));
                    Serial.println(totalProgramSteps);
                    // Auto-save to SD card to ensure persistence
                    saveCyclerToSD();
                }
            }
        }
        else if (strcmp(cmd, "SKIP_STEP_ADVANCE") == 0) {
            // SKIP_STEP_ADVANCE command received");
            if (isRunning) {
                advanceStep();
                // Reset watchdog on manual intervention
                watchdogActive = false;
                preMediaChangeOD = 0.0;
                Serial.println(F("$DEBUG,WATCHDOG_RESET_MANUAL_ADVANCE"));
            }
        }
        else if (strcmp(cmd, "RESET_WATCHDOG") == 0) {
            watchdogActive = false;
            preMediaChangeOD = 0.0;

        }
        else if (strncmp(cmd, "SET_PUMP_SPEED", 14) == 0) {
            // Format: SET_PUMP_SPEED,pump_num,speed
            char* firstComma = strchr(cmd, ',');
            char* secondComma = firstComma ? strchr(firstComma + 1, ',') : NULL;
            if (firstComma && secondComma) {
                int pumpNum = atoi(firstComma + 1);
                int speed = atoi(secondComma + 1);
                
                if (speed >= 100 && speed <= 2000) {
                    switch (pumpNum) {
                        case 1: pumpOneSpeed = speed; break;
                        case 2: pumpTwoSpeed = speed; break;
                        case 3: pumpThreeSpeed = speed; break;
                        case 4: pumpFourSpeed = speed; break;
                    }

                }
            }
        }
        else if (strncmp(cmd, "SET_MAX_DISPENSATIONS", 21) == 0) {
            // Format: SET_MAX_DISPENSATIONS,value
            char* commaPos = strchr(cmd, ',');
            if (commaPos) {
                int newMax = atoi(commaPos + 1);
                if (newMax >= 10 && newMax <= 500) {
                    max_dispensations = newMax;
                    Serial.print(F("$DEBUG,MAX_DISPENSATIONS_SET,"));
                    Serial.println(max_dispensations);
                } else {
                    Serial.print(F("$DEBUG,MAX_DISPENSATIONS_INVALID,"));
                    Serial.println(newMax);
                }
            }
        }
        else if (strncmp(cmd, "INIT_SD_CARD", 12) == 0) {
            Serial.println(F("=== Re-initializing SD Card ==="));
            Serial.print(F("SD CS Pin: "));
            Serial.println(sdCardCSPin);
            
            // Try multiple times with delays when explicitly requested
            for (int attempt = 1; attempt <= 5; attempt++) {
                Serial.print(F("SD init attempt "));
                Serial.print(attempt);
                Serial.print(F("/5... "));
                
                if (SD.begin(sdCardCSPin)) {
                    sdCardAvailable = true;
                    Serial.println(F("SUCCESS!"));
                    Serial.println(F("✅ SD Card initialized successfully"));
                    
                    // Test if we can actually access the SD card
                    if (SD.exists("OEVO.csv")) {
                        Serial.println(F("✅ OEVO.csv found on SD card"));
                    } else {
                        Serial.println(F("ℹ️ OEVO.csv not found (will be created)"));
                    }
                    break;
                }
                
                Serial.println(F("FAILED"));
                if (attempt < 5) {
                    delay(1000); // Wait between attempts
                }
            }
            
            if (!sdCardAvailable) {
                Serial.println(F("❌ SD Card initialization failed after 5 attempts"));
            }
        }
        else if (strncmp(cmd, "SD_SET_FILENAME", 15) == 0) {
            char* commaPos = strchr(cmd, ',');
            if (commaPos) {
                char* filename = commaPos + 1;
                
                // Trim whitespace manually
                while (*filename == ' ') filename++;  // Skip leading spaces
                int len = strlen(filename);
                while (len > 0 && (filename[len-1] == ' ' || filename[len-1] == '\r')) {
                    filename[--len] = '\0';
                }
                
                strncpy(sdFileName, filename, sizeof(sdFileName) - 1);
                sdFileName[sizeof(sdFileName) - 1] = '\0';  // Ensure null termination
                sdHeaderWritten = false;

            }
        }
        else if (strncmp(cmd, "SD_WRITE_HEADER", 15) == 0) {
            if (sdCardAvailable) {
                // Always write header to mark new experiment boundary
                // This allows multiple experiments in same CSV file with headers separating them
                File headerFile = SD.open(sdFileName, FILE_WRITE); // Open in append mode
                if (headerFile) {
                    headerFile.println(F("unixTime,upTime,OD940,infraredReading,ambientTemp,mediaTemp,heaterPlateTemp,totalCycleCount,currentCycle,currentStep,mediaType,Dilution_Event,LED_Channel,LED_Percent"));
                    headerFile.close();
                    sdHeaderWritten = true;
                    Serial.println(F("$DEBUG,SD_HEADER_WRITTEN"));
                } else {
                    Serial.println(F("$DEBUG,SD_HEADER_WRITE_FAILED"));
                }
            }
        }
        else if (strncmp(cmd, "SD_SYNC", 7) == 0) {
            // SD sync is now automatic with open-write-close pattern
            if (sdCardAvailable) {
                Serial.println(F("$DEBUG,SD_SYNC_NOT_NEEDED"));
            } else {
                Serial.println(F("$DEBUG,SD_NOT_AVAILABLE"));
            }
        }
        else if (strncmp(cmd, "SAVE_CYCLER_TO_SD", 17) == 0) {
            saveCyclerToSD();
        }

        else if (strncmp(cmd, "SAVE_CONFIG_TO_SD", 17) == 0) {
            saveConfigToSD();
        }
        // ===== LED CALIBRATION COMMANDS =====
        // SET_LED_CAL,<channel>,<intensity>,<wavelength>
        // Example: SET_LED_CAL,1,5.2,450 (LED 1: 5.2 mW/cm² at 450nm)
        else if (strncmp(cmd, "SET_LED_CAL", 11) == 0) {
            char* p = strchr(cmd, ',');
            if (p) {
                int channel = atoi(p + 1);
                p = strchr(p + 1, ',');
                if (p && channel >= 1 && channel <= 6) {
                    float intensity = atof(p + 1);
                    p = strchr(p + 1, ',');
                    if (p) {
                        int wavelength = atoi(p + 1);
                        ledCalibration[channel - 1].intensity = intensity;
                        ledCalibration[channel - 1].wavelength = wavelength;
                        Serial.print(F("$LED_CAL_SET,"));
                        Serial.print(channel);
                        Serial.print(F(","));
                        Serial.print(intensity, 2);
                        Serial.print(F(","));
                        Serial.println(wavelength);
                    }
                }
            }
        }
        // SAVE_LED_CAL - Save LED calibration to SD card
        else if (strncmp(cmd, "SAVE_LED_CAL", 12) == 0) {
            saveLEDCalibrationToSD();
        }
        // GET_LED_CAL - Get all LED calibration values
        else if (strncmp(cmd, "GET_LED_CAL", 11) == 0) {
            Serial.println(F("$LED_CAL_START"));
            for (int i = 0; i < 6; i++) {
                Serial.print(F("$LED_CAL,"));
                Serial.print(i + 1);
                Serial.print(F(","));
                Serial.print(ledCalibration[i].intensity, 2);
                Serial.print(F(","));
                Serial.println(ledCalibration[i].wavelength);
            }
            Serial.println(F("$LED_CAL_END"));
        }
        else if (strncmp(cmd, "CHECK_SD_STATUS", 15) == 0) {
            checkSDCardStatus();
        }
        else if (strncmp(cmd, "SET_TIME", 8) == 0) {
            // Format: SET_TIME,unixTimestamp
            // NOTE: Timestamp is in UTC for consistent data logging across timezones
            // Python interface sends time.time() which is always UTC-based Unix timestamp
            strtok(cmd, ",");  // First call to initialize strtok with the command string
            char* timestampStr = strtok(NULL, ",");  // Get the timestamp parameter
            if (timestampStr != NULL) {
                unsigned long timestamp = strtoul(timestampStr, NULL, 10);
                
                // Switch multiplexer to RTC channel before accessing RTC
                setMultiplexerFocus(1);
                
                DateTime dt = DateTime(timestamp);
                rtc.adjust(dt);
                Serial.print(F("RTC time set to UTC: "));
                Serial.println(timestamp);
                
                // Verify the time was set
                DateTime now = rtc.now();
                Serial.print(F("RTC now reads: "));
                Serial.print(now.unixtime());
                Serial.print(F(" ("));
                Serial.print(now.year(), DEC);
                Serial.print('/');
                Serial.print(now.month(), DEC);
                Serial.print('/');
                Serial.print(now.day(), DEC);
                Serial.print(' ');
                Serial.print(now.hour(), DEC);
                Serial.print(':');
                Serial.print(now.minute(), DEC);
                Serial.print(':');
                Serial.print(now.second(), DEC);
                Serial.println(F(")"));
            } else {
                Serial.println(F("Error: SET_TIME requires timestamp"));
            }
        }
        else if (strncmp(cmd, "GET_CURRENT_PROGRAM", 19) == 0) {
            Serial.print(F("Sending current program: "));
            Serial.print(totalProgramSteps);
            Serial.println(F(" steps"));
            
            // Send program header
            Serial.print("$PROGRAM_START,");
            Serial.print(totalProgramSteps);
            Serial.print(",");
            Serial.println(cyclesPerLEDChange);
            
            // Send each step
            for (int i = 0; i < totalProgramSteps; i++) {
                Serial.print("$PROGRAM_STEP,");
                Serial.print(i + 1);
                Serial.print(",");
                Serial.print(stepProgram[i].mediaType);
                Serial.print(",");
                Serial.print(stepProgram[i].ledBrightness);
                Serial.print(",");
                Serial.println(stepProgram[i].temperature);
            }
            Serial.println("$PROGRAM_END");
        }
    }
    
    // Run state machine
    machine.run();
}

// HARDWARE INITIALIZATION
void initializeHardware() {
    
    // LCD
    setMultiplexerFocus(0);
    lcd.begin(20, 4);  // 20 columns, 4 rows
    lcd.backlight();
    lcd.clear();
    lcd.print(F("Initializing..."));
    
    // Motor pins
    pinMode(motorStirringPinInputOne, OUTPUT);
    pinMode(motorStirringPinInputTwo, OUTPUT);
    pinMode(heaterPlateTopPinEnable, OUTPUT);
    pinMode(heaterPlateBottomPinEnable, OUTPUT);
    pinMode(heaterPlateTopPinInputOne, OUTPUT);
    pinMode(heaterPlateTopPinInputTwo, OUTPUT);
    pinMode(heaterPlateBottomPinInputOne, OUTPUT);
    pinMode(heaterPlateBottomPinInputTwo, OUTPUT);
    
    // Initialize thermistors
    Thermistor* mediaTempThermistor = new NTC_Thermistor(thermistorMediaPin, 10000, 10000, 25, 3380);
    Thermistor* heaterPlateTempThermistor = new NTC_Thermistor(thermistorHeaterPlatePin, 10000, 10000, 25, 3380);
    mediaTempThermistorSmooth = new SmoothThermistor(mediaTempThermistor, 5);
    heaterPlateTempThermistorSmooth = new SmoothThermistor(heaterPlateTempThermistor, 5);
    
    // Initialize PID (July 31 approach - no SetOutputLimits, use map() instead)
    mediaTempPID.SetMode(AUTOMATIC);
    
    // Initialize LED driver
    ledDriver.begin();
    ledDriver.setPWM(0, OD940LEDBrightness);  // OD LED always on
    ledDriver.write();
    
    // Initialize motor shield
    MotorShield.begin();
    
    // Initialize RTC
    setMultiplexerFocus(1);
    Serial.println(F("Initializing RTC..."));
    
    if (!rtc.begin()) {
        Serial.println(F("RTC initialization failed! Continuing without RTC."));
    } else {
        Serial.println(F("RTC found and initialized"));
        
        // Check if RTC lost power and needs to be set
        if (rtc.lostPower()) {
            Serial.println(F("⚠️ RTC lost power, setting to compile time"));
            Serial.println(F("⚠️ WARNING: Compile time may not be UTC!"));
            Serial.println(F("⚠️ Use 'Sync RTC Time' button in interface to set proper UTC time"));
            // Set RTC to the date & time this sketch was compiled
            // NOTE: This will be in the timezone of the computer that compiled the sketch
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        }
        
        // Debug: Print current RTC time
        DateTime now = rtc.now();
        Serial.print(F("RTC current time: "));
        Serial.print(now.unixtime());
        Serial.print(F(" ("));
        Serial.print(now.year(), DEC);
        Serial.print('/');
        Serial.print(now.month(), DEC);
        Serial.print('/');
        Serial.print(now.day(), DEC);
        Serial.print(' ');
        Serial.print(now.hour(), DEC);
        Serial.print(':');
        Serial.print(now.minute(), DEC);
        Serial.print(':');
        Serial.print(now.second(), DEC);
        Serial.println(F(")"));
    }
    
    // Initialize light sensor
    setMultiplexerFocus(2);
    tsl.setGain(TSL2591_GAIN_HIGH);
    tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);
    
    // Turn off heater and pumps
    heaterOFF();
    allPumpsStop();
    

    
    // Update LCD
    setMultiplexerFocus(0);
    lcd.clear();
    lcd.print("OpenEvo Ready");
}

// I2C MULTIPLEXER
void setMultiplexerFocus(uint8_t bus) {
    Wire.beginTransmission(0x70);
    Wire.write(1 << bus);
    Wire.endTransmission();
}

// STATE FUNCTIONS
void stateFunctionMenu() {
    // This state is the main idle/stopped state.
    // Actively ensure all hardware is off.
    heaterOFF();
    stirringOFF();
    allPumpsStop();

    // Display the welcome/ready message on the LCD.
    static bool menuDisplayed = false; 
    if (!menuDisplayed) {
        setMultiplexerFocus(0);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Welcome to OpenEvo"));
        lcd.setCursor(0, 1);
        lcd.print(F("Press any button"));
        lcd.setCursor(0, 2);
        lcd.print(F("   to start"));
        
        // Send LCD content to interface
        Serial.print(F("$LCD_CONTENT,"));
        Serial.print(F("Welcome to OpenEvo|Press any button|   to start|"));
        Serial.println();
        
        menuDisplayed = true;
    }
}

void stateFunctionStandby() {
    unsigned long currentMillis = millis();
    
    // ========================================================================
    // SD CARD HEALTH MONITORING & AUTO-PAUSE SAFETY
    // ========================================================================
    // Continuously monitors SD card presence to prevent data loss.
    // Runs independently of experiment state (even when not running).
    //
    // Auto-Pause Behavior:
    //   - If SD card is removed during experiment → Auto-pause immediately
    //   - Prevents writing to missing card (would cause data loss)
    //   - Requires MANUAL unpause after card is re-inserted
    //   - System verifies card is readable before allowing unpause
    //
    // Recovery Process:
    //   1. Detect card removal → auto-pause + notify interface
    //   2. User inserts SD card
    //   3. System detects card (within 5 seconds)
    //   4. User manually unpauses experiment
    //   5. Data logging resumes automatically
    //
    static unsigned long lastSDCheck = 0;
    // Adaptive check frequency: 5 seconds for first hour, then 120 seconds (2 min) after
    // This reduces memory usage and prevents flaky SD checks during long experiments
    unsigned long sdCheckInterval = (millis() < 3600000) ? 5000 : 120000;
    
    if (currentMillis - lastSDCheck > sdCheckInterval) {
        lastSDCheck = currentMillis;
        
        Serial.println(F("$DEBUG,PERIODIC_SD_CHECK_RUNNING"));
        
        // If card was previously missing, attempt to re-initialize the SD library.
        // This is necessary because after card removal, the SD library's internal
        // state needs to be reset before it can detect the card again.
        if (!sdCardAvailable) {
            Serial.println(F("$DEBUG,SD_WAS_MISSING,ATTEMPTING_REINIT"));
            if (SD.begin(sdCardCSPin)) {
                Serial.println(F("$DEBUG,SD_REINIT_SUCCESS"));
            } else {
                Serial.println(F("$DEBUG,SD_REINIT_FAILED"));
            }
        }
        
        // Robust SD card detection using 3-level verification:
        // Level 1: Can we open the root directory?
        // Level 2: Can we read an existing file (config.txt)?
        // Level 3: Can we write a test file?
        File testRoot = SD.open("/");
        bool cardPresent = false;
        
        if (testRoot) {
            testRoot.close();
            Serial.println(F("$DEBUG,SD_ROOT_OPENED"));
            
            // Root directory accessible. Now verify we can actually read files.
            // Try opening config.txt which should exist after first experiment.
            File testFile = SD.open("config.txt");
            if (testFile) {
                cardPresent = true;  // File exists and is readable
                testFile.close();
                Serial.println(F("$DEBUG,SD_CONFIG_OPENED_OK"));
            } else {
                Serial.println(F("$DEBUG,SD_CONFIG_NOT_FOUND,TRYING_TEST_WRITE"));
                // Config doesn't exist (first run?). Verify card is writable.
                File testWrite = SD.open("._sdtest", FILE_WRITE);
                if (testWrite) {
                    testWrite.println("test");
                    testWrite.close();
                    SD.remove("._sdtest");
                    cardPresent = true;  // Write successful
                    Serial.println(F("$DEBUG,SD_TEST_WRITE_OK"));
                } else {
                    cardPresent = false;  // Write failed - card may be read-only or corrupted
                    Serial.println(F("$DEBUG,SD_TEST_WRITE_FAILED"));
                }
            }
        } else {
            cardPresent = false;  // Root directory not accessible - card missing or not initialized
            Serial.println(F("$DEBUG,SD_ROOT_OPEN_FAILED"));
        }
        
        // ===== CARD RECOVERY DETECTED =====
        if (cardPresent) {
            Serial.print(F("$DEBUG,SD_CHECK,CARD_PRESENT,sdCardAvailable_was="));
            Serial.println(sdCardAvailable ? F("TRUE") : F("FALSE"));
            
            if (!sdCardAvailable) {
                // SD card has just been re-inserted or recovered!
                sdCardAvailable = true;
                showingSDError = false;  // Clear error display flag
                Serial.println(F("✅ SD CARD RECOVERED - Ready to use"));
                Serial.println(F("$DEBUG,SD_RECOVERED"));
                Serial.print(F("$DEBUG,SD_RECOVERED,sdCardAvailable_now="));
                Serial.print(sdCardAvailable ? F("TRUE") : F("FALSE"));
                Serial.print(F(",isPaused="));
                Serial.print(isPaused ? F("TRUE") : F("FALSE"));
                Serial.print(F(",isRunning="));
                Serial.println(isRunning ? F("TRUE") : F("FALSE"));
                
                // ===== CRITICAL FIX: Reopen data file and auto-unpause if experiment was running =====
                // When SD card is removed and reinserted, the file handle becomes invalid.
                // We need to reopen it immediately so data logging continues.
                if (isRunning) {
                    // Close any stale file handle first
                    if (dataFile) {
                        dataFile.close();
                        Serial.println(F("$DEBUG,CLOSED_STALE_FILE_HANDLE"));
                    }
                    
                    // Reopen the data file in append mode
                    dataFile = SD.open(sdFileName, FILE_WRITE);
                    if (dataFile) {
                        Serial.println(F("✅ Data file REOPENED after SD recovery!"));
                        Serial.println(F("$DEBUG,DATA_FILE_AUTO_REOPENED"));
                        
                        // ===== AUTO-UNPAUSE: Resume experiment automatically =====
                        // This is the "hot-swap" feature - when SD card is reinserted,
                        // the experiment should automatically resume without user intervention
                        if (isPaused) {
                            isPaused = false;
                            showingSDError = false;
                            Serial.println(F("✅ EXPERIMENT AUTO-RESUMED after SD recovery!"));
                            Serial.println(F("$DEBUG,AUTO_UNPAUSED_SD_RECOVERY"));
                            Serial.println(F("$DEBUG,EXPERIMENT_UNPAUSED"));
                            
                            // Update LCD to show running status
                            forceLCDUpdate = true;
                        }
                    } else {
                        Serial.println(F("❌ Failed to reopen data file after SD recovery"));
                        Serial.println(F("$DEBUG,DATA_FILE_REOPEN_FAILED"));
                        // Don't auto-unpause if file couldn't be opened
                    }
                }
            }
        }
        // ===== CARD REMOVAL DETECTED =====
        else {
            Serial.print(F("$DEBUG,SD_CHECK,CARD_MISSING,sdCardAvailable="));
            Serial.println(sdCardAvailable ? F("TRUE") : F("FALSE"));
            
            if (sdCardAvailable) {
                // SD card has just been removed or become inaccessible!
                sdCardAvailable = false;
                
                // Close the data file handle since it's now invalid
                if (dataFile) {
                    dataFile.close();
                    Serial.println(F("$DEBUG,DATA_FILE_CLOSED_SD_REMOVED"));
                }
                
                if (isRunning && !isPaused) {
                    // ===== AUTO-PAUSE: Pause experiment when SD card is removed =====
                    // This prevents data loss - experiment will auto-resume when card is reinserted
                    isPaused = true;
                    showingSDError = true;
                    
                    Serial.println(F("⚠️  SD CARD REMOVED - EXPERIMENT PAUSED"));
                    Serial.println(F("$DEBUG,SD_LOST,AUTO_PAUSED"));
                    Serial.println(F("$DEBUG,PAUSED"));
                    Serial.println(F("$DEBUG,EXPERIMENT_PAUSED"));
                    
                    // Display error on LCD immediately
                    setMultiplexerFocus(0);
                    lcd.clear();
                    lcd.print(F("SD CARD REMOVED!"));
                    lcd.setCursor(0, 1);
                    lcd.print(F("PAUSED -No data loss"));
                    lcd.setCursor(0, 2);
                    lcd.print(F("Insert SD card to"));
                    lcd.setCursor(0, 3);
                    lcd.print(F("auto-resume"));
                    delay(50);  // Wait for I2C LCD transaction to complete before switching channels
                    
                    // Send LCD content to interface
                    Serial.println(F("$LCD_CONTENT,SD CARD REMOVED!|PAUSED - Data safe|Insert SD card to|auto-resume"));
                } else {
                    // Not running or already paused - just notify
                    Serial.println(F("❌ SD CARD LOST"));
                    Serial.println(F("$DEBUG,SD_LOST"));
                }
                // Close data file if open
                if (dataFile) {
                    dataFile.close();
                }
            }
        }
    }

    if (isRunning) {
        // --- ACTIVE EXPERIMENT LOGIC ---
        // Always update sensors and LCD, even when paused
        setMultiplexerFocus(1);
        heaterCompute();
        calculateOD940();
        
        // Send data to interface (reduce frequency when paused)
        unsigned long dataInterval = isPaused ? (interval * 5) : interval; // 5x slower when paused
        if (currentMillis - previousMillis >= dataInterval) {
            previousMillis = currentMillis;
            
            // Sensors already read above
            
            // Send data to interface - NEW 12-FIELD FORMAT
            // Format: unixTime,upTime,OD940,infraredReading,ambientTemp,mediaTemp,heaterPlateTemp,totalCycleCount,currentCycle,currentStep,mediaType,Dilution_Event
            Serial.print("@");
            
            // unixTime (use 0 if RTC not available)
            setMultiplexerFocus(1);  // Switch to RTC channel before reading
            DateTime now = rtc.now();
            if (now.year() > 2000) {
                Serial.print(now.unixtime());
            } else {
                Serial.print(0);  // RTC not set properly
            }
            Serial.print(",");
            
            // upTime
            Serial.print(millis());
            Serial.print(",");
            
            // OD940
            Serial.print(currentOD940, 4);
            Serial.print(",");
            
            // infraredReading
            Serial.print(IR);
            Serial.print(",");
            
            // ambientTemp
            Serial.print(ambientTemp, 2);
            Serial.print(",");
            
            // mediaTemp
            Serial.print(mediaTemp, 2);
            Serial.print(",");
            
            // heaterPlateTemp
            Serial.print(heaterPlateTemp, 2);
            Serial.print(",");
            
            // totalCycleCount (total cycles completed)
            Serial.print(currentCycle - 1); // Total completed cycles
            Serial.print(",");
            
            // currentCycle
            Serial.print(currentCycle);
            Serial.print(",");
            
            // currentStep
            Serial.print(currentStep);
            Serial.print(",");
            
            // mediaType
            Serial.print(currentMediaType);
            Serial.print(",");
            
            // Dilution_Event (1 if dilution triggered, 0 otherwise)
            // CRITICAL: Send the flag value, then IMMEDIATELY reset it
            // This ensures only ONE serial message has Dilution_Event=1 per actual dilution
            // Previously, the flag was only reset in logAveragedDataToSD(), which could be skipped
            // if SD card was not available, causing multiple false peaks in the interface
            bool sendDilutionEvent = dilutionEventFlag;
            Serial.println(sendDilutionEvent ? 1 : 0);
            
            // Reset the flag immediately after sending to prevent duplicate events
            if (dilutionEventFlag) {
                dilutionEventFlag = false;
                Serial.println(F("$DEBUG,DILUTION_EVENT_FLAG_RESET_AFTER_SERIAL"));
            }
            
            // Accumulate data for SD card averaging (every 1 second) - ONLY WHEN NOT PAUSED
            if (!isPaused) {
                odSum += currentOD940;
                tempSum += mediaTemp;
                irSum += IR;
                dataPointCount++;
            }
            
            // Log to SD card every 10 seconds with averaged data - ONLY WHEN NOT PAUSED
            if (!isPaused && sdCardAvailable && (currentMillis - previousSDLogMillis >= sdLogInterval)) {
                previousSDLogMillis = currentMillis;
                logAveragedDataToSD();
                
                // Reset accumulation variables
                odSum = 0.0;
                tempSum = 0.0;
                irSum = 0;
                dataPointCount = 0;
            }
            
            // Update stirring and heater control (always for safety)
            if (!isPaused) {
                updateStirringRamp();
                stirringON();
            }
            
            // Only run experiment control logic when not paused
            if (!isPaused) {
                // Check watchdog status - auto-clear when OD drops (with minimum alert time)
                if (watchdogActive) {
                    float odDrop = preMediaChangeOD - currentOD940;
                    // Only auto-clear if watchdog has been active for at least 5 seconds
                    // This prevents false alerts from clearing immediately
                    if (odDrop >= WATCHDOG_OD_DROP_REQUIRED && 
                        (currentMillis - watchdogTriggerTime) >= WATCHDOG_MIN_ALERT_TIME) {
                        watchdogActive = false;
                        preMediaChangeOD = 0.0;
                        
                        // Send watchdog cleared message to interface
                        Serial.print(F("$DEBUG,WATCHDOG_STATUS,INACTIVE,"));
                        Serial.print(F("OD_DROP:"));
                        Serial.print(odDrop, 3);
                        Serial.print(F(",REQUIRED:"));
                        Serial.print(WATCHDOG_OD_DROP_REQUIRED, 3);
                        Serial.print(F(",ALERT_TIME_MS:"));
                        Serial.println(currentMillis - watchdogTriggerTime);
                    }
                }
                
                // Check threshold for media change (with watchdog protection)
                if (currentOD940 >= dynamicThreshold && 
                    !mediaChangeInProgress && 
                    !watchdogActive &&
                    (currentMillis - lastMediaChangeTime) > MEDIA_CHANGE_COOLDOWN) {
                    
                    // WATCHDOG: Save OD at peak detection (before dilution)
                    if (currentOD940 >= 0.5) {
                        preMediaChangeOD = currentOD940;
                        Serial.print(F("$DEBUG,WATCHDOG_PRE_OD_SAVED,"));
                        Serial.println(currentOD940, 3);
                    } else {
                        preMediaChangeOD = 0.0;  // Don't monitor empty vessels
                    }
                    
                    // Save peak info (will send to interface AFTER dilution completes)
                    float peakOD = currentOD940;
                    DateTime peakTime = rtc.now();
                    unsigned long peakUnixTime = peakTime.unixtime();
                    
                    // Set dilution event flags for serial AND SD logging, save the peak OD
                    dilutionEventFlag = true;     // For serial output (reset after send)
                    dilutionEventForSD = true;    // For SD card logging (reset after write)
                    savedPeakOD = currentOD940;   // Save OD at moment of detection
                    
                    performMediaChange();
                    
                    // NOW send peak with correct NEW media type (after advanceStep)
                    Serial.print(F("PEAK_DETECTED,"));
                    Serial.print(peakOD, 4);
                    Serial.print(F(","));
                    Serial.print(peakUnixTime);
                    Serial.print(F(","));
                    Serial.println(currentMediaType);  // NEW media type!
                }
                
                // Check watchdog timer (15 seconds after pumping completes)
                // Wait long enough for multiple fresh OD readings (sensor updates every ~1 second)
                if (!watchdogActive && preMediaChangeOD > 0 && mediaChangeStartTime > 0) {
                    unsigned long elapsed = currentMillis - mediaChangeStartTime;
                    
                    // Wait 15 seconds to ensure we have fresh OD data (at least 10-15 new readings)
                    if (elapsed > 15000) {
                        Serial.print(F("$DEBUG,WATCHDOG_CHECK,elapsed_ms:"));
                        Serial.println(elapsed);
                        
                        float odDrop = preMediaChangeOD - currentOD940;
                        
                        // Only trigger watchdog if OD drop is insufficient AND OD is still high
                        // This prevents false alarms when vessel is empty/low
                        if (odDrop < WATCHDOG_OD_DROP_REQUIRED && currentOD940 > 0.5) {
                            watchdogActive = true;
                            watchdogTriggerTime = currentMillis;  // Record when triggered
                            
                            // Send watchdog triggered message to interface
                            Serial.print(F("$DEBUG,WATCHDOG_STATUS,ACTIVE,"));
                            Serial.print(F("OD_DROP:"));
                            Serial.print(odDrop, 3);
                            Serial.print(F(",REQUIRED:"));
                            Serial.print(WATCHDOG_OD_DROP_REQUIRED, 3);
                            Serial.print(F(",PRE_OD:"));
                            Serial.print(preMediaChangeOD, 3);
                            Serial.print(F(",CURRENT_OD:"));
                            Serial.println(currentOD940, 3);
                            
                            // DON'T reset preMediaChangeOD here! Clearing logic needs it!
                            // Only reset the timer to prevent repeated triggering
                            mediaChangeStartTime = 0;

                        } else {
                            // OD drop is sufficient, no alert needed
                            Serial.print(F("$DEBUG,WATCHDOG_STATUS,INACTIVE,"));
                            Serial.print(F("OD_DROP:"));
                            Serial.print(odDrop, 3);
                            Serial.print(F(",REQUIRED:"));
                            Serial.print(WATCHDOG_OD_DROP_REQUIRED, 3);
                            Serial.print(F(",CURRENT_OD:"));
                            Serial.println(currentOD940, 3);
                            
                            // Reset for next dilution (successful dilution)
                            preMediaChangeOD = 0.0;
                            mediaChangeStartTime = 0;
                        }
                    }
                }
            }
            
            // Update LCD (but skip if showing SD error message)
            setMultiplexerFocus(0);
            static unsigned long lastLCDUpdate = 0;
            unsigned long lcdUpdateInterval = isPaused ? 500 : 2000; // Faster updates when paused
            // Force update if flag is set, or if enough time has passed
            if ((forceLCDUpdate || (currentMillis - lastLCDUpdate > lcdUpdateInterval)) && !showingSDError) {
                lastLCDUpdate = currentMillis;
                forceLCDUpdate = false;  // Clear the flag after using it
                lcd.clear();
                
                // Debug LCD update
                static unsigned long lastLCDDebug = 0;
                if (millis() - lastLCDDebug > 5000) {
                    lastLCDDebug = millis();
                    Serial.print(F("$DEBUG,LCD_UPDATE,isPaused:"));
                    Serial.print(isPaused ? F("TRUE") : F("FALSE"));
                    Serial.print(F(",watchdog:"));
                    Serial.print(watchdogActive ? F("TRUE") : F("FALSE"));
                    Serial.print(F(",sdCard:"));
                    Serial.println(sdCardAvailable ? F("TRUE") : F("FALSE"));
                }
                
                if (isPaused) {
                    Serial.println(F("$DEBUG,LCD_DISPLAY,PAUSED"));
                    lcd.print(F("PAUSED"));
                    if (watchdogActive) {
                        lcd.print(F(" + WATCHDOG"));
                    }
                    lcd.setCursor(0, 1);
                    lcd.print(F("Cycle:"));
                    lcd.print(currentCycle);
                    lcd.print(F(" Step:"));
                    lcd.print(currentStep);
                    lcd.print(F("/"));
                    lcd.print(totalProgramSteps);
                    lcd.setCursor(0, 2);
                    lcd.print(F("Press any button"));
                    lcd.setCursor(0, 3);
                    lcd.print(F("to resume"));
                    
                    // Send LCD content to interface
                    Serial.print(F("$LCD_CONTENT,"));
                    Serial.print(F("PAUSED"));
                    if (watchdogActive) {
                        Serial.print(F(" + WATCHDOG"));
                    }
                    Serial.print(F("|Cycle:"));
                    Serial.print(currentCycle);
                    Serial.print(F(" Step:"));
                    Serial.print(currentStep);
                    Serial.print(F("/"));
                    Serial.print(totalProgramSteps);
                    Serial.print(F("|Press any button|to resume"));
                    Serial.println();
                    
                    // Debug: Confirm LCD content was sent
                    Serial.println(F("$DEBUG,LCD_CONTENT_SENT,PAUSED"));
                } else if (watchdogActive) {
                    Serial.println(F("$DEBUG,LCD_DISPLAY,WATCHDOG"));
                    lcd.print(F("WATCHDOG ACTIVE!"));
                    lcd.setCursor(0, 1);
                    lcd.print(F("Media LOCKED"));
                    lcd.setCursor(0, 2);
                    lcd.print(F("OD must drop"));
                    lcd.setCursor(0, 3);
                    lcd.print(F("Current: "));
                    lcd.print(currentOD940, 2);
                    
                    // Send LCD content to interface
                    Serial.print(F("$LCD_CONTENT,"));
                    Serial.print(F("WATCHDOG ACTIVE!|Media LOCKED|OD must drop|Current: "));
                    Serial.print(currentOD940, 2);
                    Serial.println();
                } else if (!sdCardAvailable) {
                    Serial.println(F("$DEBUG,LCD_DISPLAY,SD_ERROR"));
                    lcd.print(F("SD CARD ERROR!"));
                    lcd.setCursor(0, 1);
                    lcd.print(F("PAUSED"));
                    lcd.setCursor(0, 2);
                    lcd.print(F("Insert SD & Unpause"));
                    lcd.setCursor(0, 3);
                    lcd.print(F("to resume experiment"));
                    delay(50);  // Wait for I2C LCD transaction to complete
                    
                    // Send LCD content to interface
                    Serial.println(F("$LCD_CONTENT,SD CARD ERROR!|PAUSED|Insert SD & Unpause|to resume experiment"));
                } else {
                    // Normal operation display
                    Serial.println(F("$DEBUG,LCD_DISPLAY,NORMAL"));
                    lcd.setCursor(0, 0);

                    // --- NEW LCD LAYOUT ---

                    // Line 1: Cycle and Step
                    lcd.print(F("Cycle:"));
                    lcd.print(currentCycle);
                    lcd.print(F(" Step:"));
                    lcd.print(currentStep);
                    lcd.print(F("/"));
                    lcd.print(totalProgramSteps);

                    // Line 2: OD and IR
                    lcd.setCursor(0, 1);
                    lcd.print(F("OD:"));
                    lcd.print(currentOD940, 2);
                    lcd.print(F(" IR:"));
                    lcd.print(IR);

                    // Line 3: Media Type and LED (with LED channel number)
                    lcd.setCursor(0, 2);
                    lcd.print(F("Media:"));
                    // Print media type abbreviation
                    if (strcmp(currentMediaType, "POSITIVE") == 0) {
                        lcd.print(F("POS"));
                    } else if (strcmp(currentMediaType, "NEGATIVE") == 0) {
                        lcd.print(F("NEG"));
                    } else {
                        lcd.print(F("NEU"));
                    }
                    int ledPercent = (currentLEDBrightness / 4095.0) * 100;
                    lcd.print(F(" LED"));
                    lcd.print(currentLEDPin);  // <-- LED channel number (1-6)
                    lcd.print(F(":"));
                    lcd.print(ledPercent);
                    lcd.print(F("%"));  // <-- percentage units

                    // Line 4: Temperature
                    lcd.setCursor(0, 3);
                    lcd.print(F("Media Temp:"));
                    lcd.print(mediaTemp, 1);
                    lcd.print(F("C"));
                    
                    // Send LCD content to interface
                    Serial.print(F("$LCD_CONTENT,"));
                    // Line 1
                    Serial.print(F("Cycle:"));
                    Serial.print(currentCycle);
                    Serial.print(F(" Step:"));
                    Serial.print(currentStep);
                    Serial.print(F("/"));
                    Serial.print(totalProgramSteps);
                    Serial.print(F("|"));
                    // Line 2
                    Serial.print(F("OD:"));
                    Serial.print(currentOD940, 2);
                    Serial.print(F(" IR:"));
                    Serial.print(IR);
                    Serial.print(F("|"));
                    // Line 3 - Media Type and LED (with LED channel number)
                    Serial.print(F("Media:"));
                    if (strcmp(currentMediaType, "POSITIVE") == 0) {
                        Serial.print(F("POS"));
                    } else if (strcmp(currentMediaType, "NEGATIVE") == 0) {
                        Serial.print(F("NEG"));
                    } else {
                        Serial.print(F("NEU"));
                    }
                    Serial.print(F(" LED"));
                    Serial.print(currentLEDPin);  // <-- LED channel number (1-6)
                    Serial.print(F(":"));
                    Serial.print(ledPercent);
                    Serial.print(F("%|"));  // <-- percentage units
                    // Line 4
                    Serial.print(F("Media Temp:"));
                    Serial.print(mediaTemp, 1);
                    Serial.print(F("C"));
                    Serial.println();
                }
            }
        }
    }
}

// TRANSITIONS
bool transitionMenuToStandby() {
    return isRunning;
}

// SENSOR FUNCTIONS - FROM WORKING VERSION
// =======================================================================================
// === Light Sensor Reading ==============================================================
// =======================================================================================
void getTSL2591Data(uint16_t &ir, uint16_t &full) {
    setMultiplexerFocus(2);
    uint32_t lum = tsl.getFullLuminosity();
    ir = lum >> 16;
    full = lum & 0xFFFF;
}

// =======================================================================================
// === OD Calculation ======================================================================
// =======================================================================================
/**
 * OPTICAL DENSITY (OD) CALCULATION
 * ==================================
 * Measures culture density using 940nm IR light absorption.
 * 
 * HARDWARE:
 *   - TSL2591 light sensor (reads IR intensity)
 *   - 940nm IR LED (illuminates culture)
 *   - TCA9548A multiplexer (channel 2)
 * 
 * CALIBRATION METHODS:
 *   1. Inverse (preferred, full range):
 *      OD = a/IR + b
 *      Empirically accurate for IR light scattering across the OD range
 * 
 *   2. 2-Point Linear (fallback for OD 0-2):
 *      OD = slope*IR + intercept
 *      Simpler but less accurate at high densities
 * 
 * MEASUREMENT PROCESS:
 *   1. Switch multiplexer to TSL sensor
 *   2. Turn on IR LED at configured brightness
 *   3. Read IR intensity from sensor
 *   4. Apply calibration formula
 *   5. Clamp result to valid range (0-10 OD)
 * 
 * NOTE: Lower IR readings = higher cell density (more absorption)
 */
void calculateOD940() {
    // SKIP OD measurement entirely when in LED calibration mode
    // This prevents the calibration LED from blinking during testing
    if (ledCalibrationMode) {
        return;  // Don't interrupt the calibration LED
    }
    
    // Switch I2C multiplexer to TSL2591 sensor channel
    setMultiplexerFocus(2);
    delay(5);  // Wait for multiplexer to settle
    
    // CRITICAL: Turn OFF all stimulation LEDs (channels 1-15) during measurement
    // Stimulation light would interfere with IR absorption reading
    for (int i = 1; i <= 15; i++) {
        ledDriver.setPWM(i, 0);
    }
    
    // Turn ON only the 940nm measurement LED (channel 0)
    ledDriver.setPWM(0, OD940LEDBrightness);
    ledDriver.write();
    delay(10);  // Wait for LED to stabilize and stimulation light to clear
    
    // Read infrared intensity from TSL2591 sensor
    uint16_t ir, full;
    getTSL2591Data(ir, full);  // 'full' is visible+IR, but we only use 'ir'
    IR = ir;  // Store globally for interface display
    
    // Turn OFF the 940nm measurement LED (channel 0)
    ledDriver.setPWM(0, 0);
    
    // RESTORE LED state after measurement
    // In calibration mode, restore the calibration LED instead of program LED
    if (ledCalibrationMode && calibrationLEDChannel >= 1 && calibrationLEDChannel <= 15) {
        // Calibration mode - restore the test LED
        ledDriver.setPWM(calibrationLEDChannel, calibrationLEDBrightness);
    } else if (currentLEDState && currentLEDPin >= 1 && currentLEDPin <= 15) {
        // Normal mode - restore program LED
        ledDriver.setPWM(currentLEDPin, currentLEDBrightness);
    }
    ledDriver.write();  // Apply all changes at once
    
    // Only calculate OD if calibration has been performed
    if (calibrationSet) {
        float calculatedOD;
        
        // ===== METHOD 1: INVERSE CALIBRATION (PREFERRED - Full Range Accuracy) =====
        if (useInverse && IR > 0) {
            // Formula: OD = a/IR + b
            // Empirically accurate for LIGHT SCATTERING (turbidity) measurements
            // Note: This is NOT Beer-Lambert Law (which applies to absorbance, not scattering)
            // Lower IR = more scattered light = higher cell density = higher OD
            calculatedOD = inverseA / IR + inverseB;
        }
        // ===== METHOD 2: LINEAR CALIBRATION (2-point fallback) =====
        else {
            // Formula: OD = slope*IR + intercept
            // Calculate slope and intercept from upper and lower calibration points
            float slope = (odUpper - odLower) / (irUpper - irLower);
            float intercept = odLower - (slope * irLower);
            calculatedOD = slope * IR + intercept;
        }
        
        // ===== SAFETY CLAMPING =====
        // Prevent invalid OD values from bad calibration or sensor errors
        if (calculatedOD < 0.0) {
            currentOD940 = 0.0;  // Negative OD is physically impossible
        } else if (calculatedOD > 10.0) {  // <----- CHANGE MAX OD HERE
            currentOD940 = 10.0;  // Cap at maximum realistic OD
        } else {
            currentOD940 = calculatedOD;  // Valid range
        }
    } else {
        // No calibration set - return 0
        // This prevents using uncalibrated (meaningless) measurements
        currentOD940 = 0.0;
    }
}

/**
 * TEMPERATURE CONTROL & SAFETY SYSTEM
 * ====================================
 * This function manages the heater using a 3-layer safety system:
 * 
 * LAYER 1: Emergency Temperature Limit
 *   - Hard limit at 40°C media temp (protects organisms)
 *   - Hard limit at 50°C heater plate (prevents hardware damage)
 *   - Auto-pauses experiment and turns off heater immediately
 *   - Cannot be overridden - requires manual intervention
 * 
 * LAYER 2: Pause Safety
 *   - Heater turns OFF when experiment is paused
 *   - Prevents unintended heating during manual intervention
 * 
 * LAYER 3: PID Temperature Control with Proportional Reduction
 *   - Maintains temperature at setpoint (typically 30°C)
 *   - Uses adaptive PID: aggressive when far, conservative when close
 *   - Proportional PWM reduction in final 0.3°C approach (prevents overshoot)
 *   - Stops heating AT setpoint (no offset)
 * 
 * The heater uses H-bridge motor drivers with:
 *   - PWM (0-130) controls heating power (from July 31 working version)
 *   - PID outputs 0-255, then map() scales to 0-130 safe limit
 *   - Direction pins (HIGH/LOW) enable the H-bridge
 *   - Both top and bottom heaters controlled identically
 * 
 * IMPORTANT: All paths must write to the heater pins at the end!
 * Early returns would leave pins in undefined state.
 */
void heaterCompute() {
    // Read all temperature sensors
    mediaTemp = mediaTempThermistorSmooth->readCelsius();      // Culture temperature
    heaterPlateTemp = heaterPlateTempThermistorSmooth->readCelsius();  // Heater surface temp
    ambientTemp = (double) rtc.getTemperature();               // Room temperature from RTC

    // ===== SENSOR SANITY CHECK =====
    // If MEDIA thermistor readings are impossible (unplugged/broken), emergency shutoff
    // Note: We only check media temp since that's what we're controlling
    // Heater plate thermistor is optional monitoring
    if (mediaTemp < -10.0 || mediaTemp > 100.0) {
        outputPWM = 0;
        analogWrite(heaterPlateTopPinEnable, 0);
        analogWrite(heaterPlateBottomPinEnable, 0);
        digitalWrite(heaterPlateTopPinInputOne, LOW);
        digitalWrite(heaterPlateTopPinInputTwo, LOW);
        digitalWrite(heaterPlateBottomPinInputOne, LOW);
        digitalWrite(heaterPlateBottomPinInputTwo, LOW);
        Serial.println(F("$ERROR,MEDIA_THERMISTOR_ERROR_HEATER_OFF"));
        // DISABLED: Auto-pause disabled to prevent false triggers
        // if (isRunning && !isPaused) {
        //     isPaused = true;
        //     stirringOFF();
        //     Serial.println(F("$DEBUG,AUTO_PAUSED_SENSOR_ERROR"));
        // }
        return;
    }

    // Send temperature telemetry every 10 seconds for monitoring
    static unsigned long lastTempDebug = 0;
    if (millis() - lastTempDebug > 10000) {
        lastTempDebug = millis();
        
        int rawMediaPin = analogRead(thermistorMediaPin);
        int rawHeaterPin = analogRead(thermistorHeaterPlatePin);
        
        Serial.print(F("$DEBUG,TEMPS,Media:"));
        Serial.print(mediaTemp, 2);
        Serial.print(F(",Heater:"));
        Serial.print(heaterPlateTemp, 2);
        Serial.print(F(",Ambient:"));
        Serial.print(ambientTemp, 2);
        Serial.print(F(",Setpoint:"));
        Serial.print(incubationSetpointTemp, 2);
        Serial.print(F(",PWM:"));
        Serial.print(outputPWM);
        Serial.print(F(",isPaused:"));
        Serial.print(isPaused ? "YES" : "NO");
        Serial.print(F(",RawMedia:"));
        Serial.print(rawMediaPin);
        Serial.print(F(",RawHeater:"));
        Serial.println(rawHeaterPin);
    }

    // ===== LAYER 1: EMERGENCY TEMPERATURE LIMITS =====
    // These are hard limits that cannot be exceeded. If triggered, the system
    // immediately shuts down heating and pauses the experiment to prevent damage.
    if (mediaTemp >= EMERGENCY_TEMP_LIMIT || heaterPlateTemp >= 60.0) {
        outputPWM = 0;  // Force heater OFF
        
        // Disable all heater H-bridge pins
        analogWrite(heaterPlateTopPinEnable, 0);
        analogWrite(heaterPlateBottomPinEnable, 0);
        digitalWrite(heaterPlateTopPinInputOne, LOW);
        digitalWrite(heaterPlateTopPinInputTwo, LOW);
        digitalWrite(heaterPlateBottomPinInputOne, LOW);
        digitalWrite(heaterPlateBottomPinInputTwo, LOW);
        
        // Report which temperature triggered the emergency
        Serial.print(F("$ERROR,EMERGENCY_TEMP_LIMIT_EXCEEDED,Media:"));
        Serial.print(mediaTemp, 2);
        Serial.print(F("C,Heater:"));
        Serial.print(heaterPlateTemp, 2);
        Serial.print(F("C,Limits:"));
        Serial.print(EMERGENCY_TEMP_LIMIT, 1);
        Serial.println(F("C/60.0C"));
        
        // DISABLED: Auto-pause disabled - heater is already off for safety
        // The heater is disabled above, which is sufficient protection
        // if (isRunning && !isPaused) {
        //     isPaused = true;
        //     stirringOFF();  // Also stop stirring for safety
        //     Serial.println(F("$DEBUG,AUTO_PAUSED_OVERTEMP"));
        // }
        return;  // Skip PID computation - emergency mode
    }
    
    // ===== LAYER 2: PAUSE SAFETY =====
    // When paused, disable heating but continue temperature monitoring.
    // This prevents unexpected heating during manual intervention.
    if (isPaused) {
        outputPWM = 0;  // Heater OFF when paused
    } else {
        // ===== LAYER 3: PID TEMPERATURE CONTROL =====
        // Normal operating mode: maintain setpoint temperature using PID control
        
        // Calculate distance from setpoint for adaptive tuning
        double gap = abs(incubationSetpointTemp - mediaTemp);
        
        // Adaptive PID tuning based on distance from setpoint
        if (gap < gapThreshold) {
            // Close to setpoint: use conservative tuning
            mediaTempPID.SetTunings(consKp, consKi, consKd);
        } else {
            // Far from setpoint: use aggressive tuning
            mediaTempPID.SetTunings(aggKp, aggKi, aggKd);
        }
        
        // Run PID controller only if below setpoint (no offset - turn off AT setpoint)
        if (mediaTemp < incubationSetpointTemp) {
            mediaTempPID.Compute();  // PID outputs 0-255
            outputPWM = map(outputPWM, 0, 255, 0, pwmSafetyLimit);  // Scale to safe PWM limit (0-130)
            
            // Additional proportional reduction in the final approach
            // When within 0.1°C of setpoint, scale PWM down proportionally
            double finalApproachGap = 0.1;  // Start reducing PWM at 0.1°C from setpoint (very close)
            if (gap < finalApproachGap) {
                // Scale PWM from full power at 0.1°C away to 0 at setpoint
                double scaleFactor = gap / finalApproachGap;  // 1.0 at 0.1°C away, 0.0 at setpoint
                outputPWM = outputPWM * scaleFactor;
            }
        } else {
            outputPWM = 0;  // At or above setpoint: turn off heater
        }
        
        // ===== LAYER 4: HEATER PLATE PROTECTION =====
        // Progressive PWM reduction as heater plate approaches emergency limit (60°C)
        // Starts reducing at 55°C, reaches minimum at 60°C
        const double plateStartReduce = 55.0;  // Start reducing PWM when plate reaches this temp
        const double plateMaxReduce = 60.0;    // Maximum reduction at this temp (at emergency limit)
        const double minPWMFactor = 0.5;       // Minimum PWM factor (50% of calculated PWM)
        
        if (heaterPlateTemp > plateStartReduce) {
            // Calculate reduction factor: 1.0 at 55°C, 0.5 at 60°C
            double reductionRange = plateMaxReduce - plateStartReduce;  // 8 degrees
            double tempAboveStart = heaterPlateTemp - plateStartReduce;
            double reductionFactor = 1.0 - ((1.0 - minPWMFactor) * (tempAboveStart / reductionRange));
            
            // Clamp factor between minPWMFactor and 1.0
            if (reductionFactor < minPWMFactor) reductionFactor = minPWMFactor;
            if (reductionFactor > 1.0) reductionFactor = 1.0;
            
            outputPWM = outputPWM * reductionFactor;
        }
    }
    
    // ===== APPLY HEATER CONTROL =====
    // CRITICAL: Always write to pins, even when PWM=0, to ensure proper state.
    // H-bridge motor drivers require both PWM and direction pins to be set.
    analogWrite(heaterPlateTopPinEnable, outputPWM);      // Top heater PWM
    analogWrite(heaterPlateBottomPinEnable, outputPWM);   // Bottom heater PWM
    digitalWrite(heaterPlateTopPinInputOne, HIGH);         // Enable forward direction
    digitalWrite(heaterPlateTopPinInputTwo, LOW);
    digitalWrite(heaterPlateBottomPinInputOne, HIGH);
    digitalWrite(heaterPlateBottomPinInputTwo, LOW);
}

// ============================================================================
// HARDWARE CONTROL FUNCTIONS
// ============================================================================
// These functions provide direct control over hardware components.
// Used during startup, shutdown, and emergency situations.

/**
 * Turn ON the stirring motor at the configured speed.
 * Motor uses simple PWM control (no H-bridge direction needed).
 */
void stirringON() {
    digitalWrite(motorStirringPinInputOne, LOW);
    analogWrite(motorStirringPinInputTwo, motorStirringSpeed);
}

/**
 * Turn OFF the stirring motor completely.
 * Sets both pins LOW to ensure motor is fully stopped.
 */
void stirringOFF() {
    digitalWrite(motorStirringPinInputOne, LOW);
    digitalWrite(motorStirringPinInputTwo, LOW);
}

/**
 * EMERGENCY HEATER SHUTDOWN
 * Completely disables the heater by:
 *   1. Setting PWM to 0 (no power)
 *   2. Setting all direction pins LOW (disable H-bridge)
 * 
 * Used during:
 *   - System startup (ensure heater starts OFF)
 *   - Manual stop/kill commands
 *   - State transitions (entering menu/standby)
 *   - Emergency situations
 */
void heaterOFF() {
    analogWrite(heaterPlateTopPinEnable, 0);
    analogWrite(heaterPlateBottomPinEnable, 0);
    digitalWrite(heaterPlateTopPinInputOne, LOW);
    digitalWrite(heaterPlateTopPinInputTwo, LOW);
    digitalWrite(heaterPlateBottomPinInputOne, LOW);
    digitalWrite(heaterPlateBottomPinInputTwo, LOW);
}

void allPumpsStop() {
    pumpOne->run(RELEASE);
    pumpTwo->run(RELEASE);
    pumpThree->run(RELEASE);
    pumpFour->run(RELEASE);
}

/**
 * AUTOMATED MEDIA CHANGE
 * =======================
 * Performs the core turbidostat function: replaces culture with fresh media
 * when OD exceeds threshold. This maintains cells in exponential growth phase.
 * 
 * PROCESS (2 steps):
 *   1. EMPTY: Remove 1.5x volume (e.g., 60mL if max is 40mL)
 *      - Uses pump 4 (waste pump) in reverse
 *      - Extra volume ensures thorough emptying
 *   
 *   2. FILL: Add fresh media (e.g., 40mL)
 *      - Pump selection based on NEXT step's media type:
 *        * NEUTRAL  → Pump 1
 *        * POSITIVE → Pump 2  
 *        * NEGATIVE → Pump 3
 *      - This ensures correct media for upcoming cycle
 * 
 * SAFETY FEATURES:
 *   - 2.5 second delay between empty and fill (prevents mixing)
 *   - Pump verification after emptying
 *   - Progress updates to interface (every 0.1mL)
 *   - LCD feedback (every 0.2mL)
 * 
 * WATCHDOG INTEGRATION:
 *   - Records OD before change for post-change verification
 *   - If OD doesn't drop significantly, watchdog triggers
 * 
 * CALLED BY:
 *   - Main experiment loop when currentOD940 > dynamicThreshold
 *   - SKIP_STEP command (forces media change)
 */
void performMediaChange() {
    mediaChangeInProgress = true;
    lastMediaChangeTime = millis();
    
    // ========================================================================
    // CRITICAL SAFETY: FORCE HEATER OFF DURING ALL PUMPING
    // ========================================================================
    // The main loop's heaterCompute() doesn't run during pumping (blocking delays).
    // Without this, the heater stays at whatever PWM it was before pumping started,
    // which can cause dangerous overheating during long pump cycles.
    // This was the cause of the Dec 10, 2025 overheating incident.
    heaterOFF();
    outputPWM = 0;
    Serial.println(F("$DEBUG,SAFETY_HEATER_OFF_DURING_PUMPING"));
    
    // preMediaChangeOD is now set at peak detection (before this function is called)
    // This ensures we capture the exact OD that triggered dilution
    
    Serial.print(F("$DEBUG,MEDIA_CHANGE_STARTING,max_dispensations="));
    Serial.println(max_dispensations);
    
    // ========================================================================
    // STEP 1: EMPTY CULTURE
    // ========================================================================
    // Empty 1.5x the normal volume to ensure thorough removal.
    // Example: If max_dispensations = 400 (40mL), empty 600 (60mL)

    int emptyDispensations = (max_dispensations * 3) / 2;  // 1.5x volume
    for (int i = 0; i < emptyDispensations; i++) {
        // Pulse pump: 100ms ON, 10ms OFF
        // This provides precise volume control (0.1mL per pulse)
        pumpFour->run(BACKWARD);
        pumpFour->setSpeed(pumpFourSpeed);
        delay(100);
        pumpFour->run(RELEASE);
        delay(10);
        
        // Send progress to interface every 0.1mL for real-time feedback
        Serial.print(F("$PUMP_PROGRESS,EMPTYING,"));
        Serial.print((i + 1) * volumePerDispensation, 1);  // Current volume
        Serial.print(F(","));
        Serial.print(emptyDispensations * volumePerDispensation, 1);  // Target volume
        Serial.println();
        
        // Update LCD every 0.2mL (reduces I2C traffic)
        if ((i + 1) % 2 == 0) {
            setMultiplexerFocus(0);
            lcd.clear();
            lcd.print(F("EMPTYING..."));
            lcd.setCursor(0, 1);
            lcd.print(F("Vol: "));
            lcd.print((i + 1) * volumePerDispensation, 1);
            lcd.print(F("mL"));
            
            // Mirror LCD to interface for remote monitoring
            Serial.print(F("$LCD_CONTENT,"));
            Serial.print(F("EMPTYING...|Vol: "));
            Serial.print((i + 1) * volumePerDispensation, 1);
            Serial.print(F("mL||"));
            Serial.println();
        }
        
        // SAFETY: Check temperature every 10 pulses (~1mL) during pumping
        // The main loop doesn't run during pumping, so we must check here
        if ((i + 1) % 10 == 0) {
            heaterOFF();  // Redundant but critical - ensure heater stays OFF
        }
    }
    
    // ===== CRITICAL SAFETY DELAY =====
    // Wait 2.5 seconds to ensure:
    //   1. All liquid has drained from vessel
    //   2. Pumps have completely stopped
    //   3. No turbulence remains before filling
    // This prevents old/new media mixing and ensures accurate OD drop
    Serial.println(F("$DEBUG,EMPTYING_COMPLETE"));
    delay(2000);
    pumpFour->run(RELEASE);  // Verify pump is stopped
    delay(500);
    
    Serial.println(F("$DEBUG,STARTING_FILL"));
    
    // ========================================================================
    // STEP 2: FILL WITH FRESH MEDIA
    // ========================================================================
    // Select pump based on NEXT step's media type (not current step).
    // This ensures the correct media is loaded for the upcoming cycle.
    Adafruit_DCMotor *fillPump;
    int fillPumpSpeed;
    
    // ===== DETERMINE NEXT STEP'S MEDIA TYPE =====
    // We need to look ahead to see what media the NEXT step requires,
    // because media changes happen at the END of a cycle.
    int nextStep = currentStep;
    int nextCycle = currentCycle;
    
    // Calculate next step (with cycle rollover to step 1)
    nextStep++;
    if (nextStep > totalProgramSteps && totalProgramSteps > 0) {
        nextStep = 1;
        nextCycle++;
    }
    
    // Get next step's media type from program
    char nextMediaType[9];
    if (nextStep >= 1 && nextStep <= totalProgramSteps) {
        strcpy(nextMediaType, stepProgram[nextStep - 1].mediaType);  // Array is 0-indexed
    } else {
        strcpy(nextMediaType, "NEUTRAL");  // Fallback if program not defined
    }
    
    // Debug output for troubleshooting pump selection
    Serial.print(F("$DEBUG,MEDIA_CHANGE_PUMP_SELECTION,NextMediaType:"));
    Serial.print(nextMediaType);
    Serial.print(F(",NextStep:"));
    Serial.print(nextStep);
    Serial.print(F(",NextCycle:"));
    Serial.println(nextCycle);
    
    // ===== PUMP SELECTION BASED ON MEDIA TYPE =====
    // Each media type has a dedicated pump for contamination prevention:
    //   NEUTRAL  → Pump 1 (minimal media, growth control)
    //   POSITIVE → Pump 2 (selection media, e.g., antibiotic)
    //   NEGATIVE → Pump 3 (alternative media, e.g., without selection)
    if (strcmp(nextMediaType, "NEUTRAL") == 0) {
        fillPump = pumpOne;
        fillPumpSpeed = pumpOneSpeed;
        Serial.println(F("$DEBUG,FILLING_WITH_NEUTRAL_MEDIA,PUMP_ONE"));
    } else if (strcmp(nextMediaType, "POSITIVE") == 0) {
        fillPump = pumpTwo;
        fillPumpSpeed = pumpTwoSpeed;
        Serial.println(F("$DEBUG,FILLING_WITH_POSITIVE_MEDIA,PUMP_TWO"));
    } else if (strcmp(nextMediaType, "NEGATIVE") == 0) {
        fillPump = pumpThree;
        fillPumpSpeed = pumpThreeSpeed;
        Serial.println(F("$DEBUG,FILLING_WITH_NEGATIVE_MEDIA,PUMP_THREE"));
    } else {  // Unknown media type defaults to neutral for safety
        fillPump = pumpOne;
        fillPumpSpeed = pumpOneSpeed;
        Serial.println(F("$DEBUG,FILLING_WITH_DEFAULT_NEUTRAL_MEDIA,PUMP_ONE"));
    }

    int fillDispensations = max_dispensations;
    for (int i = 0; i < fillDispensations; i++) {
        fillPump->run(BACKWARD);
        fillPump->setSpeed(fillPumpSpeed);
        delay(100);
        fillPump->run(RELEASE);
        delay(10);
        
        // Send progress to interface every dispensation (0.1mL increments)
        Serial.print(F("$PUMP_PROGRESS,FILLING,"));
        Serial.print((i + 1) * volumePerDispensation, 1);
        Serial.print(F(","));
        Serial.print(fillDispensations * volumePerDispensation, 1);
        Serial.println();
        
        // Show progress on LCD less frequently to avoid blocking (every 0.2mL)
        if ((i + 1) % 2 == 0) {
            setMultiplexerFocus(0);
            lcd.clear();
            lcd.print(F("FILLING "));
            lcd.print(nextMediaType);
            lcd.setCursor(0, 1);
            lcd.print(F("Pump"));
            // Determine which pump number is being used
            int pumpNumber = 0;
            if (fillPump == pumpOne) pumpNumber = 1;
            else if (fillPump == pumpTwo) pumpNumber = 2;
            else if (fillPump == pumpThree) pumpNumber = 3;
            else if (fillPump == pumpFour) pumpNumber = 4;
            lcd.print(pumpNumber);
            lcd.print(F(" Vol:"));
            lcd.print((i + 1) * volumePerDispensation, 1);
            lcd.print(F("mL"));
            
            // Send LCD content to interface
            Serial.print(F("$LCD_CONTENT,"));
            Serial.print(F("FILLING "));
            Serial.print(nextMediaType);
            Serial.print(F(" Pump"));
            Serial.print(pumpNumber);
            Serial.print(F("|Vol: "));
            Serial.print((i + 1) * volumePerDispensation, 1);
            Serial.print(F("mL||"));
            Serial.println();
        }
        
        // SAFETY: Check temperature every 10 pulses (~1mL) during filling
        // The main loop doesn't run during pumping, so we must check here
        if ((i + 1) % 10 == 0) {
            heaterOFF();  // Redundant but critical - ensure heater stays OFF
        }
    }
    
    // Ensure filling pump is fully stopped
    fillPump->run(RELEASE);
    delay(500);
    
    Serial.println(F("$DEBUG,FILLING_COMPLETE"));
    Serial.println(F("$DEBUG,MEDIA_CHANGE_COMPLETE"));
    
    mediaChangeInProgress = false;
    
    // Advance step after media change
    advanceStep();
    
    // Peak notification is now sent from the main loop (after advanceStep completes)
    // This ensures the correct NEW media type is included with PEAK_DETECTED
    
    // NOW start the watchdog timer (after all pumping is done)
    if (preMediaChangeOD > 0) {
        mediaChangeStartTime = millis();
        Serial.println(F("$DEBUG,WATCHDOG_TIMER_STARTED,pumping_complete"));
    }
}

// =======================================================================================
// === Step Advancement ==================================================================
// =======================================================================================
/**
 * ADVANCE TO NEXT PROGRAM STEP
 * ==============================
 * Progresses the experiment to the next step in the cycler program.
 * Called after each media change to update media type and LED settings.
 * 
 * STEP PROGRESSION:
 *   - Steps count from 1 to totalProgramSteps
 *   - When reaching the last step, rolls over to step 1 and increments cycle
 *   - Example: With 3 steps, sequence is: 1→2→3→1(cycle++)→2→3→1(cycle++)...
 * 
 * LED CYCLING:
 *   - LEDs advance every N cycles (controlled by cyclesPerLEDChange)
 *   - Example: If cyclesPerLEDChange=2, LED changes on cycles 2,4,6,8...
 *   - Each LED corresponds to a different wavelength for evolution experiments
 *   - Physical LEDs: Pin 1 (cycle 1), Pin 2 (cycle 2), ..., Pin 15 (cycle 15)
 * 
 * CALLED BY:
 *   - Main experiment loop after performMediaChange()
 *   - SKIP_STEP command (manual advancement)
 *   - JUMP_TO command (direct step change)
 */
void advanceStep() {
    Serial.print(F("$DEBUG,ADVANCE_STEP_START,Cycle:"));
    Serial.print(currentCycle);
    Serial.print(F(",Step:"));
    Serial.print(currentStep);
    Serial.print(F("/"));
    Serial.println(totalProgramSteps);
    
    // Move to next step
    currentStep++;
    
    // ===== CYCLE ROLLOVER =====
    // When we've completed all steps, start over at step 1 and increment cycle
    if (currentStep > totalProgramSteps && totalProgramSteps > 0) {
        currentStep = 1;
        currentCycle++;
        Serial.print(F("$DEBUG,CYCLE_ROLLOVER,NewCycle:"));
        Serial.println(currentCycle);
        
        // LED advancement check happens in applyStepSettings()
    }
    
    Serial.print(F("$DEBUG,ADVANCE_STEP_END,Cycle:"));
    Serial.print(currentCycle);
    Serial.print(F(",Step:"));
    Serial.println(currentStep);
    
    // ===== APPLY NEW STEP'S SETTINGS =====
    // Updates media type and LED brightness for the new step
    applyStepSettings(currentStep - 1);  // Convert to 0-indexed array position
}

// ============================================================================
// APPLY PROGRAM STEP SETTINGS
// ============================================================================
/**
 * APPLY STEP SETTINGS
 * ====================
 * Updates experiment parameters based on the current cycler program step.
 * Handles media type changes and conditional LED cycling.
 * 
 * WHAT CHANGES:
 *   - Media type (NEUTRAL/POSITIVE/NEGATIVE) - changes EVERY step
 *   - LED brightness - changes only when cycle is divisible by cyclesPerLEDChange
 * 
 * LED CYCLING LOGIC:
 *   - LEDs don't change every step - they change every N cycles
 *   - cyclesPerLEDChange = 2 means LED changes on cycles 2, 4, 6, 8...
 *   - 6 LED pins available (pins 1-6), cycles through them repeatedly
 *   - Example with cyclesPerLEDChange=2:
 *       Cycle 1-2: LED pin 1
 *       Cycle 3-4: LED pin 2
 *       Cycle 5-6: LED pin 3
 *       Cycle 7-8: LED pin 4
 *       Cycle 9-10: LED pin 5
 *       Cycle 11-12: LED pin 6
 *       Cycle 13-14: LED pin 1 (wraps around)
 * 
 * LED BRIGHTNESS PATTERN:
 *   - The brightness pattern comes from the step program
 *   - All LEDs follow the same brightness pattern (0-4095 PWM)
 *   - Brightness can cycle within a single LED (e.g., step 1=bright, step 2=dim, step 3=off)
 * 
 * PARAMETERS:
 *   stepIndex - 0-based array index (currentStep-1)
 */
void applyStepSettings(int stepIndex) {
    Serial.print(F("$DEBUG,APPLY_STEP_SETTINGS,stepIndex:"));
    Serial.print(stepIndex);
    Serial.print(F(",totalProgramSteps:"));
    Serial.print(totalProgramSteps);
    Serial.print(F(",currentStep:"));
    Serial.println(currentStep);
    
    // Bounds check
    if (stepIndex >= 0 && stepIndex < totalProgramSteps) {
        StepData currentStepData = stepProgram[stepIndex];
        
        // ===== MEDIA TYPE UPDATE (ALWAYS) =====
        // Media type changes with every step
        strcpy(currentMediaType, currentStepData.mediaType);
        Serial.print(F("$DEBUG,MEDIA_TYPE_UPDATED,"));
        Serial.print(F("Step:"));
        Serial.print(currentStep);
        Serial.print(F(",StepIndex:"));
        Serial.print(stepIndex);
        Serial.print(F(",MediaType:"));
        Serial.println(currentMediaType);
        
        // ===== LED UPDATE (ALWAYS) =====
        // LED must be set EVERY step so it can be restored after OD measurements
        // The PIN stays the same within a cycle period (controlled by cyclesPerLEDChange)
        // Example with cyclesPerLEDChange=2:
        //   Cycles 1-2: LED pin 1
        //   Cycles 3-4: LED pin 2
        //   Cycles 5-6: LED pin 3
        
        if (!ledCalibrationMode) {
            // Turn off all 15 LED channels first
            for (int i = 1; i <= 15; i++) {
                ledDriver.setPWM(i, 0);
            }
            
            // Calculate which physical LED pin to use
            // Same pin is used for multiple cycles based on cyclesPerLEDChange
            int ledCycleIndex = (currentCycle - 1) / cyclesPerLEDChange;  // Which LED period
            int ledPin = (ledCycleIndex % 6) + 1;  // Cycle through pins 1-6
            
            // Get brightness from step program
            currentLEDBrightness = currentStepData.ledBrightness;
            currentLEDState = (currentLEDBrightness > 0);
            currentLEDPin = ledPin;
            
            Serial.print(F("$DEBUG,LED_UPDATE,Cycle:"));
            Serial.print(currentCycle);
            Serial.print(F(",Pin:"));
            Serial.print(ledPin);
            Serial.print(F(",Brightness:"));
            Serial.println(currentLEDBrightness);
            
            // Activate the selected LED at the specified brightness
            if (currentLEDState) {
                ledDriver.setPWM(ledPin, currentLEDBrightness);
            }
            ledDriver.write();  // Apply changes to hardware
        }
    } else {
        // Error: step index out of bounds
        Serial.print(F("$DEBUG,APPLY_STEP_SETTINGS_ERROR,stepIndex:"));
        Serial.print(stepIndex);
        Serial.print(F(" is out of bounds (0-"));
        Serial.print(totalProgramSteps - 1);
        Serial.println(F(")"));
    }
}

void applyStepSettingsWithLED(int stepIndex) {
    if (stepIndex >= 0 && stepIndex < totalProgramSteps) {
        StepData currentStepData = stepProgram[stepIndex];
        
        // Media type always follows current step
        strcpy(currentMediaType, currentStepData.mediaType);
        Serial.print(F("$DEBUG,MEDIA_TYPE_UPDATED_JUMP,"));
        Serial.print(F("Step:"));
        Serial.print(currentStep);
        Serial.print(F(",MediaType:"));
        Serial.println(currentMediaType);
        
        // Skip LED changes if in calibration mode
        if (ledCalibrationMode) {
            Serial.println(F("$DEBUG,LED_CHANGE_SKIPPED,IN_CALIBRATION_MODE"));
            return;
        }
        
        // Force LED update regardless of cycle logic
        // Turn off all LEDs first
        for (int i = 1; i <= 15; i++) {
            ledDriver.setPWM(i, 0);
        }
        
        // Calculate which LED pin to use based on cycle number (6 LEDs available)
        int ledCycleIndex = (currentCycle - 1) / cyclesPerLEDChange; // Which LED cycle we're in
        int ledPin = (ledCycleIndex % 6) + 1; // Cycle through LED pins 1-6
        
        // Get LED brightness from current step (all LEDs use same brightness pattern)
        currentLEDBrightness = currentStepData.ledBrightness;
        currentLEDState = (currentLEDBrightness > 0);
        currentLEDPin = ledPin;
        
        if (currentLEDState) {
            ledDriver.setPWM(ledPin, currentLEDBrightness);
            Serial.print(F("LED "));
            Serial.print(ledPin);
            Serial.print(F(" set to "));
            Serial.print((currentLEDBrightness / 4095.0) * 100, 1);
            Serial.println(F("%"));
        } else {
            Serial.println(F("LED turned OFF"));
        }
        ledDriver.write();
        
        Serial.print(F("Applied step settings: Media="));
        Serial.print(currentMediaType);
        Serial.print(F(", LED="));
        Serial.print((currentLEDBrightness / 4095.0) * 100, 1);
        Serial.println(F("%"));
    }
}

void updateStirringRamp() {
    if (stirringRampUpActive) {
        unsigned long elapsed = millis() - stirringRampStartTime;
        if (elapsed >= STIRRING_RAMP_DURATION) {
            // Ramp complete - use full speed
            stirringRampUpActive = false;
            motorStirringSpeed = 90;  // Full speed

        } else {
            // Calculate ramped speed
            float progress = (float)elapsed / STIRRING_RAMP_DURATION;
            motorStirringSpeed = STIRRING_RAMP_START_SPEED + (80 * progress); // 10 to 90
        }
    }
}

// SIMPLE SD CARD FUNCTIONS
void initializeSDCard() {
    Serial.println(F("=== Initializing SD Card ==="));
    Serial.print(F("SD CS Pin: "));
    Serial.println(sdCardCSPin);
    
    // Try only once during startup to avoid blocking
    Serial.print(F("SD init attempt 1/1... "));
    
    if (SD.begin(sdCardCSPin)) {
        // SD.begin() succeeded, but verify we can actually access the card
        File testRoot = SD.open("/");
        if (testRoot) {
            sdCardAvailable = true;
            testRoot.close();
            Serial.println(F("SUCCESS!"));
            Serial.println(F("✅ SD Card initialized successfully"));
            
            // Test if we can actually access the SD card
            if (SD.exists("OEVO.csv")) {
                Serial.println(F("✅ OEVO.csv found on SD card"));
            } else {
                Serial.println(F("ℹ️ OEVO.csv not found (will be created)"));
            }
        } else {
            // SD.begin() succeeded but can't open root - no card present
            sdCardAvailable = false;
            Serial.println(F("FAILED (No card detected)"));
            Serial.println(F("⚠️ SD Card not available - insert card and retry"));
            displaySDError("No Card");
        }
    } else {
        sdCardAvailable = false;
        Serial.println(F("FAILED"));
        Serial.println(F("⚠️ SD Card not available - will retry on INIT_SD_CARD command"));
        displaySDError("Init");
        return;
    }
}

/**
 * DATA LOGGING TO SD CARD
 * ========================
 * Records experiment data to OEVO.csv on SD card for offline analysis.
 * Called regularly during experiment (typically every 2 seconds).
 * 
 * DATA FORMAT (11 fields):
 *   unixTime          - Real-world timestamp from RTC (0 if RTC not set)
 *   upTime            - Milliseconds since Arduino started
 *   OD940             - Calculated optical density (4 decimals)
 *   infraredReading   - Raw IR sensor value
 *   ambientTemp       - Room temperature from RTC (°C)
 *   mediaTemp         - Culture temperature (°C)
 *   heaterPlateTemp   - Heater surface temperature (°C)
 *   totalCycleCount   - Number of completed cycles
 *   currentCycle      - Current cycle number
 *   currentStep       - Current step in program
 *   mediaType         - NEUTRAL/POSITIVE/NEGATIVE
 * 
 * FILE HANDLING:
 *   - Opens file if not already open
 *   - Appends to end of file
 *   - Closes immediately after write (data safety)
 *   - Skips if SD card unavailable (prevents data loss)
 * 
 * ERROR HANDLING:
 *   - Checks SD card availability before writing
 *   - Displays error on LCD (rate-limited to avoid spam)
 *   - Continues experiment even if logging fails
 */
void logDataToSD() {
    // ===== SD CARD AVAILABILITY CHECK =====
    if (!sdCardAvailable) {
        // Rate-limited error display to avoid spamming LCD
        static unsigned long lastSDErrorDisplay = 0;
        if (millis() - lastSDErrorDisplay > 5000) {
            displaySDError("Log Data");
            lastSDErrorDisplay = millis();
        }
        return;
    }
    
    // ===== FILE OPENING =====
    // If file isn't open, attempt to open it for appending
    if (!dataFile) {
        dataFile = SD.open(sdFileName, FILE_WRITE);
        if (!dataFile) {
            Serial.println(F("$DEBUG,DATA_FILE_OPEN_FAILED"));
            displaySDError("Write Data");
            return;
        }
    }
    
    // Ensure we're writing at the end of the file (append mode)
    dataFile.seek(dataFile.size());
    
    // ===== TIMESTAMP COLLECTION =====
    unsigned long currentTime = millis();
    setMultiplexerFocus(1);  // Switch I2C to RTC channel
    DateTime now = rtc.now();
    unsigned long unixTime = (now.year() > 2000) ? now.unixtime() : 0;
    
    // ===== DATA WRITE (CSV FORMAT) =====
    // Format: unixTime,upTime,OD940,infraredReading,ambientTemp,mediaTemp,heaterPlateTemp,
    //         totalCycleCount,currentCycle,currentStep,mediaType
    dataFile.print(unixTime);
    dataFile.print(F(","));
    dataFile.print(currentTime);
    dataFile.print(F(","));
    dataFile.print(currentOD940, 4);  // 4 decimal places for precision
    dataFile.print(F(","));
    dataFile.print(IR);  // Raw IR sensor value
    dataFile.print(F(","));
    dataFile.print(ambientTemp, 2);
    dataFile.print(F(","));
    dataFile.print(mediaTemp, 2);
    dataFile.print(F(","));
    dataFile.print(heaterPlateTemp, 2);
    dataFile.print(F(","));
    dataFile.print(currentCycle - 1);  // Total completed cycles
    dataFile.print(F(","));
    dataFile.print(currentCycle);  // Current cycle in progress
    dataFile.print(F(","));
    dataFile.print(currentStep);  // Current step in program
    dataFile.print(F(","));
    dataFile.println(currentMediaType);  // NEUTRAL/POSITIVE/NEGATIVE
    
    // ===== FILE CLOSURE =====
    // Close immediately to ensure data is written (data safety).
    // This prevents data loss if power fails or SD card is removed.
    dataFile.close();
}

void logAveragedDataToSD() {
    if (!sdCardAvailable || !dataFile) { // Check the global handle
        return;
    }
    if (dataPointCount == 0) {
        return;
    }
    
    // NOTE: This function no longer opens/closes the file.
    // It uses the global 'dataFile' handle opened by START_TURBIDOSTAT.
    
    // Calculate averages
    float avgOD = odSum / dataPointCount;
    float avgTemp = tempSum / dataPointCount;
    int avgIR = irSum / dataPointCount;
    
    // Write averaged data matching new format: unixTime,upTime,OD940,infraredReading,ambientTemp,mediaTemp,heaterPlateTemp,totalCycleCount,currentCycle,currentStep,mediaType,Dilution_Event,LED_Channel,LED_Percent
    unsigned long currentTime = millis();
    setMultiplexerFocus(1);  // Switch to RTC channel before reading
    DateTime now = rtc.now();
    unsigned long unixTime = (now.year() > 2000) ? now.unixtime() : 0;
    
    // unixTime
    dataFile.print(unixTime);
    dataFile.print(",");
    
    // upTime
    dataFile.print(currentTime);
    dataFile.print(",");
    
    // OD940 (use saved peak OD if dilution was triggered, otherwise use averaged OD)
    dataFile.print(dilutionEventForSD ? savedPeakOD : avgOD, 4);
    dataFile.print(",");
    
    // infraredReading (averaged)
    dataFile.print(avgIR);
    dataFile.print(",");
    
    // ambientTemp (current reading from RTC)
    dataFile.print(ambientTemp, 2);
    dataFile.print(",");
    
    // mediaTemp (averaged)
    dataFile.print(avgTemp, 2);
    dataFile.print(",");
    
    // heaterPlateTemp (current reading)
    dataFile.print(heaterPlateTemp, 2);
    dataFile.print(",");
    
    // totalCycleCount
    dataFile.print(currentCycle - 1); // Fixed: total completed cycles
    dataFile.print(",");
    
    // currentCycle
    dataFile.print(currentCycle);
    dataFile.print(",");
    
    // currentStep
    dataFile.print(currentStep);
    dataFile.print(",");
    
    // mediaType
    dataFile.print(currentMediaType);
    dataFile.print(",");
    
    // Dilution_Event (1 if dilution triggered, 0 otherwise)
    dataFile.print(dilutionEventForSD ? 1 : 0);
    dataFile.print(",");
    
    // LED_Channel (1-6, which LED is currently active)
    dataFile.print(currentLEDPin);  // <-- LED channel number (1-6)
    dataFile.print(",");
    
    // LED_Percent (0-100%, effective light intensity considering OD measurement duty cycle)
    // Effective dose = PWM% × 0.885 (LED is off ~11.5% of time during OD measurements)
    int ledPercent = (currentLEDBrightness / 4095.0) * 100;  // <-- % (PWM duty cycle)
    int effectiveLedPercent = ledPercent * 0.885;  // <-- % (effective dose, accounting for OD measurement)
    dataFile.println(effectiveLedPercent);
    
    // Reset the SD dilution event flag and saved peak OD after logging
    // NOTE: dilutionEventFlag (for serial) is reset separately in the serial output section
    dilutionEventForSD = false;
    savedPeakOD = 0.0;
    
    // Periodically flush data to the card to save it without closing the file
    static unsigned long lastFlush = 0;
    if (millis() - lastFlush > 15000) { // Flush every 15 seconds
        dataFile.flush();
        lastFlush = millis();
        Serial.println(F("DEBUG: Flushed dataFile to SD card."));
    }
}

/**
 * SAVE CONFIGURATION TO SD CARD
 * ===============================
 * Saves all experiment configuration parameters to config.txt.
 * This allows the system to resume with correct settings after power loss.
 * 
 * SAVED PARAMETERS (20 fields):
 *   Temperature & Hardware:
 *     - incubationSetpointTemp  - Culture temperature setpoint (°C)
 *     - OD940LEDBrightness      - IR LED brightness (0-4095)
 *     - motorStirringSpeed      - Stirring motor PWM (0-255)
 *   
 *   Pump Configuration:
 *     - pumpOneSpeed through pumpFourSpeed - PWM values (0-4095)
 *   
 *   Experiment Settings:
 *     - threshold              - OD threshold for media change
 *     - cyclesPerLEDChange     - How many cycles before advancing LED step
 *     - maxDispensations       - Max number of media changes per cycle
 *   
 *   Calibration Data (4-point):
 *     - irLower/odLower        - Point 1: Low density calibration point
 *     - irMidLow/odMidLow      - Point 2: Mid-low density calibration point
 *     - irMidHigh/odMidHigh    - Point 3: Mid-high density calibration point
 *     - irUpper/odUpper        - Point 4: High density calibration point
 *     - inverseA/inverseB      - Inverse calibration coefficients (OD = a/IR + b)
 *     - useInverse             - Whether inverse calibration is active
 * 
 * FILE HANDLING:
 *   - Deletes old file first (ensures clean write)
 *   - Opens config.txt in write mode
 *   - Closes immediately after write (data safety)
 * 
 * CALLED BY:
 *   - SAVE_CONFIG_TO_SD serial command
 *   - Configuration updates from interface
 *   - After calibration changes
 */
void saveConfigToSD() {
    // ===== SD CARD AVAILABILITY CHECK =====
    if (!sdCardAvailable) {
        Serial.println(F("DEBUG: saveConfigToSD failed, SD not available."));
        displaySDError("Config not saved");
        return;
    }

    // ===== CLEAN WRITE: Remove old file first =====
    // This prevents corruption from partial writes
    if (SD.exists("config.txt")) {
        SD.remove("config.txt");
    }

    // ===== OPEN FILE FOR WRITING =====
    File configFile = SD.open("config.txt", FILE_WRITE);
    if (!configFile) {
        Serial.println(F("$SD_ERROR,Failed to save configuration"));
        return;
    }
    
    // ===== WRITE CONFIGURATION (KEY=VALUE FORMAT) =====
    // Temperature and hardware settings
    configFile.print("incubationSetpointTemp=");
    configFile.println(incubationSetpointTemp, 2);
    configFile.print("OD940LEDBrightness=");
    configFile.println(OD940LEDBrightness);
    configFile.print("motorStirringSpeed=");
    configFile.println(motorStirringSpeed);
    
    // Pump speeds (1-4)
    configFile.print("pumpOneSpeed=");
    configFile.println(pumpOneSpeed);
    configFile.print("pumpTwoSpeed=");
    configFile.println(pumpTwoSpeed);
    configFile.print("pumpThreeSpeed=");
    configFile.println(pumpThreeSpeed);
    configFile.print("pumpFourSpeed=");
    configFile.println(pumpFourSpeed);
    
    // Experiment control parameters
    configFile.print("threshold=");
    configFile.println(dynamicThreshold, 2);
    configFile.print("cyclesPerLEDChange=");
    configFile.println(cyclesPerLEDChange);
    configFile.print("maxDispensations=");
    configFile.println(max_dispensations);
    
    // 4-Point calibration data (IR readings and corresponding OD values)
    configFile.print("irLower=");
    configFile.println(irLower);
    configFile.print("odLower=");
    configFile.println(odLower, 3);
    configFile.print("irMidLow=");
    configFile.println(irMidLow);
    configFile.print("odMidLow=");
    configFile.println(odMidLow, 3);
    configFile.print("irMidHigh=");
    configFile.println(irMidHigh);
    configFile.print("odMidHigh=");
    configFile.println(odMidHigh, 3);
    configFile.print("irUpper=");
    configFile.println(irUpper);
    configFile.print("odUpper=");
    configFile.println(odUpper, 3);
    
    // Inverse calibration coefficients (preferred method)
    configFile.print("inverseA=");
    configFile.println(inverseA, 6);
    configFile.print("inverseB=");
    configFile.println(inverseB, 6);
    configFile.print("useInverse=");
    configFile.println(useInverse ? "true" : "false");
    
    configFile.println("END");  // End marker for parsing
    
    // ===== FILE CLOSURE =====
    configFile.close();
    Serial.println(F("DEBUG: config.txt saved successfully."));
}

void saveCyclerToSD() {
    if (!sdCardAvailable) {
        Serial.println(F("DEBUG: saveCyclerToSD failed, SD not available."));
        return;
    }

    // --- FILE SAFETY ---
    // Temporarily close the main data file if it's open.
    bool dataFileWasOpen = false;
    if (dataFile) {
        dataFile.close();
        dataFileWasOpen = true;
        Serial.println(F("DEBUG: Temporarily closed dataFile to save cycler."));
    }
    
    // Remove existing file to ensure clean overwrite
    if (SD.exists("cycler.csv")) {
        SD.remove("cycler.csv");
    }
    
    File cyclerFile = SD.open("cycler.csv", FILE_WRITE);
    if (cyclerFile) {
        // Write CSV header in simplified format
        cyclerFile.println("stepNumber,mediaType,stimulationLEDNumber,stimulationLEDintensity,cyclesPerLEDChange");
        
        // Write current step program
        for (int i = 0; i < totalProgramSteps; i++) {
            cyclerFile.print(i + 1);
            cyclerFile.print(",");
            cyclerFile.print(stepProgram[i].mediaType);
            cyclerFile.print(",");
            
            // stimulationLEDNumber: 0 for OFF, 1 for ON
            int ledNumber = (stepProgram[i].ledBrightness > 0) ? 1 : 0;
            cyclerFile.print(ledNumber);
            cyclerFile.print(",");
            
            // stimulationLEDintensity: Convert PWM (0-4095) to 0-255 scale
            int ledIntensity = map(stepProgram[i].ledBrightness, 0, 4095, 0, 255);
            cyclerFile.print(ledIntensity);
            cyclerFile.print(",");
            
            // cyclesPerLEDChange: Global setting
            cyclerFile.println(cyclesPerLEDChange);
        }
        
        cyclerFile.close();
        Serial.println("$CYCLER_SAVED,OK");
    } else {
        Serial.println("$CYCLER_SAVED,ERROR");
    }

    // --- FILE SAFETY ---
    // Re-open the main data file if it was open before.
    if (dataFileWasOpen) {
        dataFile = SD.open(sdFileName, FILE_WRITE);
        if (dataFile) {
            Serial.println(F("DEBUG: Re-opened dataFile for logging."));
        } else {
            Serial.println(F("ERROR: Failed to re-open dataFile after saving cycler!"));
        }
    }
}

// ===============================
// SAVE LED CALIBRATION TO SD CARD
// ===============================
// Saves user-measured LED intensity (mW/cm²) and wavelength (nm) for each LED
// File: led_cal.txt on SD card
void saveLEDCalibrationToSD() {
    if (!sdCardAvailable) {
        Serial.println(F("$LED_CAL_SAVE,ERROR,SD_NOT_AVAILABLE"));
        return;
    }
    
    // Remove existing file
    if (SD.exists("led_cal.txt")) {
        SD.remove("led_cal.txt");
    }
    
    File calFile = SD.open("led_cal.txt", FILE_WRITE);
    if (calFile) {
        // Write header comment
        calFile.println(F("# LED Calibration Data"));
        calFile.println(F("# Format: LED<n>=<intensity_mW_cm2>,<wavelength_nm>"));
        
        // Write calibration for each LED (1-6)
        for (int i = 0; i < 6; i++) {
            calFile.print(F("LED"));
            calFile.print(i + 1);
            calFile.print(F("="));
            calFile.print(ledCalibration[i].intensity, 2);
            calFile.print(F(","));
            calFile.println(ledCalibration[i].wavelength);
        }
        
        calFile.close();
        Serial.println(F("$LED_CAL_SAVE,OK"));
        
        // Print saved values for verification
        for (int i = 0; i < 6; i++) {
            if (ledCalibration[i].intensity > 0) {
                Serial.print(F("  LED"));
                Serial.print(i + 1);
                Serial.print(F(": "));
                Serial.print(ledCalibration[i].intensity, 2);
                Serial.print(F(" mW/cm² @ "));
                Serial.print(ledCalibration[i].wavelength);
                Serial.println(F("nm"));
            }
        }
    } else {
        Serial.println(F("$LED_CAL_SAVE,ERROR,FILE_OPEN_FAILED"));
    }
}

// ===============================
// LOAD LED CALIBRATION FROM SD CARD
// ===============================
void loadLEDCalibrationFromSD() {
    if (!sdCardAvailable) {
        return;
    }
    
    if (!SD.exists("led_cal.txt")) {
        Serial.println(F("INFO: led_cal.txt not found. Using default values (0)."));
        return;
    }
    
    File calFile = SD.open("led_cal.txt", FILE_READ);
    if (calFile) {
        Serial.println(F("Loading LED calibration from led_cal.txt..."));
        
        char line[64];
        while (calFile.available()) {
            int len = calFile.readBytesUntil('\n', line, sizeof(line) - 1);
            line[len] = '\0';
            
            // Skip comments and empty lines
            if (line[0] == '#' || len == 0) continue;
            
            // Parse LED<n>=<intensity>,<wavelength>
            if (strncmp(line, "LED", 3) == 0) {
                int ledNum = line[3] - '0';  // Get LED number (1-6)
                if (ledNum >= 1 && ledNum <= 6) {
                    char* eq = strchr(line, '=');
                    if (eq) {
                        float intensity = atof(eq + 1);
                        char* comma = strchr(eq, ',');
                        int wavelength = 0;
                        if (comma) {
                            wavelength = atoi(comma + 1);
                        }
                        ledCalibration[ledNum - 1].intensity = intensity;
                        ledCalibration[ledNum - 1].wavelength = wavelength;
                        
                        if (intensity > 0) {
                            Serial.print(F("  LED"));
                            Serial.print(ledNum);
                            Serial.print(F(": "));
                            Serial.print(intensity, 2);
                            Serial.print(F(" mW/cm² @ "));
                            Serial.print(wavelength);
                            Serial.println(F("nm"));
                        }
                    }
                }
            }
        }
        
        calFile.close();
        Serial.println(F("LED calibration loaded from SD card."));
    }
}

void loadConfigFromSD() {
    if (!sdCardAvailable) {
        return;
    }
    
    if (!SD.exists("config.txt")) {
        Serial.println(F("INFO: config.txt not found. Creating a new one with default values."));
        saveConfigToSD(); // Create the file with defaults if it doesn't exist
        return;
    }
    
    File configFile = SD.open("config.txt", FILE_READ);
    if (configFile) {
        // Loading configuration from config.txt");
        
        char line[32];  // Further reduced config line buffer
        while (configFile.available()) {
            int len = configFile.readBytesUntil('\n', line, sizeof(line) - 1);
            line[len] = '\0';
            
            // Trim whitespace
            while (len > 0 && (line[len-1] == ' ' || line[len-1] == '\r')) {
                line[--len] = '\0';
            }
            
            if (len == 0 || line[0] == '#') continue; // Skip empty lines and comments
            
            char* equalPos = strchr(line, '=');
            if (equalPos) {
                *equalPos = '\0';  // Split the string
                char* key = line;
                char* value = equalPos + 1;
                
                // Parse configuration values
                if (strcmp(key, "incubationSetpointTemp") == 0) {
                    incubationSetpointTemp = atof(value);
                    // Loaded temperature: "));
                    Serial.println(incubationSetpointTemp);
                }
                else if (strcmp(key, "OD940LEDBrightness") == 0) {
                    OD940LEDBrightness = atoi(value);
                    // Loaded OD LED brightness: "));
                    Serial.println(OD940LEDBrightness);
                }
                else if (strcmp(key, "motorStirringSpeed") == 0) {
                    motorStirringSpeed = atoi(value);
                    // Loaded stirring speed: "));
                    Serial.println(motorStirringSpeed);
                }
                else if (strcmp(key, "pumpOneSpeed") == 0) {
                    pumpOneSpeed = atoi(value);
                }
                else if (strcmp(key, "pumpTwoSpeed") == 0) {
                    pumpTwoSpeed = atoi(value);
                }
                else if (strcmp(key, "pumpThreeSpeed") == 0) {
                    pumpThreeSpeed = atoi(value);
                }
                else if (strcmp(key, "pumpFourSpeed") == 0) {
                    pumpFourSpeed = atoi(value);
                }
                else if (strcmp(key, "threshold") == 0) {
                    dynamicThreshold = atof(value);
                    // Loaded threshold: "));
                    Serial.println(dynamicThreshold);
                }
                // Note: odCalibSlope and odCalibIntercept are now calculated dynamically from calibration points
                else if (strcmp(key, "cyclesPerLEDChange") == 0) {
                    cyclesPerLEDChange = atoi(value);
                    // Loaded cycles per LED: "));
                    Serial.println(cyclesPerLEDChange);
                }
                else if (strcmp(key, "maxDispensations") == 0) {
                    max_dispensations = atoi(value);
                    // Loaded max dispensations: "));
                    Serial.println(max_dispensations);
                }
                else if (strcmp(key, "irLower") == 0) {
                    irLower = atoi(value);
                    Serial.print(F("DEBUG: Loaded irLower: "));
                    Serial.println(irLower);
                }
                else if (strcmp(key, "odLower") == 0) {
                    odLower = atof(value);
                    Serial.print(F("DEBUG: Loaded odLower: "));
                    Serial.println(odLower, 3);
                }
                else if (strcmp(key, "irMidLow") == 0) {
                    irMidLow = atoi(value);
                    Serial.print(F("DEBUG: Loaded irMidLow: "));
                    Serial.println(irMidLow);
                }
                else if (strcmp(key, "odMidLow") == 0) {
                    odMidLow = atof(value);
                    Serial.print(F("DEBUG: Loaded odMidLow: "));
                    Serial.println(odMidLow, 3);
                }
                else if (strcmp(key, "irMidHigh") == 0) {
                    irMidHigh = atoi(value);
                    Serial.print(F("DEBUG: Loaded irMidHigh: "));
                    Serial.println(irMidHigh);
                }
                else if (strcmp(key, "odMidHigh") == 0) {
                    odMidHigh = atof(value);
                    Serial.print(F("DEBUG: Loaded odMidHigh: "));
                    Serial.println(odMidHigh, 3);
                }
                // Legacy keys: only present in very old (pre-4-point) config.txt files.
                // No longer written by V3; still accepted on load and mapped onto
                // irMidHigh/odMidHigh so an ancient config can still be read.
                else if (strcmp(key, "irMid") == 0) {
                    irMidHigh = atoi(value);
                    Serial.print(F("DEBUG: Loaded irMid (legacy) -> irMidHigh: "));
                    Serial.println(irMidHigh);
                }
                else if (strcmp(key, "odMid") == 0) {
                    odMidHigh = atof(value);
                    Serial.print(F("DEBUG: Loaded odMid (legacy) -> odMidHigh: "));
                    Serial.println(odMidHigh, 3);
                }
                else if (strcmp(key, "irUpper") == 0) {
                    irUpper = atoi(value);
                    Serial.print(F("DEBUG: Loaded irUpper: "));
                    Serial.println(irUpper);
                }
                else if (strcmp(key, "odUpper") == 0) {
                    odUpper = atof(value);
                    Serial.print(F("DEBUG: Loaded odUpper: "));
                    Serial.println(odUpper, 1);
                }
                // NOTE: Legacy quadA/quadB/quadC/useQuadratic keys (if present in an
                // old config.txt) are intentionally ignored — quadratic calibration
                // was removed in firmware V2.
                // Inverse calibration (preferred method)
                else if (strcmp(key, "inverseA") == 0) {
                    inverseA = atof(value);
                    Serial.print(F("DEBUG: Loaded inverseA: "));
                    Serial.println(inverseA, 6);
                }
                else if (strcmp(key, "inverseB") == 0) {
                    inverseB = atof(value);
                    Serial.print(F("DEBUG: Loaded inverseB: "));
                    Serial.println(inverseB, 6);
                }
                else if (strcmp(key, "useInverse") == 0) {
                    useInverse = (strcmp(value, "true") == 0);
                    Serial.print(F("DEBUG: Loaded useInverse: "));
                    Serial.println(useInverse ? "true" : "false");
                }
            }
        }
        
        // Mark calibration as set if we have valid calibration points
        // Check that at least the primary points (lower and upper) have non-zero values
        if (irLower > 0 && irUpper > 0) {
            calibrationSet = true;
            Serial.println(F("DEBUG: Calibration marked as set from SD config"));
            Serial.print(F("DEBUG: irLower="));
            Serial.print(irLower);
            Serial.print(F(", odLower="));
            Serial.print(odLower, 3);
            Serial.print(F(", irUpper="));
            Serial.print(irUpper);
            Serial.print(F(", odUpper="));
            Serial.println(odUpper, 3);
        } else {
            Serial.print(F("WARNING: Calibration NOT set - irLower="));
            Serial.print(irLower);
            Serial.print(F(", irUpper="));
            Serial.println(irUpper);
        }
        
        configFile.close();
        // Configuration loaded from SD card");
    } else {
        // Failed to open config.txt");
    }
}

void loadCyclerFromSD() {
    Serial.println(F("=== loadCyclerFromSD() called ==="));
    if (!sdCardAvailable) {
        Serial.println(F("SD card not available for cycler load"));
        return;
    }
    
    Serial.println(F("SD card available, checking for cycler.csv"));
    if (!SD.exists("cycler.csv")) {
        Serial.println(F("INFO: cycler.csv not found. Creating a new one with default program."));
        saveCyclerToSD(); // Create the file with the default program if it doesn't exist
        return;
    }
    
    Serial.println(F("cycler.csv exists, attempting to open"));
    
    File cyclerFile = SD.open("cycler.csv", FILE_READ);
    if (cyclerFile) {
        Serial.println(F("Loading cycle program from cycler.csv"));
        
        // Skip header line
        char header[40];
        if (cyclerFile.available()) {
            int len = cyclerFile.readBytesUntil('\n', header, sizeof(header) - 1);
            header[len] = '\0';
            Serial.print(F("Cycler header: "));
            Serial.println(header);
        }
        
        int stepCount = 0;
        char line[48];  // Reduced buffer for cycler lines
        while (cyclerFile.available() && stepCount < 15) { // Max 15 steps
            int len = cyclerFile.readBytesUntil('\n', line, sizeof(line) - 1);
            line[len] = '\0';
            
            // Trim whitespace
            while (len > 0 && (line[len-1] == ' ' || line[len-1] == '\r')) {
                line[--len] = '\0';
            }
            
            if (len == 0) continue; // Skip empty lines
            
            // Parse CSV line: stepNumber,mediaType,stimulationLEDNumber,stimulationLEDintensity,cyclesPerLEDChange
            char* commas[5];
            commas[0] = strchr(line, ',');
            for (int i = 1; i < 5 && commas[i-1]; i++) {
                commas[i] = strchr(commas[i-1] + 1, ',');
            }
            
            if (commas[0] && commas[1] && commas[2] && commas[3]) {
                int stepNum = atoi(line);  // First field before first comma
                
                // Extract media type
                char mediaType[9];
                int mediaLen = commas[1] - commas[0] - 1;
                if (mediaLen > 8) mediaLen = 8;
                strncpy(mediaType, commas[0] + 1, mediaLen);
                mediaType[mediaLen] = '\0';
                
                int ledNumber = atoi(commas[1] + 1);
                int ledIntensity = atoi(commas[2] + 1);
                int cyclesPerLED = atoi(commas[3] + 1);
                
                // Convert LED intensity from 0-255 to PWM 0-4095
                int ledBrightness = map(ledIntensity, 0, 255, 0, 4095);
                
                // Store in step program (stepNum is 1-based, array is 0-based)
                if (stepNum >= 1 && stepNum <= 15) {
                    strcpy(stepProgram[stepNum - 1].mediaType, mediaType);
                    stepProgram[stepNum - 1].ledBrightness = ledBrightness;
                    stepProgram[stepNum - 1].temperature = 30.0; // Default temperature
                    
                    cyclesPerLEDChange = cyclesPerLED; // Update global setting
                    stepCount++;
                    
                    Serial.print(F("Loaded step "));
                    Serial.print(stepNum);
                    Serial.print(F(": "));
                    Serial.print(mediaType);
                    Serial.print(F(", LED: "));
                    Serial.print(ledBrightness);
                    Serial.println(F(" PWM"));
                }
            }
        }
        
        if (stepCount > 0) {
            totalProgramSteps = stepCount;
            Serial.print(F("Loaded "));
            Serial.print(totalProgramSteps);
            Serial.print(F(" steps with "));
            Serial.print(cyclesPerLEDChange);
            Serial.println(F(" cycles per LED change"));
        } else {
            Serial.println(F("No valid steps loaded from cycler.csv"));
        }
        
        cyclerFile.close();
    } else {
        Serial.println(F("Failed to open cycler.csv"));
    }
}

void loadLastExperimentState() {
    Serial.println(F("=== loadLastExperimentState() called ==="));
    Serial.print(F("SD card available: "));
    Serial.println(sdCardAvailable ? F("YES") : F("NO"));
    if (!sdCardAvailable) {
        Serial.println(F("SD card not available for state load"));
        return;
    }
    
    if (!SD.exists("OEVO.csv")) {
        Serial.println(F("OEVO.csv not found, keeping default cycle/step"));
        return;
    }
    
    Serial.println(F("OEVO.csv exists, attempting to open..."));
    File dataFile = SD.open("OEVO.csv", FILE_READ);
    if (!dataFile) {
        Serial.println(F("Failed to open OEVO.csv for state load"));
        return;
    }
    
    Serial.println(F("OEVO.csv opened successfully, reading last valid data line..."));
    
    // Read from the end to robustly get the last complete data line (skip headers/partials)
    char lastLine[96] = "";
    char line[96];
    long fileSize = dataFile.size();
    long startPos = max(0L, fileSize - 2048L); // read last ~2KB
    dataFile.seek(startPos);
    
    // Skip a possible partial first line if not at the beginning
    if (startPos > 0) {
        char temp[80];
        dataFile.readBytesUntil('\n', temp, sizeof(temp) - 1);
    }
    
    while (dataFile.available()) {
        int len = dataFile.readBytesUntil('\n', line, sizeof(line) - 1);
        line[len] = '\0';
        
        // Trim whitespace
        while (len > 0 && (line[len-1] == ' ' || line[len-1] == '\r')) {
            line[--len] = '\0';
        }
        
        if (len <= 0) continue;
        
        // Count commas to ensure full format (10 commas for 11-field, 11 commas for 12-field with Dilution_Event)
        int commaCount = 0;
        for (int i = 0; line[i] != '\0'; i++) {
            if (line[i] == ',') commaCount++;
        }
        
        // Skip header-like lines and only accept full data lines (backward compatible: 10 or 11 commas)
        if (commaCount >= 10 && strncmp(line, "unixTime", 8) != 0) {
            strcpy(lastLine, line);
        }
    }
    dataFile.close();
    
    Serial.print(F("Last line length: "));
    Serial.println(strlen(lastLine));
    if (strlen(lastLine) > 0) {
        Serial.print(F("Last line content: "));
        Serial.println(lastLine);
        
        // Parse the CSV line to extract currentCycle and currentStep
        // Handle both 11-field (old) and 12-field (new with Dilution_Event) formats
        // 11-field: unixTime,upTime,OD940,infraredReading,ambientTemp,mediaTemp,heaterPlateTemp,totalCycleCount,currentCycle,currentStep,mediaType
        // 12-field: same as above + Dilution_Event
        char* commas[12];
        int commaCount = 0;
        commas[0] = strchr(lastLine, ',');
        if (commas[0]) commaCount = 1;
        
        for (int i = 1; i < 12 && commas[i-1]; i++) {
            commas[i] = strchr(commas[i-1] + 1, ',');
            if (commas[i]) commaCount++;
        }
        
        Serial.print(F("Found "));
        Serial.print(commaCount);
        Serial.println(F(" commas"));
        
        int cycleVal = 0, stepVal = 0;
        
        if (commaCount >= 10) { // Backward compatible: handles both 11-field (10 commas) and 12-field (11 commas)
            // commas[0] points to separator after field 1 (unixTime)
            // currentCycle (field 9) starts after commas[7]
            // currentStep  (field 10) starts after commas[8]
            cycleVal = atoi(commas[7] + 1);
            stepVal = atoi(commas[8] + 1);
            Serial.print(F("Parsed cycle: "));
            Serial.print(cycleVal);
            Serial.print(F(", step: "));
            Serial.println(stepVal);
        } else {
            Serial.println(F("Not enough commas for valid format"));
        }
        
        if (cycleVal > 0 && stepVal > 0) {
            // Validate step is within bounds of loaded program
            if (stepVal <= totalProgramSteps) {
                currentCycle = cycleVal;
                currentStep = stepVal;
                Serial.print(F("Restored state - Cycle: "));
                Serial.print(currentCycle);
                Serial.print(F(", Step: "));
                Serial.print(currentStep);
                Serial.print(F("/"));
                Serial.println(totalProgramSteps);
            } else {
                Serial.print(F("Invalid step "));
                Serial.print(stepVal);
                Serial.print(F(" > totalProgramSteps "));
                Serial.print(totalProgramSteps);
                Serial.println(F(", resetting to step 1"));
                currentCycle = cycleVal;
                currentStep = 1; // Reset to valid step
            }
        } else {
            Serial.println(F("Invalid cycle/step values (both must be > 0)"));
        }
    } else {
        Serial.println(F("No valid data line found in OEVO.csv"));
    }
    Serial.println(F("=== loadLastExperimentState() complete ==="));
}

void checkSDCardStatus() {
    // Try to access SD card root directory
    File root = SD.open("/");
    if (root) {
        // SD card is accessible
        int freeSpaceMB = 1000; // Placeholder - actual free space detection is complex
        
        if (!sdCardAvailable) {
            // SD card just recovered!
            sdCardAvailable = true;
            Serial.println(F("✅ SD CARD RECOVERED"));
            Serial.println(F("$DEBUG,SD_CARD_RECOVERED"));
        }
        
        Serial.print("$SD_STATUS,OK,");
        Serial.println(freeSpaceMB);
        root.close();
    } else {
        // SD card is not accessible
        if (sdCardAvailable) {
            // SD card just failed
            sdCardAvailable = false;
            Serial.println(F("❌ SD CARD LOST"));
            Serial.println(F("$DEBUG,SD_CARD_LOST"));
            displaySDError("Status Check");
        }
        Serial.println("$SD_STATUS,ERROR,0");
    }
}

void testSDCardConnection() {
    Serial.println(F("$SD_TEST,STARTING"));
    
    // Test 1: Basic SD.begin()
    Serial.print(F("$SD_TEST,INIT,"));
    if (SD.begin(sdCardCSPin)) {
        Serial.println(F("PASS"));
        sdCardAvailable = true;
    } else {
        Serial.println(F("FAIL"));
        sdCardAvailable = false;
        Serial.println(F("$SD_TEST,COMPLETE,FAIL"));
        return;
    }
    
    // Test 2: Open root directory
    Serial.print(F("$SD_TEST,ROOT,"));
    File root = SD.open("/");
    if (root) {
        Serial.println(F("PASS"));
        root.close();
    } else {
        Serial.println(F("FAIL"));
        Serial.println(F("$SD_TEST,COMPLETE,FAIL"));
        return;
    }
    
    // Test 3: Create test file
    Serial.print(F("$SD_TEST,WRITE,"));
    File testFile = SD.open("test.txt", FILE_WRITE);
    if (testFile) {
        testFile.println(F("SD test file"));
        testFile.close();
        Serial.println(F("PASS"));
    } else {
        Serial.println(F("FAIL"));
        Serial.println(F("$SD_TEST,COMPLETE,FAIL"));
        return;
    }
    
    // Test 4: Read test file
    Serial.print(F("$SD_TEST,READ,"));
    testFile = SD.open("test.txt");
    if (testFile && testFile.available()) {
        String content = testFile.readString();
        testFile.close();
        if (content.indexOf("SD test file") >= 0) {
            Serial.println(F("PASS"));
        } else {
            Serial.println(F("FAIL"));
            Serial.println(F("$SD_TEST,COMPLETE,FAIL"));
            return;
        }
    } else {
        Serial.println(F("FAIL"));
        if (testFile) testFile.close();
        Serial.println(F("$SD_TEST,COMPLETE,FAIL"));
        return;
    }
    
    // Test 5: Delete test file
    Serial.print(F("$SD_TEST,DELETE,"));
    if (SD.remove("test.txt")) {
        Serial.println(F("PASS"));
    } else {
        Serial.println(F("FAIL"));
        Serial.println(F("$SD_TEST,COMPLETE,FAIL"));
        return;
    }
    
    // Test 6: Check if OEVO.csv exists
    Serial.print(F("$SD_TEST,OEVO_CSV,"));
    if (SD.exists("OEVO.csv")) {
        Serial.println(F("EXISTS"));
    } else {
        Serial.println(F("NOT_FOUND"));
    }
    
    Serial.println(F("$SD_TEST,COMPLETE,PASS"));
}

void listSDCardFiles() {
    Serial.println(F("$SD_FILES,STARTING"));
    
    if (!SD.begin(sdCardCSPin)) {
        Serial.println(F("$SD_FILES,ERROR,SD card not available"));
        return;
    }
    
    File root = SD.open("/");
    if (!root) {
        Serial.println(F("$SD_FILES,ERROR,Cannot open root directory"));
        return;
    }
    
    int fileCount = 0;
    while (true) {
        File entry = root.openNextFile();
        if (!entry) {
            break; // No more files
        }
        
        Serial.print(F("$SD_FILES,FILE,"));
        Serial.print(entry.name());
        Serial.print(F(","));
        Serial.print(entry.size());
        Serial.print(F(","));
        if (entry.isDirectory()) {
            Serial.println(F("DIR"));
        } else {
            Serial.println(F("FILE"));
        }
        
        entry.close();
        fileCount++;
        
        // Prevent infinite loops
        if (fileCount > 50) {
            Serial.println(F("$SD_FILES,WARNING,Too many files, stopping at 50"));
            break;
        }
    }
    
    root.close();
    
    Serial.print(F("$SD_FILES,COMPLETE,"));
    Serial.println(fileCount);
}

void getLastSDLine() {
    // === getLastSDLine() called ==="));
    // Simple function to read the last line from OEVO.csv (not header)
    if (!SD.begin(sdCardCSPin)) {
        Serial.println("$LAST_LINE,ERROR,SD card not available");
        return;
    }
    
    File dataFile = SD.open("OEVO.csv", FILE_READ);
    if (!dataFile) {
        Serial.println("$LAST_LINE,ERROR,No data file found");
        return;
    }
    
    long fileSize = dataFile.size();
    if (fileSize == 0) {
        dataFile.close();
        Serial.println("$LAST_LINE,ERROR,Data file is empty");
        return;
    }

    // Read from end to find last valid data line
    char lastLine[60] = "";
    char line[60];
    dataFile.seek(0);
    
    while (dataFile.available()) {
        int len = dataFile.readBytesUntil('\n', line, sizeof(line) - 1);
        line[len] = '\0';
        
        // Trim whitespace
        while (len > 0 && (line[len-1] == ' ' || line[len-1] == '\r')) {
            line[--len] = '\0';
        }
        
        // Skip header and empty lines
        if (len > 0 && strncmp(line, "unixTime", 8) != 0) {
            strcpy(lastLine, line);
        }
    }
    dataFile.close();
    
    if (strlen(lastLine) > 0) {
        // Last line found: "));
        Serial.println(lastLine);
        Serial.print("$LAST_LINE,OK,");
        Serial.println(lastLine);
    } else {
        Serial.println("$LAST_LINE,ERROR,No valid data lines found");
    }
}

void getRecentSDData(int numLines) {
    // Simple function to read last ~20 lines for chart pre-population
    // Limit to max 25 lines to avoid memory issues
    if (numLines > 25) numLines = 25;
    
    if (!SD.begin(sdCardCSPin)) {
        Serial.println("$SD_DATA,ERROR,SD card not available");
        return;
    }
    
    File dataFile = SD.open("OEVO.csv", FILE_READ);
    if (!dataFile) {
        Serial.println("$SD_DATA,ERROR,No data file found");
        return;
    }
    
    // Simple approach: read from end of file backwards
    long fileSize = dataFile.size();
    if (fileSize == 0) {
        dataFile.close();
        Serial.println("$SD_DATA,ERROR,Data file is empty");
        return;
    }
    
    // Start from near the end and work backwards to find lines
    long startPos = max(0L, fileSize - 2000L); // Read last ~2KB
    dataFile.seek(startPos);
    
    // Skip partial first line if we're not at the beginning
    if (startPos > 0) {
        char temp[80];
        dataFile.readBytesUntil('\n', temp, sizeof(temp) - 1);
    }
    
    // Read remaining lines into a small buffer
    char lines[5][60]; // Further reduced array size to save memory
    int lineCount = 0;
    
    while (dataFile.available() && lineCount < 5) {
        int len = dataFile.readBytesUntil('\n', lines[lineCount], 59);
        lines[lineCount][len] = '\0';
        
        // Trim whitespace
        while (len > 0 && (lines[lineCount][len-1] == ' ' || lines[lineCount][len-1] == '\r')) {
            lines[lineCount][--len] = '\0';
        }
        
        if (len > 0 && strncmp(lines[lineCount], "unixTime", 8) != 0) {
            lineCount++;
        }
    }
    dataFile.close();
    
    // Send the last numLines (or all if fewer)
    int startIdx = max(0, lineCount - numLines);
    for (int i = startIdx; i < lineCount; i++) {
        Serial.print("$SD_DATA,OK,");
        Serial.println(lines[i]);
        delay(10); // Small delay
    }
    
    Serial.println("$SD_DATA,END");
}

void initializeDefaultProgram() {
    // Initialize default program settings
    totalProgramSteps = 3; // <-- Edit number of default steps here
    cyclesPerLEDChange = 1; // <-- Edit default cycles per LED change here
    
    // Step 1: NEUTRAL
    strcpy(stepProgram[0].mediaType, "NEUTRAL"); // <-- Edit step 1 media type here
    stepProgram[0].ledBrightness = 0; // <-- Edit step 1 LED brightness here (0-4095)
    stepProgram[0].temperature = 30.0; // <-- Edit step 1 temperature here
    
    // Step 2: POSITIVE  
    strcpy(stepProgram[1].mediaType, "POSITIVE"); // <-- Edit step 2 media type here
    stepProgram[1].ledBrightness = 0; // <-- Edit step 2 LED brightness here (1% of 4095)
    stepProgram[1].temperature = 30.0; // <-- Edit step 2 temperature here
    
    // Step 3: NEGATIVE
    strcpy(stepProgram[2].mediaType, "NEGATIVE"); // <-- Edit step 3 media type here
    stepProgram[2].ledBrightness = 0; // <-- Edit step 3 LED brightness here (0-4095)
    stepProgram[2].temperature = 30.0; // <-- Edit step 3 temperature here
    
    // Default program initialized"));
}

void loadConfiguration() {
    // Load configuration from SD card
    loadConfigFromSD();
    
    // Load cycle program from SD card
    loadCyclerFromSD();
    
    // Load LED calibration from SD card (user-measured intensity/wavelength)
    loadLEDCalibrationFromSD();
    
    // Load last experiment state if available
    loadLastExperimentState();
    
    // Initialize media change cooldown to prevent immediate media change on startup
    lastMediaChangeTime = millis();
    Serial.println(F("DEBUG: Initialized lastMediaChangeTime to prevent startup emptying"));
    
    Serial.println(F("=== OpenEvo FIRMWARE STARTUP COMPLETE ==="));
    Serial.println(F("Ready to receive commands..."));
    
    // Automatically broadcast SD card status at startup
    checkSDCardStatus();
}

void displaySDError(const char* operation) {
    // Set flag to keep this message on screen
    showingSDError = true;
    
    // Display SD card error on LCD with helpful instructions
    setMultiplexerFocus(0);
    lcd.clear();
    lcd.setCursor(0, 0);
    
    // Check if this is an unpause failure - show detailed instructions
    if (strcmp(operation, "Unpause Failed") == 0) {
        lcd.print(F("UNPAUSE FAILED!"));
        lcd.setCursor(0, 1);
        lcd.print(F("Insert SD card"));
        lcd.setCursor(0, 2);
        lcd.print(F("Wait 5 seconds"));
        lcd.setCursor(0, 3);
        lcd.print(F("Then try again"));
        delay(50);  // Wait for I2C LCD transaction to complete
        
        // Send LCD content to interface
        Serial.println(F("$LCD_CONTENT,UNPAUSE FAILED!|Insert SD card|Wait 5 seconds|Then try again"));
    } else {
        // Generic SD error display
        lcd.print(F("SD Card Error!"));
        lcd.setCursor(0, 1);
        lcd.print(F("Insert SD CARD"));
        lcd.setCursor(0, 2);
        lcd.print(operation);
        delay(50);  // Wait for I2C LCD transaction to complete
        
        // Send LCD content to interface
        Serial.print(F("$LCD_CONTENT,"));
        Serial.print(F("SD Card Error!|Insert SD CARD|"));
        Serial.println(operation);
    }
    
    // Also send debug message
    Serial.print(F("$DEBUG,SD_ERROR,"));
    Serial.println(operation);
}
