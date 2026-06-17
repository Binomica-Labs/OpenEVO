// *******************************************************************************************************************************************
// easyEVO Type D Directed Evolution Engine
// Version: 1.5.0.0
// Copyright 2024 Binomica Labs
// Author: Sebastian S. Cocioba
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
//
// *******************************************************************************************************************************************



// ---------------------------------------***-PINS-***----------------------------------------------------------------------------------------
//
//
//
//  Heater Controller
//      ENA   44
//      IN1   42
//      IN2   40
//      IN3   38
//      IN4   36
//      ENB   34
//
//  Stirring Controller
//      IN1   48
//      IN2   46
//
//  Heater Thermistor
//            A9
//
//  Media Thermistor
//            A8
//
//  SD Card Reader Module:
//      MOSI  51
//      MISO  50
//      SCK   52
//      CS    53
//
//  Adafruit 24-Channel LED Driver
//      940-945nm LED 0   //OD940 sensor light
//      660-665nm LED 1   
//      720-725nm LED 2   
//      760-765nm LED 3   
//      780-785nm LED 4   
//      800-805nm LED 5   
//      820-825nm LED 6
//
//  Interface Buttons:
//      UP   26
//      SEL  24
//      DWN  22
//
//  I2C Bus Rail
//      SDA   20
//      SCL   21
//
//  I2C Multiplexer Connections
//      LCD Screen        SDA0/SCL0   setMultiplexerFocus(0) //function call to listen to specific I2C pin on multiplexer, call before use
//      Real-Time Clock   SDA1/SCL1   setMultiplexerFocus(1)
//      TSL2591 Sensor    SDA2/SCL2   setMultiplexerFocus(2)
// -------------------------------------------------------------------------------------------------------------------------------------------



// LIBRARY STUFF
#include <PID_v1.h> //PID by Brett Beauregard
#include <math.h>
#include <RTClib.h> //RTClib by Adafruit
#include <SPI.h>    // SPI library for SD card module
#include <SD.h>     // SD card library
#include <Wire.h>
#include <LiquidCrystal_I2C.h> //LiquidCrystalI2C by Frank de Brabander
#include <Adafruit_Sensor.h>  //Adafruit Unified Sensor Library
#include "Adafruit_TSL2591.h" //install all dependencies including Adafruit Unified Sensor Library
#include <StateMachine.h> //StateMachine by Jose Jullan + LinkedList dependency
#include "Arduino.h"
#include "avdweb_Switch.h" //Switch by Albert van Dalen
#include "Adafruit_TLC5947.h" //Adafruit TLC5947 by Adafruit
#include <NTC_Thermistor.h> //***INSTALL VERSION 2.0.3 ONLY; library has a bug!!!*** NTC_Thermistor by Yurii Salimov
#include <SmoothThermistor.h> //SmoothThermistor by Gianni Van Hoecke
#include <Adafruit_MotorShield.h> //Adafruit Motor Shield V2 Library + ALL Dependencies



// CRITICAL SETTINGS
const String softwareVersion = "v1.6.0.0";
const String csvFileHeader =   "unixTime,"
                          "upTime,"
                          "currentProgramTime,"
                          "ambientTemp,mediaTemp,"
                          "heaterPlateTemp,"
                          "heaterPWM,"
                          "growthDuration,"
                          "growthDurationChange,"
                          "neutralCycleCount,"
                          "positiveCycleCount,"
                          "negativeCycleCount,"
                          "totalCycleCount,"
                          "fullSpectrumReading,"
                          "visibleSpectrumReading,"
                          "infraredReading,"
                          "OD940\n";

//the temperature you wish your culture vessel to hold in degrees Celsius.
float incubationSetpointTemp = 30.00;

//keep constant to ensure consistent stirring speed
int motorStirringSpeed = 60;

//initial brightness of the 940nm Optical Density LED; adjust to reduce saturation
int OD940LEDBrightness = 2100;

//motor turning speed, calibrate your motors to determine these values below
int pumpOneSpeed = 1080;

int pumpTwoSpeed = 1140;

int pumpThreeSpeed = 1140;

int pumpFourSpeed = 1110;



//how many media refresh cycles for neutral media in Light Cycler mode
int neutralCycleMax = 1;

//how many media refresh cycles for positive selection media in Light Cycler mode
int positiveCycleMax = 1;

//how many media refresh cycles for negative selection media in Light Cycler mode
int negativeCycleMax = 1;

//how many media refresh cycles for the second neutral selection media (same bottle; for cell recovery) in Light Cycler mode
int neutralCycleTwoMax = 1;

//1Hz master loop interval in milliseconds
const int interval = 1000;

//0.1Hz turbidostat loop interval (adjusted for time spent dispensing)
const int intervalTurbidostat = 1000;

//0.1Hz LightCycler loop interval (adjusted for time spent dispensing)
const int intervalLightcycler = 1000;



// STATE STUFF
bool flagGoToStandby = false;
bool flagGoToMenu = false;
bool flagGoToTurbidostat = false;
bool flagGoToIncubate = false;
bool flagGoToHeaterTest = false;
bool flagGoToPrimePumps = false;
bool flagGoToCalibrateStirring = false;
bool flagGoToCalibrateOptics = false;
bool flagGoToCalibratePumpOne = false;
bool flagGoToCalibratePumpTwo = false;
bool flagGoToCalibratePumpThree = false;
bool flagGoToCalibratePumpFour = false;
bool displayStatPage = false;



// CRITICAL VARIABLES
unsigned long currentUnixTime = 0;
float currentOD940 = 0.0000;
unsigned long robotStartTime = 0;
unsigned long programStartTime = 0;
unsigned long currentProgramTime = 0;
unsigned long upTime = 0;
double mediaTemp = 0.00;                        //temp inside culture vessel; the media itself
double heaterPlateTemp = 0.00;                  //temp on surface of heater plate
double ambientTemp = 0.00;                      //ambient room temp as measured by the real-time clock module



int growthStartTime = 0;
int growthStopTime = 0;
int growthDuration = 0;
int growthDurationChange = 0;
int previousGrowthDuration = 0;

int cycleDuration = 0;

int totalCycleCount = 0;

int neutralCycleCount = 0;
int positiveCycleCount = 0;
int negativeCycleCount = 0;

String mediaCycleArray[20][9];    //limit the total size to less than 50 entries or memory will be full. Rely on patterns.
int currentMediaDispenseCount = 0;
int currentMediaPumpNumber = 1;
int wastePumpNumber = 4;

int currentStepNumber = 0;
String currentMediaType = "";
double currentMediaTempSetpoint = 0.00;
int currentStimulationLEDNumber = 0;
int currentStimulationLEDIntensity = 0;
float currentOpticalDensityFloor = 0.000;
float currentOpticalDensityCeiling = 2.000; //
int currentStirringSpeed = 0;
int currentCycleTimeoutDuration = 0;
int currentCycleStep = 1;
int totalProgramCycles = 0;


//  PERISTALTIC PUMP CONTROLLER STUFF (DC Motor Shield)
int pumpSelection = 1;                                //an incremented variable to easily select which pump in a for-loop during priming (1 to 4)
Adafruit_MotorShield MotorShield = Adafruit_MotorShield();
Adafruit_DCMotor *pumpOne = MotorShield.getMotor(1);
Adafruit_DCMotor *pumpTwo = MotorShield.getMotor(2);
Adafruit_DCMotor *pumpThree = MotorShield.getMotor(3);
Adafruit_DCMotor *pumpFour = MotorShield.getMotor(4);



//  STIRRING CONTROLLER STUFF
const int motorStirringPinInputOne = 48;
const int motorStirringPinInputTwo = 46;



// HEATER CONTROLLER STUFF
const int heaterPlateTopPinEnable = 44;       // Top Heater Plate PWM Pin
const int heaterPlateTopPinInputOne = 42;
const int heaterPlateTopPinInputTwo = 40;
const int heaterPlateBottomPinEnable = 34;    // Bottom Heater Plate PWM Pin
const int heaterPlateBottomPinInputOne = 38;
const int heaterPlateBottomPinInputTwo = 36;



// THERMOMETER STUFF
//makes a null object of a thermistor to pass original thermistor into smoothing function
Thermistor* mediaTempThermistorSmooth = NULL;
Thermistor* heaterPlateTempThermistorSmooth = NULL;
const int thermistorMediaPin = A8;                    //arduino analog pin 8 for media thermistor data line
const int thermistorHeaterPlatePin = A9;              //arduino analog pin 9 for media thermistor data line
const int thermistorReferenceResistance = 10000;      //resistance value of Vishay PTF5610K000BZEK precision resistor
const int thermistorNominalResistance = 10000;        //rated thermistor resistance
const int thermistorNominalTemp = 25;                 //temp curves were calibrated for at thermistor factory
const int thermistorBValue = 3380;                    //taken from Murata NXFT15XH103FA1B150 NTC thermistor datasheet
const int thermistorSmoothingFactor = 5;              //how many samples to take and average



// PID STUFF
const double setpointOffset = 0.25; //added an extra 0.25 so average is more centered around 30C since gap trigger is setPointTemp - 0.5
double outputPWM = 0.00;
const double aggKp = 4, aggKi = 0.2, aggKd = 1;        //Define the aggressive and conservative Tuning Parameters
const double consKp = 1, consKi = 0.05, consKd = 0.25;
const double gapThreshold = 0.5;
//************SAFETY VARIABLE********************
const float pwmSafetyLimit = 130.00;    //DO **NOT** LET HEATER EXCEED GLASS TRANSITION POINT OF PRINTED MATERIAL (~106C)!!!
//************************************************/
PID mediaTempPID(&mediaTemp, &outputPWM, &currentMediaTempSetpoint, consKp, consKi, consKd, DIRECT);   //Specify the links and initial tuning parameters



// BUTTON STUFF
const int btnPinUp = 26;
const int btnPinSelect = 24;
const int btnPinDown = 22;
Switch BtnUp = Switch(btnPinUp);    //debounced switch objects
Switch BtnSelect = Switch(btnPinSelect);
Switch BtnDown = Switch(btnPinDown);



// LIGHT SENSOR STUFF
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
uint32_t lum;
uint16_t ir, full;
unsigned long IR;
unsigned long IRtotal;
float IRavg;
unsigned long IRblank;
unsigned long VIS;
unsigned long FULL;



// LED DRIVER STUFF
const int ledDriverPinData = 25;
const int ledDriverPinClock = 27;
const int ledDriverPinLatch = 23;
const int ledDriverPinEnable = -1;  //set to -1 to not use enable pin (optional)
Adafruit_TLC5947 ledDriver = Adafruit_TLC5947(1, ledDriverPinClock, ledDriverPinData, ledDriverPinLatch);
   
const int totalStimulationLEDCount = 6; //adjust according to how many stimulation LEDs you have; for-looped during standby state

// REAL-TIME CLOCK STUFF
RTC_DS3231 rtc;                           // defines real-time clock object
unsigned long previousMillis = 0;



// SD CARD STUFF
const int sdCardCSPin = 53;
File csvFile;
File configFile;
File cycleProgramFile;


// LCD SCREEN STUFF
LiquidCrystal_I2C lcd(0x27, 20, 4);     // set I2C address to 0x27 and a 20 chars x 4 line screen



StateMachine machine = StateMachine();
State* stateMenu = machine.addState(&stateFunctionMenu);
State* stateStandby = machine.addState(&stateFunctionStandby);
State* stateTurbidostat = machine.addState(&stateFunctionTurbidostat);
State* stateIncubate = machine.addState(&stateFunctionIncubate);
State* statePrimePumps = machine.addState(&stateFunctionPrimePumps);
State* stateHeaterTest = machine.addState(&stateFunctionHeaterTest);
State* stateCalibrateStirring = machine.addState(&stateFunctionCalibrateStirring);
State* stateCalibrateOptics = machine.addState(&stateFunctionCalibrateOptics);
State* stateCalibratePumpOne = machine.addState(&stateFunctionCalibratePumpOne);
State* stateCalibratePumpTwo = machine.addState(&stateFunctionCalibratePumpTwo);
State* stateCalibratePumpThree = machine.addState(&stateFunctionCalibratePumpThree);
State* stateCalibratePumpFour = machine.addState(&stateFunctionCalibratePumpFour);




//MENU STUFF
const String MenuItems[] =
{
  "TURBIDOSTAT",
  "INCUBATE",
  "PRIME PUMPS",
  "HEATER TEST",
  "CALIBRATE STIRRING",
  "CALIBRATE OPTICS",
  "CALIBRATE PUMP 1",
  "CALIBRATE PUMP 2",
  "CALIBRATE PUMP 3",
  "CALIBRATE PUMP 4"
};



// Select I2C BUS
void setMultiplexerFocus(uint8_t bus)
{
  Wire.beginTransmission(0x70);  // setMultiplexerFocus address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}


void menuFunctions(int menuItem, byte selectPressed, byte selectPressedLong)  // Your menu functions
{
  setMultiplexerFocus(0);
   if (menuItem == 1) // select TURBIDOSTAT mode
  {
    if (selectPressed == 1)
    {
      setMultiplexerFocus(0);
      lcd.clear();
      setMultiplexerFocus(1);
      DateTime now = rtc.now();
      programStartTime = now.unixtime();
      growthStartTime = now.unixtime() - programStartTime;  //start growth cycle timer
      machine.transitionTo(stateStandby);
    }
  }

  if (menuItem == 2) // select INCUBATE mode
  {
    if (selectPressed == 1)
    {
      setMultiplexerFocus(0);
      lcd.clear();
      setMultiplexerFocus(1);
      DateTime now = rtc.now();
      programStartTime = now.unixtime();
      growthStartTime = now.unixtime() - programStartTime;
      machine.transitionTo(stateIncubate);
    }
  }


  if (menuItem == 3) // select PRIME PUMPS mode
  {
    if (selectPressed == 1)
    {
      setMultiplexerFocus(0);
      lcd.clear();
      machine.transitionTo(statePrimePumps);
    }
  }


  if (menuItem == 4) // select HEATER TEST mode
  {
    if (selectPressed == 1)
    {
      setMultiplexerFocus(0);
      lcd.clear();
      machine.transitionTo(stateHeaterTest);
    }
  }

  if (menuItem == 5) // select CALIBRATE STIRRING mode
  {
    if (selectPressed == 1)
    {
      setMultiplexerFocus(0);
      lcd.clear();
      machine.transitionTo(stateCalibrateStirring);
    }
  }

    if (menuItem == 6) // select CALIBRATE OPTICS mode
  {
    if (selectPressed == 1)
    {
      setMultiplexerFocus(0);
      lcd.clear();
      machine.transitionTo(stateCalibrateOptics);
    }
  }

     if (menuItem == 7) // select CALIBRATE PUMP ONE mode
  {
    if (selectPressed == 1)
    {
      setMultiplexerFocus(0);
      lcd.clear();
      machine.transitionTo(stateCalibratePumpOne);
    }
  }

   if (menuItem == 8) // select CALIBRATE PUMP TWO mode
  {
    if (selectPressed == 1)
    {
      setMultiplexerFocus(0);
      lcd.clear();
      machine.transitionTo(stateCalibratePumpTwo);
    }
  }

   if (menuItem == 9) // select CALIBRATE PUMP THREE mode
  {
    if (selectPressed == 1)
    {
      setMultiplexerFocus(0);
      lcd.clear();
      machine.transitionTo(stateCalibratePumpThree);
    }
  }

   if (menuItem == 10) // select CALIBRATE PUMP FOUR mode
  {
    if (selectPressed == 1)
    {
      setMultiplexerFocus(0);
      lcd.clear();
      machine.transitionTo(stateCalibratePumpFour);
    }
  }
}


//MENU NAVIGATION LOGIC
template< typename T, size_t NumberOfSize >

size_t MenuItemsSize(T (&) [NumberOfSize])
{
  return NumberOfSize;
}

int numberOfMenuItems = MenuItemsSize(MenuItems) - 1;
int currentMenuItem = 0;
int previousMenuItem = 1;
byte button_flag = 0;



void setup()
{
  Serial.begin(115200);
  Wire.begin();
  BtnUp.doubleClickPeriod = 0;        //turn off double-click on all buttons
  BtnSelect.doubleClickPeriod = 0;
  BtnDown.doubleClickPeriod = 0;

  setMultiplexerFocus(0);
  //lcd.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();
  //poll the SD card and make sure it's working
  initializeSDCard();

  //read the configuration and calibration data from sdCard's config.txt file
  readConfigFile();
  readCycleProgramFile();

  //Serial.print("Start Pump Number: ");
  //Serial.println(currentMediaPumpNumber);
  //Serial.print("Start Cycle Step: ");
  //Serial.println(currentCycleStep); 
  //Serial.print("Start Temp Setpoint: ");
  //Serial.println(currentMediaTempSetpoint);

  //Initialize Stirrer Controller
  pinMode(motorStirringPinInputOne, OUTPUT);
  pinMode(motorStirringPinInputTwo, OUTPUT);

  //Initialize Heater Controller
  pinMode(heaterPlateTopPinEnable, OUTPUT); //top heater plate PWM
  pinMode(heaterPlateBottomPinEnable, OUTPUT); //bottom heater plate PWM
  pinMode(heaterPlateTopPinInputOne, OUTPUT);
  pinMode(heaterPlateTopPinInputTwo, OUTPUT);
  pinMode(heaterPlateBottomPinInputOne, OUTPUT);
  pinMode(heaterPlateBottomPinInputTwo, OUTPUT);

  //start heater off to ensure no accidental activation
  heaterOFF();

  //Initialize Thermometers
  Thermistor* mediaTempThermistor = new NTC_Thermistor(
    thermistorMediaPin,
    thermistorReferenceResistance,
    thermistorNominalResistance,
    thermistorNominalTemp,
    thermistorBValue
  );

  Thermistor* heaterPlateTempThermistor = new NTC_Thermistor(
    thermistorHeaterPlatePin,
    thermistorReferenceResistance,
    thermistorNominalResistance,
    thermistorNominalTemp,
    thermistorBValue
  );

  //apply smoothing to thermistor values using a smoothing object that takes the thermistor object as input
  mediaTempThermistorSmooth = new SmoothThermistor(mediaTempThermistor, thermistorSmoothingFactor);
  heaterPlateTempThermistorSmooth = new SmoothThermistor(heaterPlateTempThermistor, thermistorSmoothingFactor);

  //start PID temp control system
  mediaTempPID.SetMode(AUTOMATIC);

  //start the LED driver board
  ledDriver.begin();

  //turn on the OD940 LED to preheat, set the intensity (0 to 4096) to match TSL2591 maximum reading of 37888 or a tiny bit below. 
  //This ensure full ADC range when reading optical density.
  ledDriver.setPWM(0, OD940LEDBrightness);

  ledDriver.write();    //actually send the above values to the LED driver; do not forget to add this line or LED levels will not change!!!

  if (!MotorShield.begin(300))                                          // start motor shield at 300Hz instead of default 1.6KHz
  {
    setMultiplexerFocus(0);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MOTOR ERROR");
    while (1) delay(1000);
  }

  //set the I2C multiplexer focus to I2C port 1, the real-time clock line
  setMultiplexerFocus(1);
  if (!rtc.begin())
  {
    //set the I2C multiplexer focus to I2C port 0, the LCD screen
    setMultiplexerFocus(0);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Couldn't find clock!");
    while (1) delay(1000);
  }

  //write the gain and integration time parameters to the TSL2591 light sensor
  configureLightSensor();

  //set the I2C multiplexer focus to I2C port 0, the LCD screen
  setMultiplexerFocus(0);
  lcd.clear();

  //attach specific functions to each state transition event
  stateStandby->addTransition(&transitionStandbyToMenu, stateMenu); //go back to main menu

  stateStandby->addTransition(transitionStandbyToTurbidostat, stateTurbidostat);

  stateHeaterTest->addTransition(transitionHeaterTestToMenu, stateMenu);

  stateIncubate->addTransition(transitionIncubateToMenu, stateMenu);

  statePrimePumps->addTransition(transitionPrimePumpsToMenu, stateMenu);

  stateTurbidostat->addTransition(transitionTurbidostatToMenu, stateMenu);

  stateTurbidostat->addTransition(transitionTurbidostatToStandby, stateStandby);

  stateCalibrateStirring->addTransition(transitionCalibrateStirringToMenu, stateMenu);

  stateCalibrateOptics->addTransition(transitionCalibrateOpticsToMenu, stateMenu);

  stateCalibratePumpOne->addTransition(transitionCalibratePumpOneToMenu, stateMenu);

  stateCalibratePumpTwo->addTransition(transitionCalibratePumpTwoToMenu, stateMenu);

  stateCalibratePumpThree->addTransition(transitionCalibratePumpThreeToMenu, stateMenu);

  stateCalibratePumpFour->addTransition(transitionCalibratePumpFourToMenu, stateMenu);

  //set the I2C multiplexer focus to I2C port 1, the real-time clock line
  setMultiplexerFocus(1);
  if (rtc.lostPower())
  {
    // this will adjust to the date and time at compilation
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  //check the time on the real-time clock
  DateTime now = rtc.now();

  //SUPER IMPORTANT VARIABLE; MASTER TIME REFERENCE
  robotStartTime = now.unixtime();
}



void loop()
{
  //poll each of the buttons for any user input
  BtnUp.poll();
  BtnSelect.poll();
  BtnDown.poll();

  //activate the statemachine
  machine.run();
}



//=======================================
void stateFunctionMenu()
{
  setMultiplexerFocus(0);
  lcd.setCursor(0, 0);
  lcd.print("EasyEVO Type D");
  lcd.setCursor(0, 1);
  lcd.print(">");
  lcd.setCursor(1, 1);
  lcd.print(MenuItems [currentMenuItem]);

  if (BtnSelect.singleClick() == HIGH && button_flag == 0)
  {
    menuFunctions(currentMenuItem + 1, 1, 0);
    button_flag = 1;
    previousMillis = millis();
  }

  if (BtnSelect.longPress() == HIGH && button_flag == 0)
  {
    menuFunctions(currentMenuItem + 1, 0, 1);
    button_flag = 1;
    previousMillis = millis();
  }

  if (BtnDown.singleClick() == HIGH && button_flag == 0)
  {
    ++currentMenuItem;
    if (currentMenuItem > numberOfMenuItems )
    {
      currentMenuItem = numberOfMenuItems ;
    }
    button_flag = 1;
    previousMillis = millis();
  }

  else if (BtnUp.singleClick() == HIGH && button_flag == 0)
  {
    currentMenuItem--;
    if (currentMenuItem < 0)
    {
      currentMenuItem = 0;
    }
    button_flag = 1;
    previousMillis = millis();
  }

  if (currentMenuItem != previousMenuItem)
  {
    setMultiplexerFocus(0);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Main Menu 1/6");
    lcd.setCursor(0, 1);
    lcd.print(">");
    lcd.setCursor(1, 1);
    lcd.print(MenuItems [currentMenuItem]);
    menuFunctions(currentMenuItem + 1, 0, 0);
    previousMenuItem = currentMenuItem;
  }

  if (millis() - previousMillis >= 50)
  {
    previousMillis = millis();
    button_flag = 0;
  }
}


// the main active state of the easyEVO in TURBIDOSTAT mode. This records OD940 values and checks if the OD hit a user defined ceiling
void stateFunctionStandby()
{
  setMultiplexerFocus(0);
  stirringON();

 
  unsigned long currentMillis = millis();

  //ensure the bracketed function calls happen only once per user defined interval (usually one second)
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;

    
    //Serial.print("Stdby Pump Number: ");
    //Serial.println(currentMediaPumpNumber);

    //calculate the heater plate's current temperature and the PID controller's output to maintain user defined media temp
    heaterCompute(currentMediaTempSetpoint);

    //reset the transition flag to keep the easyEVO in standby mode until optical density trigger occurs
    flagGoToTurbidostat = false;

    //measure the optical density of the reaction tube
    calculateOD940();

    //check if optical density hit ceiling threshold, if so then transition into the turbidostat state function to dispense media
    if (currentOD940 >= currentOpticalDensityCeiling)
    {
      flagGoToTurbidostat = true;
    }

      ledDriver.setPWM(currentStimulationLEDNumber, currentStimulationLEDIntensity); 
      ledDriver.write();

    //display relevant information on the LCD screen, oscillating between two pages
    if (displayStatPage == true)
    {
       displayStats();
       displayStatPage = false;
    }
    else
    {
      displaySecondaryStats();
      displayStatPage = true;
    }
   

    //save light values
    writeToCard();
  }
}

//conditional which if true, executes a transition for standby mode to the main menu
bool transitionStandbyToMenu()
{
  if (BtnSelect.longPress() == HIGH)
  {
    //set I2C focus back to LCD screen and turn off all the motors, heater, and pumps
    setMultiplexerFocus(0);
    stirringOFF();
    heaterOFF();
    allPumpsStop();
    //set all stimulation LED lights to zero, save for the OD940 LED at position zero
    resetStimulationLights();
    currentCycleStep = 1;   //comment out to resume from last step instead of rebooting the cycler upon escape to menu
    growthStartTime = 0;
    growthStopTime = 0;
    programStartTime = 0;
    flagGoToStandby = false;
    lcd.clear();
    return true;
  }

  return false;
}



//TURBIDOSTAT STATE
void stateFunctionTurbidostat()
{
  setMultiplexerFocus(0);
  stirringON();
    
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= intervalTurbidostat)
  {
    previousMillis = currentMillis;
    heaterCompute(currentMediaTempSetpoint);

    //Serial.print("Pump Number: ");
    //Serial.println(currentMediaPumpNumber);
    dispenseMedia(currentMediaPumpNumber); 

    currentMediaDispenseCount++;
    float totalOD940 = 0.00;
    float averageOD940 = 0.00;
    for (int i = 0; i <= 9; i++)
    {
      calculateOD940();
      totalOD940 = totalOD940 + currentOD940;
    }

    averageOD940 = totalOD940 / 10;
    currentOD940 = averageOD940;      //THIS IS FOR A TSL2591 LIGHT SENSOR INTEGRATION TIME OF 100ms ONLY!

    removeMedia(wastePumpNumber); //typically pump 4 is dedicated waste line  

    totalOD940 = 0.00;
    averageOD940 = 0.00;
    for (int i = 0; i <= 9; i++)
    {
      calculateOD940();
      totalOD940 = totalOD940 + currentOD940;
    }

    averageOD940 = totalOD940 / 10;
    currentOD940 = averageOD940;      //THIS IS FOR A TSL2591 LIGHT SENSOR INTEGRATION TIME OF 100ms ONLY!

    //dilute to the next step's optical density floor value to allow full growth starting from desired density of next cycle
    int nextCycleStep = currentCycleStep + 1;
    currentOpticalDensityFloor = mediaCycleArray[nextCycleStep][5].toFloat();
    
    if (currentCycleStep == totalProgramCycles - 1)
    {
      //if current step is last step in cycle, set the optical density floor to the first step's value
      currentOpticalDensityFloor = mediaCycleArray[1][5].toFloat();
    }

    if (currentOD940 <= currentOpticalDensityFloor)
    {
      allPumpsStop();
      flagGoToStandby = true;
    }
    displayStats();
    writeToCard();
  }
}

bool transitionTurbidostatToMenu()
{
  if (BtnSelect.longPress() == HIGH)
  {
    setMultiplexerFocus(0);
    stirringOFF();
    heaterOFF();
    allPumpsStop();
    //set all stimulation LED lights to zero, save for the OD940 LED at position zero
    resetStimulationLights();
    currentCycleStep = 1; //reset the current step count when exiting to menu
    growthStartTime = 0;
    growthStopTime = 0;
    programStartTime = 0;
    lcd.clear();
    return true;
  }

  return false;
}

bool transitionTurbidostatToStandby()
{
  if (flagGoToStandby == true)
  {
    setMultiplexerFocus(1);

    DateTime now = rtc.now();

    totalCycleCount++;
    currentCycleStep++;

    
  if (currentCycleStep >= totalProgramCycles)
    {
      currentCycleStep = 1; //reset the current step count to loop the cycler program forever
    }

      currentStepNumber = mediaCycleArray[currentCycleStep][0].toInt();
      currentMediaType = mediaCycleArray[currentCycleStep][1];      //"NEUTRAL", "POSITIVE", or "NEGATIVE" (media pumps 1, 2, and 3 respectively)
      currentMediaTempSetpoint = mediaCycleArray[currentCycleStep][2].toDouble();
      currentStimulationLEDNumber = mediaCycleArray[currentCycleStep][3].toInt();
      currentStimulationLEDIntensity = mediaCycleArray[currentCycleStep][4].toInt();
      currentOpticalDensityFloor = mediaCycleArray[currentCycleStep][5].toFloat();
      currentOpticalDensityCeiling = mediaCycleArray[currentCycleStep][6].toFloat();
      currentStirringSpeed = mediaCycleArray[currentCycleStep][7].toInt();
      currentCycleTimeoutDuration = mediaCycleArray[currentCycleStep][8].toInt();
   

    if (currentMediaType == "NEUTRAL")
    {
      currentMediaPumpNumber = 1;
    }
    
    if (currentMediaType == "POSITIVE")
    {
      currentMediaPumpNumber = 2;
    }
    
    if (currentMediaType == "NEGATIVE")
    {
      currentMediaPumpNumber = 3;
    }

    growthStartTime = now.unixtime() - programStartTime;  //start timing new growth cycle

    //set I2C multiplexer to I2C channel zero (LCD screen);
    setMultiplexerFocus(0); 

    //shut off all the pumps, for safety
    allPumpsStop();

    //set all stimulation LED lights to zero, save for the OD940 LED at position zero
    resetStimulationLights();

    //reset the flag that triggers transition between standby and turbidostat states                                                           
    flagGoToTurbidostat = false;                                              
    return true;
  }

  return false;
}


//conditional which if true, executes a state transition from standby mode into turbidostat mode where media will be dispensed
bool transitionStandbyToTurbidostat()
{
  if (flagGoToTurbidostat == true)
  {
    //set I2C focus to real-time clock to measure cycle times
    setMultiplexerFocus(1);

    DateTime now = rtc.now();

    //log the end of the current growth cycle
    growthStopTime = now.unixtime() - programStartTime;

    //calculate the growth cycle time end to end
    growthDuration = growthStopTime - growthStartTime;

    //calculate the difference in time between last cycle and current cycle
    growthDurationChange = growthDuration - previousGrowthDuration;

    //set the current growth duration as the old one to make room for next cycle
    previousGrowthDuration = growthDuration;

    setMultiplexerFocus(0);
    allPumpsStop();
    flagGoToStandby = false;
    return true;
  }

  return false;
}



//INCUBATE STATE - grow a batch culture without using pumps. Useful for making starter cultures or characterizing microbial growth.
void stateFunctionIncubate()
{
  stirringON();
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval)
  {
    setMultiplexerFocus(0);
    previousMillis = currentMillis;
    heaterCompute(currentMediaTempSetpoint);
    calculateOD940();
    displayStats();
    writeToCard();
  }
}

bool transitionIncubateToMenu()
{
  if (BtnSelect.longPress() == HIGH)
  {
    setMultiplexerFocus(0);
    stirringOFF();
    heaterOFF();
    allPumpsStop();
    growthStartTime = 0;
    growthStopTime = 0;
    flagGoToIncubate = false;
    lcd.clear();
    return true;
  }

  return false;
}

bool transitionMenuToIncubate()
{
  if (flagGoToIncubate == true)
  {
    setMultiplexerFocus(0);
    allPumpsStop();
    return true;
  }

  return false;
}



//PRIME PUMPS STATE
void stateFunctionPrimePumps()
{
  //set I2C multiplexer to I2C channel zero (LCD screen);
  setMultiplexerFocus(0);

  //goto the primePumps function
  primePumps();                                                                
}

bool transitionPrimePumpsToMenu()
{
  if (BtnSelect.longPress() == HIGH)
  {
    setMultiplexerFocus(0);
    heaterOFF();
    allPumpsStop();
    lcd.clear();
    return true;
  }

  return false;
}



//HEATER TEST STATE
void stateFunctionHeaterTest()
{
  setMultiplexerFocus(0);
  stirringON();
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    heaterCompute(currentMediaTempSetpoint);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("HEATER TEST");
    lcd.setCursor(0, 1);
    lcd.print("Ambient Temp:");
    lcd.print(ambientTemp);
    lcd.setCursor(0, 2);
    lcd.print("Media Temp:");
    lcd.print(mediaTemp);
    lcd.setCursor(0, 3);
    lcd.print("PID:");
    lcd.print(int(outputPWM));
    lcd.print(" HTR:");
    lcd.print(heaterPlateTemp);
  }
}

bool transitionHeaterTestToMenu()
{
  if (BtnSelect.longPress() == HIGH)
  {
    setMultiplexerFocus(0);
    stirringOFF();
    heaterOFF();
    lcd.clear();
    return true;
  }

  return false;
}

bool transitionMenuToHeaterTest()
{
  if (flagGoToHeaterTest == true)
  {
    setMultiplexerFocus(0);
    return true;
  }

  return false;
}


//CALIBRATE STIRRING STATE
void stateFunctionCalibrateStirring()
{
  setMultiplexerFocus(0);
  stirringON();

    //lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("CALIBRATE STIRRING");
    lcd.setCursor(0, 1);
    lcd.print("Stirring Speed:");
    lcd.print(motorStirringSpeed);

    if (BtnUp.pushed() == true)
  {
    motorStirringSpeed = motorStirringSpeed + 2;
  }

  if (BtnUp.released() == true)
  {
    
  }

  if (BtnDown.pushed() == true)
  {
    motorStirringSpeed = motorStirringSpeed - 2; 
  }

  if (BtnDown.released() == true)
  {
    
  }

}

bool transitionCalibrateStirringToMenu()
{
  if (BtnSelect.longPress() == HIGH)
  {
    setMultiplexerFocus(0);
    writeConfigFile();
    stirringOFF();
    heaterOFF();
    lcd.clear();
    return true;
  }

  return false;
}

bool transitionMenuToCalibrateStirring()
{
  if (flagGoToCalibrateStirring == true)
  {
    setMultiplexerFocus(0);
    return true;
  }

  return false;
}



//CALIBRATE OPTICS STATE
void stateFunctionCalibrateOptics()
{
  stirringON();
    setMultiplexerFocus(2);
    uint32_t lum = tsl.getFullLuminosity();

    //set lower 16bits from TSL2591 sensor to variable IR
    ir = lum >> 16;

    full = lum & 0xFFFF;
    setMultiplexerFocus(0);
    lcd.setCursor(0, 0);
    lcd.print("CALIBRATE OPTICS");
    lcd.setCursor(0, 1);
    lcd.print("IR Signal:");
    lcd.print(String(ir));
    lcd.setCursor(0, 2);
    lcd.print("LED PWR:");
    lcd.print(String(OD940LEDBrightness));

    if (BtnUp.pushed() == true)
  {
      OD940LEDBrightness = OD940LEDBrightness + 10;
     ledDriver.setPWM(0, OD940LEDBrightness);
     ledDriver.write();
  }

  if (BtnUp.released() == true)
  {
    
  }

  if (BtnDown.pushed() == true)
  {
    OD940LEDBrightness = OD940LEDBrightness - 10;
     ledDriver.setPWM(0, OD940LEDBrightness);
     ledDriver.write();    
  }

  if (BtnDown.released() == true)
  {
    
  }

}

bool transitionCalibrateOpticsToMenu()
{
  if (BtnSelect.longPress() == HIGH)
  {
    setMultiplexerFocus(0);
    writeConfigFile();
    stirringOFF();
    heaterOFF();
    lcd.clear();
    return true;
  }

  return false;
}

bool transitionMenuToCalibrateOptics()
{
  if (flagGoToCalibrateOptics == true)
  {
    setMultiplexerFocus(0);
    return true;
  }

  return false;
}



//CALIBRATE PUMP ONE STATE
void stateFunctionCalibratePumpOne()
{
    setMultiplexerFocus(0);
    lcd.setCursor(0, 0);
    lcd.print("CALIBRATE PUMP 1");
    lcd.setCursor(0, 1);
    lcd.print("Motor Speed:");
    lcd.print(pumpOneSpeed);

    if (BtnUp.released() == true)
  {
    pumpOneSpeed = pumpOneSpeed + 1;
  }

  if (BtnDown.released() == true)
  {
    pumpOneSpeed = pumpOneSpeed - 1; 
  }

  if (BtnSelect.released() == true)
  {
    for (int dispensations = 1; dispensations <= 10; dispensations++)
    {
    dispenseMedia(1); 
    lcd.clear();
    lcd.setCursor(0, 2);
    lcd.print("Dispensations: " + String(dispensations));
    }
  }
}

bool transitionCalibratePumpOneToMenu()
{
  if (BtnSelect.longPress() == HIGH)
  {
    setMultiplexerFocus(0);
    allPumpsStop();
    writeConfigFile();
    stirringOFF();
    heaterOFF();
    lcd.clear();
    return true;
  }

  return false;
}

bool transitionMenuToCalibratePumpOne()
{
  if (flagGoToCalibratePumpOne == true)
  {
    setMultiplexerFocus(0);
    allPumpsStop();
    return true;
  }

  return false;
}



//CALIBRATE PUMP TWO STATE
void stateFunctionCalibratePumpTwo()
{
    setMultiplexerFocus(0);
    lcd.setCursor(0, 0);
    lcd.print("CALIBRATE PUMP 2");
    lcd.setCursor(0, 1);
    lcd.print("Motor Speed:");
    lcd.print(pumpTwoSpeed);

    if (BtnUp.released() == true)
  {
    pumpTwoSpeed = pumpTwoSpeed + 1;
  }

  if (BtnDown.released() == true)
  {
    pumpTwoSpeed = pumpTwoSpeed - 1; 
  }

  if (BtnSelect.released() == true)
  {
    for (int dispensations = 1; dispensations <= 10; dispensations++)
    {
    dispenseMedia(2); 
    lcd.clear();
    lcd.setCursor(0, 2);
    lcd.print("Dispensations: " + String(dispensations));
    }
  }
}

bool transitionCalibratePumpTwoToMenu()
{
  if (BtnSelect.longPress() == HIGH)
  {
    setMultiplexerFocus(0);
    allPumpsStop();
    writeConfigFile();
    stirringOFF();
    heaterOFF();
    lcd.clear();
    return true;
  }

  return false;
}

bool transitionMenuToCalibratePumpTwo()
{
  if (flagGoToCalibratePumpTwo == true)
  {
    setMultiplexerFocus(0);
    allPumpsStop();
    return true;
  }

  return false;
}



//CALIBRATE PUMP THREE STATE
void stateFunctionCalibratePumpThree()
{
    setMultiplexerFocus(0);
    lcd.setCursor(0, 0);
    lcd.print("CALIBRATE PUMP 3");
    lcd.setCursor(0, 1);
    lcd.print("Motor Speed:");
    lcd.print(pumpThreeSpeed);

    if (BtnUp.released() == true)
  {
    pumpThreeSpeed = pumpThreeSpeed + 1;
  }

  if (BtnDown.released() == true)
  {
    pumpThreeSpeed = pumpThreeSpeed - 1; 
  }

  if (BtnSelect.released() == true)
  {
    for (int dispensations = 1; dispensations <= 10; dispensations++)
    {
    dispenseMedia(3); 
    lcd.clear();
    lcd.setCursor(0, 2);
    lcd.print("Dispensations: " + String(dispensations));
    }
  }
}

bool transitionCalibratePumpThreeToMenu()
{
  if (BtnSelect.longPress() == HIGH)
  {
    setMultiplexerFocus(0);
    allPumpsStop();
    writeConfigFile();
    stirringOFF();
    heaterOFF();
    lcd.clear();
    return true;
  }

  return false;
}

bool transitionMenuToCalibratePumpThree()
{
  if (flagGoToCalibratePumpThree == true)
  {
    setMultiplexerFocus(0);
    allPumpsStop();
    return true;
  }

  return false;
}



//CALIBRATE PUMP FOUR STATE
void stateFunctionCalibratePumpFour()
{
    setMultiplexerFocus(0);
    lcd.setCursor(0, 0);
    lcd.print("CALIBRATE PUMP 4");
    lcd.setCursor(0, 1);
    lcd.print("Motor Speed:");
    lcd.print(pumpFourSpeed);

    if (BtnUp.released() == true)
  {
    pumpFourSpeed = pumpFourSpeed + 1;
  }

  if (BtnDown.released() == true)
  {
    pumpFourSpeed = pumpFourSpeed - 1; 
  }

  if (BtnSelect.released() == true)
  {
    for (int dispensations = 1; dispensations <= 10; dispensations++)
    {
    dispenseMedia(4); 
    lcd.clear();
    lcd.setCursor(0, 2);
    lcd.print("Dispensations: " + String(dispensations));
    }
  }
}

bool transitionCalibratePumpFourToMenu()
{
  if (BtnSelect.longPress() == HIGH)
  {
    setMultiplexerFocus(0);
    allPumpsStop();
    writeConfigFile();
    stirringOFF();
    heaterOFF();
    lcd.clear();
    return true;
  }

  return false;
}

bool transitionMenuToCalibratePumpFour()
{
  if (flagGoToCalibratePumpFour == true)
  {
    setMultiplexerFocus(0);
    allPumpsStop();
    return true;
  }

  return false;
}



//---------------------------------------TEST FUNCTIONS----------------------------------------

void primePumps()
{
  setMultiplexerFocus(0);

  if (BtnSelect.pushed() == true)
  {
    pumpSelection++;
    if (pumpSelection > 4)
    {
      pumpSelection = 1;
    }
  }

  if (BtnUp.pushed() == true)
  {
    removeMedia(pumpSelection);
  }

  if (BtnUp.released() == true)
  {
    allPumpsStop();
  }

  if (BtnDown.pushed() == true)
  {
    dispenseMedia(pumpSelection);
  }

  if (BtnDown.released() == true)
  {
    allPumpsStop();
  }

  lcd.setCursor(0, 0);
  lcd.print("UP or DOWN to pump");
  lcd.setCursor(0, 1);
  lcd.print("SEL for next pump");
  lcd.setCursor(0, 2);
  lcd.print("Hold SEL to Exit");
  lcd.setCursor(0, 3);
  lcd.print("Current Pump:");

  switch (pumpSelection)
  {
    case 1:
      lcd.print("1");
      break;
    case 2:
      lcd.print("2");
      break;
    case 3:
      lcd.print("3");
      break;
    case 4:
      lcd.print("4");
      break;
    default:
      lcd.print("ERR");
      break;
  }
}



void heaterCompute(double tempSetpoint)
{
  mediaTemp = mediaTempThermistorSmooth->readCelsius();
  heaterPlateTemp = heaterPlateTempThermistorSmooth->readCelsius();
  ambientTemp = rtc.getTemperature();

  double gap = abs((tempSetpoint + setpointOffset) - mediaTemp); //distance away from setpoint

  if (gap < gapThreshold)
  { //we're close to setpoint, use conservative tuning parameters
    mediaTempPID.SetTunings(consKp, consKi, consKd);
  }

  else
  {
    //we're far from setpoint, use aggressive tuning parameters
    mediaTempPID.SetTunings(aggKp, aggKi, aggKd);
  }

  mediaTempPID.Compute();

  outputPWM = map(outputPWM, 0, 255, 0, pwmSafetyLimit);           //limit PWM level to safety limit

  analogWrite(heaterPlateTopPinEnable, outputPWM);  //send PID-adjusted PWM to heater controller
  analogWrite(heaterPlateBottomPinEnable, outputPWM);

  digitalWrite(heaterPlateTopPinInputOne, HIGH);  //turn on H-Bridge configuration in heater controller
  digitalWrite(heaterPlateTopPinInputTwo, LOW);   //this config actually turns on the heater
  digitalWrite(heaterPlateBottomPinInputOne, HIGH);
  digitalWrite(heaterPlateBottomPinInputTwo, LOW);
}



void allPumpsStop()
{
  pumpOne->run(RELEASE);
  pumpOne->setSpeed(0);
  pumpOne->run(RELEASE);

  pumpTwo->run(RELEASE);
  pumpTwo->setSpeed(0);
  pumpTwo->run(RELEASE);

  pumpThree->run(RELEASE);
  pumpThree->setSpeed(0);
  pumpThree->run(RELEASE);

  pumpFour->run(RELEASE);
  pumpFour->setSpeed(0);
  pumpFour->run(RELEASE);
}
//-----------------------------------------------------------------------------------



//---------------------IMPORTANT FUNCTIONS-------------------------------------------
void stirringON()
{
  digitalWrite(motorStirringPinInputOne, LOW);                //keep this pin LOW to make stirrer go only in one direction
  analogWrite(motorStirringPinInputTwo, motorStirringSpeed);  //send PWM signal to adjust stirring speed
}



void stirringOFF()
{
  digitalWrite(motorStirringPinInputOne, LOW);
  digitalWrite(motorStirringPinInputTwo, LOW);
}



void heaterON()
{
  digitalWrite(heaterPlateTopPinInputOne, HIGH);            //turn on H-Bridge configuration in heater controller
  digitalWrite(heaterPlateTopPinInputTwo, LOW);             //this config actually turns on the heater
  digitalWrite(heaterPlateBottomPinInputOne, HIGH);
  digitalWrite(heaterPlateBottomPinInputTwo, LOW);
}



void heaterOFF()
{
  analogWrite(heaterPlateTopPinEnable, 0);                  //send PID-adjusted PWM to heater controller
  analogWrite(heaterPlateBottomPinEnable, 0);

  digitalWrite(heaterPlateTopPinInputOne, LOW);             //turn on H-Bridge configuration in heater controller
  digitalWrite(heaterPlateTopPinInputTwo, LOW);             //this config actually turns on the heater
  digitalWrite(heaterPlateBottomPinInputOne, LOW);
  digitalWrite(heaterPlateBottomPinInputTwo, LOW);
}



void dispenseMedia(int pumpNumber)
{
    switch (pumpNumber)
    {
      case 1:
          pumpOne->run(BACKWARD);
          pumpOne->setSpeed(pumpOneSpeed);
          delay(1000);
          pumpOne->run(RELEASE);
        break;
      case 2:
          pumpTwo->run(BACKWARD);
          pumpTwo->setSpeed(pumpTwoSpeed);
          delay(1000);
          pumpTwo->run(RELEASE);
        break;
      case 3:
          pumpThree->run(BACKWARD);
          pumpThree->setSpeed(pumpThreeSpeed);
          delay(1000);
          pumpThree->run(RELEASE);
          break;
      case 4:
          pumpFour->run(BACKWARD);
          pumpFour->setSpeed(pumpFourSpeed);
          delay(1000);
          pumpFour->run(RELEASE);
          break;
      default:
        allPumpsStop();
        break;
    }
}



void removeMedia(int pumpNumber)
{
    switch (pumpNumber)
    {
      case 1:
          pumpOne->run(FORWARD);
          pumpOne->setSpeed(pumpOneSpeed);
          delay(2000);
          pumpOne->run(RELEASE);
        break;
      case 2:
          pumpTwo->run(FORWARD);
          pumpTwo->setSpeed(pumpTwoSpeed);
          delay(2000);
          pumpTwo->run(RELEASE);
        break;
      case 3:
          pumpThree->run(FORWARD);
          pumpThree->setSpeed(pumpThreeSpeed);
          delay(2000);
          pumpThree->run(RELEASE);
          break;
      case 4:
          pumpFour->run(FORWARD);
          pumpFour->setSpeed(pumpFourSpeed);
          delay(2000);
          pumpFour->run(RELEASE);
          break;
      default:
        allPumpsStop();
        break;
    }
}



void initializeSDCard()
{
  if (!SD.begin(sdCardCSPin))
  {
    setMultiplexerFocus(0);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SD CARD ERROR!");
    while (1) delay(1000);
  }

  File csvFile = SD.open("EEVO.csv", FILE_WRITE);

  if (csvFile)
  {
    csvFile.print(csvFileHeader);
  }
  csvFile.close();
}



void writeToCard()
{
  File csvFile = SD.open("EEVO.csv", FILE_WRITE);

  if (csvFile)
  {
    setMultiplexerFocus(1);
    digitalWrite(LED_BUILTIN, HIGH);

    // saves date and time right before writing to memory card so it's as precise as possible to SD card
    DateTime now = rtc.now();                                 
    currentUnixTime = now.unixtime();
    upTime = now.unixtime() - robotStartTime;
    currentProgramTime = now.unixtime() - programStartTime;

     // saves unix time value as single 64-bit integer for easy seconds counting to SD card
    csvFile.print(currentUnixTime);                            
    csvFile.print(',');

    csvFile.print(upTime);
    csvFile.print(',');

    csvFile.print(currentProgramTime);
    csvFile.print(',');

    csvFile.print(ambientTemp);
    csvFile.print(',');

    csvFile.print(mediaTemp);
    csvFile.print(',');

    csvFile.print(heaterPlateTemp);
    csvFile.print(',');

    csvFile.print(outputPWM);
    csvFile.print(',');

    csvFile.print(growthDuration);
    csvFile.print(',');

    csvFile.print(growthDurationChange);
    csvFile.print(',');

    csvFile.print(neutralCycleCount);
    csvFile.print(',');

    csvFile.print(positiveCycleCount);
    csvFile.print(',');

    csvFile.print(negativeCycleCount);
    csvFile.print(',');

    csvFile.print(totalCycleCount);
    csvFile.print(',');

    csvFile.print(FULL);
    csvFile.print(',');

    csvFile.print(VIS);
    csvFile.print(',');

    csvFile.print(IR);
    csvFile.print(',');

    csvFile.println(currentOD940, 4);

    digitalWrite(LED_BUILTIN, LOW);

    csvFile.close();                                          
  }

  else
  {
    setMultiplexerFocus(0);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SD CARD ERROR!");
    while (1) delay(1000);
  }
}



void writeConfigFile()
{
  if (!SD.begin(sdCardCSPin))
  {
    setMultiplexerFocus(0);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SD CARD ERROR!");
    while (1) delay(1000);
  }

//check if old CONFIG file exists and, if so, remove it
  if (SD.exists("CONFIG.TXT")) 
  {
    SD.remove("CONFIG.TXT");
  }

// create a new config file using the declared variables
  File configFile = SD.open("CONFIG.TXT", FILE_WRITE);

  if (configFile)
   {
    configFile.println("incubationSetpointTemp=" + String(incubationSetpointTemp, 2));
    configFile.println("OD940LEDBrightness=" + String(OD940LEDBrightness));
    configFile.println("motorStirringSpeed=" + String(motorStirringSpeed));
    configFile.println("pumpOneSpeed=" + String(pumpOneSpeed));
    configFile.println("pumpTwoSpeed=" + String(pumpTwoSpeed)); 
    configFile.println("pumpThreeSpeed=" + String(pumpThreeSpeed));
    configFile.println("pumpFourSpeed=" + String(pumpFourSpeed));
    configFile.println("END");
    configFile.close();
    }
    else 
    {
      // File not found
      setMultiplexerFocus(0);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("CONFIG WRITE ERROR!");
    while (1) delay(1000);
    }
}



void readConfigFile()
{
  if (!SD.begin(sdCardCSPin))
  {
    setMultiplexerFocus(0);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SD CARD ERROR!");
    while (1) delay(1000);
  }

  File configFile = SD.open("CONFIG.TXT", FILE_READ);

  if (configFile)
  {
     while (configFile.available()) {
      String configData = configFile.readStringUntil('\n');
      String key = configData.substring(0, configData.indexOf('='));
      String value = configData.substring(configData.indexOf('=') + 1);
      if (key == "incubationSetpointTemp") {
        incubationSetpointTemp = value.toFloat();
      } else if (key == "OD940LEDBrightness") {
        OD940LEDBrightness = value.toInt();
      } else if (key == "motorStirringSpeed") {
        motorStirringSpeed = value.toInt();
      } else if (key == "pumpOneSpeed") {
        pumpOneSpeed = value.toInt();
      }  else if (key == "pumpTwoSpeed") {
        pumpTwoSpeed = value.toInt();
      } else if (key == "pumpThreeSpeed") {
        pumpThreeSpeed = value.toInt();
      } else if (key == "pumpFourSpeed") {
        pumpFourSpeed = value.toInt();
      }
    }
    configFile.close();
  }

  else
  {
    setMultiplexerFocus(0);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("CONFIG READ ERROR!");
    while (1) delay(1000);
  }
}



void readCycleProgramFile()
{
  if (!SD.begin(sdCardCSPin))
  {
    setMultiplexerFocus(0);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SD CARD ERROR!");
    while (1) delay(1000);
  }

  File cycleProgramFile = SD.open("CYCLER.CSV", FILE_READ);

  if (cycleProgramFile)
  {
    int cycleLineNumber = 0; //start on cycle number 1 since line 0 of the CSV is the human readable header
     while (cycleProgramFile.available()) 
      {
        String cycleProgramData = cycleProgramFile.readStringUntil('\n');
        int start = 0;
        for (int parameterNumber = 0; parameterNumber < 10; parameterNumber++)
          {
            int end = cycleProgramData.indexOf(',', start);
            if (end == -1) end = cycleProgramData.length();
            mediaCycleArray[cycleLineNumber][parameterNumber] = cycleProgramData.substring(start, end);
            start = end + 1;
            //Serial.println(mediaCycleArray[cycleLineNumber][parameterNumber]);
          }
          cycleLineNumber++;
      }
    cycleProgramFile.close(); // close SD card file

    totalProgramCycles = cycleLineNumber;

  }

  else
  {
    setMultiplexerFocus(0);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("CYCLE PRGM READ ERROR!");
    while (1) delay(1000);
  }

      currentStepNumber = mediaCycleArray[currentCycleStep][0].toInt();
      currentMediaType = mediaCycleArray[currentCycleStep][1];      //"NEUTRAL", "POSITIVE", or "NEGATIVE" (media pumps 1, 2, and 3 respectively)
      currentMediaTempSetpoint = mediaCycleArray[currentCycleStep][2].toDouble();
      currentStimulationLEDNumber = mediaCycleArray[currentCycleStep][3].toInt();
      currentStimulationLEDIntensity = mediaCycleArray[currentCycleStep][4].toInt();
      currentOpticalDensityFloor = mediaCycleArray[currentCycleStep][5].toFloat();
      currentOpticalDensityCeiling = mediaCycleArray[currentCycleStep][6].toFloat();
      currentStirringSpeed = mediaCycleArray[currentCycleStep][7].toInt();
      currentCycleTimeoutDuration = mediaCycleArray[currentCycleStep][8].toInt();

    if (currentMediaType == "NEUTRAL")
    {
      currentMediaPumpNumber = 1;
    }

    if (currentMediaType == "POSITIVE")
    {
      currentMediaPumpNumber = 2;
    }

    if (currentMediaType == "NEGATIVE")
    {
      currentMediaPumpNumber = 3;
    }
}



void configureLightSensor(void)
{
  setMultiplexerFocus(2);
  // settings for the TSL2591 light sensor; adjust these to match your LED intensity. 
  // Currently set to be just at the cusp of saturation for max bit depth
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  //tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain

  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)
}



void resetStimulationLights()
{
   for(int i = 1; i < totalStimulationLEDCount + 1; i++)
    {
      ledDriver.setPWM(i, 0); //shut off all stimulation lights aside from the optical density light
    }

    ledDriver.write();
}



void showSplashScreen()
{
  setMultiplexerFocus(0);
  lcd.setCursor(2, 0);                                       // draw Binomica Labs splash page
  lcd.print("easyEVO v");
  lcd.print(softwareVersion);
  lcd.setCursor(3, 1);
  lcd.print("Binomica Labs");
  lcd.setCursor(2, 2);
  lcd.print("Small Thoughtful");
  lcd.setCursor(6, 3);
  lcd.print("Science");

  delay(4000);
}



void calculateOD940()
{
  setMultiplexerFocus(2);
  uint32_t lum = tsl.getFullLuminosity();
  ir = lum >> 16;                                         //set lower 16bits from TSL2591 sensor to variable IR
  full = lum & 0xFFFF;

  IR = ir;
  FULL = full;
  VIS = full - ir;
  IRblank = 37889;
  currentOD940 = log10(float(IRblank) / float(IR));       //forcing it to read as a float so integer does not take priority, IMPORTANT
}



void displayStats()
{
  setMultiplexerFocus(1);
  DateTime now = rtc.now(); // check the time on the real-time clock
  currentUnixTime = now.unixtime();
  currentProgramTime = now.unixtime() - programStartTime;
  ambientTemp = float(rtc.getTemperature());
  mediaTemp = mediaTempThermistorSmooth->readCelsius();
  heaterPlateTemp = heaterPlateTempThermistorSmooth->readCelsius();

  setMultiplexerFocus(0);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TIME:");
  lcd.print(currentProgramTime);
  lcd.setCursor(13, 0);
  lcd.print("CYC:");
  lcd.print(totalCycleCount);
  lcd.setCursor(0, 1);
  lcd.print("Amb:");
  lcd.print(ambientTemp);
  lcd.print(" Med:");
  lcd.print(mediaTemp);
  lcd.setCursor(0, 2);
  lcd.print("Heater:");
  lcd.print(heaterPlateTemp);
  lcd.print(" PWM:");
  lcd.print(int(outputPWM));
  lcd.setCursor(0, 3);
  lcd.print("IR:");
  lcd.print(String(ir));
  lcd.print(" ");
  lcd.print(" OD:");
  //display OD940 value to 4 decimal places (7 is max on most AVR chips)
  lcd.print(currentOD940, 4);                   
}



void displaySecondaryStats()
{
  setMultiplexerFocus(0);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("STP:");
  lcd.print(currentStepNumber);
  lcd.setCursor(12, 0);
  lcd.print(" MED:");
  lcd.print(currentMediaType);
  lcd.setCursor(0, 1);
  lcd.print("SET:");
  lcd.print(currentMediaTempSetpoint);
  lcd.print(" LED:");
  lcd.print(currentStimulationLEDNumber);
  lcd.setCursor(0, 2);
  lcd.print("ODF:");
  lcd.print(currentOpticalDensityFloor);
  lcd.print(" ODC:");
  lcd.print(currentOpticalDensityCeiling);
  lcd.setCursor(0, 3);
  lcd.print("GRW:");
  lcd.print(growthDuration);
  lcd.print(" DIF:");
  lcd.print(growthDurationChange);                  }