// *******************************************************************************************************************************************
// easyEVO Type S Directed Evolution Engine
// Version: 1.0.0.0
// Copyright 2023 Binomica Labs
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
//      940-945nm LED 0   //doubles as OD940 sensor light
//      455-460nm LED 1
//      465-470nm LED 2
//      480-485nm LED 3
//      500-505nm LED 4
//      520-535nm LED 5
//      530-535nm LED 6
//      660-665nm LED 7
//      680-685nm LED 8
//      700-705nm LED 9
//      720-725nm LED 10
//      760-765nm LED 11
//      780-785nm LED 12
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
#include <PID_v1.h>
#include <math.h>
#include <RTClib.h>                       // real-time clock library
#include <SPI.h>                          // SPI library for SD card module
#include <SD.h>                           // SD card library
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
#include <StateMachine.h>
#include "Arduino.h"
#include "avdweb_Switch.h"
#include "Adafruit_TLC5947.h"
#include <NTC_Thermistor.h>
#include <SmoothThermistor.h>
#include <SpeedyStepper.h>



// CRITICAL SETTINGS
String softwareVersion = "v1.2.5.0";
String csvFileHeader = "unixTime,upTime,currentProgramTime,ambientTemp,mediaTemp,heaterPlateTemp,heaterPWM,growthDuration,growthDurationChange,dilutionDuration,dilutionDurationChange,neutralMediaDispenseCount,cycleDuration,neutralCycleCount,positiveCycleCount,neutralCycleTwoCount,negativeCycleCount,totalCycleCount,fullSpectrumReading,visibleSpectrumReading,infraredReading,OD940\n";

//the temperature you wish your culture vessel to hold in degrees Celsius.
double setpointTemp = 30.00;

//keep constant to ensure consistent stirring speed
int motorStirringSpeed = 120;

//upper optical density trigger level, starts the media refresh cycle
float OD940UpperBound = 0.6000;

//lower optical density trigger level, stops the media refresh cycle
float OD940LowerBound = 0.3000;

//initial brightness of the 940nm Optical Density LED; adjust to reduce saturation
int OD940LEDBrightness = 2100;

//dispensing distance in rotational steps, calibrate your motors to determine these values below
int neutralMediaRotationStepsForward = 1080;

int neutralMediaRotationStepsReverse = neutralMediaRotationStepsForward * -1;

int positiveMediaRotationStepsForward = 1140;

int positiveMediaRotationStepsReverse = positiveMediaRotationStepsForward * -1;

int negativeMediaRotationStepsForward = 1110;

int negativeMediaRotationStepsReverse = negativeMediaRotationStepsForward * -1;

int wasteRotationStepsForward = 1140;

//waste removal rotations is double any dispensing motor rotations to protect against overflow
int wasteRotationStepsReverse = (wasteRotationStepsForward * 2) * -1;

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
const int intervalTurbidostat = 10000;

//0.1Hz LightCycler loop interval (adjusted for time spent dispensing)
const int intervalLightCycler = 10000;



// STATE STUFF
bool flagGoToStandby = false;
bool flagGoToTurbidostat = false;
bool flagGoToLightCycler = false;
bool flagGoToIncubate = false;
bool flagGoToHeaterTest = false;
bool flagGoToPrimePumps = false;
bool flagGoToCalibrateStirring = false;
bool flagGoToCalibrateOptics = false;
bool flagGoToCalibratePumpOne = false;
bool flagGoToCalibratePumpTwo = false;
bool flagGoToCalibratePumpThree = false;
bool flagGoToCalibratePumpFour = false;
bool flagNeutralCycle = true;     //starts off true to begin cycling on neutral media as default
bool flagPositiveCycle = false;
bool flagNegativeCycle = false;



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


int dilutionStartTime = 0;
int dilutionStopTime = 0;
int dilutionDuration = 0;
int dilutionDurationChange = 0;
int previousDilutionDuration = 0;

int cycleDuration = 0;

int neutralMediaDispenseCount = 0;

int totalCycleCount = 0;

int neutralCycleCount = 0;
int neutralCycleTwoCount = 0;
int positiveCycleCount = 0;
int negativeCycleCount = 0;
int mediaCycleSelection = 0;



//  PERISTALTIC PUMP CONTROLLER STUFF (CNC Sheild; hardwired pins)
const int motorNeutralMediaStepPin = 2;
const int motorNeutralMediaDirectionPin = 5;
const int motorWasteStepPin = 3;
const int motorWasteDirectionPin = 6;
const int motorPositiveMediaStepPin = 4;
const int motorPositiveMediaDirectionPin = 7;
const int motorNegativeMediaStepPin = 12;
const int motorNegativeMediaDirectionPin = 13;
int pumpSelection = 0;                                //an incremented variable to easily select which pump in a for-loop during priming
SpeedyStepper motorNeutralMedia;
SpeedyStepper motorWaste;
SpeedyStepper motorNegativeMedia;
SpeedyStepper motorPositiveMedia;



//  STIRRING CONTROLLER STUFF
int motorStirringPinInputOne = 48;
int motorStirringPinInputTwo = 46;



// HEATER CONTROLLER STUFF
int heaterPlateTopPinEnable = 44;       // Top Heater Plate PWM Pin
int heaterPlateTopPinInputOne = 42;
int heaterPlateTopPinInputTwo = 40;
int heaterPlateBottomPinEnable = 34;    // Bottom Heater Plate PWM Pin
int heaterPlateBottomPinInputOne = 38;
int heaterPlateBottomPinInputTwo = 36;



// THERMOMETER STUFF
//makes a null object of a thermistor to pass original thermistor into smoothing function
Thermistor* mediaTempThermistorSmooth = NULL;
Thermistor* heaterPlateTempThermistorSmooth = NULL;
int thermistorMediaPin = A8;                    //arduino analog pin 8 for media thermistor data line
int thermistorHeaterPlatePin = A9;              //arduino analog pin 9 for media thermistor data line
int thermistorReferenceResistance = 10000;      //resistance value of Vishay PTF5610K000BZEK precision resistor
int thermistorNominalResistance = 10000;        //rated thermistor resistance
int thermistorNominalTemp = 25;                 //temp curves were calibrated for at thermistor factory
int thermistorBValue = 3380;                    //taken from Murata NXFT15XH103FA1B150 NTC thermistor datasheet
int thermistorSmoothingFactor = 5;              //how many samples to take and average



// PID STUFF
double setpointOffset = 0.25; //added an extra 0.25 so average is more centered around 30C since gap trigger is setPointTemp - 0.5
double outputPWM = 0.00;
double aggKp = 4, aggKi = 0.2, aggKd = 1;        //Define the aggressive and conservative Tuning Parameters
double consKp = 1, consKi = 0.05, consKd = 0.25;
double gapThreshold = 0.5;
//************SAFETY VARIABLE********************
float pwmSafetyLimit = 120.00;    //DO **NOT** LET HEATER EXCEED PLA PLASTIC SOFTENING POINT (~120C)!!!
//************************************************/
PID mediaTempPID(&mediaTemp, &outputPWM, &setpointTemp, consKp, consKi, consKd, DIRECT);   //Specify the links and initial tuning parameters



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
int ledDriverPinData = 25;
int ledDriverPinClock = 27;
int ledDriverPinLatch = 23;
int ledDriverPinEnable = -1;  //set to -1 to not use enable pin (optional)
Adafruit_TLC5947 ledDriver = Adafruit_TLC5947(1, ledDriverPinClock, ledDriverPinData, ledDriverPinLatch);



// REAL-TIME CLOCK STUFF
RTC_DS3231 rtc;                           // defines real-time clock object
unsigned long previousMillis = 0;



// SD CARD STUFF
const int sdCardCSPin = 53;
File csvFile;
File configFile;



// LCD SCREEN STUFF
LiquidCrystal_I2C lcd(0x27, 20, 4);     // set I2C address to 0x27 and a 20 chars x 4 line screen



StateMachine machine = StateMachine();
State* stateMenu = machine.addState(&stateFunctionMenu);
State* stateStandby = machine.addState(&stateFunctionStandby);
State* stateTurbidostat = machine.addState(&stateFunctionTurbidostat);
State* stateIncubate = machine.addState(&stateFunctionIncubate);
State* stateLightCycler = machine.addState(&stateFunctionLightCycler);
State* statePrimePumps = machine.addState(&stateFunctionPrimePumps);
State* stateHeaterTest = machine.addState(&stateFunctionHeaterTest);
State* stateCalibrateStirring = machine.addState(&stateFunctionCalibrateStirring);
State* stateCalibrateOptics = machine.addState(&stateFunctionCalibrateOptics);
State* stateCalibratePumpOne = machine.addState(&stateFunctionCalibratePumpOne);
State* stateCalibratePumpTwo = machine.addState(&stateFunctionCalibratePumpTwo);
State* stateCalibratePumpThree = machine.addState(&stateFunctionCalibratePumpThree);
State* stateCalibratePumpFour = machine.addState(&stateFunctionCalibratePumpFour);



//MENU STUFF
String MenuItems[] =
{
  "TURBIDOSTAT",
  "INCUBATE",
  "LIGHT CYCLER",
  "PRIME PUMPS",
  "HEATER TEST",
  "CALIBRATE STIRRING",
  "CALIBRATE OPTICS",
  "CALIBRATE PUMP 1",
  "CALIBRATE PUMP 2",
  "CALIBRATE PUMP 3",
  "CALIBRATE PUMP 4"
};



// Set the focus of the I2C Multiplexer. 0 is LCD screen. 1 is Real-time clock. 2 is light sensor.
void setMultiplexerFocus(uint8_t bus)
{
  Wire.beginTransmission(0x70);  // TCA9548A I2C Multiplexer bit address
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
      growthStartTime = now.unixtime() - programStartTime;
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

  if (menuItem == 3) // select LIGHT CYCLER mode
  {
    if (selectPressed == 1)
    {
      setMultiplexerFocus(0);
      lcd.clear();
      setMultiplexerFocus(1);
      DateTime now = rtc.now();
      programStartTime = now.unixtime();
      growthStartTime = now.unixtime() - programStartTime;
      machine.transitionTo(stateLightCycler);
    }
  }

  if (menuItem == 4) // select PRIME PUMPS mode
  {
    if (selectPressed == 1)
    {
      setMultiplexerFocus(0);
      lcd.clear();
      machine.transitionTo(statePrimePumps);
    }
  }

  if (menuItem == 5) // select HEATER TEST mode
  {
    if (selectPressed == 1)
    {
      setMultiplexerFocus(0);
      lcd.clear();
      machine.transitionTo(stateHeaterTest);
    }
  }

  if (menuItem == 6) // select CALIBRATE STIRRING mode
  {
    if (selectPressed == 1)
    {
      setMultiplexerFocus(0);
      lcd.clear();
      machine.transitionTo(stateCalibrateStirring);
    }
  }

    if (menuItem == 7) // select CALIBRATE OPTICS mode
  {
    if (selectPressed == 1)
    {
      setMultiplexerFocus(0);
      lcd.clear();
      machine.transitionTo(stateCalibrateOptics);
    }
  }

     if (menuItem == 8) // select CALIBRATE PUMP ONE mode
  {
    if (selectPressed == 1)
    {
      setMultiplexerFocus(0);
      lcd.clear();
      machine.transitionTo(stateCalibratePumpOne);
    }
  }

   if (menuItem == 9) // select CALIBRATE PUMP TWO mode
  {
    if (selectPressed == 1)
    {
      setMultiplexerFocus(0);
      lcd.clear();
      machine.transitionTo(stateCalibratePumpTwo);
    }
  }

   if (menuItem == 10) // select CALIBRATE PUMP THREE mode
  {
    if (selectPressed == 1)
    {
      setMultiplexerFocus(0);
      lcd.clear();
      machine.transitionTo(stateCalibratePumpThree);
    }
  }

   if (menuItem == 11) // select CALIBRATE PUMP FOUR mode
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
  //Serial.begin(9600);
  Wire.begin();
  BtnUp.doubleClickPeriod = 0;        //turn off double-click on all buttons
  BtnSelect.doubleClickPeriod = 0;
  BtnDown.doubleClickPeriod = 0;

  setMultiplexerFocus(0);
  // initialize the lcd
  lcd.begin();
  // turn on LCD backlight
  lcd.backlight();
  lcd.clear();

  //poll the SD card and make sure it's working
  initializeSDCard();

  //read the configuration and calibration data from sdCard's config.txt file
  readConfigFile();

  //Initialize Pump Steppers and put them in HALT mode (all steps LOW)
  intializePumps();

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

  //turn on the OD940 LED to preheat, set the intensity (0 to 4096) to match TSL2591 maximum reading of 37888 or a tiny bit below. This ensure full ADC range when reading optical density.
  ledDriver.setPWM(0, OD940LEDBrightness);

  //Visible LED Sixpack
  ledDriver.setPWM(1, 0);     //455-460nm
  ledDriver.setPWM(2, 0);     //465-470nm
  ledDriver.setPWM(3, 0);     //480-485nm
  ledDriver.setPWM(4, 0);     //500-505nm
  ledDriver.setPWM(5, 0);     //520-535nm
  ledDriver.setPWM(6, 0);     //530-535nm

  //NIR LED Sixpack
  ledDriver.setPWM(7, 0);     //660-665nm
  ledDriver.setPWM(8, 0);     //680-685nm
  ledDriver.setPWM(9, 0);     //700-705nm
  ledDriver.setPWM(10, 0);    //720-725nm
  ledDriver.setPWM(11, 0);    //760-765nm
  ledDriver.setPWM(12, 0);    //780-785nm

  ledDriver.write();    //actually send the above values to the LED driver; do not forget to add this line or LED levels will not change!!!

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

  stateLightCycler->addTransition(transitionLightCyclerToMenu, stateMenu);

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

  //activate the statemachine; most important function call of the entire robot
  machine.run();
}



void stateFunctionMenu()
{
  setMultiplexerFocus(0);
  lcd.setCursor(0, 0);
  lcd.print("EasyEVO Type S");
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

    //calculate the heater plate's current temperature and the PID controller's output to maintain user defined media temp
    heaterCompute();

    //reset the transition flag to keep the easyEVO in standby mode until optical density trigger occurs
    flagGoToTurbidostat = false;

    //measure the optical density of the reaction tube
    calculateOD940();

    //check if OD hit ceiling threshold, if so then transition into the turbidostat state function to dispense media
    if (currentOD940 >= OD940UpperBound)
    {
      flagGoToTurbidostat = true;
    }

    //display relevant information on the LCD screen
    displayStats();

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
    flagGoToStandby = false;
    lcd.clear();
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

    growthStopTime = now.unixtime() - programStartTime;

    growthDuration = growthStopTime - growthStartTime;

    growthDurationChange = growthDuration - previousGrowthDuration;

    previousGrowthDuration = growthDuration;

    dilutionStartTime = now.unixtime() - programStartTime;

    setMultiplexerFocus(0);
    allPumpsStop();
    flagGoToStandby = false;
    return true;
  }

  return false;
}

//conditional which if true, transitions from the main menu to standby mode
bool transitionMenuToStandby()
{
  if (flagGoToStandby == true)
  {
    setMultiplexerFocus(1);
    DateTime now = rtc.now();

    growthStartTime = now.unixtime() - programStartTime;

    setMultiplexerFocus(0);

    allPumpsStop();

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
    heaterCompute();
    neutralMediaFWD();  //dispense neutral media
    wasteREV();         //suck up from waste line to keep volume constant
    neutralMediaDispenseCount++;
    float totalOD940 = 0.0000;
    float averageOD940 = 0.0000;
    for (int i = 0; i <= 9; i++)
    {
      calculateOD940();
      totalOD940 = totalOD940 + currentOD940;
    }

    averageOD940 = totalOD940 / 10;
    currentOD940 = averageOD940;      //THIS IS FOR A TSL2591 LIGHT SENSOR INTEGRATION TIME OF 100ms ONLY!
   
    if (currentOD940 <= OD940LowerBound)
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

    dilutionStopTime = now.unixtime() - programStartTime;

    dilutionDuration = dilutionStopTime - dilutionStartTime;

    dilutionDurationChange = dilutionDuration - previousDilutionDuration;

    previousDilutionDuration = dilutionDuration;

    neutralMediaDispenseCount = 0;

    totalCycleCount++;

    cycleDuration = growthDuration + dilutionDuration;

    growthStartTime = now.unixtime() - programStartTime;


    setMultiplexerFocus(0);                                                              //set I2C multiplexer to I2C channel zero (LCD screen);
    allPumpsStop();                                                           //shut off all the pumps, for safety
    flagGoToTurbidostat = false;                                              //reset the flag that triggers transition between standby and turbidostat states
    return true;
  }

  return false;
}



//INCUBATE STATE - grow a batch culture without using pumps. Useful for making starter cultures or characterizing microbial growth.
void stateFunctionIncubate()
{
  setMultiplexerFocus(0);
  stirringON();
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    heaterCompute();
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
    flagGoToStandby = false;
    lcd.clear();
    return true;
  }

  return false;
}



//LIGHTCYCLER STATE - utilize all the pumps to oscillate between positive, neutral, and negative selection media and activate certain LED lights during these cycles
void stateFunctionLightCycler()
{
  setMultiplexerFocus(0);
  stirringON();
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= intervalLightCycler)
  {
    previousMillis = currentMillis;
    heaterCompute();

    //check to see which media pump to dispense from depending on the cycle counts of each media prior
    switch (mediaCycleSelection)
    {
      case 0:
        neutralMediaFWD();
        wasteREV();
        break;
      case 1:
        positiveMediaFWD();
        wasteREV();
        break;
      case 2:
        neutralMediaFWD();  //always dispense neutral media between each selective media to ensure cells recover and desired genes have time to express
        wasteREV();
        break;
      case 3:
        negativeMediaFWD();
        wasteREV();
        break;

      default:
        allPumpsStop();
        break;
    }

    //this region of code carefully averages the OD940 values to be certain the current OD is not due to light scattering noise
    float totalOD940 = 0.00;
    float averageOD940 = 0.00;
    for (int i = 0; i <= 9; i++)
    {
      calculateOD940();
      totalOD940 = totalOD940 + currentOD940;
    }

    averageOD940 = totalOD940 / 10;
    currentOD940 = averageOD940;      //THIS IS FOR A TSL2591 LIGHT SENSOR INTEGRATION TIME OF 100ms ONLY!

    if (currentOD940 <= OD940LowerBound)
    {
      allPumpsStop();
      flagGoToStandby = true;
    }
    displayStats();
    writeToCard();
  }
}

bool transitionLightCyclerToMenu()
{
  if (BtnSelect.longPress() == HIGH)
  {
    setMultiplexerFocus(0);
    stirringOFF();
    heaterOFF();
    allPumpsStop();
    lcd.clear();
    return true;
  }

  return false;
}

bool transitionLightCyclerToStandby()
{
  if (flagGoToStandby == true)
  {
    setMultiplexerFocus(1);
    DateTime now = rtc.now();                                                 //set the current cyle stop time to the current real time

    //upon leaving Light Cycler mode and returning to standby mode, increment the current media cycle counts depending on the current media selected and remaining cycles predefined by the user
    switch (mediaCycleSelection)
    {
      case 0:
        neutralCycleCount++;    //increment the neutral media cycle count
        totalCycleCount++;      //increment the global cycle count
        if (neutralCycleCount >= neutralCycleMax)
        {
          mediaCycleSelection = 1;
          neutralCycleCount = 0;
        }
        break;

      case 1:
        positiveCycleCount++;   //increment the positive media cycle count
        totalCycleCount++;      //increment the global cycle count
        if (positiveCycleCount >= positiveCycleMax)
        {
          mediaCycleSelection = 2;
          positiveCycleCount = 0;
        }
        break;

      case 2:
        neutralCycleTwoCount++;    //increment the neutral media cycle count
        totalCycleCount++;      //increment the global cycle count
        if (neutralCycleTwoCount >= neutralCycleTwoMax)
        {
          mediaCycleSelection = 3;
          neutralCycleTwoCount = 0;
        }
        break;

      case 3:
        negativeCycleCount++;   //increment the negative media cycle count
        totalCycleCount++;      //increment the global cycle count
        if (negativeCycleCount >= negativeCycleMax)
        {
          mediaCycleSelection = 0;
          negativeCycleCount = 0;
        }
        break;

      default:
        allPumpsStop();
        break;
    }

    setMultiplexerFocus(0);                                                              //set I2C multiplexer to I2C channel zero (LCD screen);
    allPumpsStop();                                                           //shut off all the pumps, for safety
    flagGoToLightCycler = false;                                              //reset the flag that triggers transition between standby and light cycler states
    return true;
  }

  return false;
}



//PRIME PUMPS STATE
void stateFunctionPrimePumps()
{
  setMultiplexerFocus(0);                                                                 //set I2C multiplexer to I2C channel zero (LCD screen);
  primePumps();                                                                //goto the primePumps function
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

bool transitionMenuToPrimePumps()
{
  if (flagGoToPrimePumps == true)
  {
    setMultiplexerFocus(0);
    allPumpsStop();
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
    heaterCompute();
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
    ir = lum >> 16;                                         //set lower 16bits from TSL2591 sensor to variable IR
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
    lcd.print("Motor Steps:");
    lcd.print(neutralMediaRotationStepsForward);

    if (BtnUp.released() == true)
  {
    neutralMediaRotationStepsForward = neutralMediaRotationStepsForward + 10;
  }

  if (BtnDown.released() == true)
  {
    neutralMediaRotationStepsForward = neutralMediaRotationStepsForward - 10; 
  }

  if (BtnSelect.released() == true)
  {
    for (int dispensations = 1; dispensations <= 10; dispensations++)
    {
    neutralMediaFWD(); 
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
    lcd.print("Motor Steps:");
    lcd.print(wasteRotationStepsForward);

    if (BtnUp.released() == true)
  {
    wasteRotationStepsForward = wasteRotationStepsForward + 10;
  }

  if (BtnDown.released() == true)
  {
    wasteRotationStepsForward = wasteRotationStepsForward - 10; 
  }

  if (BtnSelect.released() == true)
  {
    for (int dispensations = 1; dispensations <= 10; dispensations++)
    {
    wasteFWD(); 
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
    lcd.print("Motor Steps:");
    lcd.print(positiveMediaRotationStepsForward);

    if (BtnUp.released() == true)
  {
    positiveMediaRotationStepsForward = positiveMediaRotationStepsForward + 10;
  }

  if (BtnDown.released() == true)
  {
    positiveMediaRotationStepsForward = positiveMediaRotationStepsForward - 10; 
  }

  if (BtnSelect.released() == true)
  {
    for (int dispensations = 1; dispensations <= 10; dispensations++)
    {
    positiveMediaFWD(); 
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
    lcd.print("Motor Steps:");
    lcd.print(negativeMediaRotationStepsForward);

    if (BtnUp.released() == true)
  {
    negativeMediaRotationStepsForward = negativeMediaRotationStepsForward + 10;
  }

  if (BtnDown.released() == true)
  {
    negativeMediaRotationStepsForward = negativeMediaRotationStepsForward - 10; 
  }

  if (BtnSelect.released() == true)
  {
    for (int dispensations = 1; dispensations <= 10; dispensations++)
    {
    negativeMediaFWD(); 
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



//---------------------------------------CORE FUNCTIONS----------------------------------------



void intializePumps()
{
  motorNeutralMedia.connectToPins(motorNeutralMediaStepPin, motorNeutralMediaDirectionPin);
  motorWaste.connectToPins(motorWasteStepPin, motorWasteDirectionPin);
  motorPositiveMedia.connectToPins(motorPositiveMediaStepPin, motorPositiveMediaDirectionPin);
  motorNegativeMedia.connectToPins(motorNegativeMediaStepPin, motorNegativeMediaDirectionPin);
}



void primePumps()
{
  setMultiplexerFocus(0);

  if (BtnSelect.pushed() == 1)
  {
    pumpSelection++;
    if (pumpSelection > 3)
    {
      pumpSelection = 0;
    }
  }

  if (BtnUp.pushed() == true)
  {
    switch (pumpSelection)
    {
      case 0:
        neutralMediaREV();
        break;
      case 1:
        wasteREV();
        break;
      case 2:
        positiveMediaREV();
        break;
      case 3:
        negativeMediaREV();
        break;
      default:
        allPumpsStop();
        break;
    }
  }

  if (BtnUp.released() == true)
  {
    allPumpsStop();
  }

  if (BtnDown.pushed() == true)
  {
    switch (pumpSelection)
    {
      case 0:
        neutralMediaFWD();
        break;
      case 1:
        wasteFWD();
        break;
      case 2:
        positiveMediaFWD();
        break;
      case 3:
        negativeMediaFWD();
        break;
      default:
        allPumpsStop();
        break;
    }
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
    case 0:
      lcd.print("NEU");
      break;
    case 1:
      lcd.print("WST");
      break;
    case 2:
      lcd.print("POS");
      break;
    case 3:
      lcd.print("NEG");
      break;
    default:
      lcd.print("ERR");
      break;
  }
}



void heaterCompute()
{
  mediaTemp = mediaTempThermistorSmooth->readCelsius();
  heaterPlateTemp = heaterPlateTempThermistorSmooth->readCelsius();
  ambientTemp = rtc.getTemperature();

  double gap = abs((setpointTemp + setpointOffset) - mediaTemp); //distance away from setpoint

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



void allPumpsReverse()
{
  wasteREV();
  neutralMediaREV();
  positiveMediaREV();
  negativeMediaREV();
}



void allPumpsStop()
{
  wasteSTOP();
  neutralMediaSTOP();
  positiveMediaSTOP();
  negativeMediaSTOP();
}



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



void wasteFWD()
{
  motorWaste.setSpeedInStepsPerSecond(500);
  motorWaste.setAccelerationInStepsPerSecondPerSecond(65000);
  motorWaste.moveRelativeInSteps(wasteRotationStepsForward); 
}



void wasteREV()
{
  motorWaste.setSpeedInStepsPerSecond(500);
  motorWaste.setAccelerationInStepsPerSecondPerSecond(65000);
  motorWaste.moveRelativeInSteps(wasteRotationStepsReverse);      //double the time it takes for any dispensing motor; prevents overflow and keeps media volume constant
}



void wasteSTOP()
{
  motorWaste.setSpeedInStepsPerSecond(0);
  motorWaste.setAccelerationInStepsPerSecondPerSecond(65000);
  motorWaste.moveRelativeInSteps(0);
}



void neutralMediaFWD()
{
  motorNeutralMedia.setSpeedInStepsPerSecond(500);
  motorNeutralMedia.setAccelerationInStepsPerSecondPerSecond(65000);
  motorNeutralMedia.moveRelativeInSteps(neutralMediaRotationStepsForward);
}



void neutralMediaREV()
{
  motorNeutralMedia.setSpeedInStepsPerSecond(500);
  motorNeutralMedia.setAccelerationInStepsPerSecondPerSecond(65000);
  motorNeutralMedia.moveRelativeInSteps(neutralMediaRotationStepsReverse);
}



void neutralMediaSTOP()
{
  motorNeutralMedia.setSpeedInStepsPerSecond(0);
  motorNeutralMedia.setAccelerationInStepsPerSecondPerSecond(65000);
  motorNeutralMedia.moveRelativeInSteps(0);
}



void positiveMediaFWD()
{
  motorPositiveMedia.setSpeedInStepsPerSecond(500);
  motorPositiveMedia.setAccelerationInStepsPerSecondPerSecond(65000);
  motorPositiveMedia.moveRelativeInSteps(positiveMediaRotationStepsForward);
}



void positiveMediaREV()
{
  motorPositiveMedia.setSpeedInStepsPerSecond(500);
  motorPositiveMedia.setAccelerationInStepsPerSecondPerSecond(65000);
  motorPositiveMedia.moveRelativeInSteps(positiveMediaRotationStepsReverse);
}



void positiveMediaSTOP()
{
  motorPositiveMedia.setSpeedInStepsPerSecond(0);
  motorPositiveMedia.setAccelerationInStepsPerSecondPerSecond(65000);
  motorPositiveMedia.moveRelativeInSteps(0);
}



void negativeMediaFWD()
{
  motorNegativeMedia.setSpeedInStepsPerSecond(500);
  motorNegativeMedia.setAccelerationInStepsPerSecondPerSecond(65000);
  motorNegativeMedia.moveRelativeInSteps(negativeMediaRotationStepsForward); //FOURTH MOTOR BACKWARDS(?)
}



void negativeMediaREV()
{
  motorNegativeMedia.setSpeedInStepsPerSecond(500);
  motorNegativeMedia.setAccelerationInStepsPerSecondPerSecond(65000);
  motorNegativeMedia.moveRelativeInSteps(negativeMediaRotationStepsReverse);
}



void negativeMediaSTOP()
{
  motorNegativeMedia.setSpeedInStepsPerSecond(0);
  motorNegativeMedia.setAccelerationInStepsPerSecondPerSecond(65000);
  motorNegativeMedia.moveRelativeInSteps(0);
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
    DateTime now = rtc.now();                                 // saves date and time right before writing to memory card so it's as precise as possible to SD card
    currentUnixTime = now.unixtime();
    upTime = now.unixtime() - robotStartTime;
    currentProgramTime = now.unixtime() - programStartTime;
    csvFile.print(currentUnixTime);                             // saves unix time value as single 64-bit integer for easy seconds counting to SD card
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

    csvFile.print(dilutionDuration);
    csvFile.print(',');

    csvFile.print(dilutionDurationChange);
    csvFile.print(',');

    csvFile.print(neutralMediaDispenseCount);
    csvFile.print(',');

    csvFile.print(cycleDuration);
    csvFile.print(',');

    csvFile.print(neutralCycleCount);
    csvFile.print(',');

    csvFile.print(positiveCycleCount);
    csvFile.print(',');

    csvFile.print(neutralCycleTwoCount);
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

    digitalWrite(LED_BUILTIN, LOW);            // turn off SD card writing cycle signal light

    csvFile.close();                                           // close SD card file
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
    configFile.println("setpointTemp=" + String(setpointTemp, 2));
    configFile.println("OD940UpperBound=" + String(OD940UpperBound, 4));
    configFile.println("OD940LowerBound=" + String(OD940LowerBound, 4));
    configFile.println("OD940LEDBrightness=" + String(OD940LEDBrightness));
    configFile.println("motorStirringSpeed=" + String(motorStirringSpeed));
    configFile.println("neutralMediaRotationStepsForward=" + String(neutralMediaRotationStepsForward));
    configFile.println("positiveMediaRotationStepsForward=" + String(positiveMediaRotationStepsForward));
    configFile.println("negativeMediaRotationStepsForward=" + String(negativeMediaRotationStepsForward));
    configFile.println("wasteRotationStepsForward=" + String(wasteRotationStepsForward));
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
      //Serial.println(key);
      //Serial.println(value);
      if (key == "setpointTemp") {
        setpointTemp = value.toFloat();
      } else if (key == "OD940UpperBound") {
        OD940UpperBound = value.toFloat();
      } else if (key == "OD940LowerBound") {
        OD940LowerBound = value.toFloat();
      } else if (key == "OD940LEDBrightness") {
        OD940LEDBrightness = value.toInt();
      } else if (key == "motorStirringSpeed") {
        motorStirringSpeed = value.toInt();
      } else if (key == "neutralMediaRotationStepsForward") {
        neutralMediaRotationStepsForward = value.toInt();
      } else if (key == "positiveMediaRotationStepsForward") {
        positiveMediaRotationStepsForward = value.toInt();
      } else if (key == "negativeMediaRotationStepsForward") {
        negativeMediaRotationStepsForward = value.toInt();
      }  else if (key == "wasteRotationStepsForward") {
          wasteRotationStepsForward = value.toInt();
        }
      }
    configFile.close();                                           // close SD card file
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



void configureLightSensor(void)
{
  setMultiplexerFocus(2);
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  //tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain

  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)
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
  lcd.print(currentOD940, 4);                   //display OD940 value to 4 decimal places (7 is max on most AVR chips)
}