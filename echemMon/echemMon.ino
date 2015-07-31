

/* Monitor code for echem328 toxic gas sensor board.
Released under Creative Commons SA-BY 4.0 license
written by Ken McGary (ken@circuitsci.com), Elliott Dicus and Peter Sand (manylabs.org)
echemMon_v0_0  3/19/2015 Initial Release.
echemMon_v0_1  5/3/2015  Add many new features, see circuitsci.com/wiki/ for more info.

Compiling and uploading this Arduino sketch requires downloading and installing two other libraries:
https://github.com/JonHub/Filters enables the real-time filtering and statistics features of this software
https://github.com/ycheneval/Adafruit_ADS1X15 is a forked version of an AdaFruit library with better differential-mode operation)
*/

//************************************************************************************
//*                                                                                  *
//*                           #defines and #includes                                 *                             
//*                                                                                  *
//************************************************************************************
//#define USE_HIH // use Honeywell HIH-type rH/temp sensor

//#include <EEPROMStore.h>
//#include <avr/wdt.h>
#include <Wire.h>
#include <EEPROM.h>

//#include <FilterDerivative.h>
//#include <FilterOnePole.h>
//  library for real-time signal filtering
// http://playground.arduino.cc/Code/Filters
#include <Filters.h>
#include <FilterTwoPole.h>
#include <FloatDefine.h>
#include <RunningStatistics.h>

#include "AQSettings.h"
#include "AQSensorValues.h"

/* Echem Includes */
#include <Adafruit_ADS1015.h>
#include "echem.h"

/* Settings */
AQSettings g_settings;

//************************************************************************************
//*                                                                                  *
//*                           #define DEFAULTs                                       *                             
//*                                                                                  *
//************************************************************************************
// in case cal constants aren't available, or seem corrupted, or when board is first used, 
// or when user chooses to load defaults via menu command.
#define DEFAULT_DATA_SET_ID 001        // this value is appended to CSV files and other output streams and can be changed via the monitor commands
#define DEFAULT_READ_INTERVAL 2000     // milliseconds between each sensor reading
#define DEFAULT_STARTUP_DELAY 15       // seconds bebefore sampling routine starts (to allow setup of terminal software, tc)
#define DEFAULT_TRIGGER_MODE 1         // 0, = no triggering, 1 = continuous interval, 2
#define DEFAULT_TRIGGER_QUALIFY 0      // 0, = no external TTL triggering, 1 -127 = positive trigger debounced, -126-1 = negative polarity triggerred and debounced
#define DEFAULT_OFFSET -50.0           // * 1e-9   - typical number in case cal constants aren't available
#define DEFAULT_SENSITIVITY 500.0      // * 1e-9 -typical number in case cal constants aren't available
#define DEFAULT_SENSOR_TYPE 1          // * Alphasense CO-B4
#define DEFAULT_LOAD_RESISTOR 2        // AFE Rload setting 0,1,2,3 corresponding to 10, 33, 50, 100 ohms
#define DEFAULT_GAIN_RESISTOR 0        // AFE Gain Resistor setting , 0 = external
#define DEFAULT_AFE_BIAS 0             // * AFE Bias, normally zero except for NO2 sensor and some other oddballs
#define DEFAULT_AFE_BIAS_POLARITY 1    // * 1 = positive, 0 = neg
#define DEFAULT_AUX_OFFSET 0.0         // * mV typical number, in case cal constants aren't available
#define DEFAULT_ADC_OFFSET_0 0.0       // * mV typical number when adcGain = 0, in case cal constants aren't available
#define DEFAULT_ADC_OFFSET_1 0.0       // * mV typical number when adcGain = 1, in case cal constants aren't available
#define DEFAULT_ADC_OFFSET_2 0.0       // * mV typical number when adcGain = 2, in case cal constants aren't available
#define DEFAULT_ADC_OFFSET_3 0.0       // * mV typical number when adcGain = 3, in case cal constants aren't available
#define DEFAULT_ADC_GAIN 1             // ADC Gain setting, 0-5 corresponding to x2/3, x1, x2, x4, x8, x16
#define DEFAULT_AVG_NUMBER 8           // typical number in case cal constants aren't available
#define DEFAULT_AVG_DELAY 100          // milliseconds - typical number in case cal constants aren't available
#define DEFAULT_FILTER_FREQUENCY 0.02  // Filter frequency in Hz for two-pole lowpass filter
#define DEFAULT_FILTER_Q 0.707           // Filter Q factor, 0.5 = critically damped, 0.707 = Butterworth, 0.577 = Bessel, can affect freq as well
                                       // http://en.wikipedia.org/wiki/Q_factor
#define DEFAULT_GAS_TEMP 99            // > 50 switches to on-board xtemp analog temperature sensor, <50 replaces tempC for temp compensation euqations
#define DEFAULT_BOARD_NUMBER 0         // * -typical number in case cal constants aren't available
#define DEFAULT_SENSOR_NUMBER 0        // *  -typical number in case cal constants aren't available
#define TIA_X        200.0             // in Kohms for AUX pin amplifier scaling

const int default_data_fields [] = { 1, 1, 2, 2, 2, 2, 2, 3, 3, 4,   4, 0, 0, 0, 3, 0, 0, 0, 0, 0,   0, 0, 0, 0 }; 

//************************************************************************************
//*                                                                                  *
//*                           define operating parameters                            *                             
//*                                                                                  *
//************************************************************************************
// This could be anything except FF (the default when fresh). This should be
// incremented if the defaults are changed.
#define VERIFICATION_CHECK_VALUE 0x01 // this should/can be incremented along with software version number, will reload defaults 
                                      //if not matching previously stored value
/* Command Processing */
#define INPUT_BUFFER_LENGTH 21 // maybe more than we need?
#define DATA_FIELDS_ARRAY_SIZE 23 // 24 elements, starting at 0
#define MAX_COMMAND_ARGS 2
#define MAX_READ_INTERVAL 600000 // = 60000 milliseconds 600 seconds = 10 minutes
#define MAX_STARTUP_DELAY 600 // = 600 seconds = 10 minutes
#define MAX_AVG_NUMBER 5000 // milliseconds
#define MAX_AFE_BIAS 13 // 0x0B, max allowed by LMP91000, see datasheet
#define MAX_AVG_DELAY 5000 // milliseconds
#define MAX_DECIMAL_PLACES 6 // for serial prints of floats
#define DEFAULT_DECIMAL_PLACES 2 // default decimal places if data_fields [] input is out of range
#define MIN_READ_INTERVAL 500 // milliseconds
#define MIN_TRIG_QUAL -100
#define MAX_TRIG_QUAL 100

//************************************************************************************
//*                                                                                  *
//*                           define global variables & constants                    *                             
//*                                                                                  *
//************************************************************************************
char g_inputBuffer[ INPUT_BUFFER_LENGTH ];
int g_inputIndex = 0;
char charBuf = 0;
int tempIndex = 0;
int loopIndex = 0;
int intBuf = 0;
boolean triggerFlag = 0;
int memTest  = 0;

// Commonly Used Flash Strings
#define PGMSTR(x) (__FlashStringHelper*)(x)
const char flash_menu_separator[] PROGMEM  = "## =====";
const char flash_invalid[] PROGMEM  = "#! Invalid: ";
const char flash_success[] PROGMEM  = "## Success";
const char comma_space[] PROGMEM  = ", ";
const char pound_space[] PROGMEM  = "# ";

byte fetch_humidity_temperature(unsigned int *p_Humidity, unsigned int *p_Temperature); // for optional HIH sensor under echem socket

unsigned long loopCount = 1;        // start at the beginning
unsigned long sampleDelay = 2000;   // initial value, updated by values stored in EEPROM
unsigned long time = 0;             // what it is
unsigned long sampleTime = 0;       // exact MCU time in milliseconds that a sensor value was read
unsigned long nextTime = 0;         // the next time to take a reading in MCU milliseconds

float gasTemp = 0;
float printTemp = 0;
float windowLength = 1;
float filteredPPM = 0;
float filteredAUX = 0;
float fixedAUX = 0;
float filteredRAW = 0;
float ppmComp = 0;
float auxppm = 0;
float auxOffset = -0.75;
//  RunningStatistics inputStats;                 
//  RunningStatistics auxStats;                 

    AQSensorValues currentSensorValues;
FilterTwoPole rawFilter;
FilterTwoPole ppmFilter;
FilterTwoPole auxFilter;
RunningStatistics ppmFilterStats;    // create statistics to look at the main filtered signal output 
RunningStatistics auxFilterStats;    // create statistics to look at the aux filtered output 
RunningStatistics rawFilterStats;    // create statistics to look at the aux filtered output 


//    AQSensorValues currentSensorValues;
//************************************************************************************
//*                                                                                  *
//*                                   setup()                                        *                             
//*                                                                                  *
//************************************************************************************
void setup(void) {
  Serial.begin(9600);  // start serial port to receive commands and send sensor data
  Wire.begin();                       // initialize the I2C functions

  // We have to use this before we call analogRead, otherwise we'd short AREF to
  // the active reference voltage which could damage the microcontroller. 
  // Requires jumper from AREF to VREF (2.048V) on PX connector.
  analogReference(EXTERNAL); 
  //set up digital output status/LED pins
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(3,INPUT_PULLUP);



//  Serial.print("# freeRAM = "); Serial.println(freeRam());
  loadSettings();
  initEchem();
  windowLength = 20.0/g_settings.filterFrequency;
  g_settings.gasTemp = DEFAULT_GAS_TEMP;

  // set up filter
  rawFilter.setAsFilter( LOWPASS_BUTTERWORTH, DEFAULT_FILTER_FREQUENCY );
  ppmFilter.setAsFilter( LOWPASS_BUTTERWORTH, DEFAULT_FILTER_FREQUENCY );
  auxFilter.setAsFilter( LOWPASS_BUTTERWORTH, DEFAULT_FILTER_FREQUENCY );
  float tempQ = g_settings.filterQfactor;
  float tempF = g_settings.filterFrequency;
  ppmFilter.setFrequency0( tempF );
  ppmFilter.setQ( tempQ );
  rawFilter.setFrequency0( tempF );
  rawFilter.setQ( tempQ );
  auxFilter.setFrequency0( tempF );
  auxFilter.setQ( tempQ );
  ppmFilterStats.setWindowSecs( windowLength );
  ppmFilterStats.setInitialValue( 0 );
  auxFilterStats.setWindowSecs( windowLength );
  auxFilterStats.setInitialValue( 0 );
  rawFilterStats.setWindowSecs( windowLength );
  rawFilterStats.setInitialValue( 0 );
  
  
  time = millis();
  while ((millis() - time) < (g_settings.startupDelay*1000)) { 
    // wait for operator to get serial term open, convert to milliseconds
    checkSerialCommands(); // read and execute incoming serial commands, if any
  }

  if (g_settings.triggerMode == 1 || g_settings.triggerMode == 3) { // if  verbose Interval or Serial Trigger mode
    Serial.println( F(" # Hello from the echem328!  More info at circuitsci.com"));  
    // all comment outputs start with "# "
    Serial.println(F("# Setup"));
    if(!settingsVerified()){
      Serial.println( F("# Using defaults") );  
      // start with # comment delimiter for KST and other csv-based graphing packages
      loadDefaultSettings();
    }
    else {
      Serial.println( F( "# Settings loaded") );  
      // start with # comment delimiter for KST and other csv-based graphing packages
    }
  printCalibration();
  printFieldLabels();
  }
 
  nextTime = millis() + g_settings.readInterval;
//  if (g_settings.readInterval > MIN_READ_INTERVAL) {
//    sampleDelay = g_settings.readInterval; // minimum milliseconds between each sensor read operation
//  }
} //                                                                                 *
//************************************************************************************

//************************************************************************************
//*                                                                                  *
//*                           MAIN  loop()                                           *                             
//*                                                                                  *
//************************************************************************************
// here we go, again and again forever, until reset or something breaks.             *
void loop(void) { //                                                                 *
  //  time = millis();
  // nextTime = millis() + g_settings.readInterval;
  /*  if (nextTime == 0) { // first time through
    nextTime = millis() + g_settings.readInterval;
  }*/
  if (g_settings.triggerMode == 0 || g_settings.triggerMode == 1) { // if 0 quiet mode or 1 verbose Interval Trigger mode
    // wait here for next sample time
    while (millis() < nextTime) {
      checkSerialCommands(); // read incoming serial commands, if any
    }
    nextTime = millis() + g_settings.readInterval;
    sampleTime = millis();
    digitalWrite( 7, HIGH );  // read cycle indicator for external trigger and/or LED
    processSensors( currentSensorValues );
    digitalWrite( 7, LOW );
    triggerFlag = 0;
    printSensors( currentSensorValues );
    loopCount++;  // increment loop counter
  }
  int trigWait = 0;
  int trigPolarity = 0;
  if (g_settings.triggerMode == 2 || g_settings.triggerMode == 3) { // if 2 quiet mode or 3 verbose Serial Trigger mode
    // wait here for next sample time
    if (g_settings.triggerQualify != 0) {
      if ( g_settings.triggerQualify < 0) {
        trigWait = abs(g_settings.triggerQualify);
        trigPolarity = 0;
        while (digitalRead(3) == 1) {
          checkSerialCommands(); // read incoming serial commands
        }
        delay (trigWait);
        if (digitalRead(3) == 0) {
          triggerFlag = 1;
        }
        else {
          triggerFlag = 0;
          return;
        }
       }  
      if ( g_settings.triggerQualify > 0) {
        trigWait = g_settings.triggerQualify;
        trigPolarity = 1;
        while (digitalRead(3) == 0) {
          checkSerialCommands(); // read and execute incoming serial commands, if any
        }
        delay (trigWait);
        if (digitalRead(3) == 1) {
          triggerFlag = 1;
        }  
        else {
          triggerFlag = 0;
          return;
        }
      }  
    }
    sampleTime = millis();
    digitalWrite( 7, HIGH );  // read cycle indicator for external trigger and/or LED
    processSensors( currentSensorValues );
    digitalWrite( 7, LOW );
    triggerFlag = 0;
    printSensors( currentSensorValues );
    loopCount++;  // increment loop counter
  }
}
//                                                                                   *
//************************************************************************************

//************************************************************************************
//*                                                                                  *
//*                             processSensors()                                     *                             
//*                                                                                  *
//************************************************************************************

// Takes an AQSensorValues object and fills it with the current sensor values
void processSensors( AQSensorValues &sensorValues ) {
  // Temp / Humidity
  #ifdef USE_HIH
    uint8_t status = readTemperatureAndHumidity( sensorValues.temperature,
      sensorValues.humidity);
    if(status){
      Serial.println( F("# HIH6121 ERR:") );
      Serial.println( status );
    }
  #endif
  // get sensor and diagnostic values
  readEchem( sensorValues.tempC, sensorValues.tempDelta,
    sensorValues.ppmraw, sensorValues.ppm, sensorValues.sensorCurrent,
    sensorValues.volt1diff, sensorValues.volt2diff, sensorValues.volt3 );

  // Echem power supply diagnostic voltages @ hardwired MCU analog inputs - 10-bit resolution, 2.048V reference, v divider ratio 7.66
  sensorValues.echemVdd = (((float)analogRead(6) / 1024.0) * 2.048) * 7.666;
  sensorValues.echemVin = (((float)analogRead(7) / 1024.0) * 2.048) * 7.666;
  ppmFilter.input( sensorValues.ppm );
  auxFilter.input( sensorValues.volt1diff );
  rawFilter.input( sensorValues.ppmraw );
  filteredPPM = ppmFilter.output();
  filteredAUX = auxFilter.output();
  filteredRAW = rawFilter.output();
  ppmFilterStats.input( filteredPPM );
  rawFilterStats.input( filteredRAW );
  auxFilterStats.input( filteredAUX );

  float cgain_per_ppm = g_settings.sensorSensitivity * 1e-3; // microamps per ppm
  
// experimental "compensated" output using AUX pin for zero offset compensation
// correction factors belong here depending on sensor sgas species per Alphasense app note AAN803
//   fixedAUX = filteredAUX
/*  if ( sensorValues.tempDelta < 30 ) {
    fixedAUX = filteredAUX;
  }
  else {
    fixedAUX = ( filteredAUX * 3.8 );
  }
  */
//  auxppm = ( fixedAUX / TIA_X ) * cgain_per_ppm; // Aux Electrode current in ppm considering sensitivity 
//   ppmComp = ( filteredRAW - auxppm );  
 

  }
//                                                                                   *
//                                                                                   *
//************************************************************************************


//***************************** PRINT ROUTINES ***************************************
//                                                                                   *
//************************************************************************************
//*                                                                                  *
//*                             printSensors()                                       *                             
//*                                                                                  *
//************************************************************************************
//                                                                                   *
//                                                                                   *
// Takes an AQSensorValues object. Print the sensor values, 1 per line, to the
// serial port
void printSensors( const AQSensorValues &sensorValues ) {

// print raw a/d and processed values to serial port
  if (g_settings.data_fields [ 0 ] > 0) {
    Serial.print(loopCount);// 
  }
  Serial.print ( PGMSTR(comma_space) );  
  if (g_settings.data_fields [ 1 ] > 0) {
     Serial.print(sampleTime);
  }
  Serial.print ( PGMSTR(comma_space) );  
  if (g_settings.data_fields [ 2 ] > 0) {
     Serial.print(sensorValues.echemVin, g_settings.data_fields [ 2 ]);
  }
  Serial.print ( PGMSTR(comma_space) );  
  if (g_settings.data_fields [ 3 ] > 0) {
     Serial.print(sensorValues.echemVdd, g_settings.data_fields [ 3 ]);
  }
  Serial.print ( PGMSTR(comma_space) );  
  if (g_settings.data_fields [ 4 ] > 0) { // aux raw a/d signal
    Serial.print(sensorValues.volt1diff, g_settings.data_fields [ 4 ]); 
  }
  Serial.print ( PGMSTR(comma_space) );  
  if (g_settings.data_fields [ 5 ] > 0) { // main Working Electrode raw a/d signal
     Serial.print(sensorValues.volt2diff, g_settings.data_fields [ 5 ]); 
  }
  Serial.print ( PGMSTR(comma_space) );  
  if (g_settings.data_fields [ 6 ] > 0) { // AFE internal reference (half of board ref or 1.024V)
     Serial.print(sensorValues.volt3, g_settings.data_fields [ 6 ]); 
  }
  Serial.print ( PGMSTR(comma_space) );  
  if (g_settings.data_fields [ 7 ] > 0) {
 /*   if ( g_settings.gasTemp < 50 ) {
      printTemp = gasTemp;
    }  
    else {
      printTemp = sensorValues.tempDelta;
    }
*/
     Serial.print(sensorValues.tempC, g_settings.data_fields [ 7 ]); 
    // tempDelta is holdover from AQ Station code, is actually xtemp here 
    // or echem xtemp sensor value as read from a/d channel 0 
  }
  Serial.print ( PGMSTR(comma_space) );                                                              
  if (g_settings.data_fields [ 8 ] > 0) {
     Serial.print(sensorValues.ppmraw, g_settings.data_fields [ 8 ]);
  }
  Serial.print ( PGMSTR(comma_space) );  
  if (g_settings.data_fields [ 9 ] > 0) { 
     Serial.print(sensorValues.ppm, g_settings.data_fields [ 9 ]);
  }
  Serial.print ( PGMSTR(comma_space) );  
  if (g_settings.data_fields [ 10 ] > 0) { 
     Serial.print(filteredPPM, g_settings.data_fields [ 10 ]);
  }
  Serial.print ( PGMSTR(comma_space) );  
  if (g_settings.data_fields [ 11 ] > 0) { 
     Serial.print(ppmFilterStats.mean(), g_settings.data_fields [ 11 ]);
  }
  Serial.print ( PGMSTR(comma_space) );  
  if (g_settings.data_fields [ 12 ] > 0) { 
     Serial.print(ppmFilterStats.variance(), g_settings.data_fields [ 12 ]);
  }
  Serial.print ( PGMSTR(comma_space) );  
  if (g_settings.data_fields [ 13 ] > 0) { 
     Serial.print(ppmFilterStats.sigma(), g_settings.data_fields [ 13 ]);
  }
  Serial.print ( PGMSTR(comma_space) );  
  if (g_settings.data_fields [ 14 ] > 0) { 
     Serial.print(filteredAUX, g_settings.data_fields [ 14 ]);
  }
  Serial.print ( PGMSTR(comma_space) );  
  if (g_settings.data_fields [ 15 ] > 0) { 
     Serial.print(auxFilterStats.mean(), g_settings.data_fields [ 15 ]);
  }
  Serial.print ( PGMSTR(comma_space) );  
  if (g_settings.data_fields [ 16 ] > 0) { 
     Serial.print(auxFilterStats.variance(), g_settings.data_fields [ 16 ]);
  } 
  Serial.print ( PGMSTR(comma_space) ); 
  if (g_settings.data_fields [ 17 ] > 0) { 
     Serial.print(auxFilterStats.sigma(), g_settings.data_fields [ 17 ]);
  }
  Serial.print ( PGMSTR(comma_space) );  
  if (g_settings.data_fields [ 18 ] > 0) { 
     Serial.print(filteredRAW, g_settings.data_fields [ 18 ]);
  }
  Serial.print ( PGMSTR(comma_space) );  
  if (g_settings.data_fields [ 19 ] > 0) { 
     Serial.print(g_settings.gasTemp, g_settings.data_fields [ 19 ]);
  }
  Serial.print ( PGMSTR(comma_space) );  
  if (g_settings.data_fields [ 20 ] > 0) { 
     Serial.print((((float)analogRead(0) / 1024.0) * 2.048), g_settings.data_fields [ 20 ]);
  }
  Serial.print ( PGMSTR(comma_space) );  
  if (g_settings.data_fields [ 21 ] > 0) { 
     Serial.print((((float)analogRead(1) / 1024.0) * 2.048), g_settings.data_fields [ 21 ]);
  }
  Serial.print ( PGMSTR(comma_space) );  
  if (g_settings.data_fields [ 22 ] > 0) { 
     Serial.print((((float)analogRead(2) / 1024.0) * 2.048), g_settings.data_fields [ 22 ]);
  }
  Serial.print ( PGMSTR(comma_space) );  
  if (g_settings.data_fields [ 23 ] > 0) { 
     Serial.print((((float)analogRead(3) / 1024.0) * 2.048), g_settings.data_fields [ 23 ]);
  }  
//Serial.print ( PGMSTR(comma_space) ); Serial.print ( ppmComp,4 );
//Serial.print ( PGMSTR(comma_space) ); Serial.print ( auxppm,4 );

Serial.println(""); // final carriage return
}

//************************************************************************************
//*                                                                                  *
//*                             printCalibration()                                   *                             
//*                                                                                  *
//************************************************************************************
// Prints the current calibration/settings & values
void printCalibration() {
  Serial.print( F("# dataSetId: ") ); Serial.println(g_settings.dataSetId);
  Serial.print( F("# boardNumber: ") ); Serial.println(g_settings.boardNumber);
  Serial.print( F("# sensorNumber: ") ); Serial.println(g_settings.sensorNumber);
  Serial.print( F("# version: ") ); Serial.println(g_settings.verification_check, HEX);
  Serial.print( F("# freeRAM = ") ); Serial.println(freeRam());

  Serial.print( F("# avgNumber: ") ); Serial.println(g_settings.avgNumber);
  Serial.print( F("# avgDelay: ") ); Serial.println(g_settings.avgDelay);
  Serial.print( F("# readInterval: ") ); Serial.println(g_settings.readInterval);
  Serial.print( F("# startupDelay: ") ); Serial.println(g_settings.startupDelay);
  Serial.print( F("# triggerMode: ") ); Serial.println(g_settings.triggerMode);
  Serial.print( F("# triggerQualify: ") ); Serial.println(g_settings.triggerQualify);

  Serial.print( F("# filterFrequency = ") ); Serial.println (g_settings.filterFrequency);
  Serial.print( F("# filterQfactor = ") ); Serial.println (g_settings.filterQfactor);
  Serial.print( F("# windowLength = ") ); Serial.println (windowLength);

  Serial.print( F("# afeLoadResistor: ") ); Serial.println(g_settings.afeLoadResistor);
  Serial.print( F("# afeGainResistor: ") ); Serial.println(g_settings.afeGainResistor);
  Serial.print( F("# afeBias: ") ); Serial.println(g_settings.afeBias);
  Serial.print( F("# afeBiasPolarity: ") ); Serial.println(g_settings.afeBiasPolarity);

  Serial.print( F("# auxOffset: ") ); Serial.println(g_settings.auxOffset, 2);
  Serial.print( F("# adcOffset(0-3): ") ); Serial.print(g_settings.adcOffset0, 2);
  Serial.print( PGMSTR(comma_space) ); Serial.print(g_settings.adcOffset1, 2);
  Serial.print( PGMSTR(comma_space) ); Serial.print(g_settings.adcOffset2, 2);
  Serial.print( PGMSTR(comma_space) ); Serial.println(g_settings.adcOffset3, 2);
  Serial.print( F("# adcGain: ") ); Serial.println(g_settings.adcGain);

  Serial.print( F("# sensorType: ") ); Serial.println(g_settings.sensorType);
  Serial.print( F("# sensorSensitivity: ") ); Serial.println(g_settings.sensorSensitivity, 2);
  Serial.print( F("# sensorOffset: ") ); Serial.println(g_settings.sensorOffset, 2);
  
  Serial.print( F("# sensitivity factors (-30 to +50C)  = [ ") );
  for ( tempIndex = 0; tempIndex < ( SENSITIVITY_TABLE_LENGTH ); tempIndex++ ) {
    Serial.print(g_settings.sensitivity_factor_values [ tempIndex ]);
    if (tempIndex < (SENSITIVITY_TABLE_LENGTH - 1)) {
      Serial.print (PGMSTR(comma_space));
    }  
  }
  Serial.println F((" ]") );
  Serial.print ( F("# zero current factors (-30 to +50C) = [ ") );
  for (tempIndex = 0; tempIndex < ZERO_CURRENT_TABLE_LENGTH; tempIndex++) {
    Serial.print (g_settings.zero_current_factor_values [ tempIndex ]);
    if (tempIndex < (ZERO_CURRENT_TABLE_LENGTH - 1)) {
      Serial.print (PGMSTR(comma_space));
    }  
  }
  Serial.println ( F(" ]"));

  Serial.print( F("# data_fields = [ ") );
  for (tempIndex = 0; tempIndex <= DATA_FIELDS_ARRAY_SIZE; tempIndex++) {
    Serial.print (g_settings.data_fields [ tempIndex ]);
    if (tempIndex < (DATA_FIELDS_ARRAY_SIZE)) {
      Serial.print (PGMSTR(comma_space));
      if (tempIndex == 9 || tempIndex ==19) { // add extra space in blocks of 10 for better readability
        Serial.print ( F("   ") );
      }  
    }  
  }
  Serial.println ( F(" ]") );
  

}

//************************************************************************************
//*                                                                                  *
//*                             printMenu()                                          *                             
//*                                                                                  *
//************************************************************************************
// Prints the menu
void printMenu() {
  Serial.println( F("# 0: Show Main Menu") );
  Serial.println( F("# 1: Send Current Data Values(CSV)") );
  Serial.println( F("# 2: Show All Settings") );
  Serial.println( F("# 3: Edit General Settings") );
  Serial.println( F("# 4: Edit Sensitivity Correction Factors") );
  Serial.println( F("# 5: Edit Zero Current Correction Factors") );
  Serial.println( F("# 6: Edit Data Fields") );
  Serial.println( F("# 9: Load Defaults") );
}

//************************************************************************************
//*                                                                                  *
//*                          printFieldLabels()                                      *                             
//*                                                                                  *
//************************************************************************************
void printFieldLabels() {
    // print column headers based on data_fields array, 
    // 0 = "field not used"
    // 1 = "field used" for integer fields
    // >0 = "field used", number determines decimal places shown
    Serial.println ( F(" ") );// blank first line

    if (g_settings.data_fields [ 0 ] > 0 ) {
      Serial.print ( F("INDEX, ") );// 
    }
    if (g_settings.data_fields [ 1 ] > 0 ) {
      Serial.print ( F("TIME, ") );// 
    }  
    if (g_settings.data_fields [ 2 ] > 0 ) {
      Serial.print ( F("Vin, ") );
    }   
    if (g_settings.data_fields [ 3 ] > 0 ) {
      Serial.print ( F("Vdd, ") ); 
    }  
    if (g_settings.data_fields [ 4 ] > 0 ) {
      Serial.print ( F("Vaux, ") );
    }  
    if (g_settings.data_fields [ 5 ] > 0 ) {
      Serial.print ( F("Vwe, ") );
    }   
    if (g_settings.data_fields [ 6 ] > 0 ) {
      Serial.print ( F("Vref, ") ); 
    }
    if (g_settings.data_fields [ 7 ] > 0 ) {
      Serial.print ( F("TEMP, ") ); 
    }  
    if (g_settings.data_fields [ 8 ] > 0 ) {
      Serial.print ( F("PPMraw, ") ); 
    }  
    if (g_settings.data_fields [ 9 ] > 0 ) {
      Serial.print ( F("PPM, ") ); 
    }
    if (g_settings.data_fields [ 10 ] > 0 ) {
      Serial.print ( F("PPMf, ") ); 
    }
    if (g_settings.data_fields [ 11 ] > 0 ) {
      Serial.print ( F("PPMm, ") ); 
    }
    if (g_settings.data_fields [ 12 ] > 0 ) {
      Serial.print ( F("PPMv, ") ); 
    }
    if (g_settings.data_fields [ 13 ] > 0 ) {
      Serial.print ( F("PPMs, ") ); 
    }
    if (g_settings.data_fields [ 14 ] > 0 ) {
      Serial.print ( F("AUXf, ") ); 
    }
    if (g_settings.data_fields [ 15 ] > 0 ) {
      Serial.print ( F("AUXm, ") ); 
    }
    if (g_settings.data_fields [ 16 ] > 0 ) {
      Serial.print ( F("AUXv, ") ); 
    }
    if (g_settings.data_fields [ 17 ] > 0 ) {
      Serial.print ( F("AUXs, ") ); 
    }
    if (g_settings.data_fields [ 18 ] > 0 ) {
      Serial.print ( F("RAWf, ") ); 
    }
    if (g_settings.data_fields [ 19 ] > 0 ) {
      Serial.print ( F("gTemp, ") ); 
    }
    if (g_settings.data_fields [ 20 ] > 0 ) {
      Serial.print ( F("A0, ") ); 
    }
    if (g_settings.data_fields [ 21 ] > 0 ) {
      Serial.print ( F("A1, ") ); 
    }
    if (g_settings.data_fields [ 22 ] > 0 ) {
      Serial.print ( F("A2, ") ); 
    }
    if (g_settings.data_fields [ 23 ] > 0 ) {
      Serial.print ( F("A3") ); 
    }
//      Serial.print ( F("fRAW, ppmC, auxppm") ); // test print
  Serial.println(""); // final carriage return
}
//                                                                                   *
//                                                                                   *
//************************************************************************************


//****************************** COMMAND ROUTINES ************************************
//                                                                                   *
//************************************************************************************
//*                                                                                  *
//*                                 indexOf()                                        *                             
//*                                                                                  *
//************************************************************************************
// get index of given character; return -1 if not found
// fix(smaller): replace with strchr
inline int indexOf ( const char *str, char c ) {
  int index = 0;
  while (true) {
    char s = *str++;
    if (s == c)
      return index;
    if (s == 0)
      break;
    index++;
  }
  return -1;
}

//************************************************************************************
//*                                                                                  *
//*                             executeCommand()                                     *                             
//*                                                                                  *
//************************************************************************************
// execute an incoming serial command
boolean executeCommand( const char *command, byte argCount, char *args[] ) {


//  if (g_settings.triggerMode == 1 || g_settings.triggerMode == 3) { // if verbose mode/Interval Trigger, print menu seperator... otherwise don't
//    Serial.println( PGMSTR(flash_menu_separator) );
//  }  
  if ( command[0] == '0' ){ // Show Menu
    printMenu();
    return true;
  }

  else if ( command[0] == '1' ){ // take a reading and print values
/*
    if (g_settings.triggerMode == 2 || g_settings.triggerMode == 3)  { // Serial / external triggered modes
//       AQSensorValues currentSensorValues;
     sampleTime = millis();
digitalWrite( A0, HIGH );
      AQSensorValues currentSensorValues;
      processSensors( currentSensorValues );
digitalWrite( A0, LOW );
    }
    printSensors( currentSensorValues );
    if (g_settings.triggerMode == 2 || g_settings.triggerMode == 3)  { // Serial / external triggered modes
      loopCount++;  // increment loop counter
    }  
*/    
  triggerFlag = 1;
  return true;
  }      
  
  
  else if ( command[0] == '2' ){ // Show settings
    printCalibration();
    return true;
  }

  else if ( command[0] == '9' ){ // Load default settings
    if (argCount == 0){
      Serial.println ( F("#! Load Default Settings -- WARNING, this will overwrite all parameters except for Correction Factors!!!" ));
      Serial.println ( F("# Submit \"9:confirm\"") );
    }
    else if ( argCount == 1 && strcmp_P(args[0], PSTR("confirm")) == 0 ){
      loadDefaultSettings();
      saveSettings();
    }
    return true;
  }
  
  else if ( command[0] == '3' ){ // Edit settings
    if (argCount == 0){ // if no args, print the settings options menu
      Serial.println ( F("# Edit General Settings" ) );
      Serial.println ( F("# Submit \"3:<number>,<value>\"") );  Serial.println (PGMSTR(pound_space) );

      Serial.println ( F("# i: readInterval (ms)") );
      Serial.println ( F("# m: triggerMode (0 = Interval, 1 = Interrupt)") );
      Serial.println ( F("# o: triggerQualify (0 = no ext trigger, 1-100 = pos trigger, <0 = neg trigger milliseconds)") );
      Serial.println ( F("# e: avgDelay (ms)") );
      Serial.println ( F("# r: avgNumber") );
      Serial.println ( F("# f: filterFrequency") );
      Serial.println ( F("# q: filterQfactor") );
      Serial.println ( F("# t: gasTemp") );
      Serial.println ( F("# u: startupDelay (s)") );
      Serial.println ( F("# v: datasetId") );  Serial.println (PGMSTR(pound_space) );

      Serial.println ( F("# A: adcOffset0 (mV)") );
      Serial.println ( F("# B: adcOffset1 (mV)") );
      Serial.println ( F("# C: adcOffset2 (mV)") );
      Serial.println ( F("# D: adcOffset3 (mV)") );
      Serial.println ( F("# K: auxOffset  (mV)") ); Serial.println (PGMSTR(pound_space) );

      Serial.println ( F("# N: adcGain (0-3) = [2/3, 1, 2, 4]") );  
      Serial.println ( F("# G: afeGainResistor ((0 = external, 1-7) = [ 2.75K, 3.5K, 7K, 14K, 35K, 120K, 350K ] ohms") );
      Serial.println ( F("# L: afeLoadResistor (0-3) = [ 10, 33, 50, 100 ] ohms") );
      Serial.println ( F("# P: afeBiasPolarity: (0=neg, 1=pos)") ); 
      Serial.println ( F("# S: afeBias: (see lookup table, 0=0V/off)") );   Serial.println (PGMSTR(pound_space) );

      Serial.println ( F("# J: boardNumber") );
      Serial.println ( F("# W: sensorNumber") );Serial.println (PGMSTR(pound_space) );

      Serial.println ( F("# X: sensorSensitivity (nA/ppm)") );
      Serial.println ( F("# Y: sensorOffset (nA)") );
      Serial.println ( F("# Z: sensorType (0=custom, 1=CO, 2=H2S, 3=NO, 4=NO2, 5=O3, 6=SO2, 9=MY)") );  Serial.println (PGMSTR(pound_space) );
      return true;
    }
    else if (argCount == 2){
      // Check for the value to edit

      int charBuf = static_cast<int>(*args[0]);
      switch( charBuf ) {
 
        case 'v':
          g_settings.dataSetId = atoi(args[1]);
          Serial.print ( F("## dataSetId = ") );
          Serial.println ( g_settings.dataSetId );
          break;
        case 'J':
          g_settings.boardNumber = atoi(args[1]);
          Serial.print ( F("## boardNumber = ") );
          Serial.println ( g_settings.boardNumber);
          break;
 
        case 'i':
          g_settings.readInterval = atoi(args[1]);
   //       if (g_settings.readInterval >MAX_READ_INTERVAL) { // = 600 seconds = 10 minutes
  //          g_settings.readInterval = 2000; //  if input out of range, default = 2 seconds
  //        }
          if ((g_settings.readInterval > MAX_READ_INTERVAL) | (g_settings.avgNumber * (20 + g_settings.avgDelay) > (g_settings.readInterval - 100))) { 
            //check against valid timing parameters, adc readings take 20 ms
            Serial.print ( F("#! max = ") );
            // Serial.print ((g_settings.readInterval - 100) / (g_settings.avgNumber + 20));  // debug test
            g_settings.readInterval = DEFAULT_READ_INTERVAL; // if input out of range, load defaults
            g_settings.avgDelay = DEFAULT_AVG_DELAY; //  better error checking would be good here...
            g_settings.avgNumber = DEFAULT_AVG_NUMBER; // 
           Serial.println ( F(", defaults loaded.") );
          Serial.print ( F("## avgNumber = ") );
          Serial.println ( g_settings.avgNumber );
          Serial.print ( F("## avgDelay = ") );
          Serial.println ( g_settings.avgDelay );
          }
          Serial.print ( F("## readInterval = ") );
          Serial.println ( g_settings.readInterval );
          break;
        case 'm':
          g_settings.triggerMode = atoi(args[1]);
          if (g_settings.triggerMode >3) { 
            g_settings.triggerMode = 0; //  if input out of range, default = positive bias
            Serial.print ( F("#! limit exceeded, default loaded") );
          }
          Serial.print ( F("## triggerMode = ") );
          Serial.println ( g_settings.triggerMode );
          break;
        case 'o':
          g_settings.triggerQualify = atoi(args[1]);
          if ((g_settings.triggerQualify > MAX_TRIG_QUAL || g_settings.triggerQualify < MIN_TRIG_QUAL) && g_settings.triggerQualify != 0) { 
            g_settings.triggerQualify = 0; //  if input out of range, default = positive bias
            Serial.print ( F("#! limit exceeded, default loaded") );
          }
          Serial.print ( F("## triggerQualify = ") );
          Serial.println ( g_settings.triggerQualify );
          break;
        case 'e':
          g_settings.avgDelay = atoi(args[1]);
          if ( (g_settings.avgDelay > MAX_AVG_DELAY) | (g_settings.avgNumber * (20 + g_settings.avgDelay) > (g_settings.readInterval - 100))) { 
            //check against valid timing parameters, adc readings take 20 ms
            Serial.print ( F("#! max = ") );
            Serial.print ((g_settings.readInterval - 100) / (g_settings.avgNumber + 20));  //test
            g_settings.avgDelay = 10; // if input out of range, default = 10 milliseconds
           Serial.println ( F(", default loaded, increase readInterval or decrease avgNumber.") );
          }
          Serial.print ( F("## avgDelay = ") );
          Serial.println ( g_settings.avgDelay );
          break;
        case 'r':
          g_settings.avgNumber = atoi(args[1]);
          Serial.println (g_settings.avgNumber * (20+g_settings.avgDelay));  //test
          if ( (g_settings.avgNumber > MAX_AVG_NUMBER) | (g_settings.avgNumber * (22 + g_settings.avgDelay) > (g_settings.readInterval - 100))) {
            //check against valid timing parameters, adc readings take 20 ms
            Serial.print ( F("#! max = ") );
            Serial.print ((g_settings.readInterval - 100) / (22+g_settings.avgDelay));  
            g_settings.avgNumber = DEFAULT_AVG_NUMBER; // if input out of range, averaged samples per reading
            Serial.println ( F(", default loaded, increase readInterval or decrease avgDelay.") );
          }
          Serial.print ( F("## avgNumber = ") );
          Serial.println ( g_settings.avgNumber);
          break;
        case 'f':
          g_settings.filterFrequency = atof(args[1]);
          Serial.print ( F("## filterFrequency = ") );
          Serial.println ( g_settings.filterFrequency, 2 );
          windowLength = 20.0/g_settings.filterFrequency;
          Serial.print ( F("## windowLength = ") );
          Serial.println ( windowLength );
          break;
        case 'q':
          g_settings.filterQfactor = atof(args[1]);
          Serial.print ( F("## filterQfactor = ") );
          Serial.println ( g_settings.filterQfactor, 4 );
          break;
       case 'u':
          g_settings.startupDelay = atoi(args[1]);
          if (g_settings.startupDelay > MAX_STARTUP_DELAY) {
            g_settings.startupDelay = DEFAULT_STARTUP_DELAY; // if input out of range
            Serial.print ( F("#! limit exceeded, default loaded") );
          }
          Serial.print ( F("## startupDelay = ") );
          Serial.println ( g_settings.startupDelay );
          break;
       case 't':
          g_settings.gasTemp = atof(args[1]);
          if ((g_settings.gasTemp > 100) || (g_settings.gasTemp < -30)) {
            g_settings.startupDelay = DEFAULT_GAS_TEMP; // if input out of range
            Serial.print ( F("#! limit exceeded, default loaded") );
          }
          Serial.print ( F("## gasTemp = ") );
          Serial.println ( g_settings.gasTemp, 2 );
          break;

        case 'W':
          g_settings.sensorNumber = atol(args[1]);
          Serial.print ( F("## sensorNumber = ") );
          Serial.println ( g_settings.sensorNumber );
          break;
        case 'X':
          g_settings.sensorSensitivity = atof(args[1]);
          Serial.print ( F("## sensorSensitivity = ") );
          Serial.println ( g_settings.sensorSensitivity, 2 );
          break;
        case 'Y':
          g_settings.sensorOffset = atof(args[1]);
          Serial.print ( F("## sensorOffset = ") );
          Serial.println ( g_settings.sensorOffset, 2 );
          break;
        case 'Z':
          g_settings.sensorType = atoi(args[1]);
          initEchem();
          if (g_settings.sensorType >9) {
            g_settings.sensorType = DEFAULT_SENSOR_TYPE; //  if input out of range
            Serial.print ( F("#! limit exceeded, default loaded") );
        }
          Serial.print ( F("## sensorType = ") );
          Serial.println ( g_settings.sensorType);
          break;

        case 'A':
          g_settings.adcOffset0 = atof(args[1]);
          initEchem();
          Serial.print ( F("## adcOffset0 = ") );
          Serial.println ( g_settings.adcOffset0, 4 );
          break;
        case 'B':
          g_settings.adcOffset1 = atof(args[1]);
          initEchem();
          Serial.print ( F("## adcOffset1 = ") );
          Serial.println ( g_settings.adcOffset1, 4 );
          break;
        case 'C':
          g_settings.adcOffset2 = atof(args[1]);
           initEchem();
         Serial.print ( F("## adcOffset2 = ") );
          Serial.println ( g_settings.adcOffset2, 4 );
          break;
        case 'D':
          g_settings.adcOffset3 = atof(args[1]);
          initEchem();
          Serial.print ( F("## adcOffset3 = ") );
          Serial.println ( g_settings.adcOffset3, 4 );
          break;
        case 'K':
          g_settings.auxOffset = atof(args[1]);
          Serial.print ( F("## auxOffset = ") );
          Serial.println ( g_settings.auxOffset, 4 );
          break;
        case 'N':
          g_settings.adcGain = atoi(args[1]);
          if (g_settings.adcGain > 3) { // limited by ADS1115 as well as max input voltage ranges in this circuit, see datasheet
            g_settings.adcGain = DEFAULT_ADC_GAIN; // if input out of range
            Serial.print ( F("#! limit exceeded, default loaded ") );
          }
          initEchem();
          Serial.print ( F("## adcGain = ") );
          Serial.println ( g_settings.adcGain );
          break;
        case 'G':
          g_settings.afeGainResistor = atoi(args[1]);
          if (g_settings.afeGainResistor > 7) { // limited by LMP91000, see datasheet
            g_settings.afeGainResistor = DEFAULT_GAIN_RESISTOR; //  if input out of range
            Serial.print ( F("#! limit exceeded, default loaded ") );
          }
          LMP_CFG(); //Configure the LMP91000 AFE device
          Serial.print ( F("## afeGainResistor = ") );
          Serial.println ( g_settings.afeGainResistor );
          break;
        case 'L':
          g_settings.afeLoadResistor = atoi(args[1]);
          if (g_settings.afeLoadResistor > 3) { // limited by LMP91000, see datasheet
            g_settings.afeLoadResistor = DEFAULT_LOAD_RESISTOR; //  if input out of range
            Serial.print ( F("#! limit exceeded, default loaded ") );
          }
          LMP_CFG(); //Configure the LMP91000 AFE device
          Serial.print ( F("## afeLoadResistor = ") );
          Serial.println ( g_settings.afeLoadResistor );
          break;
        case 'P':
          g_settings.afeBiasPolarity = atoi(args[1]);
          if (g_settings.afeBiasPolarity > 1) { // limited by LMP91000, see datasheet
            g_settings.afeBiasPolarity = DEFAULT_AFE_BIAS_POLARITY; //  if input out of range
            Serial.print ( F("#! limit exceeded, default loaded ") );
          }
          initEchem();
          Serial.print ( F("## afeBiasPolarity = ") );
          Serial.println ( g_settings.afeBiasPolarity );
          break;
        case 'S':
          g_settings.afeBias = atoi(args[1]);
          if (g_settings.afeBias > MAX_AFE_BIAS) {
            g_settings.afeBias = DEFAULT_AFE_BIAS; //  if input out of range
            Serial.print ( F("#! limit exceeded, default loaded ") );
          }
          initEchem();
          Serial.print ( F("## afeBias = ") );
          Serial.println ( g_settings.afeBias );
          break;

        default:
          Serial.print ( PGMSTR(flash_invalid) );
          Serial.println (command);
          break;
      }
    saveSettings();
    return true;
    }
  } 
   
  else if( command[0] == '4' ){ // Edit sensor sensitivity correction factors

    if( argCount == 0 ){ // if no args, print the sensitivity settings menu
      Serial.println ( F("# Edit Sensor Sensitivity Correction Factors") );
      Serial.println ( F("# Submit \"4:<number>,<value>\"") );
      Serial.println ( F("# 0: -30 C") );
      Serial.println ( F("# 1: -20 C") );
      Serial.println ( F("# 2: -10 C") );
      Serial.println ( F("# 3:  0  C") );
      Serial.println ( F("# 4: +10 C") );
      Serial.println ( F("# 5: +20 C") );
      Serial.println ( F("# 6: +30 C") );
      Serial.println ( F("# 7: +40 C") );
      Serial.println ( F("# 8: +50 C") );
      return true;
    }
    else if( argCount == 2 ){
      // Check for the value to edit
      tempIndex = atoi(args[0]);
      if ( tempIndex < 9 ) {
        g_settings.sensitivity_factor_values [ tempIndex ] = atof(args[1]);
        Serial.print( F("## ")); // make it an output comment
        Serial.print ( F("sensitivity_factor[") );
        Serial.print (tempIndex);
        Serial.print ("] = ");
        Serial.println ( g_settings.sensitivity_factor_values [ tempIndex ], 4 );
        Serial.println ( F( "#! sensorType set to 0 (custom)" ));
        g_settings.sensorType = 0; // now  non-standard temp curve so set type to 0 "custom"
        saveSettings();
        return true;
      }  
    }
  }

  else if( command[0] == '5' ){ // Edit zero current correction factors

    if( argCount == 0 ){ // if no args, print the zero current settings menu
      Serial.println ( F("# Edit Zero Current Correction Factors") );
      Serial.println ( F("# Submit \"5:<number>,<value>\"") );
      Serial.println ( F("# 0: -30 C") );
      Serial.println ( F("# 1: -20 C") );
      Serial.println ( F("# 2: -10 C") );
      Serial.println ( F("# 3:  0  C") );
      Serial.println ( F("# 4: +10 C") );
      Serial.println ( F("# 5: +20 C") );
      Serial.println ( F("# 6: +30 C") );
      Serial.println ( F("# 7: +40 C") );
      Serial.println ( F("# 8: +50 C") );
      return true;
    }
    else if( argCount == 2 ){
      // Check for the value to edit
      tempIndex = atoi(args[0]);
      if ( tempIndex < 9 ) {
        g_settings.zero_current_factor_values [ tempIndex ] = atof(args[1]);
        Serial.print( F("## ") ); // make it status OK comment
        Serial.print ( F("zero_current_factor[") );
        Serial.print (tempIndex);
        Serial.print ("] = ");
        Serial.println ( g_settings.zero_current_factor_values [ tempIndex ], 4 );
        Serial.println ( F( "#! sensorType set to 0 (custom)" ) );
        g_settings.sensorType = 0; // now  non-standard temp curve so set type to 0 "custom"
        saveSettings();
        return true;
      }  
    }
  }

   else if( command[0] == '6' ){ // Edit data_fields array

    if( argCount == 0 ){ // if no args, print the data field options menu
      Serial.println ( F("# Edit Data Fields") );
      Serial.println ( F("# 0 = field inactive, >0 = number of decimal places (max = 8)") ) ;
      Serial.println ( F("# Submit \"6:<number>,<value>\"") );
      Serial.println ( F("#  0: INDEX") );
      Serial.println ( F("#  1: TIME") );
      Serial.println ( F("#  2: Vin") );
      Serial.println ( F("#  3: Vdd") );
      Serial.println ( F("#  4: Vaux") );
      Serial.println ( F("#  5: Vwe") );
      Serial.println ( F("#  6: Vref") );
      Serial.println ( F("#  7: TEMP") );
      Serial.println ( F("#  8: PPMraw") );
      Serial.println ( F("#  9: PPM") );
      Serial.println ( F("# 10: PPMf") );
      Serial.println ( F("# 11: PPMm") );
      Serial.println ( F("# 12: PPMv") );
      Serial.println ( F("# 13: PPMs") );
      Serial.println ( F("# 14: AUXf") );
      Serial.println ( F("# 15: AUXm") );
      Serial.println ( F("# 16: AUXv") );
      Serial.println ( F("# 17: AUXs") );
      Serial.println ( F("# 18: RAWf") );
      Serial.println ( F("# 14: gTemp") );
      Serial.println ( F("# 20: A0") );
      Serial.println ( F("# 21: A1") );
      Serial.println ( F("# 22: A2") );
      Serial.println ( F("# 23: A3") );
      return true;
    }
    else if( argCount == 2 ){
      // Check for the value to edit
      tempIndex = atoi(args[0]);
      if ( tempIndex == 123 ){ // if secret code 123 entered, load data field defaults
        for ( loopIndex = 0; loopIndex <= DATA_FIELDS_ARRAY_SIZE; loopIndex++) {
          g_settings.data_fields [ loopIndex ]  = default_data_fields [ loopIndex ];     
          Serial.println ( F("#! Data field defaults loaded") );
        }
        saveSettings();
        return true;
      }
      else if ( tempIndex <= DATA_FIELDS_ARRAY_SIZE ) {
        if (  atoi(args[1])  <= MAX_DECIMAL_PLACES ) {
          g_settings.data_fields [ tempIndex ] = atoi(args[1]);
        } 
        else {   
          g_settings.data_fields [ tempIndex ] = DEFAULT_DECIMAL_PLACES; // default decimal places if input is out of range
          Serial.println ( F("#! Data field defaults loaded") );
        }  
        Serial.print( F("## ") ); // make it a status comment
        Serial.print ( F("data_field["));
        Serial.print (tempIndex);
        Serial.print ( F("] = ") );
        Serial.println( g_settings.data_fields [ tempIndex ] );
        saveSettings();
        return true;
       }
     }    
  }


  // Any invalid options
  Serial.print ( PGMSTR(flash_invalid) );
  Serial.println (command);
  printMenu();
  Serial.println ( PGMSTR(flash_menu_separator) );
  return false;
}


//************************************************************************************
//*                                                                                  *
//*                             processMessage()                                     *                             
//*                                                                                  *
//************************************************************************************
// Parse the incoming serial buffer and execute any command it contains
void processMessage() {
  g_inputBuffer[ g_inputIndex ] = 0; // truncate the string
  g_inputIndex = 0;
  char *command = g_inputBuffer;
  char *args[ MAX_COMMAND_ARGS ];
  int argCount = 0;

  // check for command arguments
  int colonPos = indexOf( g_inputBuffer, ':' );
  if (colonPos > 0) {
    g_inputBuffer[ colonPos ] = 0;
    char *argStr = g_inputBuffer + colonPos + 1;
    int commaPos = 0;
    argCount = 0;
    do {
      // strip leading spaces
      while (*argStr == ' ')
        argStr++;
      // store in argument array
      args[ argCount ] = argStr;
      // find end of arg
      commaPos = indexOf( argStr, ',' );
      char *argEnd = 0;
      if (commaPos > 0) {
        argStr[ commaPos ] = 0;
        argStr += commaPos + 1;
        argEnd = argStr - 2;
      } else {
        argEnd = argStr + strlen( argStr ) - 1;
      }
      // strip off tailing spaces (note: we're protected by 0 that replaced
      // colon)
      while (*argEnd == ' ') {
        *argEnd = 0;
        argEnd--;
      }
      // done with this arg
      argCount++;
    } while (commaPos > 0 && argCount < MAX_COMMAND_ARGS);
  }

  // execute the command now that we have parsed it
  if (command[ 0 ]) {
      boolean handled = executeCommand( command, argCount, args );
  }
}

//************************************************************************************
//*                                                                                  *
//*                             feedCommandByte()                                    *                             
//*                                                                                  *
//************************************************************************************
// Process a single incoming command byte; store in buffer or execute command if
// complete
void feedCommandByte( char c ) {  // if carriage return or line feed
  if (c == 10 || c == 13) {
    if (g_inputIndex)
      processMessage();

  // want to be able to write 0 into last position after increment
  } else if (g_inputIndex >= INPUT_BUFFER_LENGTH - 1) {
    g_inputIndex = 0;
  } else {
    g_inputBuffer[ g_inputIndex ] = c;
    g_inputIndex++;
  }
}

//************************************************************************************
//*                                                                                  *
//*                             checkSerialCommands()                                *                             
//*                                                                                  *
//************************************************************************************

// Read incoming serial commands; will execute any received commands
void checkSerialCommands() {
  while (Serial.available()) {
//    wdt_reset();
    char c = Serial.read();
    if (c > 0) {
      feedCommandByte( c );
    }
  }
}

//****************************** SETTINGS ROUTINES ***********************************
//                                                                                   *
//************************************************************************************
//*                                                                                  *
//*                             loadDefaultSettings()                                *                             
//*                                                                                  *
//************************************************************************************
//                                                                                   *
//                                                                                   *
// Load the default settings and save them to the eeprom
void loadDefaultSettings() {
  g_settings.verification_check = VERIFICATION_CHECK_VALUE;
  g_settings.dataSetId          = DEFAULT_DATA_SET_ID;
  g_settings.readInterval       = DEFAULT_READ_INTERVAL;     // milliseconds between readings (1-1000)
  g_settings.triggerMode        = DEFAULT_TRIGGER_MODE;      // 
  g_settings.triggerQualify     = DEFAULT_TRIGGER_QUALIFY;   // 
  g_settings.afeLoadResistor    = DEFAULT_LOAD_RESISTOR;     // AFE Rload setting in ohms, options are 10, 33, 50, 100
  g_settings.afeGainResistor    = DEFAULT_GAIN_RESISTOR;     // AFE Gain Resistor setting , 0 = external
  g_settings.adcGain            = DEFAULT_ADC_GAIN;          // ADC Gain Setting
  g_settings.sensorSensitivity  = DEFAULT_SENSITIVITY;       // * 1e-9
  g_settings.sensorOffset       = DEFAULT_OFFSET;            // * 1e-9
  g_settings.sensorType         = DEFAULT_SENSOR_TYPE;       // CO-B4 = 1
  g_settings.afeBias            = DEFAULT_AFE_BIAS;          // * 100mV steps
  g_settings.afeBiasPolarity    = DEFAULT_AFE_BIAS_POLARITY;   
  g_settings.avgNumber          = DEFAULT_AVG_NUMBER;   
  g_settings.avgDelay           = DEFAULT_AVG_DELAY;         // * ms
  g_settings.boardNumber        = DEFAULT_BOARD_NUMBER;     
  g_settings.sensorNumber       = DEFAULT_SENSOR_NUMBER;    
  g_settings.startupDelay       = DEFAULT_STARTUP_DELAY;     // * seconds
  g_settings.auxOffset          = DEFAULT_AUX_OFFSET;        // * mV
  g_settings.adcOffset0         = DEFAULT_ADC_OFFSET_0;      // * mV
  g_settings.adcOffset1         = DEFAULT_ADC_OFFSET_1;      // * mV
  g_settings.adcOffset2         = DEFAULT_ADC_OFFSET_2;      // * mV
  g_settings.adcOffset3         = DEFAULT_ADC_OFFSET_3;      // * mV
  g_settings.filterFrequency    = DEFAULT_FILTER_FREQUENCY;  // * Hz
  g_settings.filterQfactor      = DEFAULT_FILTER_Q;          
  for ( loopIndex = 0; loopIndex <= DATA_FIELDS_ARRAY_SIZE; loopIndex++) {
    g_settings.data_fields [ loopIndex ]  = default_data_fields [ loopIndex ]; 
  }    
  for (arrayIndex = 0; arrayIndex < SENSITIVITY_TABLE_LENGTH; arrayIndex++) {
    g_settings.sensitivity_factor_values [ arrayIndex ] = co_sensitivity_factor_values[ arrayIndex ];
  }
  for (arrayIndex = 0; arrayIndex < ZERO_CURRENT_TABLE_LENGTH; arrayIndex++) {
    g_settings.zero_current_factor_values [ arrayIndex ] = co_zero_current_factor_values[ arrayIndex ];
  }
}

//************************************************************************************
//*                                                                                  *
//*                             settingsVerified()                                   *                             
//*                                                                                  *
//************************************************************************************

// Returns true if the current settings match the verification check value and
// both the readInterval and afeLoadResistor > 0. False otherwise.
bool settingsVerified() {
  // Check the verification value and some small sanity checks.
  if(
    g_settings.verification_check == VERIFICATION_CHECK_VALUE &&
    g_settings.readInterval > 0 && // should be more than 500 as main timing / data processing loop takes about this much.
    g_settings.afeLoadResistor < 4 && // must be 0-3
    g_settings.adcGain      < 4    // must be 0-3
  ){
    return true;
  }
  return false;
}

//************************************************************************************
//*                                                                                  *
//*                             loadSettings()                                       *                             
//*                                                                                  *
//************************************************************************************
// Load settings from eeprom
int loadSettings() {
//  Serial.println( F("# Load") );
  int address = 0;
  uint8_t* p = (uint8_t*)(void*)&g_settings;
  uint16_t i;
  for (i = 0; i < sizeof(g_settings); i++){
    
//    wdt_reset(); // Reset the watchdog timer
    *p++ = EEPROM.read(address++);
  }
//  Serial.print("# Load "); 
//  Serial.println( PGMSTR(flash_success) );
// took these out as they happen before the startup delay and can cause weird line counts in CSV output
  return i;
}

//************************************************************************************
//*                                                                                  *
//*                             saveSettings()                                       *                             
//*                                                                                  *
//************************************************************************************
// Save settings back to eeprom
int saveSettings() {
//  Serial.println ( F("## Save") );
  int address = 0;
  const uint8_t* p = (const uint8_t*)(const void*)&g_settings;
  uint16_t i;
  for(i = 0; i < sizeof(g_settings); i++){
//    wdt_reset(); // Reset the watchdog timer
    EEPROM.write(address++, *p++);
  }
//  Serial.println ( F("## Save") );
  if (g_settings.triggerMode == 1 || g_settings.triggerMode == 3) { // if  verbose Interval or Serial Trigger mode
    Serial.println ( PGMSTR(flash_success) );
  }
  return i;
}
//                                                                                   *
//                                                                                   *
//************************************************************************************


//************************************************************************************
//*                                                                                  *
//*                       readTemperatureAndHumidity()                               *                             
//*              (optional HIH sensor mounted under echem socket)                    *
//************************************************************************************
#ifdef USE_HIH
// Takes a float for temperature and humidity and fills them with the current
// values.
// Returns a status code:
// 0: Normal
// 1: Stale - this data has already been read
// 2: Command Mode - the sensor is in command mode
// 3: Diagnostic - The sensor has had a diagnostic condition and data is invalid
// 4: Invalid - The sensor did not return 4 bytes of data. Usually this means it's not attached.
uint8_t readTemperatureAndHumidity( float &temperature_c, float &humidity_percent ) {
  // From: http://www.phanderson.com/arduino/hih6130.html
  uint8_t address = 0x27;
  uint8_t humHigh, humLow, tempHigh, tempLow, status;
  uint16_t humData, tempData;

  // Request read
  Wire.beginTransmission(address);
  Wire.endTransmission();

  // According to the data sheet, the measurement cycle is typically ~36.56 ms
  // We'll give a little extra time

  // Request data
  uint8_t bytesReceived = Wire.requestFrom((int)address, (int)4);
  if(bytesReceived != 4){

    // This is our own error to specify that we didn't receive 4 bytes from the
    // sensor.
    return 4; // temp and humidity will be unchanged
  }

  humHigh  = Wire.read();
  humLow   = Wire.read();
  tempHigh = Wire.read();
  tempLow  = Wire.read();

  // Status is the top two bits of the high humidity byte
  status = (humHigh >> 6) & 0x03;
  if(status == 3){
    return status; // temp and humidity will be unchanged
  }

  // Keep the rest
  humHigh = humHigh & 0x3f;

  // OR in the low bytes
  humData  = (((uint16_t)humHigh)  << 8) | humLow;
  tempData = (((uint16_t)tempHigh) << 8) | tempLow;

  // The bottom two bits of the low temp byte are invalid, so we'll remove
  // those
  tempData = tempData >> 2;

  // Convert to floating point
  humidity_percent = (float) humData * 6.10e-3; // 100 / (2^14 - 1)
  temperature_c = (float) tempData * 1.007e-2 - 40.0; // 165 / (2^14 - 1)

  // Status can be
  // 0: Normal
  // 1: Stale - this data has already been read
  // 2: Command Mode - the sensor is in command mode
  // 3: Diagnostic - The sensor has had a diagnostic condition and data is
  //    invalid
  return status;
}
#endif

int freeRam () 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
