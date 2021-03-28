#ifndef _CONFIG_H
#define _CONFIG_H
/*
 * 
 * This is where you should do all the configuration
 * 
 * First you need to make sure that you have all the require Arduino libraries to compile it
 * 
 * required libraries:
 * Adafruit_BMP085 
 * avdweb_VirtualDelay
 */



/////////////// config changes start here ///////////


// If you want to have additionnal debugging uncomment it
//#define SERIAL_DEBUG
#undef SERIAL_DEBUG


////////////// config changes end here /////////////
//////////// do not change anything after unless you know what you are doing /////////////////////

#define MAJOR_VERSION 1
#define MINOR_VERSION 7
#define CONFIG_START 32




#define BOARD_FIRMWARE "AltiDuo"
#define NBR_PYRO_OUT3

#define SerialCom Serial

#include "Arduino.h"
//used for writing in the microcontroler internal eeprom
#include <EEPROM.h>

#include "avdweb_VirtualDelay.h"

//pyro out 1
extern const int pyroOut1;

//pyro out 2
extern const int pyroOut2;

extern int continuityPins[4];

struct ConfigStruct {
  int unit;             //0 = meter 1 = feet
  int beepingMode;      // decide which way you want to report the altitude
  int outPut1;          // assign a function to each pyro
  int outPut2;
  int outPut3;                // unused but left for compatibility
  int mainAltitude;           //deployment altitude for the main chute
  int superSonicYesNo;        // if set to yes do not do any altitude measurement when altimeter starts
  int outPut1Delay;           // delay output by x ms
  int outPut2Delay;
  int outPut3Delay;           // unused but left for compatibility
  int beepingFrequency;       // this beeping frequency can be changed
  int nbrOfMeasuresForApogee; //how many measure to decide that apogee has been reached
  int endRecordAltitude;      // stop recording when landing define under which altitude we are not recording
  int recordTemperature;      //decide if we want to record temperature
  int superSonicDelay;        //nbr of ms during when we ignore any altitude measurements
  long connectionSpeed;       //altimeter connection baudrate
  int altimeterResolution;    // BMP sensor resolution
  int eepromSize;             // unused but left for compatibility
  int noContinuity;           // Swith on or off continuity
  int outPut4;                // unused but left for compatibility
  int outPut4Delay;           // unused but left for compatibility
  int liftOffAltitude;        //Lift Altitude in meters
  int batteryType;            // 0= Unknown, 1= "2S (7.4 Volts)", 2 = "9 Volts",3 = "3S (11.1 Volts)
  int cksum;  
};
extern ConfigStruct config;

extern void defaultConfig();
extern boolean readAltiConfig();
extern int getOutPin(int );
extern bool writeAltiConfig( char * );
extern void printAltiConfig();
extern void writeConfigStruc();
extern bool CheckValideBaudRate(long);
extern unsigned int CheckSumConf( ConfigStruct );
extern unsigned int msgChk( char * buffer, long length );
#endif
