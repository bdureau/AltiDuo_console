/*
  Model Rocket dual altimeter Ver 1.7
  Copyright Boris du Reau 2012-2021

  This is using a BMP085 presure sensor and an Atmega 328
  The following should fire the main at apogee if it is at least 50m above ground of the launch site
  and fire the main 100m before landing
  The Arduino board that I am using to load the program is an Arduino UNO ATmega328P

  For the BMP085 pressure sensor
  Connect VCC of the BMP085 sensor to 5.0V! make sure that you are using the 5V sensor (GY-65 model)
  Connect GND to Ground
  Connect SCL to i2c clock - on 328 Arduino Uno/Duemilanove/etc thats Analog 5
  Connect SDA to i2c data - on 328 Arduino Uno/Duemilanove/etc thats Analog 4
  EOC is not used, it signifies an end of conversion
  XCLR is a reset pin, also not used here
  The micro swiches are connected on pin D6 and D7
  The main is connected to pin D8
  The apogee is connected to pin D9
  The main continuity test is connected to pin D10
  The apogee continuity test is connected to pin D11
  The speaker/buzzer is connected to pin D12

  Major changes on  version 1.5
  Added support so that it can use the Android console
  Major changes on  version 1.6
  Adding checksum
  Major changes on  version 1.7
  Allow multiple outputs of the main type
  allow events
  major re-write
*/

//altimeter configuration lib
#include "config.h"
#include <Wire.h> //I2C library

//#include <Adafruit_BMP085.h>
#include "Bear_BMP085.h"

#include "kalman.h"
#include "beepfunc.h"

//////////////////////////////////////////////////////////////////////
// Global variables
//////////////////////////////////////////////////////////////////////
int mode = 0; //0 = read; 1 = write;

//Adafruit_BMP085 bmp;
BMP085 bmp;

//ground level altitude
long initialAltitude;
long liftoffAltitude;
long lastAltitude;
//current altitude
long currAltitude;
//Apogee altitude
long apogeeAltitude;
long mainAltitude;
boolean liftOff = false;
unsigned long initialTime;

boolean FastReading = false;

//nbr of measures to do so that we are sure that apogee has been reached
unsigned long measures = 5;
unsigned long mainDeployAltitude;

// pin used by the jumpers
const int pinAltitude1 = 8;
const int pinAltitude2 = 7;

//soft configuration
boolean softConfigValid = false;

//by default apogee pin
const int pinChannel1Continuity = 10;
// by default continuity for the main
const int pinChannel2Continuity = 11;

float FEET_IN_METER = 1;
boolean canRecord = true;

boolean Output1Fired = false;
boolean Output2Fired = false;
long lastTelemetry = 0;

boolean exitRecording = false;
boolean allApogeeFiredComplete = false;
boolean allMainFiredComplete = false;
boolean allTimerFiredComplete = false;
boolean allLiftOffFiredComplete = false;
boolean allLandingFiredComplete = false;
boolean allAltitudeFiredComplete = false;

boolean telemetryEnable = false;

void MainMenu();

double ReadAltitude()
{
  return KalmanCalc(bmp.readAltitude());
}

/*

  ResetGlobalVar()

*/
void ResetGlobalVar() {

  exitRecording = false;

  allApogeeFiredComplete = false;
  allMainFiredComplete = false;
  allTimerFiredComplete = false;
  allLiftOffFiredComplete = false;
  allLandingFiredComplete = false;
  allAltitudeFiredComplete = false;

  liftOff = false;
  apogeeAltitude = 0;
  mainAltitude = 0;

  Output1Fired = false;
  Output2Fired = false;


  lastAltitude = 0;//initialAltitude;

}
/*

   initAlti()

*/
void initAlti() {
  ResetGlobalVar();

  // if the baud rate is invalid let's default it
  if (!CheckValideBaudRate(config.connectionSpeed))
  {
    config.connectionSpeed = 38400;
    writeConfigStruc();
  }

  // set main altitude (if in feet convert to metrics)
  if (config.unit == 0)
    FEET_IN_METER = 1;
  else
    FEET_IN_METER = 3.28084 ;

  mainDeployAltitude = int(config.mainAltitude / FEET_IN_METER);
  // beepFrequency
  beepingFrequency = config.beepingFrequency;

  //number of measures to do to detect Apogee
  measures = config.nbrOfMeasuresForApogee;

  //check which pyro are enabled
  pos = -1;

  if (config.outPut1 != 3) {
    pos++;
    continuityPins[pos] = pinChannel1Continuity;
  }
  if (config.outPut2 != 3) {
    pos++;
    continuityPins[pos] = pinChannel2Continuity;
  }
}
//================================================================
// Start program
//================================================================
void setup()
{
  int val = 0;     // variable to store the read value
  int val1 = 0;     // variable to store the read value

  // Read altimeter softcoded configuration
  softConfigValid = readAltiConfig();

  // check if configuration is valid
  if (!softConfigValid)
  {
    //default values
    defaultConfig();
    writeConfigStruc();
  }

  initAlti();
  // init Kalman filter
  KalmanInit();

  // initialise the connection
  Wire.begin();

  //You can change the baud rate here
  //and change it to 57600, 115200 etc..
  //Serial.begin(BAUD_RATE);
  SerialCom.begin(config.connectionSpeed);

  pinMode(PD0, INPUT_PULLUP);
  //Presure Sensor Initialisation
  // Note that BMP180 is compatible with the BMP085 library
  // Low res should work better at high speed
  bmp.begin( config.altimeterResolution);

  //our drogue has not been fired
  //apogeeHasFired = false;
  //mainHasFired = false;

  SerialCom.print(F("Start program\n"));

  //SerialCom.print(F("Set outputs\n"));
  //Initialise the output pin
  pinMode(pyroOut1, OUTPUT);
  pinMode(pyroOut2, OUTPUT);

  pinMode(pinSpeaker, OUTPUT);

  pinMode(pinAltitude1, INPUT);
  pinMode(pinAltitude2, INPUT);

  pinMode(pinChannel1Continuity , INPUT);
  pinMode(pinChannel2Continuity , INPUT);


  //Make sure that the output are turned off
  digitalWrite(pyroOut1, LOW);
  digitalWrite(pyroOut2, LOW);

  digitalWrite(pinSpeaker, LOW);

  //enable or disable continuity check
  if (config.noContinuity == 1)
    noContinuity = true;
  else
    noContinuity = false;

  //initialisation give the version of the altimeter
  //One long beep per major number and One short beep per minor revision
  //For example version 1.2 would be one long beep and 2 short beep
  beepAltiVersion(MAJOR_VERSION, MINOR_VERSION);



  if (!softConfigValid)
  {
    //initialise the deployement altitude for the main
    mainDeployAltitude = 100;

    // On the Alti duo when you close the jumper you set it to 1
    // val is the left jumper and val1 is the right jumper
    //as of version 1.4 only use the jumper if no valid softconfiguration

    val = digitalRead(pinAltitude1);
    val1 = digitalRead(pinAltitude2);
    if (val == 0 && val1 == 0)
    {
      mainDeployAltitude = 50;
    }
    if (val == 0 && val1 == 1)
    {
      mainDeployAltitude = 100;
    }
    if (val == 1 && val1 == 0)
    {
      mainDeployAltitude = 150;
    }
    if (val == 1 && val1 == 1)
    {
      mainDeployAltitude = 200;
    }
  }

  //number of measures to do to detect Apogee
  //measures = config.nbrOfMeasuresForApogee;

  // let's do some dummy altitude reading
  // to initialise the Kalman filter
  for (int i = 0; i < 50; i++) {
    ReadAltitude();
  }

  //let's read the launch site altitude
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += ReadAltitude();
    delay(50);
  }
  initialAltitude = (sum / 10.0);
  lastAltitude = 0;//initialAltitude;
  liftoffAltitude = config.liftOffAltitude; //20;
}

/*
   setEventState(int pyroOut, boolean state)
    Set the state of the output
*/
void setEventState(int pyroOut, boolean state)
{
  if (pyroOut == pyroOut1)
  {
    Output1Fired = state;
#ifdef SERIAL_DEBUG
    SerialCom.println(F("Output1Fired"));
#endif
  }

  if (pyroOut == pyroOut2)
  {
    Output2Fired = state;
#ifdef SERIAL_DEBUG
    SerialCom.println(F("Output2Fired"));
#endif
  }
}

/*
   SendTelemetry(long sampleTime, int freq)
   Send telemety so that we can plot the flight

*/
void SendTelemetry(long sampleTime, int freq) {
  char altiTelem[150] = "";
  char temp[10] = "";
  if (telemetryEnable && (millis() - lastTelemetry) > freq) {
    lastTelemetry = millis();
    int val = 0;
    //check liftoff
    int li = 0;
    if (liftOff)
      li = 1;

    //check apogee
    int ap = 0;
    if (allApogeeFiredComplete)
      ap = 1;

    //check main
    int ma = 0;
    if (allMainFiredComplete)
      ma = 1;
    int landed = 0;
    if ( allMainFiredComplete && currAltitude < 10)
      landed = 1;

    strcat(altiTelem, "telemetry," );
    sprintf(temp, "%i,", currAltitude);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", li);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", ap);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", apogeeAltitude);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", ma);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", mainAltitude);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", landed);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", sampleTime);
    strcat(altiTelem, temp);
    if (config.outPut1 != 3) {
      //check continuity
      val = digitalRead(pinChannel1Continuity);
      if (val == 0)
        strcat(altiTelem, "0,");
      else
        strcat(altiTelem, "1,");
    }
    else {
      strcat(altiTelem, "-1,");
    }
    SerialCom.print(F(","));
    if (config.outPut1 != 3) {
      //check continuity
      val = digitalRead(pinChannel2Continuity);
      delay(20);
      if (val == 0)
        strcat(altiTelem, "0,");
      else
        strcat(altiTelem, "1,");
    }
    else {
      strcat(altiTelem, "-1,");
    }
    unsigned int chk;
    chk = msgChk(altiTelem, sizeof(altiTelem));
    sprintf(temp, "%i", chk);
    strcat(altiTelem, temp);
    strcat(altiTelem, ";\n");
    SerialCom.print("$");
    SerialCom.print(altiTelem);
  }
}
//================================================================
// Main loop which call the menu
//================================================================
void loop()
{
  MainMenu();
}
/*
   Calculate the current velocity
*/
int currentVelocity(int prevTime, int curTime, int prevAltitude, int curAltitude)
{
  int curSpeed = int ((curAltitude - prevAltitude) / ( curTime - prevTime));
  return curSpeed;
}

//================================================================
// Function:  recordAltitude()
// called for normal recording
//================================================================
void recordAltitude()
{
  ResetGlobalVar();

  boolean OutputFiredComplete[2] = {false, false};
  int OutputDelay[2] = {0, 0};
  OutputDelay[0] = config.outPut1Delay;
  OutputDelay[1] = config.outPut2Delay;

  // 0 = main 1 = drogue 2 = timer 4 = landing 5 = liftoff 3 = disable
  int OutputType[2] = {3, 3};
  OutputType[0] = config.outPut1;
  OutputType[1] = config.outPut2;

  int OutputPins[2] = { -1, -1};
  if (config.outPut1 != 3)
    OutputPins[0] = pyroOut1;
  if (config.outPut2 != 3)
    OutputPins[1] = pyroOut2;

  boolean apogeeReadyToFire = false;
  boolean mainReadyToFire = false;
  boolean landingReadyToFire = false;
  boolean liftOffReadyToFire = false;
  unsigned long apogeeStartTime = 0;
  unsigned long mainStartTime = 0;
  unsigned long landingStartTime = 0;
  unsigned long liftOffStartTime = 0;
  boolean ignoreAltiMeasure = false;
  unsigned long altitudeStartTime[] = {0, 0, 0, 0};

  boolean liftOffHasFired = false;
  //hold the state of all our outputs
  boolean outputHasFired[2] = {false, false};

  if (config.outPut1 == 3) Output1Fired = true;
  if (config.outPut2 == 3) Output2Fired = true;

#ifdef SERIAL_DEBUG
  SerialCom.println(F("Config delay:"));
  SerialCom.println(config.outPut1Delay);
  SerialCom.println(config.outPut2Delay);
#endif

  while (!exitRecording)
  {
    //read current altitude
    currAltitude = (ReadAltitude() - initialAltitude);
    if (liftOff)
      SendTelemetry(millis() - initialTime, 200);
    if (( currAltitude > liftoffAltitude) && !liftOff && !allMainFiredComplete )
    {
      liftOff = true;
      SendTelemetry(0, 200);
      // save the time
      initialTime = millis();
      if (config.superSonicYesNo == 1)
        ignoreAltiMeasure = true;
    }
    if (liftOff)
    {
#ifdef SERIAL_DEBUG
      SerialCom.println(F("we have lift off\n"));
#endif

      unsigned long prevTime = 0;
      long prevAltitude = 0;
      // loop until we have reach an altitude of 3 meter
      //while(currAltitude > 3 && MainFiredComplete==false && liftOff ==true;)
      while (liftOff)
      {
        unsigned long currentTime;
        unsigned long diffTime;

        currAltitude = (ReadAltitude() - initialAltitude);

        currentTime = millis() - initialTime;
        if (allMainFiredComplete && !allLandingFiredComplete && !landingReadyToFire) {

          if (abs(currentVelocity(prevTime, currentTime, prevAltitude, currAltitude)) < 1  ) {
            //we have landed
            landingReadyToFire = true;
            landingStartTime = millis();
          }
        }
        prevAltitude = currAltitude;
        SendTelemetry(currentTime, 200);
        diffTime = currentTime - prevTime;
        prevTime = currentTime;

        if (!liftOffHasFired && !liftOffReadyToFire) {
          liftOffReadyToFire = true;
          liftOffStartTime = millis();
        }

        if (!allLiftOffFiredComplete) {
          //fire all liftoff that are ready
          for (int li = 0; li < 2; li++ ) {
            if (!outputHasFired[li] && ((millis() - liftOffStartTime) >= OutputDelay[li] ) && OutputType[li] == 5) {
              digitalWrite(OutputPins[li], HIGH);
              outputHasFired[li] = true;
            }
          }
          for (int li = 0; li < 2; li++ ) {
            if ((millis() - liftOffStartTime ) >= (1000 + OutputDelay[li])  && !OutputFiredComplete[li] && OutputType[li] == 5)
            {
              digitalWrite(OutputPins[li], LOW);
              setEventState(OutputPins[li], true);
              OutputFiredComplete[li] = true;
            }
          }

          allLiftOffFiredComplete = true;

          for (int li = 0; li < 2; li++ ) {
            if (!OutputFiredComplete[li] && OutputType[li] == 5)
            {
              allLiftOffFiredComplete = false;
            }
          }
          SendTelemetry(millis() - initialTime, 200);
        }
        //altitude events
        if (!allAltitudeFiredComplete) {
          //fire all altitude that are ready
          for (int al = 0; al < 4; al++ ) {
            if (!outputHasFired[al] && ((currAltitude >= OutputDelay[al]) ) && OutputType[al] == 6) {
              digitalWrite(OutputPins[al], HIGH);
              outputHasFired[al] = true;
              altitudeStartTime[al] = millis();
            }
          }
          for (int al = 0; al < 4; al++ ) {
            if (( millis()  >= (1000 + altitudeStartTime[al]))  && !OutputFiredComplete[al] && OutputType[al] == 6 && outputHasFired[al])
            {
              digitalWrite(OutputPins[al], LOW);
              setEventState(OutputPins[al], true);
              OutputFiredComplete[al] = true;
            }
          }

          allAltitudeFiredComplete = true;

          for (int al = 0; al < 4; al++ ) {
            if (!OutputFiredComplete[al] && OutputType[al] == 6)
            {
              allAltitudeFiredComplete = false;
            }
          }
          SendTelemetry(millis() - initialTime, 200);
        }
        // timer events
        if (!allTimerFiredComplete) {
          //fire all timers that are ready
          for (int ti = 0; ti < 4; ti++ ) {
            if (!outputHasFired[ti] && ((currentTime >= OutputDelay[ti]) ) && OutputType[ti] == 2) {
              digitalWrite(OutputPins[ti], HIGH);
              outputHasFired[ti] = true;
            }
          }
          for (int ti = 0; ti < 4; ti++ ) {
            if ((currentTime  >= (1000 + OutputDelay[ti]))  && !OutputFiredComplete[ti] && OutputType[ti] == 2)
            {
              digitalWrite(OutputPins[ti], LOW);
              setEventState(OutputPins[ti], true);
              OutputFiredComplete[ti] = true;
            }
          }

          allTimerFiredComplete = true;

          for (int ti = 0; ti < 4; ti++ ) {
            if (!OutputFiredComplete[ti] && OutputType[ti] == 2)
            {
              allTimerFiredComplete = false;
            }
          }
          SendTelemetry(millis() - initialTime, 200);
        }

        if (config.superSonicYesNo == 1)
        {
          //are we still in superSonic mode?
          if (currentTime > 3000)
            ignoreAltiMeasure = false;
        }
        if (currAltitude < lastAltitude && !apogeeReadyToFire && !ignoreAltiMeasure)
        {
          measures = measures - 1;
          if (measures == 0)
          {
            //fire drogue
            apogeeReadyToFire = true;
            apogeeStartTime = millis();
            //apogeeAltitude = currAltitude;
            apogeeAltitude = lastAltitude;
          }
        }
        else
        {
          lastAltitude = currAltitude;
          measures = config.nbrOfMeasuresForApogee;
        }
        if (apogeeReadyToFire && !allApogeeFiredComplete)
        {

          //fire all drogues if delay ok
          for (int ap = 0; ap < 2; ap++ ) {
            if (!outputHasFired[ap] && ((millis() - apogeeStartTime) >= OutputDelay[ap]) && OutputType[ap] == 1) {
              digitalWrite(OutputPins[ap], HIGH);
              outputHasFired[ap] = true;
            }
          }

          for (int ap = 0; ap < 2; ap++ ) {
            if ((millis() - apogeeStartTime ) >= (1000 + OutputDelay[ap]) && !OutputFiredComplete[ap] && OutputType[ap] == 1)
            {
              digitalWrite(OutputPins[ap], LOW);
              setEventState(OutputPins[ap], true);
              OutputFiredComplete[ap] = true;
            }
          }

          allApogeeFiredComplete = true;

          for (int ap = 0; ap < 2; ap++ ) {
            if (!OutputFiredComplete[ap] && OutputType[ap] == 1)
            {
              allApogeeFiredComplete = false;
            }
          }
          SendTelemetry(millis() - initialTime, 200);
        }


        if ((currAltitude  < mainDeployAltitude) && allApogeeFiredComplete && !mainReadyToFire && !allMainFiredComplete)
        {
          // Deploy main chute  X meters or feet  before landing...
          mainReadyToFire = true;
#ifdef SERIAL_DEBUG
          SerialCom.println(F("preparing main"));
#endif
          mainStartTime = millis();
          mainAltitude = currAltitude;
#ifdef SERIAL_DEBUG
          SerialCom.println(F("main altitude"));
          SerialCom.println(mainAltitude);
#endif
        }
        if (mainReadyToFire && !allMainFiredComplete)
        {
          //fire main
#ifdef SERIAL_DEBUG
          SerialCom.println(F("firing main"));
#endif
          for (int ma = 0; ma < 2; ma++ ) {
            if (!outputHasFired[ma] && ((millis() - mainStartTime) >= OutputDelay[ma]) && OutputType[ma] == 0) {
              digitalWrite(OutputPins[ma], HIGH);
              outputHasFired[ma] = true;
            }
          }


          for (int ma = 0; ma < 2; ma++ ) {
            if ((millis() - mainStartTime ) >= (1000 + OutputDelay[ma]) && !OutputFiredComplete[ma] && OutputType[ma] == 0)
            {
              digitalWrite(OutputPins[ma], LOW);
              setEventState(OutputPins[ma], true);
              OutputFiredComplete[ma] = true;
            }
          }
          allMainFiredComplete = true;

          for (int ma = 0; ma < 2; ma++ ) {
            if (!OutputFiredComplete[ma] && OutputType[ma] == 0)
            {
              allMainFiredComplete = false;
            }
          }
          SendTelemetry(millis() - initialTime, 200);
        }


        if (landingReadyToFire && !allLandingFiredComplete) {
          //fire all landing that are ready
          for (int la = 0; la < 4; la++ ) {
            if (!outputHasFired[la] && ((millis() - landingStartTime) >= OutputDelay[la] ) && OutputType[la] == 4) {
              digitalWrite(OutputPins[la], HIGH);
              outputHasFired[la] = true;
            }
          }
          for (int la = 0; la < 4; la++ ) {
            if ((millis() - landingStartTime ) >= (1000 + OutputDelay[la])  && !OutputFiredComplete[la] && OutputType[la] == 4)
            {
              digitalWrite(OutputPins[la], LOW);
              setEventState(OutputPins[la], true);
              OutputFiredComplete[la] = true;
            }
          }

          allLandingFiredComplete = true;

          for (int la = 0; la < 4; la++ ) {
            if (!OutputFiredComplete[la] && OutputType[la] == 4)
            {
              allLandingFiredComplete = false;
            }
          }
          SendTelemetry(millis() - initialTime, 200);
        }
        if ((allMainFiredComplete && currAltitude < 10) && allLandingFiredComplete)
        {
          liftOff = false;
          SendTelemetry(millis() - initialTime, 200);
        }

        if (Output1Fired  && Output2Fired  )
        {
#ifdef SERIAL_DEBUG
          SerialCom.println(F("all event have fired"));
#endif
          exitRecording = true;
          SendTelemetry(millis() - initialTime, 200);
        }
      }
    }
  }
}




//================================================================
// Main menu to interpret all the commands sent by the altimeter console
//================================================================
void MainMenu()
{
  char readVal = ' ';
  int i = 0;

  char commandbuffer[200];

  /*SerialCom.println(F("Rocket flight data logger. A maximum of 25 flight can be logged \n"));
    SerialCom.println(F("Commands are: \n"));
    SerialCom.println(F("w = record flight \n"));
    SerialCom.println(F("r (followed by the flight number) = read flight data\n"));
    SerialCom.println(F("l  = print flight list \n"));
    SerialCom.println(F("e  = erase all flight data \n"));
    SerialCom.println(F("c  = toggle continuity on/off \n"));
    SerialCom.println(F("b  = print alti config \n"));
    SerialCom.println(F("Enter Command and terminate it by a ; >>\n"));*/
  i = 0;
  readVal = ' ';
  while ( readVal != ';')
  {
    if (FastReading == false)
    {
      currAltitude = (ReadAltitude() - initialAltitude);
      if (liftOff)
        SendTelemetry(millis() - initialTime, 200);
      if (( currAltitude > liftoffAltitude) != true)
      {
        continuityCheckAsync();
        SendTelemetry(0, 200);
      }
      else
      {
        recordAltitude();
      }
      long savedTime = millis();
      while (allApogeeFiredComplete  && allMainFiredComplete )
      {
        // check if we have anything on the serial port
        if (SerialCom.available())
        {
          readVal = SerialCom.read();
          if (readVal != ';' )
          {
            if (readVal != '\n')
              commandbuffer[i++] = readVal;
          }
          else
          {
            commandbuffer[i++] = '\0';
            resetFlight();
            interpretCommandBuffer(commandbuffer);
          }
        }


        //beep last altitude every 10 second
        while ((millis() - savedTime) > 10000) {

          beginBeepSeq();

          if (config.beepingMode == 0)
            beepAltitude(apogeeAltitude * FEET_IN_METER);
          else
            beepAltitudeNew(apogeeAltitude * FEET_IN_METER);
          beginBeepSeq();

          if (config.beepingMode == 0)
            beepAltitude(mainAltitude * FEET_IN_METER);
          else
            beepAltitudeNew(mainAltitude * FEET_IN_METER);

          savedTime = millis();
        }
      }
    }

    while (SerialCom.available())
    {
      readVal = SerialCom.read();
      if (readVal != ';' )
      {
        if (readVal != '\n')
          commandbuffer[i++] = readVal;
      }
      else
      {
        commandbuffer[i++] = '\0';
        break;
      }
    }
  }
  interpretCommandBuffer(commandbuffer);
}


void interpretCommandBuffer(char *commandbuffer) {
  SerialCom.println((char*)commandbuffer);
    //get all flight data
   if (commandbuffer[0] == 'a')
  {
    SerialCom.print(F("Not implemented\n"));
    SerialCom.print(F("$OK;\n"));
  }
  //get altimeter config
  else if (commandbuffer[0] == 'b')
  {
    SerialCom.print(F("$start;\n"));

    printAltiConfig();

    SerialCom.print(F("$end;\n"));
  }
  //toggle continuity on and off
  else if (commandbuffer[0] == 'c')
  {
    if (noContinuity == false)
    {
      noContinuity = true;
      SerialCom.println(F("Continuity off \n"));
    }
    else
    {
      noContinuity = false;
      SerialCom.println(F("Continuity on \n"));
    }
  }
   //reset alti config
  else if (commandbuffer[0] == 'd')
  {
    defaultConfig();
    writeConfigStruc();
    initAlti();
  }
  //this will erase all flight
  else if (commandbuffer[0] == 'e')
  {
    SerialCom.println(F("Not implemented"));
    SerialCom.print(F("$OK;\n"));
  }
  //FastReading
  else if (commandbuffer[0] == 'f')
  {
    FastReading = true;
    SerialCom.print(F("$OK;\n"));

  }
  //FastReading off
  else if (commandbuffer[0] == 'g')
  {
    FastReading = false;
    SerialCom.print(F("$OK;\n"));
  }
  //hello
  else if (commandbuffer[0] == 'h')
  {
    //FastReading = false;
    SerialCom.print(F("$OK;\n"));
  }
   else if (commandbuffer[0] == 'i')
  {
    //exit continuity mode
  }
  //turn on or off the selected output
  else if (commandbuffer[0] == 'k')
  {
    char temp[2];
    boolean fire = true;

    temp[0] = commandbuffer[1];
    temp[1] = '\0';
    if (commandbuffer[2] == 'F')
      fire = false;

    if (atol(temp) > -1)
    {
      switch (atoi(temp))
      {
        case 1:
          fireOutput(pyroOut1, fire);
          break;
        case 2:
          fireOutput(pyroOut2, fire);
          break;
      }
    }
  }
  //list all flights
  else if (commandbuffer[0] == 'l')
  {
    SerialCom.println(F("Not implemented\n"));
    SerialCom.print(F("$OK;\n"));
  }
  //mainloop on/off
  else if (commandbuffer[0] == 'm')
  {
    if (commandbuffer[1] == '1') {
#ifdef SERIAL_DEBUG
      SerialCom.print(F("main Loop enabled\n"));
#endif
      //mainLoopEnable = true;
      FastReading = false;
    }
    else {
#ifdef SERIAL_DEBUG
      SerialCom.print(F("main loop disabled\n"));
#endif
      //mainLoopEnable = false;
      FastReading = true;
    }
    SerialCom.print(F("$OK;\n"));
  }
  //Number of flight
  else if (commandbuffer[0] == 'n')
  {
    SerialCom.println(F("Not implemented\n"));
    SerialCom.print(F("$OK;\n"));
  }
  // send test tram
  else if (commandbuffer[0] == 'o')
  { 
    SerialCom.print(F("$start;\n"));
    sendTestTram();
    SerialCom.print(F("$end;\n"));
  }
  //altimeter config param
  //write  config
  else if (commandbuffer[0] == 'p')
  {
    if (writeAltiConfigV2(commandbuffer)) {
      SerialCom.print(F("$OK;\n"));
    }
    else
      SerialCom.print(F("$KO;\n"));
  }
  else if (commandbuffer[0] == 'q')
  {
    writeConfigStruc();
    readAltiConfig();
    initAlti();
    SerialCom.print(F("$OK;\n"));
  }
  //this will read one flight
  else if (commandbuffer[0] == 'r')
  {
    SerialCom.println(F("Not implemented"));
    SerialCom.print(F("$OK;\n"));
  }
  //write altimeter config
  else if (commandbuffer[0] == 's')
  {
   /* if (writeAltiConfig(commandbuffer)) {

      SerialCom.print(F("$OK;\n"));
      readAltiConfig();
      initAlti();
    }
    else {
      SerialCom.print(F("$KO;\n"));
    }*/
    SerialCom.println(F("Not implemented"));
    SerialCom.print(F("$OK;\n"));
  }
  else if (commandbuffer[0] == 't')
  {
    //reset config
    defaultConfig();
    writeConfigStruc();
    initAlti();
    SerialCom.print(F("config reseted\n"));
  }
  // Recording
  else if (commandbuffer[0] == 'w')
  {
    SerialCom.println(F("Not implemented \n"));
    SerialCom.print(F("$OK;\n"));
  }
  //telemetry on/off
  else if (commandbuffer[0] == 'y')
  {
    if (commandbuffer[1] == '1') {
      SerialCom.print(F("Telemetry enabled\n"));
      telemetryEnable = true;
    }
    else {
      SerialCom.print(F("Telemetry disabled\n"));
      telemetryEnable = false;
    }
    SerialCom.print(F("$OK;\n"));
  }
  else if (commandbuffer[0] == ' ')
  {
    SerialCom.print(F("$K0;\n"));
  }
  else
  {
    // Serial.println(F("Unknown command" ));
    SerialCom.print(F("$UNKNOWN;"));
    SerialCom.println(commandbuffer[0]);
  }
}

void resetFlight() {
  // re-nitialise all flight related global variables
  allApogeeFiredComplete  = false;
  allMainFiredComplete = false;
  allTimerFiredComplete = false;
  allLiftOffFiredComplete = false;
  allLandingFiredComplete = false;
  liftOff = false;
  Output1Fired = false;
  Output2Fired = false;

}


void fireOutput(int pin, boolean fire) {
  if (fire)
    digitalWrite(pin, HIGH );
  else
    digitalWrite(pin, LOW );
}

/*
    Test tram
*/
void sendTestTram() {

  char altiTest[100] = "";
  char temp[10] = "";

  strcat(altiTest, "testTrame," );
  strcat(altiTest, "Bear altimeters are the best!!!!,");
  unsigned int chk;
  chk = msgChk(altiTest, sizeof(altiTest));
  sprintf(temp, "%i", chk);
  strcat(altiTest, temp);
  strcat(altiTest, ";\n");
  SerialCom.print("$");
  SerialCom.print(altiTest);

}
