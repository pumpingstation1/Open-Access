/*
 * Open Source RFID Access Controller - v3 Standard Beta hardware
 *
 * 4/3/2011 v1.4 beta 1
 * Last build test with Arduino v1.01
 * Arclight - arclight@23.org
 * Danozano - danozano@gmail.com
 *
 * Notice: This is free software and is probably buggy. Use it at
 * at your own peril.  Use of this software may result in your
 * doors being left open, your stuff going missing, or buggery by
 * high seas pirates. No warranties are expressed on implied.
 * You are warned.
 *
 *
 * For latest downloads, including Eagle CAD files for the hardware, check out
 * http://code.google.com/p/open-access-control/downloads/list
 *
 * Latest update moves strings to PROGMEM to free up memory and adds a 
 * console password feature.
 * 
 *
 * This program interfaces the Arduino to RFID, PIN pad and all
 * other input devices using the Wiegand-26 Communications
 * Protocol. It is recommended that the keypad inputs be
 * opto-isolated in case a malicious user shorts out the 
 * input device.
 * Outputs go to a Darlington relay driver array for door hardware/etc control.
 * Analog inputs are used for alarm sensor monitoring.  These should be
 * isolated as well, since many sensors use +12V. Note that resistors of
 * different values can be used on each zone to detect shorting of the sensor
 * or wiring.
 *
 * Version 3.00+ of the hardware implements these features and emulates an
 * Arduino Duemilanova.
 * The "standard" hardware uses the MC23017 i2c 16-channel I/O expander.
 * I/O pins are addressed in two banks, as GPA0..7 and GPB0..7
 *
 * Relay outpus on digital pins:  D7, D8, GPB0, GPB1
 * DS1307 Real Time Clock (I2C):  A4 (SDA), A5 (SCL)
 * Analog pins (for alarm):       A0,A1,A2,A3 
 * Reader 1:                      D2,D3
 * Reader 2:                      D4,D5
 * RS485 TX enable / RX disable:  D6
 * Reader1 Green LED:             GPB2
 * Reader1 Buzzer:                GPB3
 * Reader2 Green LED:             GPB4 
 * Reader2 Buzzer:                GPB5
 * RS-485 TX LED:                 GPB6
 * RS-485 RX LED:                 GPB7
 * LCD RS:                        GPA0
 * LCD EN:                        GPA1
 * LCD D4..D7:                    GPA2..GPA5
 * Status LED:                    GPA6
 * Ethernet/SPI:                  D10..D1313  (Not used, reserved for the Ethernet shield)
 * 
 * Quickstart tips: 
 * Set the console password(PRIVPASSWORD) value to a numeric DEC or HEX value.
 * Define the static user list by swiping a tag and copying the value received into the #define values shown below 
 * Compile and upload the code, then log in via serial console at 57600,8,N,1
 *
 */
#include "user.h"         // User preferences file. Use this to select hardware options, passwords, etc.
#include <Wire.h>         // Needed for I2C Connection to the DS1307 date/time chip
#include <EEPROM.h>       // Needed for saving to non-voilatile memory on the Arduino.
#include <avr/pgmspace.h> // Allows data to be stored in FLASH instead of RAM

/*
#include <Ethernet.h>     // Ethernet stuff, comment out if not used.
 #include <SPI.h>          
 #include <Server.h>
 #include <Client.h>
 */


#include <DS1307.h>             // DS1307 RTC Clock/Date/Time chip library
#include <WIEGAND26.h>          // Wiegand 26 reader format libary

#ifdef MCU328
#include <PCATTACH.h>           // Pcint.h implementation, allows for >2 software interupts.
#endif

#ifdef MCPIOXP
#include <Adafruit_MCP23017.h>  // Library for the MCP23017 i2c I/O expander
#endif

#ifdef AT24EEPROM
#include <E24C1024.h>           // AT24C i2C EEPOROM library
#endif

#ifdef LCDBOARD
#include <cLCD.h>               // Special LCD library for use with I/O expander
#endif

#ifdef STDLOG
#include "stdlog.h"
#endif

#ifdef RHYSLOG
#include "rhyslog.h"
#endif

#define EEPROM_ALARM 0                  // EEPROM address to store alarm triggered state between reboots (0..511)
#define EEPROM_ALARMARMED 1             // EEPROM address to store alarm armed state between reboots
#define EEPROM_ALARMZONES 20            // Starting address to store "normal" analog values for alarm zone sensor reads.


#define EEPROM_FIRSTUSER 24
#define EEPROM_LASTUSER 1024
#define NUMUSERS  ((EEPROM_LASTUSER - EEPROM_FIRSTUSER)/5)  //Define number of internal users (200 for UNO/Duemillanova)


#ifdef HWV3STD                          // Use these pinouts for the v3 Standard hardware
#define DOORPIN1       7                // Define the pin for electrified door 1 hardware. D7=first relay
#define DOORPIN2       8                // Define the pin for electrified door 2 hardware  D8=second relay
#define ALARMSTROBEPIN 8                // Define the "non alarm: output pin. Can go to a strobe, small chime, etc. Uses GPB0 (MCP pin 8).
#define ALARMSIRENPIN  9                // Define the alarm siren pin. This should be a LOUD siren for alarm purposes. Uses GPB1 (MCP pin9).
#define READER1GRN     10
#define READER1BUZ     11
#define READER2GRN     12
#define READER2BUZ     13

#define RS485TXLED     14
#define RS485RXLED     15
#define RS485ENA        6               // Arduino Pin D6
#define STATUSLED       6
#define R1ZERO          2
#define R1ONE           3
#define R2ZERO          4
#define R2ONE           5
#endif

uint8_t reader1Pins[]={
  R1ZERO, R1ONE};           // Reader 1 pin definition
uint8_t reader2Pins[]={
  R2ZERO, R2ONE};           // Reade  2 pin definition

const uint8_t analogsensorPins[] = {
  0,1,2,3};    // Alarm Sensors connected to other analog pins

/*  Global Boolean values
 *
 */
bool door1Locked=true;                          // Keeps track of whether the doors are supposed to be locked right now
bool door2Locked=true;
boolean doorChime=false;                        // Keep track of when door chime last activated
boolean doorClosed=false;                       // Keep track of when door last closed for exit delay
boolean sensor[4]={
  false};                      // Keep track of tripped sensors, do not log again until reset.



/*  Global Timers
 *
 */
unsigned long door1locktimer=0;                 // Keep track of when door is supposed to be relocked
unsigned long door2locktimer=0;                 // after access granted.
unsigned long alarmDelay=0;                     // Keep track of alarm delay. Used for "delayed activation" or level 2 alarm.
unsigned long alarmSirenTimer=0;                // Keep track of how long alarm has gone off
unsigned long consolefailTimer=0;               // Console password timer for failed logins
unsigned long sensorDelay[2]={
  0};               // Used with sensor[] above, but sets a timer for 2 of them. Useful for logging
// motion detector hits for "occupancy check" functions.


#define numUsers (sizeof(superUserList)/sizeof(long))                  //User access array size (used in later loops/etc)
#define NUMDOORS (sizeof(doorPin)/sizeof(uint8_t))
#define numAlarmPins (sizeof(analogsensorPins)/sizeof(uint8_t))

//Other global variables
uint8_t second, minute, hour, dayOfWeek, dayOfMonth, month, year;     // Global RTC clock variables. Can be set using DS1307.getDate function.

uint8_t alarmActivated = EEPROM.read(EEPROM_ALARM);                   // Read the last alarm state as saved in eeprom.
uint8_t alarmArmed = EEPROM.read(EEPROM_ALARMARMED);                  // Alarm level variable (0..5, 0==OFF) 

uint8_t consoleFail=0;                          // Tracks failed console logins for lockout

// Enable the door access readers.
volatile long reader1 = 0;
volatile int  reader1Count = 0;
volatile long reader2 = 0;
volatile int  reader2Count = 0;
int userMask1=0;
int userMask2=0;
boolean keypadGranted=0;                                       // Variable that is set for authenticated users to use keypad after login

//volatile long reader3 = 0;                                   // Uncomment if using a third reader.
//volatile int  reader3Count = 0;

unsigned long keypadTime = 0;                                  // Timeout counter for  reader with key pad
unsigned long keypadValue=0;


// Serial terminal buffer (needs to be global)
char inString[40]={
  0};                                         // Size of command buffer (<=128 for Arduino)
uint8_t inCount=0;
boolean privmodeEnabled = false;                               // Switch for enabling "priveleged" commands



/* Create an instance of the various C++ libraries we are using.
 */

DS1307 ds1307;        // RTC Instance
WIEGAND26 wiegand26;  // Wiegand26 (RFID reader serial protocol) library

#ifdef MCPIOXP
Adafruit_MCP23017 mcp;
#endif

#ifdef MCU328
PCATTACH pcattach;    // Software interrupt library
#endif

#ifdef LCDBOARD
cLCD lcd;
#endif

logStruct logger = { 
  255, 255, 255, 255, 255 }; // keep a 5 byte logger struct in global memory

void setup(){           // Runs once at Arduino boot-up

    Wire.begin();   // start Wire library as I2C-Bus Master
  mcp.begin();      // use default address 0

  pinMode(2,INPUT);                // Initialize the Arduino built-in pins
  pinMode(3,INPUT);
  pinMode(4,INPUT);
  pinMode(5,INPUT);
  pinMode(DOORPIN1, OUTPUT); 
  pinMode(DOORPIN2, OUTPUT);
  pinMode(6,OUTPUT);

  for(int i=0; i<=15; i++)        // Initialize the I/O expander pins
  {
    mcp.pinMode(i, OUTPUT);
  }

  mcp.digitalWrite(RS485TXLED, HIGH);     // Set the LEDs to off
  mcp.digitalWrite(RS485RXLED, HIGH);  
  digitalWrite(RS485ENA, HIGH);           // Set the RS485 chip to HIGH (not asserted)


  /* Attach pin change interrupt service routines from the Wiegand RFID readers
   */
#ifdef MCU328
  pcattach.PCattachInterrupt(reader1Pins[0], callReader1Zero, CHANGE); 
  pcattach.PCattachInterrupt(reader1Pins[1], callReader1One,  CHANGE);  
  pcattach.PCattachInterrupt(reader2Pins[1], callReader2One,  CHANGE);
  pcattach.PCattachInterrupt(reader2Pins[0], callReader2Zero, CHANGE);
#endif

  //Clear and initialize readers
  wiegand26.initReaderOne(); //Set up Reader 1 and clear buffers.
  wiegand26.initReaderTwo(); 

  digitalWrite(DOORPIN1, LOW);                               // Sets the relay outputs to LOW (relays off)
  digitalWrite(DOORPIN2, LOW);
  mcp.digitalWrite(ALARMSTROBEPIN, LOW);
  mcp.digitalWrite(ALARMSIRENPIN, LOW);   

  //Initialize LEDs


  //ds1307.setDateDs1307(0,18,17,2,25,6,12);         
  /*  Sets the date/time (needed once at commissioning)
   
   uint8_t second,        // 0-59
   uint8_t minute,        // 0-59
   uint8_t hour,          // 1-23
   uint8_t dayOfWeek,     // 1-7
   uint8_t dayOfMonth,    // 1-28/29/30/31
   uint8_t month,         // 1-12
   uint8_t year);          // 0-99
   */



  Serial.begin(UBAUDRATE);	               	       // Set up Serial output at 8,N,1,57600bps
  logger.l1 = 1;
  logSystem(logger);
  chirpAlarm(1);                               // Chirp the alarm to show system ready.

#ifdef LCDBOARD
  lcd.begin(16,2);
  lcd.setCursor(0,0);
  lcd.print("Open Access ");
  lcd.print(VERSION);
  delay(500);
  lcd.clear();
#endif
  //Set up the MCP23017 IO expander and initialize
#ifdef MCPIOXP 
  mcp.digitalWrite(STATUSLED, LOW);           // Turn the status LED green
#endif

  //hardwareTest(20);                          // IO Pin testing routing (use to check your inputs with hi/lo +(5-12V) sources)
  // Also checks relays

}




void loop()                                     // Main branch, runs over and over again
{                         


  readCommand();                                 // Check for commands entered at serial console


  /* Check if doors are supposed to be locked and lock/unlock them 
   * if needed. Uses global variables that can be set in other functions.
   */

  if(((millis() - door1locktimer) >= DOORDELAY) && (door1locktimer !=0))
  { 
    if(door1Locked==true){
      doorLock(1);
      door1locktimer=0;    


    }

    else {                        
      doorUnlock(1); 
      door1locktimer=0;
    }                         
  }



  if(((millis() - door2locktimer) >= DOORDELAY) && (door2locktimer !=0))
  { 
    if(door2Locked==true) {
      doorLock(2); 
      door2locktimer=0;
    }

    else {
      doorUnlock(2); 
      door2locktimer=0;
    }   

  }   
#ifdef LCDBOARD
  lcdStatus(1,door1Locked);
  lcdStatus(2,door2Locked);
#endif

  /*  Set optional "failsafe" time to lock up every night.
   */

  ds1307.getDateDs1307(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);   // Get the current date/time

  if(hour==23 && minute==59 && door1Locked==false){
    doorLock(1);
    door1Locked==true;      
    logger.l1 = 1;
    logger.l2 = 1;
    logger.l3 = 2;
    logger.detail = 2359;
    logDoor(logger);
  }

  // Notes: RFID polling is interrupt driven, just test for the reader1Count value to climb to the bit length of the key
  // change reader1Count & reader1 et. al. to arrays for loop handling of multiple reader output events
  // later change them for interrupt handling as well!
  // currently hardcoded for a single reader unit

  /* This code checks a reader with a 26-bit keycard input. Use the second routine for readers with keypads.  
   * A 5-second window for commands is opened after each successful key access read.
   */

  if(reader1Count >= 26){                           //  When tag presented to reader1 (No keypad on this reader)
    logger.l1 = 1;
    logger.l2 = 1;
    logAccess(logger,reader1);                       //  write log entry to serial port


    /* Check a user's security level and take action as needed. The
     *  usermask is a variable from 0..255. By default, 0 and 255 are for
     *  locked out users or uninitialized records.
     *  Modify these for each door as needed.
     */

    userMask1=checkUser(reader1);    

    if(userMask1>=0) {    

      switch(userMask1) {

      case 0:                                      // No outside privs, do not log denied.
        {                                           // authenticate only.
          logger.l1 = 1;
          logger.l2 = 2;
          logAccess(logger,reader1);
          break;
        }

      case 20:                                                // Example Limited hours user
        {                                                      // Can enter from 5:00pm to 11:00pm
          ds1307.getDateDs1307(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);    
          if((hour >=17) && (hour <=23)){
            logger.l1 = 1;
            logger.l2 = 2;
            logAccess(logger,reader1);                    // Log and unlock door 2
            alarmState(0);
            armAlarm(0);                                     //  Deactivate Alarm
            //   chirpAlarm(1);                            
            door1locktimer=millis();
            doorUnlock(1);                                   // Unlock the door.
          }
          break;
        }      

      case 255:                                              // Locked out user     
        {
          logger.l1 = 1;
          logger.l2 = 5;
          logAccess(logger,userMask1);     
          break;
        }

      default:  
        {            
          logger.l1 = 1;
          logger.l2 = 2;
          logAccess(logger,reader1);        // Log and unlock door 1
          alarmState(0);
          armAlarm(0);                            //  Deactivate Alarm                  
          door1locktimer=millis();
          doorUnlock(1);                          // Unlock the door.
          break;
        }
      }                                      

    }
    else 
    {                                           
      if(checkSuperuser(reader1) >= 0) {              // Check if a superuser, grant access.
        logger.l1 = 1;
        logger.l2 = 6;
        logAccess(logger,reader1);                 // Log and unlock door 1
        logger.l2 = 2;
        logAccess(logger,reader1);
        alarmState(0);
        armAlarm(0);                               //  Deactivate Alarm
        door1locktimer=millis();
        doorUnlock(1);                             // Unlock the door.
      }
      else{                                
        logger.l1 = 1;
        logger.l2 = 3;
        logAccess(logger,reader1);                   // No tickee, no laundree
        chirpAlarm(1);
      }
    }

    wiegand26.initReaderOne();                     // Reset for next tag scan

  }

  if(reader2Count >= 26){                                // Tag presented to reader 2
    logger.l1 = 2;
    logger.l2 = 1;
    logAccess(logger,reader2);                            // Write log entry to serial port
    chirpAlarm(1);                                       // Chirp alarm to show that tag input done              
    // CHECK TAG IN OUR LIST OF USERS. -1 = no match                                  
    keypadGranted=false;                                   // Reset the keypad authorized variable

      userMask2=checkUser(reader2);    

    if(userMask2>=0){    
      switch(userMask2) {

      case 0:                         // No outside privs, do not log denied.
        {                              // authenticate and log only.
          logger.l1 = 2;
          logger.l2 = 2;
          logAccess(logger,reader2);
          break;
        }

      case 10:                         // Authenticating immediately locks up and arms alarm
        {                              // 
          logger.l1 = 2;
          logger.l2 = 2;
          logAccess(logger,reader2);
          //runCommand(0x2);

          break;
        }

      case 20:                                               //Limited hours user
        {
          ds1307.getDateDs1307(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);    
          if((hour >=17) && (hour <=23)){
            logger.l1 = 2;
            logger.l2 = 2;
            logAccess(logger,reader2);                    // Log
            alarmState(0);
            armAlarm(0);                                     //  Deactivate Alarm                           
            door2locktimer=millis();
            doorUnlock(2);                                   // Unlock the door.
            keypadGranted=1;
          }
          break;
        }

      case 255:                                               // Locked out      
        {
          logger.l1 = 2;
          logger.l2 = 5;
          logAccess(logger,userMask2);
          break;
        }

      default:  
        {           
          logger.l1 = 2;
          logger.l2 = 2;
          logAccess(logger,reader2);           // Log
          alarmState(0);
          armAlarm(0);                            //  Deactivate Alarm                          
          door1locktimer=millis();
          doorUnlock(1);                          // Unlock the door.
          keypadGranted=1;
          Serial.println(keypadGranted);
          break;
        }
      }                                      

    }
    else 
    {                                             
      if(checkSuperuser(reader2) >= 0) {              // Check if a superuser, grant access.
        logger.l1 = 1;
        logger.l2 = 2;
        logAccess(logger,reader2);                 // Log and unlock door 2
        alarmState(0);
        armAlarm(0);                              //  Deactivate Alarm
        chirpAlarm(1);                            
        door1locktimer=millis();
        doorUnlock(1);                            // Unlock the door.
        keypadGranted=1;
      }
      else{
        logger.l1 = 2;
        logger.l2 = 3;                                
        logAccess(logger,reader2);                 //  no tickee, no laundree

      }
    }

    wiegand26.initReaderTwo();                   //  Reset for next tag scan
    unsigned long keypadTime=0;                  //  Timeout counter for  reader with key pad
    long keypadValue=0;
    keypadTime=millis();  
                                   
    if(keypadGranted==1) 
    {

      while((millis() - keypadTime)  <=KEYPADTIMEOUT){

        // If access granted, open 5 second window for pin pad commands.
        if(reader2Count >=4){
          if(reader2 !=0xB){                                  // Pin pad command can be any length, terminated with '#' on the keypad.
            if(keypadValue ==0){                              // This 0..9, A..F encoding works with many Wiegand-format keypad or reader 
              keypadValue = reader2;                          // plus keypad units.

            }
            else if(keypadValue !=0) {
              keypadValue = keypadValue <<4;
              keypadValue |= reader2;               
            }
            wiegand26.initReaderTwo();                         //Reset reader two and move on.
          } 
          else break;

        }

      }
      logger.l1 = 2;
      logger.l2 = 4;
      logAccess(logger,keypadValue);
      runCommand(keypadValue);                              // Run any commands entered at the keypads.
      wiegand26.initReaderTwo();

    }
    wiegand26.initReaderTwo();                    
  } 


  /* Check physical sensors with 
   the logic below. Behavior is based on
   the current alarmArmed value.
   0=disarmed 
   1=armed
   2=
   3=
   4=door chime only (Unlock DOOR1, Check zone 0/chirp alarm if active)
   
   Modify the alarm sequence to meet your needs.
   */

  switch(alarmArmed) {


  case 0:
    {
      break;                                        // Alarm is not armed, do nothing.  
    }

  case 1:                                       // Alarm is armed
    {


      if(alarmActivated==0){                       // If alarm is armed but not currently alarming, check sensor zones.

        if(pollAlarm(0) == 1 ){                   // If this zone is tripped, immediately set Alarm State to 2 (alarm delay).
          alarmState(2);                        // Also starts the delay timer    
          alarmDelay=millis();
          if(sensor[0]==false) {                // Only log and save if sensor activation is new.
            logger.l1 = 0;
            logger.l2 = 1;
            logAlarm(logger);
            EEPROM.write(EEPROM_ALARM,0);        // Save the alarm sensor tripped to eeprom                                      
            sensor[0]=true;                      // Set value to not log this again                                                                        
          }
        } 
        if(pollAlarm(1) == 1 ){                  // If this zone is tripped, immediately set Alarm State to 1 (alarm immediate).
          alarmState(1);      
          if(sensor[1]==false) {                // Only log and save if sensor activation is new.
            logger.l1 = 1;
            logger.l2 = 1;
            logAlarm(logger);
            EEPROM.write(EEPROM_ALARM,1);        // Save the alarm sensor tripped to eeprom                                     
            sensor[1]=true;                      // Set value to not log this again
          }  
        }
        if(pollAlarm(2) == 1 ){                  // If this zone is tripped, immediately set Alarm State to 1 (alarm immediate).
          alarmState(1);      
          if(sensor[2]==false) {                // Only log and save if sensor activation is new.
            logger.l1 = 2;
            logger.l2 = 1;
            logAlarm(logger);
            EEPROM.write(EEPROM_ALARM,2);        // Save the alarm sensor tripped to eeprom                                     
            sensor[2]=true;                      // Set value to not log this again
          }    

        } 

        if(pollAlarm(3) == 1 ){                   // If this zone is tripped, immediately set Alarm State to 2 (alarm delay).
          alarmState(2);                        // Also starts the delay timer    
          alarmDelay=millis();
          if(sensor[3]==false) {                // Only log and save if sensor activation is new.
            logger.l1 = 3;
            logger.l2 = 1;
            logAlarm(logger);
            EEPROM.write(EEPROM_ALARM,3);        // Save the alarm sensor tripped to eeprom                                      
            sensor[3]=true;                      // Set value to not log this again                                                                        
          }
        }    



      }
      if(alarmActivated==1)  {                         // If alarm is actively going off (siren/strobe) for 10 min (6e5=10min)
        if(millis()-alarmSirenTimer >=3.6e6)            // Check for alarm interval expired and turn off if needed
        {
          mcp.digitalWrite(ALARMSIRENPIN,LOW);              // Turn on the chime instead  
          mcp.digitalWrite(ALARMSTROBEPIN,HIGH);     
        }
      }  

      if(alarmActivated==2)  {                         // If alarm is activated on delay, take this action
        if(millis()-alarmDelay >=60000)                 // Turn on the siren once delay exceeds 60sec.
        {
          alarmState(1);                          
        }
      }  


      break;

    }

  case 4: 
    {                                                // Door chime mode

      if((pollAlarm(3) !=0) && (doorChime==false)) {   // Only activate door chime once per opening
        chirpAlarm(3);
        logger.l1 = 2;                  
        logSystem(logger);
        doorChime=true;   
      }
      if(pollAlarm(3) ==0){
        doorChime=false;   
      }
      break;    

    }

  default: 
    {
      break;  
    }
  }

  // Log all motion detector activations regardless of alarm armed state. Useful for "occupancy detection"

  if(pollAlarm(0) == 1 ){                  // If this zone is tripped, log the action only
    //  if(sensor[0]==false) 
    if((millis() - sensorDelay[0]) >=7500) {
      logger.l1 = 0;
      logger.l2 = 1;
      logAlarm(logger);
      sensorDelay[0]=millis();                                                                  
      sensor[0]=true;      
    }                 // Set value to not log this again for 7.5s              
  }

  if(pollAlarm(1) == 1 ){                  // If this zone is tripped, log the action only
    //   if(sensor[1]==false) 
    if((millis() - sensorDelay[1]) >=7500) {
      logger.l1 = 1;
      logger.l2 = 1;
      logAlarm(logger);
      sensorDelay[1]=millis();                                                            
      sensor[1]=true;                       // Set value to not log this again for 7.5s
    }           
  }
} // End of loop()

/* Alarm System Functions - Modify these as needed for your application. 
 Sensor zones may be polled with digital or analog pins. Unique reader2
 resistors can be used to check more zones from the analog pins.
 */

void alarmState(uint8_t alarmLevel) {                    //Changes the alarm status based on this flow

  logger.l1 = 255;
  logger.l2 = 3;
  logger.l3 = alarmLevel;
  logAlarm(logger); 
  switch (alarmLevel) {                              
  case 0: 
    {                                                 // If alarmLevel == 0 turn off alarm.   
      mcp.digitalWrite(ALARMSIRENPIN, LOW);
      mcp.digitalWrite(ALARMSTROBEPIN, LOW);
      alarmActivated = alarmLevel;                    //Set global alarm level variable
      break;  
    }        
  case 1: 
    { 
      mcp.digitalWrite(ALARMSIRENPIN, HIGH);               // If alarmLevel == 1 turn on strobe lights and siren
      //    mcp.digitalWrite(ALARMSTROBEPIN, HIGH);            // Optionally activate yoru strobe/chome
      alarmSirenTimer=millis();
      alarmActivated = alarmLevel;                    //Set global alarm level variable
      logger.l1 = 255;
      logger.l2 = 2;
      logAlarm(logger);

      break;  
    }        

  case 2:                                        
    {
      mcp.digitalWrite(ALARMSTROBEPIN, HIGH);   
      alarmActivated = alarmLevel;
      break;    
    }

  case 3:                                        
    {

      alarmActivated = alarmLevel;
      break;    
    }
    /*
      case 4: {
     vaporize_intruders(STUN);
     break;
     }
     
     case 5: {
     vaporize_intruders(MAIM);
     }  etc. etc. etc.
     break;
     */

  default: 
    {                                            // Exceptional cases kill alarm outputs
      mcp.digitalWrite(ALARMSIRENPIN, LOW);          // Turn off siren and strobe
      // mcp.digitalWrite(ALARMSTROBEPIN, LOW);        
      break;
    } 




  }

  if(alarmActivated != EEPROM.read(EEPROM_ALARM)){    // Update eeprom value
    EEPROM.write(EEPROM_ALARM,alarmActivated); 
  }

}  //End of alarmState()

void chirpAlarm(uint8_t chirps){            // Chirp the siren pin or strobe to indicate events.      
  for(uint8_t i=0; i<chirps; i++) {
    mcp.digitalWrite(ALARMSTROBEPIN, HIGH);
    delay(100);
    mcp.digitalWrite(ALARMSTROBEPIN, LOW);
    delay(200);                              
  }    
}                                   

void chirpReader(uint8_t chirps, uint8_t reader){            // Chirp the siren pin or strobe to indicate events.      
  for(uint8_t i=0; i<chirps; i++) {
    if(reader==1)
    {
      mcp.digitalWrite(READER1BUZ, LOW);
      delay(100);
      mcp.digitalWrite(READER1BUZ, HIGH);
      delay(200);                              
    }
    if(reader==2)
    {
      mcp.digitalWrite(READER2BUZ, LOW);
      delay(100);
      mcp.digitalWrite(READER2BUZ, HIGH);
      delay(200);                              
    }
  }
}




uint8_t pollAlarm(uint8_t input){

  // Return 1 if sensor shows < pre-defined voltage.
  delay(20);
  if(abs((analogRead(analogsensorPins[input])/4) - EEPROM.read(EEPROM_ALARMZONES+input)) >SENSORTHRESHOLD){
    return 1;

  }
  else return 0;
}

void trainAlarm(){                       // Train the system about the default states of the alarm pins.
  armAlarm(0);                           // Disarm alarm first
  alarmState(0);

  int temp[5]={
    0  };
  int avg;

  for(int i=0; i<numAlarmPins; i++) {         

    for(int j=0; j<5;j++){                          
      temp[j]=analogRead(analogsensorPins[i]);
      delay(50);                                         // Give the readings time to settle
    }
    avg=((temp[0]+temp[1]+temp[2]+temp[3]+temp[4])/20);  // Average the results to get best values
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" ");
    Serial.print("value:");
    Serial.println(avg);
    EEPROM.write((EEPROM_ALARMZONES+i),uint8_t(avg));   //Save results to EEPROM
    avg=0;
  }

  logDate();
  PROGMEMprintln(alarmtrainMessage);


}

void armAlarm(uint8_t level){                       // Arm the alarm and set to level
  alarmArmed = level;
  logger.l1 = 255;
  logger.l2 = 4;
  logger.l3 = level;
  logAlarm(logger);

  sensor[0] = false;                             // Reset the sensor tripped values
  sensor[1] = false;
  sensor[2] = false;
  sensor[3] = false;

  if(level != EEPROM.read(EEPROM_ALARMARMED)){ 
    EEPROM.write(EEPROM_ALARMARMED,level); 
  }
}


/* Access System Functions - Modify these as needed for your application. 
 These function control lock/unlock and user lookup.
 */

int checkSuperuser(long input){       // Check to see if user is in the user list. If yes, return their index value.
  int found=-1;
  for(int i=0; i<=numUsers; i++){   
    if(input == superUserList[i]){
      found=i;
      return found;    
    }
  }                   

  return found;             //If no, return -1
}


void doorUnlock(int input) {          //Send an unlock signal to the door and flash the Door LED

  uint8_t dp=1;
  uint8_t dpLED=1;

  if(input == 1) 
  {
    dp=DOORPIN1; 
    dpLED=READER1GRN;
  }
  else 
  {
    dp=DOORPIN2;
    dpLED=READER2GRN;
  }
  digitalWrite(dp, HIGH);
  mcp.digitalWrite(dpLED, HIGH);
  logger.l1 = input;
  logger.l2 = 2;
  logDoor(logger);

#ifdef LCDBOARD
  lcdStatus(input, false);
#endif  
}

void doorLock(int input) {          //Send an unlock signal to the door and flash the Door LED
  uint8_t dp=1;
  uint8_t dpLED=1;
  if(input == 1) 
  {
    dp=DOORPIN1; 
    dpLED=READER1GRN;
  }
  else 
  {
    dp=DOORPIN2;
    dpLED=READER2GRN;
  }

  digitalWrite(dp, LOW);
  mcp.digitalWrite(dpLED, LOW);
  logger.l1 = input;
  logger.l2 = 1;
  logger.l3 = 0;
  logDoor(logger);
#ifdef LCDBOARD
  lcdStatus(input, true);
#endif  
}
void lockall() {                      //Lock down all doors. Can also be run periodically to safeguard system.

  digitalWrite(DOORPIN1, LOW);
  mcp.digitalWrite(READER1GRN, LOW);
  digitalWrite(DOORPIN2,LOW);
  mcp.digitalWrite(READER2GRN, LOW);
  door1Locked=true;
  door2Locked=true;
  PROGMEMprintln(doorslockedMessage);
  logger.l1 = 0;
  logger.l2 = 1;
  logger.l3 = 3;
  logDoor(logger);

}

/* Logging Functions - Modify these as needed for your application. 
 Logging may be serial to USB or via Ethernet (to be added later)
 */

void logDate()
{
  ds1307.getDateDs1307(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
  Serial.print(hour, DEC);
  Serial.print(":");
  Serial.print(minute, DEC);
  Serial.print(":");
  Serial.print(second, DEC);
  Serial.print("  ");
  Serial.print(month, DEC);
  Serial.print("/");
  Serial.print(dayOfMonth, DEC);
  Serial.print("/");
  Serial.print(year, DEC);
  Serial.print(' ');

  switch(dayOfWeek){

  case 1:
    {
      Serial.print("SUN");
      break;
    }
  case 2:
    {
      Serial.print("MON");
      break;
    }
  case 3:
    {
      Serial.print("TUE");
      break;
    }
  case 4:
    {
      Serial.print("WED");
      break;
    }
  case 5:
    {
      Serial.print("THU");
      break;
    }
  case 6:
    {
      Serial.print("FRI");
      break;
    }
  case 7:
    {
      Serial.print("SAT");
      break;
    }  
  }

  Serial.print(" ");

}                       

#ifdef LCDBOARD
void lcdStatus(uint8_t door, bool status)
{
  delay(10);
  if(door == 1) {
    lcd.setCursor(0,0);
  }
  else {
    lcd.setCursor(0,1);
  }
  lcd.print("Door ");  
  lcd.print(door, DEC);
  if(status==false) {
    lcd.print(" unlocked.");
  }
  else {
    lcd.print("   locked.");    
  }

}
#endif

void logprivFail() {
  //  Serial.println("Priv mode disabled");
  PROGMEMprintln(privsdeniedMessage);
}


void hardwareTest(long iterations)
{

  /* Hardware testing routing. Performs a read of all digital inputs and
   * a write to each relay output. Also reads the analog value of each
   * alarm pin. Use for testing hardware. Wiegand26 readers should read 
   * "HIGH" or "1" when connected.
   */

  pinMode(2,INPUT);
  pinMode(3,INPUT);
  pinMode(4,INPUT);
  pinMode(5,INPUT);

  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);

  for(long counter=1; counter<=iterations; counter++) {                                  // Do this number of times specified
    digitalWrite(6, HIGH);
    logDate();

    Serial.print("\n"); 
    Serial.println("Pass: "); 
    Serial.println(counter); 
    Serial.print("Input 2:");                    // Digital input testing
    Serial.println(digitalRead(2));
    Serial.print("Input 3:");
    Serial.println(digitalRead(3));
    Serial.print("Input 4:");
    Serial.println(digitalRead(4));
    Serial.print("Input 5:");
    Serial.println(digitalRead(5));
    Serial.print("Input A0:");                   // Analog input testing
    Serial.println(analogRead(0));
    Serial.print("Input A1:");
    Serial.println(analogRead(1));
    Serial.print("Input A2:");
    Serial.println(analogRead(2));
    Serial.print("Input A3:");
    Serial.println(analogRead(3));
    delay(5000);

    for(int i=0; i<=9; i++)                        // Set all relay/LCD outputs low on MCP chip (low=off)
    {
      mcp.digitalWrite(i,LOW);                     
    }   
    for(int i=10; i<=15; i++)                        // Set all LED outputs high on MCP chip (high=off)
    {
      mcp.digitalWrite(i,HIGH);                     
    }     

    mcp.digitalWrite(8,HIGH);                       // Close relays
    mcp.digitalWrite(9,HIGH);
    digitalWrite(7,HIGH);
    digitalWrite(8,HIGH);    

    for(int i=10; i<=15; i++)                        // Turn on LEDs
    {
      mcp.digitalWrite(i,LOW);                     
    }                          // Turn on LEDs
    Serial.println("Relays and LEDs on");


    delay(2000);
    mcp.digitalWrite(8,LOW);                       // Close relays
    mcp.digitalWrite(9,LOW);
    digitalWrite(7,LOW);
    digitalWrite(8,LOW);
    for(int i=10; i<=15; i++)                        // Turn off LEDs
    {
      mcp.digitalWrite(i,HIGH);                     
    }             
    Serial.println("Relays and LEDs off");

  }
}

void clearUsers()    //Erases all users from EEPROM
{
  for(int i=EEPROM_FIRSTUSER; i<=EEPROM_LASTUSER; i++){
    EEPROM.write(i,0);  
    logger.l1 = 3;
    logSystem(logger);  
  }
}

void addUser(int userNum, uint8_t userMask, unsigned long tagNumber)       // Modifies a user an entry in the local database.
{                                                                       // Users number 0..NUMUSERS
  int offset = (EEPROM_FIRSTUSER+(userNum*5));                          // Find the offset to write this user to
  uint8_t EEPROM_buffer[5] ={
    0  };                                           // Buffer for creating the 4 byte values to write. Usermask is stored in byte 5.

  if((userNum <0) || (userNum > NUMUSERS)) {                            // Do not write to invalid EEPROM addresses.

    logger.l1 = 4;
    logSystem(logger);
  }
  else
  {




    EEPROM_buffer[0] = uint8_t(tagNumber &  0xFFF);   // Fill the buffer with the values to write to bytes 0..4 
    EEPROM_buffer[1] = uint8_t(tagNumber >> 8);
    EEPROM_buffer[2] = uint8_t(tagNumber >> 16);
    EEPROM_buffer[3] = uint8_t(tagNumber >> 24);
    EEPROM_buffer[4] = uint8_t(userMask);



    for(int i=0; i<5; i++){
      EEPROM.write((offset+i), (EEPROM_buffer[i])); // Store the resulting value in 5 bytes of EEPROM.

    }

    logger.l1 = 6;
    logger.detail = userNum;
    logSystem(logger);


  }
}

void deleteUser(int userNum)                                            // Deletes a user from the local database.
{                                                                       // Users number 0..NUMUSERS
  int offset = (EEPROM_FIRSTUSER+(userNum*5));                          // Find the offset to write this user to

  if((userNum <0) || (userNum > NUMUSERS)) {                            // Do not write to invalid EEPROM addresses.

    logger.l1 = 5;
    logSystem(logger);
  }
  else
  {



    for(int i=0; i<5; i++){
      EEPROM.write((offset+i), 0xFF); // Store the resulting value in 5 bytes of EEPROM.
      // Starting at offset.



    }

    logger.l1 = 7;
    logger.detail = userNum;
    logSystem(logger);

  }

}



int checkUser(unsigned long tagNumber)                                  // Check if a particular tag exists in the local database. Returns userMask if found.
{                                                                       // Users number 0..NUMUSERS
  // Find the first offset to check

  unsigned long EEPROM_buffer=0;                                         // Buffer for recreating tagNumber from the 4 stored bytes.
  int found=-1;

  for(int i=EEPROM_FIRSTUSER; i<=(EEPROM_LASTUSER-5); i=i+5){


    EEPROM_buffer=0;
    EEPROM_buffer=(EEPROM.read(i+3));
    EEPROM_buffer= EEPROM_buffer<<8;
    EEPROM_buffer=(EEPROM_buffer ^ EEPROM.read(i+2));
    EEPROM_buffer= EEPROM_buffer<<8;
    EEPROM_buffer=(EEPROM_buffer ^ EEPROM.read(i+1));
    EEPROM_buffer= EEPROM_buffer<<8;
    EEPROM_buffer=(EEPROM_buffer ^ EEPROM.read(i));


    if((EEPROM_buffer == tagNumber) && (tagNumber !=0xFFFFFFFF) && (tagNumber !=0x0)) {    // Return a not found on blank (0xFFFFFFFF) entries 
      logger.l1 = 8;
      logger.detail = ((i-EEPROM_FIRSTUSER)/5);
      logSystem(logger);
      found = EEPROM.read(i+4);
      return found;
    }                             

  }
  logger.l1 = 9;
  logger.detail = 999;
  dumpUser(tagNumber);

  logSystem(logger);
  delay(1000);                                                            // Delay to prevent brute-force attacks on reader
  return found;                        
}





void dumpUser(uint8_t usernum)                                            // Return information ona particular entry in the local DB
{                                                                      // Users number 0..NUMUSERS


  unsigned long EEPROM_buffer=0;                                       // Buffer for recreating tagNumber from the 4 stored bytes.


  if((0<=usernum) && (usernum <=199)){

    int i=usernum*5+EEPROM_FIRSTUSER; //29

    EEPROM_buffer=0;
    EEPROM_buffer=(EEPROM.read(i+3)); //32
    EEPROM_buffer= EEPROM_buffer<<8;
    EEPROM_buffer=(EEPROM_buffer ^ EEPROM.read(i+2));
    EEPROM_buffer= EEPROM_buffer<<8;
    EEPROM_buffer=(EEPROM_buffer ^ EEPROM.read(i+1));
    EEPROM_buffer= EEPROM_buffer<<8;
    EEPROM_buffer=(EEPROM_buffer ^ EEPROM.read(i));



    Serial.print(((i-EEPROM_FIRSTUSER)/5),DEC);
    Serial.print("\t");
    Serial.print(EEPROM.read(i+4),DEC);
    Serial.print("\t");

    if(DEBUG==2){
      Serial.println(EEPROM_buffer,HEX);
    }
    else {
      if(EEPROM_buffer != 0xFFFFFFFF) {
        Serial.print("********");
      }
    }




  }
  else Serial.println("Bad user number!");
}


/* Displays a serial terminal menu system for
 * user management and other tasks
 */

void readCommand() {                                               


  uint8_t stringSize=(sizeof(inString)/sizeof(char));                    
  char cmdString[4][9];                                             // Size of commands (4=number of items to parse, 10 = max length of each)


  uint8_t j=0;                                                          // Counters
  uint8_t k=0;
  char cmd=0;


  char ch;

  if (Serial.available()) {                                       // Check if user entered a command this round	                                  
    ch = Serial.read();                                            
    if( ch == '\r' || inCount >=stringSize-1)  {                   // Check if this is the terminating carriage return
      inString[inCount] = 0;
      inCount=0;
    }
    else{
      (inString[inCount++] = ch); 
    }
    //Serial.print(ch);                        // Turns echo on or off


    if(inCount==0) {
      for(uint8_t i=0;  i<stringSize; i++) {
        cmdString[j][k] = inString[i];
        if(k<9) k++;
        else break;

        if(inString[i] == ' ') // Check for space and if true, terminate string and move to next string.
        {
          cmdString[j][k-1]=0;
          if(j<3)j++;
          else break;
          k=0;             
        }

      }
      cmd = cmdString[0][0];

      switch(cmd) {


      case 'e': 
        {                                                 // Enable "privileged" commands at console

            if((consoleFail>=5) && (millis()-consolefailTimer<300000))  // Do not allow priv mode if more than 5 failed logins in 5 minute
          {  
            PROGMEMprintln(privsAttemptsMessage);
            break;
          }
          if (strtoul(cmdString[1],NULL,16) == PRIVPASSWORD)
          {
            consoleFail=0;                    
            PROGMEMprintln(privsenabledMessage);
            privmodeEnabled=true;
          }
          else {
            PROGMEMprintln(privsdisabledMessage);
            privmodeEnabled=false;                                          
            if(consoleFail==0) {                                   // Set the timeout for failed logins
              consolefailTimer=millis();
            }
            consoleFail++;                                    // Increment the login failure counter
          }

          break;

        }

        //privmodeEnabled=true;            //Debugging statement


      case 'a': 
        {                                                 // List whole user database
          if(privmodeEnabled==true) {                 
            Serial.println("User dump started.");
            Serial.print("UserNum:");
            Serial.print(" ");
            Serial.print("Usermask:");
            Serial.print(" ");
            Serial.println("TagNum:");

            for(int i=0; i<(NUMUSERS); i++){
              dumpUser(i);
              Serial.println();
            }
          }
          else{
            logprivFail();
          }
          break;
        }

      case 's': 
        {                                                 // List user 
          if(privmodeEnabled==true) {
            Serial.print("UserNum:");
            Serial.print(" ");
            Serial.print("Usermask:");
            Serial.print(" ");
            Serial.println("TagNum:");
            dumpUser(atoi(cmdString[1]));
            Serial.println();
          }
          else{
            logprivFail();
          }
          break;
        }

      case 'd': 
        {                                                 // Display current time
          Serial.println();
          break;
        }

      case '1': 
        {                                               // Deactivate alarm                                       
          if(privmodeEnabled==true) {
            armAlarm(0);
            alarmState(0);
            chirpAlarm(1);  
          }
          else{
            logprivFail();
          }
          break;
        }
      case '2': 
        {                                               // Activate alarm with delay.
          chirpAlarm(20);                                          // 200 chirps = ~30 seconds delay
          armAlarm(1);                           
          break; 
        } 

      case 'u': 
        {
          if(privmodeEnabled==true) {
            alarmState(0);                                       // Set to door chime only/open doors                                                          

            armAlarm(4);
            doorUnlock(1);
            doorUnlock(2);
            door1Locked=false;
            door2Locked=false;
            chirpAlarm(3);   
          }

          else{
            logprivFail();
          }
          break;  
        }
      case 'l': 
        {                                             // Lock all doors          
          lockall();
          chirpAlarm(1);   
          break;  
        }                            

      case '3': 
        {                                            // Train alarm sensors
          if(privmodeEnabled==true) {
            trainAlarm();
          }
          else{
            logprivFail();
          }
          break;
        }
      case '9': 
        {                                            // Show site status
          PROGMEMprint(statusMessage1);
          Serial.println(alarmArmed,DEC);
          PROGMEMprint(statusMessage2);
          Serial.println(alarmActivated,DEC);
          PROGMEMprint(statusMessage3);
          Serial.println(pollAlarm(3),DEC);
          PROGMEMprint(statusMessage4);
          Serial.println(pollAlarm(2),DEC);                  
          PROGMEMprint(statusMessage5); 
          Serial.println(door1Locked);                    
          PROGMEMprint(statusMessage6); 
          Serial.println(door2Locked); 
          break;
        }

      case 'o': 
        {  
          if(privmodeEnabled==true) {
            if(atoi(cmdString[1]) == 1){                                     
              alarmState(0);                                       // Set to door chime only/open doors                                                          

              armAlarm(4);
              doorUnlock(1);                                       // Open the door specified
              door1locktimer=millis();
              break;
            }                    
            if(atoi(cmdString[1]) == 2){  
              alarmState(0);                                       // Set to door chime only/open doors                                                          

              armAlarm(4);
              doorUnlock(2);                                        
              door2locktimer=millis();
              break;               
            }
            Serial.print("Invalid door number!");
          }

          else{
            logprivFail();
          }
          break;
        } 

      case 'r': 
        {                                                 // Remove a user
          if(privmodeEnabled==true) {
            dumpUser(atoi(cmdString[1]));
            deleteUser(atoi(cmdString[1]));
          }
          else{
            logprivFail();
          }
          break; 
        }              

      case 'm': 
        {                                                                // Add/change a user                   
          if(privmodeEnabled==true) {
            dumpUser(atoi(cmdString[1]));
            addUser(atoi(cmdString[1]), atoi(cmdString[2]), strtoul(cmdString[3],NULL,16));                
            dumpUser(atoi(cmdString[1]));
          }
          else{
            logprivFail();
          }                                    

          break;
        }

      case '?': 
        {                                                  // Display help menu
          PROGMEMprintln(consolehelpMessage1);
          PROGMEMprintln(consolehelpMessage2);
          PROGMEMprintln(consolehelpMessage3);
          PROGMEMprintln(consolehelpMessage4);
          PROGMEMprintln(consolehelpMessage5);                     
          PROGMEMprintln(consolehelpMessage6);                  
          break;
        }

      default:  
        PROGMEMprintln(consoledefaultMessage);
        break;
      }  

    }                                    // End of 'if' statement for Serial.available
  }                                     // End of 'if' for string finished
}                                      // End of function 




/* Wrapper functions for interrupt attachment
 Could be cleaned up in library?
 */
void callReader1Zero(){
  wiegand26.reader1Zero();
}
void callReader1One(){
  wiegand26.reader1One();
}
void callReader2Zero(){
  wiegand26.reader2Zero();
}
void callReader2One(){
  wiegand26.reader2One();
}
void callReader3Zero(){
  wiegand26.reader3Zero();
}
void callReader3One(){
  wiegand26.reader3One();
}



