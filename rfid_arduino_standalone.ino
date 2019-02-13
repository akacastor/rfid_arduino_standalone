/*
 RDM6300 RFID interface connects to software serial interface (pins D10/D11)
 Uses SoftwareSerial library (included with Arduino IDE)
 
 What this Arduino sketch does:
 
 When a valid RFID fob is presented to the reader, the status LED will turn green and relay switched on to enable power to target machine. (session started - power is provided to machine)
 The input signal runningPin, controlled by the target machine, is used to determine if the machine has been turned on.
 If runningPin does not get pulled to +5V within startupTimeout (10 seconds) after a valid fob is presented, the status LED will turn off and the relay will be switched off. (session ended - timeout)
 If runningPin changes from active (high) to inactive (floating/GND), the status LED will turn off and the relay will be switched off. (session ended because machine was turned off)
 A second input, runningPinInv is available that is pulled low (to GND) to indicate the machine is active.

 When a fob is presented to the reader that is not in the EEPROM list, the status LED will turn red for 3 seconds and the relay will not be switched on. (access denied)

 If the ADMIN button is held down while presenting an admin fob, the LED will flash green and the next fob swiped will be added to the authorized list in EEPROM.
 
 If the ADMIN button is held down while restarting the Arduino, the LED will flash red indicating admin mode is active.
 If the ADMIN button remains still held down when an admin tag is swiped, the entire authorization list in EEPROM will be erased.
 The status LED will stay solid red for a few seconds while the auth list is deleted, then will return to flashing red.


 The circuit: 
 * RX is digital pin 10 (connect to TX of RDM6300 RFID reader)
 * TX is digital pin 11 (connect to RX of RDM6300 RFID reader)
 * bicolor LED (green/red) w/series resistor between pins 2 & 3
 * output pin 4 controls relay (goes to +5V to switch relay on)
   using FOTEK SSR-25 DA module (rated for 24-380 VAC @ 25 amps, control input 3-32 VDC) 
 * input pin 6 indicates whether machine is currently running (externally pulled to +5V to indicate machine is running)
   recommended to connect +5V machine running to Arduino through a 33-100 ohm series resistor
 * input pin 7 is inverted 'machine is running' input
 * a 'ADMIN' button connected to pin 5 is connected to GND when depressed (normally open)


 SoftwareSerial Note:
 Not all pins on the Mega and Mega 2560 support change interrupts, 
 so only the following can be used for RX: 
 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69
 
 Not all pins on the Leonardo support change interrupts, 
 so only the following can be used for RX: 
 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).


 Chris Gerlinsky, 9 Feb 2019
 based on SoftwareSerialExample: 
 created back in the mists of time
 modified 25 May 2012
 by Tom Igoe
 based on Mikal Hart's example
*/

#include <SoftwareSerial.h>
#include <EEPROM.h>


//------------------------------------------------------------
// list of tags that are valid for access to the machine 
//------------------------------------------------------------
// (relay will switch ON when a tag from this list is presented to the RFID reader)
// note: adminTagsList[] is terminated with a '0' entry at the end of the list
// the first two digits of 10-digit ID is ignored - only last 8 digits (32-bits) is checked (limitation of 32-bit unsigned long)
const unsigned long adminTagsList[] PROGMEM  = 
{
  0xb0009afb7f,  // Chris G (5890046)
  0              // 0 marks the end of adminTagsList
};
//------------------------------------------------------------


//------------------------------------------------------------
// settings
//------------------------------------------------------------
// time (in milliseconds) to wait for a runningPin signal after enabling the power relay
const unsigned long startupTimeout = 10000;


// time (in milliseconds) to keep status LED red when unauthorized tag was presented
const unsigned long deniedTimeout = 3000;


// time (in milliseconds) to allow a new tag to be added to authorized list in EEPROM, after admin tag was swiped with admin button pressed
const unsigned long addTagTimeout = 5000;


// time (in milliseconds) of grace time after sensing machine stop before locking out system. (disconnecting power and forgetting active fob id)
const unsigned long runningStopTimeout = 3000;


// duration of buzzer noises
const int SHORTBUZZ = 150;     // duration in milliseconds of short buzz (for access granted, etc)
const int LONGBUZZ = 500;      // duration in milliseconds of long buzz (for access denied / logout)

// tone to play buzzer noises
const int GOODBUZZTONE = 2000;  // (2000 = 2 KHz) frequency of tone to play on buzzer as a 'good' tone (access granted, etc)
const int BADBUZZTONE = 1000;   // (1000 = 1 KHz) frequency of tone to play on buzzer as a 'bad' tone (access denied, etc)


// set keepRunningOnPowerup to 1 to switch the power relay on at startup if the runningPin indicates machine is currently running
// this is meant as a safety in case the Arduino resets, to attempt to prevent accidental machine turn-off
// set keepRunningOnPowerup = 0  to always start with the relay switched off when the Arduino code boots
const int keepRunningOnPowerup = 0;

//------------------------------------------------------------


//------------------------------------------------------------
// hardware configuration
//------------------------------------------------------------
// set pin number for status LED pin
const int ledGreenPin =  2;      // the number of the LED pin
const int ledRedPin = 3;


// set pin number for ADMIN button
const int adminButtonPin = 5;    // the number of the 'admin' button pin (hold this button down while swiping admin fob to add user fob to eeprom)
const int ADMINDOWN = 0;         // adminButtonPin is pulled low when 'admin' button is depressed


// set pin # for relay switch on/off
const int relayPin = 4;     // the number of the relay control pin

// relayPin values
const int RELAYON = 1;      // set pin to 1 (5v) for relay switch on
const int RELAYOFF = 0;     // set pin to 0 for relay switch off


// runningPin must be pulled to +5V by the machine or sensor to indicate when the machine is running.
// external pull-down resistor is required on runningPin
const int runningPin = 6;    // the number of the 'is machine running' input pin
const int runningPinInv = 7; // the number of the 'is machine running' input pin (inverted logic)

// runningPin values (RUNON = 1 indicates that runningPin is pulled high while the machine is running)
const int RUNON = 1;
const int RUNOFF = 0;


// buzzerPin is connected to a piezo buzzer
const int buzzerPin = 8;      // output pin to use to control buzzer


// setup SoftwareSerial interface to receive data from RFID reader
SoftwareSerial rfidSerial(10, 11);  // RX = D10, TX = D11


unsigned int eepLength = 1024;    // using old Arduino IDE, EEPROM.length() doesn't exist, so this is hardcoded (Arduino Uno = 1024 byte EEPROM)

//------------------------------------------------------------


// possible current system states
const int STATUSUNK = -1;
const int STATUSOFF = 0;     // machine is off (power relay switched off)
const int STATUSON = 1;      // machine is on (running)
const int STATUSTURNON = 2;  // machine is turning on (power available but runningPin hasn't sensed activity yet)
const int STATUSDENIED = 3;  // an unauthorized RFID tag was presented (and status LED is flashing)
const int STATUSADD = 4;     // admin button was held down while a valid tag in adminTagsList[] was presented
const int STATUSADDED = 5;   // admin button was held down while a new authorized tag was added to EEPROM
const int STATUSDEL = 6;     // admin button was held down while booting, then released, then held down while admin tag was presented - delete all tags from EEPROM
const int STATUSTURNOFF = 7; // 'machine running' signal indicates the machine has stopped, give grace time before STATUSOFF


int systemStatus = STATUSUNK;  // system status = STATUSUNK (unknown status)


// values for entryStatus byte in EEPROM table, and return values for checkAuthTag()
const int statusEmpty = 0;
const int statusAuthorized = 1;
const int statusAdmin = 2;


int bootAdminMode =  0;      // this is set if 'admin' button is held down during Arduino startup, to enter admin mode


unsigned long fobPresentMilli;    // time from millis() when the last fob was presented
                                  // used to track when power was switched on to the machine (for a timeout in case the machine never turns on after fob activation)

unsigned long runTimeMilli;       // time from millis() when the relay was switched on
                                  // used to calculate how long machine was switched on (to output via Serial.print for logging on PC)

unsigned long runningGraceMilli;  // time from millis() when the machine run input indicated the machine was powered up.
                                  // used to calculate grace time before locking out machine power after sensing power-down.


unsigned long activeFobId;      // the fobId that was used to start the machine

//------------------------------------------------------------


void setup()  
{
  // startup up with status LED off
  digitalWrite( ledGreenPin, 0 );
  digitalWrite( ledRedPin, 0 );
  pinMode( ledGreenPin, OUTPUT );
  pinMode( ledRedPin, OUTPUT );


  // set runningPin as input. external pulldown resistor is used.
  // runningPin must be pulled to +5V by the machine or sensor to indicate when the machine is running.
  pinMode( runningPin, INPUT );

  // set runningPinInv as input with pullup resistor enabled
  // runningPinInv must be pulled to GND by the machine or sensor to indicate when the machine is running.
  pinMode( runningPinInv, INPUT_PULLUP );

  // !NOTE! - if the Arduino restarts and detects that the machine is running, the relay will be switched on to continue powering the machine.
  //          this is to reduce risk of an accidental Arduino restart (glitch) interrupting active work with a running machine.
  //  ???  It may be better to always start up with the machine turned off, but there may be a risk of accidental Arduino restart causing unexpected machine powerdown.
  if( keepRunningOnPowerup && (digitalRead(runningPin) == RUNON || digitalRead(runningPinInv) == !RUNON) )
  {  // Arduino restarted, and machine is running - let it keep running
    systemStatus = STATUSON;
    digitalWrite( relayPin, RELAYON );
    digitalWrite( ledGreenPin, 1 );        // turn status = green on
    digitalWrite( ledRedPin, 0 );

    // Open serial communications and wait for port to open:
    Serial.begin(57600);      // open serial port interface to PC
    while( !Serial )
      ; // wait for serial port to connect. Needed for Leonardo only

    Serial.println( F("bootrun, 0, 0, Arduino restarted with machine running - machine turned on.") );
    
    runTimeMilli = millis();              // record millis() timestamp when machine powered up
  }
  else
  {  // Arduino restarted, turn power to machine off.
    systemStatus = STATUSOFF;
    digitalWrite( relayPin, RELAYOFF );
    digitalWrite( ledGreenPin, 0 );        // turn status green/red LED off
    digitalWrite( ledRedPin, 0 );

    // Open serial communications and wait for port to open:
    Serial.begin(57600);      // open serial port interface to PC
    while( !Serial )
      ; // wait for serial port to connect. Needed for Leonardo only

    Serial.println( F("boot, 0, 0, Arduino restarted. Machine disabled.") );
  }
  pinMode( relayPin, OUTPUT );
  pinMode( adminButtonPin, INPUT_PULLUP );


  // set the data rate for the SoftwareSerial port
  rfidSerial.begin(9600);      // open serial port interface to RDM6300 RFID reader
  

  if( digitalRead(adminButtonPin) == ADMINDOWN )
  {
    bootAdminMode = 1;        // Arduino was started with 'admin' button depressed, run in admin mode
    systemStatus = STATUSOFF;
    digitalWrite( relayPin, RELAYOFF );
    digitalWrite( ledGreenPin, 0 );        // turn status green/red LED off
    digitalWrite( ledRedPin, 0 );
    Serial.println( F("bootadmin, 0, 0, Booted in admin mode.") );
  }
  else
    bootAdminMode = 0;
    

  if( systemStatus == STATUSOFF && !bootAdminMode )
  {  // blink status LED green twice when booting up
    digitalWrite( ledGreenPin, 1 );      // turn status green LED on
    digitalWrite( ledRedPin, 0 );
    delay(150);
    digitalWrite( ledGreenPin, 0 );      // turn status green LED off
    digitalWrite( ledRedPin, 0 );
    delay(150);
    digitalWrite( ledGreenPin, 1 );      // turn status green LED on
    digitalWrite( ledRedPin, 0 );
    delay(150);
    digitalWrite( ledGreenPin, 0 );      // turn status green LED off
    digitalWrite( ledRedPin, 0 );
  }
}

//------------------------------------------------------------


void loop() // run over and over
{
  char input[3];
  int charsRead;
  int val;
  int fobIdPacket[6];
  int idPacketLen;
  int i;
  unsigned long fobId;
  

  if( systemStatus == STATUSTURNON && (digitalRead(runningPin) == RUNON || digitalRead(runningPinInv) == !RUNON) )
  {  // STATUSTURNON indicates the relay has been switched on, and runningPin == RUNON indicates the machine has powered up.
    systemStatus = STATUSON;              // STATUSON indicates that the machine is currently running
    Serial.print( F("start, ") );
    Serial.print( activeFobId, HEX );
    Serial.print( F(", 0, Machine turned on. Fob id: ") );
    Serial.println( activeFobId, HEX );
    runTimeMilli = millis();              // record millis() timestamp when machine powered up
  }
  else if( systemStatus == STATUSON && !(digitalRead(runningPin) == RUNON || digitalRead(runningPinInv) == !RUNON) )
  {  // STATUSON indicates that the machine was running, !(runningPin == RUNON) indicates the machine has powered down.
    systemStatus = STATUSTURNOFF;        // STATUSTURNOFF indicates that machine turn-off was detected, now allow grace time for a quick restart before logging out activeFobId
    runningGraceMilli = millis();        // record millis() timestamp when machine turn-off detected (to allow grace time before fob logout)

    Serial.print( F("stop, ") );
    Serial.print( activeFobId, HEX );
    Serial.print( F(", ") );
    Serial.print( (millis() - runTimeMilli) / 1000 );
    Serial.print( F(", Machine shutdown detected. Fob id: ") );
    Serial.print( activeFobId, HEX );
    Serial.print( F(". Seconds running: ") );
    Serial.println( (millis() - runTimeMilli) / 1000 );
  }
  else if( systemStatus == STATUSTURNOFF && (digitalRead(runningPin) == RUNON || digitalRead(runningPinInv) == !RUNON) )
  {  // STATUSTURNOFF indicates that the machine was turned-off, runningPin == RUNON indicates the machine was restarted
    systemStatus = STATUSON;              // STATUSON indicates the machine is running again.

    Serial.print( F("restart, ") );
    Serial.print( activeFobId, HEX );
    Serial.print( F(", ") );
    Serial.print( (millis() - runTimeMilli) / 1000 );
    Serial.print( F(", Machine restart detected. Fob id: ") );
    Serial.println( activeFobId, HEX );
  }
  else if( systemStatus == STATUSTURNOFF && millis() > runningGraceMilli + runningStopTimeout )
  {  // STATUSTURNOFF indicates that the machine was turned off, millis() checks if the grace time has expired and no machine restart was detected.
    systemStatus = STATUSOFF;              // STATUSOFF indicates the relay has been switched off and session is finished for activeFobId
    digitalWrite( ledGreenPin, 0 );        // turn status green/red LED off
    digitalWrite( ledRedPin, 0 );
    digitalWrite( relayPin, RELAYOFF );  // turn relay off

    Serial.print( F("logout, ") );
    Serial.print( activeFobId, HEX );
    Serial.print( F(", ") );
    Serial.print( (millis() - runTimeMilli) / 1000 );
    Serial.print( F(", Machine turned off and logged out fob id: ") );
    Serial.print( activeFobId, HEX );
    Serial.print( F(". Seconds running: ") );
    Serial.println( (millis() - runTimeMilli) / 1000 );

    tone( buzzerPin, BADBUZZTONE, LONGBUZZ );   // long buzzer indicates session finished
  }
  else if( systemStatus == STATUSTURNON && millis() > fobPresentMilli + startupTimeout )
  {  // STATUSTURNON indicates the relay was switched on and machine running signal has not been detected, millis() checks if the startup timeout has expired.
    systemStatus = STATUSOFF;              // STATUSOFF indicates the relay has been switched off and session is finished for activeFobId (due to timeout waiting for the machine to be turned on)
    digitalWrite( ledGreenPin, 0 );        // turn status green/red LED off
    digitalWrite( ledRedPin, 0 );
    digitalWrite( relayPin, RELAYOFF );  // turn relay off

    Serial.print( F("timeout, ") );
    Serial.print( activeFobId, HEX );
    Serial.print( F(", 0, Machine disabled due to time-out before turn-on. Fob id: ") );
    Serial.println( activeFobId, HEX );

    tone( buzzerPin, BADBUZZTONE, LONGBUZZ );   // long buzzer indicates session finished
  }
  else if( systemStatus == STATUSDENIED && millis() > fobPresentMilli + deniedTimeout )
  {  // STATUSDENIED indicates an unauthorized fob was presented to the reader, millis() checks if the red LED indication timeout has expired
    systemStatus = STATUSOFF;              // STATUSOFF Indicates the relay remains switched off and no session is opened for the presented fob (it was not found in authorized fob list)
    digitalWrite( ledGreenPin, 0 );        // turn status green/red LED off
    digitalWrite( ledRedPin, 0 );    
  }
  else if( systemStatus == STATUSADD && millis() > fobPresentMilli + addTagTimeout )
  {  // STATUSADD indicates the admin button has been held down while presenting an admin fob, to add a new fob to authorized fob list. millis() checks if timeout has occured without a new fob being presented.
    systemStatus = STATUSOFF;              // STATUSOFF indicates the relay remains switched off, no session is open for a fob, and the admin 'add a new fob' process was canceled.
    digitalWrite( ledGreenPin, 0 );        // turn status green/red LED off
    digitalWrite( ledRedPin, 0 );    
  }
  else if( systemStatus == STATUSADD && digitalRead(adminButtonPin) != ADMINDOWN )
  {  // STATUSADD indicates the admin button has been held down while presenting an admin fob, to add new fob to authorized fob list, then admin button was released.
    systemStatus = STATUSOFF;              // STATUSOFF indicates the relay remains switched off, no session is open for a fob, and the admin 'add a new fob' process was canceled.
    digitalWrite( ledGreenPin, 0 );        // turn status green/red LED off
    digitalWrite( ledRedPin, 0 );    
  }
  else if( systemStatus == STATUSADD )
  {  // STATUSADD indicates the admin button has been held down while presenting an admin fob, to add new fob to authorized fob list.
    // while in state STATUSADD, blink the status LED green
    if( ((millis() - fobPresentMilli) / 500) % 2 )
    {
      digitalWrite( ledGreenPin, 0 );        // turn status green/red LED off (blink off)
      digitalWrite( ledRedPin, 0 );    
    }
    else
    {
      digitalWrite( ledGreenPin, 1 );        // turn status green LED on
      digitalWrite( ledRedPin, 0 );    
    }
  }
  else if( systemStatus == STATUSADDED && millis() > fobPresentMilli + deniedTimeout )
  {  // STATUSADDED indicates a new fob was added to the authorized fobs list, millis() checks if the status LED has been turned on long enough.
    systemStatus = STATUSOFF;              // STATUSOFF indicates the relay remains switched off, no session is open for a fob, and the 'add a new fob' process is complete.
    digitalWrite( ledGreenPin, 0 );        // turn status green/red LED off
    digitalWrite( ledRedPin, 0 );    
  }
  else if( systemStatus == STATUSDEL && millis() > fobPresentMilli + deniedTimeout )
  {  // STATUSDEL indicates the authorized fobs list was erased, millis() checks if the status LED has blinked long enough to indicate completion.
    systemStatus = STATUSOFF;              // STATUSOFF indicates the relay remains switched off, no session is open for a fob, and the 'delete all authorized fobs' operation completed.
    digitalWrite( ledGreenPin, 0 );        // turn status green/red LED off
    digitalWrite( ledRedPin, 0 );    
  }
  else if( systemStatus == STATUSDEL )
  {  // STATUSDEL indicates the authorized fobs list was erased, blink the status LED red to indicate completion.
    if( ((millis() - fobPresentMilli) / 500) % 2 )
    {
      digitalWrite( ledGreenPin, 0 );        // turn status green/red LED off (blink off)
      digitalWrite( ledRedPin, 0 );    
    }
    else
    {
      digitalWrite( ledGreenPin, 0 );        // turn status red LED on
      digitalWrite( ledRedPin, 1 );    
    }
  }
  else if( bootAdminMode && digitalRead(adminButtonPin) != ADMINDOWN )
  {  // booted in admin mode, but the button was released - exit admin mode
    bootAdminMode = 0;
    digitalWrite( ledGreenPin, 0 );        // turn status green/red LED off
    digitalWrite( ledRedPin, 0 );    
  }
  else if( bootAdminMode )
  {  // booted in admin mode, and the button is still being held down - blink the status LED between red/green to indicate admin mode is active.
    if( (millis() / 500) % 2 )
    {
      digitalWrite( ledGreenPin, 1 );        // turn status green LED off (blink between red/green)
      digitalWrite( ledRedPin, 0 );
    }
    else
    {
      digitalWrite( ledGreenPin, 0 );        // turn status red LED on (blink between red/green)
      digitalWrite( ledRedPin, 1 );    
    }
  }
  

  // check for received data from the RFID reader
  if (rfidSerial.available() > 0)
  {
    // messages received from RDM6300 RFID reader @ 9600bps look like:
    // 02 30 44 30 30 35 39 44 46 46 45 37 35 03 
    // six byte message between '02' and '03', XOR'd together the bytes = 0 to indicate valid checksum
    // convert the ASCII hex values to binary
    // then reverse bit order in each byte (swap order of bits from 0..7 to 7..0) to match the values in Makerspace Nanaimo RFID fob database
    if( rfidSerial.read() != 0x02 )
      return;    // didn't receive 0x02 message start indicator

    idPacketLen = 0;

    // receive 6-byte message from RFID reader
    while( rfidSerial.available() > 0 && idPacketLen < 6)
    {
      charsRead = rfidSerial.readBytesUntil( 0x03, input, 2 );  // fetch the two characters until 0x03 message complete indicator
      input[charsRead] = '\0';                                // Make it a string

      val = strtol(input, 0, 0x10);                           // Convert it from ASCII hex to int

      // reverse bit order in each received hex byte (to match reversed bit order in existing Makerspace Nanaimo database)
      fobIdPacket[ idPacketLen ] = 0;
      for( i=0; i<8; i++ )
      {
        if( val & (1<<i) )
          fobIdPacket[ idPacketLen ] |= (1<<7-i);
      }
      idPacketLen++;
    }

    // message received from RDM6300 must be 6 bytes long
    if( idPacketLen != 6 )
      return;              // didn't receive complete fob ID message from RDM6300 - throw away what we did receive

    // test checksum on received fobIdPacket    
    val = 0;
    for( i=0; i<6; i++ )
      val ^= fobIdPacket[i];
    
    if( val != 0 )
      return;              // after XORing all 6 message bytes together, the result must be 0 for the message to be valid

    // extra sanity checking - test that all the received bytes are not identical (ie: not a string if 6x 00 or 6x of any value, it could pass checksum)
    for( i=1; i<6; i++ )
    {
      if( fobIdPacket[i] != fobIdPacket[0] )
        break;            // break the for() loop if we find a message byte different from the first byte
    }
    if( i>=6 )
      return;             // if the for() loop completed without breaking early, that means all message bytes are the same - throw this packet away

    // six-byte fobIdPacket[] is considered valid at this point

    // convert fobIdPacket to 32-bit integer fobId value
    fobId = 0;
    for( i=0; i<4; i++ )
      fobId = (fobId<<8) + fobIdPacket[1+i];
      
    // 32-bit int fobId is considered valid at this point

    fobPresentMilli = millis();      // record time when valid fob was presented (used for timeout if machine isn't turned on after activation)
      
    if( digitalRead(adminButtonPin) == ADMINDOWN && systemStatus != STATUSON && systemStatus != STATUSTURNON )
    {  // 'admin' button is pressed - check if a tag in adminTagsList[] was presented      
      if( checkAdminTag(fobId) == statusAdmin )
      {  // valid tag in adminTagsList[] was presented while 'admin' button held down
        if( bootAdminMode )
        {   // running in 'admin mode', an admin fob presented = delete all fobs from the authorized fobs list
          if( systemStatus != STATUSDEL || activeFobId != fobId )
          {  // if systemStatus already was STATUSDEL and the fobId hasn't changed, it's not a new fob - it's just been read more than once while being presented to the reader.
            digitalWrite( ledGreenPin, 0 );        // turn status red LED on
            digitalWrite( ledRedPin, 1 );    

            clearAuthTags();                      // remove all authorized tags from EEPROM
            systemStatus = STATUSDEL;

            Serial.print( F("deleted, ") );
            Serial.print( fobId, HEX );
            Serial.print( F(", 0, Admin fob presented for auth tags list erase: ") );
            Serial.println( fobId, HEX );    // print the fob id#

            tone( buzzerPin, GOODBUZZTONE, SHORTBUZZ );    // three short buzzer sounds indicate fob auth list cleared
            delay(SHORTBUZZ);            
            tone( buzzerPin, GOODBUZZTONE, SHORTBUZZ );    // three short buzzer sounds indicate fob auth list cleared
            delay(SHORTBUZZ);
            tone( buzzerPin, GOODBUZZTONE, SHORTBUZZ );    // three short buzzer sounds indicate fob auth list cleared
          }
          activeFobId = fobId;
          fobPresentMilli = millis();      // record time when valid fob was presented (used for timeout of flashing red LED)
        }
        else
        {  // not running in admin mode (normal condition) - an admin fob was presented while 'admin' button held down, add a new fob to the authorized fobs list
          if( systemStatus != STATUSADD || activeFobId != fobId )
          {  // if systemStatus already was STATUSADD and the fobId hasn't changed, it's not a new fob - it was read multiple times while fob was presented to the reader.
            systemStatus = STATUSADD;      // set status to STATUSADD to indicate next fob swiped gets added to EEPROM
            Serial.print( F("admin, ") );
            Serial.print( fobId, HEX );
            Serial.print( F(", 0, Admin fob presented: ") );
            Serial.println( fobId, HEX );    // print the fob id#
            digitalWrite( ledGreenPin, 1 );  // turn status green LED on
            digitalWrite( ledRedPin, 0 );    
            
            tone( buzzerPin, GOODBUZZTONE, SHORTBUZZ );    // short buzzer indicates fob accepted
          }
          activeFobId = fobId;
        }
      }
      else if( systemStatus == STATUSADD && !checkAuthTag(fobId) )
      {  // a valid admin fob was previously swiped with admin button depressed, a new fob is allowed to be added and isn't already in the auth list
        if( addAuthTag( fobId ) )
          systemStatus = STATUSADDED;
        else
          systemStatus = STATUSDEL;      // blink LED red to indicate fob was not added
        Serial.print( F("added, ") );
        Serial.print( fobId, HEX );
        Serial.print( F(", 0, Fob added to auth list: ") );
        Serial.println( fobId, HEX );    // print the fob id#

        tone( buzzerPin, GOODBUZZTONE, SHORTBUZZ );    // two short buzzer sounds indicate fob added to list
        delay(SHORTBUZZ);            
        tone( buzzerPin, GOODBUZZTONE, SHORTBUZZ );    
      }
    }
    else
    {  // admin button wasn't held down - check if this tag is authorized to turn on the relay
      if( checkAuthTag(fobId) || checkAdminTag(fobId) )
      {  // a fob that is in the list of authorized tags has been presented        
        digitalWrite( ledGreenPin, 1 );        // turn status = green on
        digitalWrite( ledRedPin, 0 );

        digitalWrite( relayPin, RELAYON );     // turn relay on

        if( systemStatus != STATUSON )
        {
          if( systemStatus != STATUSTURNON )
          {
            tone( buzzerPin, GOODBUZZTONE, SHORTBUZZ );    // short buzzer indicates fob accepted / switching relay on
            
            Serial.print( F("login, ") );
            Serial.print( fobId, HEX );
            Serial.print( F(", 0, Access granted: ") );
            Serial.println( fobId, HEX );    // print the fob id#

            activeFobId = fobId;
          }
          systemStatus = STATUSTURNON;
        }
      }
      else if( systemStatus == STATUSOFF || systemStatus == STATUSTURNON )     // only care about unauthorized tags if the machine isn't already running
      {  // the presented fob id isn't in the list of authorized tags
        digitalWrite( ledGreenPin, 0 );        // turn status LED red
        digitalWrite( ledRedPin, 1 );

        if( systemStatus != STATUSDENIED || activeFobId != fobId )
        {  // if systemStatus was already STATUSDENIED and the fobId hasn't changed, the same fob was read multiple times by the reader.
          digitalWrite( relayPin, RELAYOFF );  // turn relay off

          tone( buzzerPin, BADBUZZTONE, LONGBUZZ );    // long buzzer indicates fob denied

          systemStatus = STATUSDENIED;
          Serial.print( F("denied, ") );
          Serial.print( fobId, HEX );
          Serial.print( F(", 0, Access DENIED:  ") );
          Serial.println( fobId, HEX );    // print the fob id#
        }
      }
    }  // end of check whether 'admin' button was held down or not while fob was detected
  }  // end of rfidSerial.available() data read from RFID reader  
}

//------------------------------------------------------------


// checkAdminTag( fobId ) - check if fobId is in list of admin tags (adminTagsList[] in flash)
// returns 0 if fobId isn't found in list, or returns value statusAdmin if fobId is found in the admin tags list
int checkAdminTag( unsigned long fobId )
{
  int i;
  
  for( i=0; adminTagsList[i] != 0; i++ )    // step through all entries in adminTagsList[] until 0 (end) is reached
  {
    if( fobId == adminTagsList[i] )
      return statusAdmin;
  }
  
  
  return 0;
}

//------------------------------------------------------------


// checkAuthTag( fobId ) - check if fobId is in EEPROM list of authorized tags
int checkAuthTag( unsigned long fobId )
{
  unsigned int eepAddr;
  int i;
  unsigned long entryId;
  unsigned char entryStatus = 0;            // entryStatus 0 indicates fobId was not found in EEPROM


  // each entry is 4 byte id# followed by one byte entryStatus
  for( eepAddr=0; eepAddr<eepLength-5; eepAddr+=5 )
  {
    // read 4-byte entryId from EEPROM
    entryId = 0;
    for( i=0; i<4; i++ )
    {  // read 4 bytes from EEPROM to check if it is a matching id #
      entryId <<= 8;
      entryId += EEPROM.read( eepAddr + i );
    }

    // check if this entry in EEPROM list matches the fobId passed to this function by caller    
    if( entryId == fobId )
    {  // found matching entry in EEPROM
      entryStatus = EEPROM.read( eepAddr + 4 );
    }
  }


  return entryStatus;
}

//------------------------------------------------------------


// addAuthTag( fobId ) - add fobId to the authorized tag list in EEPROM
int addAuthTag( unsigned long fobId )
{
  unsigned int eepAddr;
  int i;
  unsigned char entryStatus = 0;            // entryStatus 0 indicates fobId was not found in EEPROM


  for( eepAddr=0; eepAddr<eepLength-5; eepAddr+=5 )
  {
    entryStatus = EEPROM.read( eepAddr + 4 );    // read entryStatus byte from EEPROM tags list
    if( entryStatus == statusEmpty || entryStatus == 0xFF )
    {
      EEPROM.write( eepAddr, (fobId>>24)&0xFF );
      EEPROM.write( eepAddr+1, (fobId>>16)&0xFF );
      EEPROM.write( eepAddr+2, (fobId>>8)&0xFF );
      EEPROM.write( eepAddr+3, fobId&0xFF );

      entryStatus = statusAuthorized;
      EEPROM.write( eepAddr+4, entryStatus );
      
      return entryStatus;
    }
  }
  
  
  return 0;      // unable to add tag to EEPROM (full)
}

//------------------------------------------------------------

// delete all entries from the authorized tags list in EEPROM (write entire EEPROM = 0)
void clearAuthTags(void)
{
  unsigned int eepAddr;
  
  
  for( eepAddr=0; eepAddr<eepLength-5; eepAddr+=5 )
    EEPROM.write( eepAddr+4, statusEmpty );
}

//------------------------------------------------------------

