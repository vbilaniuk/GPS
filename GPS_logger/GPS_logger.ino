/****************************************************************************************
 * 
 * GPS Data Logger
 * Intended use:  Throw this on something that moves and track it.  Generates
 * Google Maps friendly data.
 * 
 * Hardware:
 * Arduino Uno
 * Adafruit Ultimate GPS on pins TX to A1 and RX to A0
 * DFRobotics 5V SD card module on Arduino Uno SPI pins, chip select on pin 10
 * Adafruit basic 16x2 LCD on pins 4-9 with discrete resistors for both contrast and backlight
 * 
 * Power:  Just use a USB cable plugged into a car's USB port, or a battery-powered phone charger.
 * 
 * DATE FORMAT:  YY-MM-DD
 * NOTE: These will be stored in UTC.  Convert to your time zone in other software.
 * 
 * Bit manipulation used in this program:
 * Say you have byte test = 10000000
 * 
 * test bit numbers: | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * In hex:            80  40  20  10   8   4   2   1
 * To check bit 7: if (test & 0x80) { }
 * To set bit 0: test |= 0x01
 * To clear bit 2: test &= ~0x04
 * 
 * Bit manipulation testing:
 * byte test = 0;
 * Serial.print(F("Bit manipulation test."));
 * // set 0th bit to 1 and check it
 * // first test to make sure it's zero
 * if (test & 0x01) Serial.println(F("It's a one."));
 * else Serial.println(F("It's a zero."));
 * // now set it:
 * test |= 0x01;
 * // test again:
 * if (test & 0x01) Serial.println(F("It's a one."));
 * else Serial.println(F("It's a zero."));
 * // clear it:
 * test &= ~0x01;
 * // test again:
 * if (test & 0x01) Serial.println(F("It's a one."));
 * else Serial.println(F("It's a zero."));
 * 
 * // set other bits:
 * test |= 0x02; test |= 0x10; test |= 0x80; // should get 10010010
 * // try printing the byte:
 * Serial.println(test, BIN);
 * 
 * // test bit 6 and print result
 * if (test & 0x40) Serial.println(F("true")); else Serial.println(F("False")); // should get False
 * 
 * 
 * Vicky Bilaniuk
 *****************************************************************************************/
#include <avr/interrupt.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <SD.h>
#include "Adafruit_GPS.h" 

#define tempFlag 0x80
#define recordingMessageSet 0x40
#define lostMsgPrinted 0x20
#define timeToSave 0x10
#define pressed 0x08
#define recording 0x04
#define stoppedMessageSet 0x02
#define hadFix 0x01

// LCD:
// set up LCD and display a message
LiquidCrystal lcd(4, 5, 6, 7, 8, 9);

//SD card:
// SCK:  pin 13
// MISO:  pin 12
// MOSI:  pin 11
// SS:  10
#define SdChipSelect 10
File file;
char filename[13];

// GPS
#define GPSECHO true // set to false to stop echoing to console.  
SoftwareSerial GPSConnect(A1, A0);
Adafruit_GPS GPS(&GPSConnect);
float convertedLongitude;
float convertedLatitude;

// random globals
volatile int count = 0;
volatile byte myFlags = B00000100; // structure: 7 = unused, 6 = recordingMessageSet, 5 = lostMsgPrinted, 4 = timeToSave, 3 = pressed,
// 2 = recording, 1 = stoppedMessageSet, 0 = hadFix.  Default is recording is true, rest are false.
char input;

void setup()
{ 
  // set up button: (note that if you try to put it on pretty much any other pin except 2 or 3, SoftwareSerial will freak out)
  pinMode(2, INPUT); // PCINT0
  digitalWrite(2, HIGH); // turn on pullup

  // set up timer 2 for basic count up operation, and set up pin change interrupt as well
  // note that this interrupt setup is not possible with SoftwareSerial running
  cli();
  attachInterrupt(0, ButtonPressed, FALLING);
  // timer:
  TCCR1A = 0; 
  TCCR1B = 0; // something in the Arduino stuff changes these and you have to reset them before you can use them
  TCCR1B = (0<<CS12) | (1<<CS11) | (1<<CS10); // divide by 64
  TIFR1 = (1<<TOV1); // clear TOV0 (it's one of those "write a 1 to clear it" types=
  TCNT1 = 0; // timer counter
  TIMSK1 = (1<<TOIE1); // enable interrupt
  sei();

  // clock calculation:
  // timer speed (Hz) = (CPU speed Hz) / prescaler
  //         1               1
  //  ----------------- / ------------ - 1 = timer overflow count value
  //  desired frequency   timer speed
  //
  // timer count value is going to be greater than 65535, so divide it by this to get how many overflows you need
  // If you want 5 seconds given the above timer settings, then:
  //
  //         1               1
  //  ----------------- / ------------ - 1 = 1250000, divided by 65535 to get 19 overflows
  //        1/5           (16M / 64)


  lcd.begin(16,2);

  if (!SD.begin(10, 11, 12, 13)) {
    lcd.clear();
    lcd.home();
    lcd.print(F("SD failure!"));
    cli();
    abort();
  }
  else lcd.print(F("SD card OK."));

  // set up GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

  // wait for GPS fix before proceeding
  // we want to set the time from the satellites
  lcd.setCursor(0,1);
  lcd.print(F("Waiting for fix."));

  Serial.begin(230400); // this is the fastest I can reliably get
  Serial.setTimeout(100);
}

void loop()
{    
  GPS.read(); // easiest way to update the GPS object

  // this is just an "I'm alive" message to print when it's waiting for a fix
  if ((myFlags & timeToSave) && !GPS.fix) {
    myFlags &= ~timeToSave;
  }

#ifdef VERBOSE
  if (myFlags & pressed) {
    Serial.print(F("RAM: ")); 
    Serial.println(ShowFreeRam());
    Serial.print(F("File handle: ")); 
    Serial.println(file);
    myFlags &= ~pressed;
    if (myFlags & recording) Serial.println(F("Not paused."));
  }
#endif

  if (Serial.available() > 0) {
    // user typed something
    // set recording to false:
    myFlags &= ~recording;
    file.close();

    input = Serial.read();
    if (input == 'D' || input == 'd') {   
#ifdef DEBUG
      Serial.print(F("Ram: ")); 
      Serial.println(ShowFreeRam());
#endif
      printRoot();
    } 
    else if (input == 'F' || input == 'f') {
      // asked for file
      
      while (!Serial.available()); // wait
      Serial.readBytesUntil('\n', filename, 13);

      // print it
      file = SD.open(filename);

      // if the file is available, read and print it:
      if (file) {
        while (file.available()) {
          Serial.write(file.read());
        }
        file.close();
      }
      else {
        Serial.print(F("File ")); Serial.print(filename); Serial.println(F(" not found."));
      }
    }

    // go back to what you were doing before...
    // set recording flag to true.  File will be reopened or made new if we rolled past midnight.
    myFlags |= recording;
  }

  // the following if statement is from the Adafruit parsing.ino sample code
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    GPS.lastNMEA(); 

    if (!GPS.parse(GPS.lastNMEA()))
      return; 
  }

  if ((myFlags & recording) && GPS.fix)
  {
    // set lostMsgPrinted to false:
    myFlags &= ~lostMsgPrinted;

    // set hadFix to true:
    myFlags |= hadFix;

    if (!(myFlags & recordingMessageSet)) {
      lcd.clear();
      lcd.home();
      lcd.print(F("Fix obtained."));
      lcd.setCursor(0,1);
      lcd.print(F("Recording..."));

      // set recordingMessageSet to true to stop LCD from being written again and again 
      myFlags |= recordingMessageSet;
    }

    if (!file) {
      // create or open the file
      // this is first run

      /********************************************************************/
      // TODO:  add in code to deal with removal of SD card 
      // Update:  Not possible.  Needs pin broken out to detect card.
      /********************************************************************/
      FileNameGenerator(filename, 0);

      // search for file first:
      if (SD.exists(filename)) {
        file = SD.open(filename, FILE_WRITE); 
      } 
      else {
        // does not exist so create it
        file = SD.open(filename, FILE_WRITE); 
        // header:
        file.println(F("Car GPS Tracker. Times are in UTC. Lat/Lon are in decimal degrees (Google maps friendly)."));
        file.println(F("Date,       Time,     Latitude,  Longitude, Quality, Altitude, Satellites")); 
        file.flush();
      }
      if (!file) {
        lcd.clear();
        lcd.home();
        lcd.print(F("SD card error!"));
        lcd.setCursor(0,1);
        lcd.print(F("Hit Reset"));
      }
    }

    if (myFlags & timeToSave) {
      SaveData();
      //Serial.println(F("Saved."));
      myFlags &= ~timeToSave;
    }
  } 
  else {
    // TODO:  Test loss of signal
    if ((myFlags & hadFix) && !GPS.fix && (myFlags & recording)) {
      if (!(myFlags & lostMsgPrinted)) {
        lcd.clear();
        lcd.home();
        lcd.print(F("Fix lost!"));
        // clear recordingMessageSet
        myFlags &= ~recordingMessageSet;
        // set lostMsgPrinted:
        myFlags |= lostMsgPrinted;
      }
    }
    else if (!(myFlags & recording)) {
      if (!(myFlags & stoppedMessageSet)) {
        lcd.clear();
        lcd.home();
        lcd.print(F("Paused."));
        lcd.setCursor(0,1);
        lcd.print(F("Press button"));
        Serial.println(F("Paused."));
        // clear recordingMessageSet
        myFlags &= ~recordingMessageSet;
        // set stoppedMessageSet:
        myFlags |= stoppedMessageSet;
      }
    }
  } 
}

// This function converts from degrees minutes to decimal degrees
// the GPS chip gives degrees minutes but Google Maps only understands either degrees minutes seconds, or decimal degrees. 
// latitude is DDMM.MMMM and longitude is DDDMM.MMMM
// W and S are negative, N and E are positive
void ConvertLatLon()
{
  float degrees = 0; 
  float minutes = 0; 
  int latsign, lonsign;
  if (GPS.lat == 'S') latsign = -1; 
  else latsign = 1;
  if (GPS.lon == 'W') lonsign = -1; 
  else lonsign = 1;
  degrees = floor(fabs(GPS.latitude/100));
  minutes = fabs(GPS.latitude) - degrees*100;
  if (latsign == 1) convertedLatitude = (float)(degrees + minutes/60);
  else  convertedLatitude = (float)(degrees + minutes/60)*-1;

  degrees = floor(fabs(GPS.longitude/100));
  minutes = fabs(GPS.longitude) - degrees*100;
  if (lonsign == 1) convertedLongitude = (degrees + minutes/60);
  else  convertedLongitude = (degrees + minutes/60)*-1;

}

// to avoid clutter in the Loop function, the saving part of the program has been split off and put here
void SaveData()
{
  ConvertLatLon();

  file.print(F("20"));
  file.print(GPS.year, DEC);
  file.print(F("/"));
  if (GPS.month < 10) file.print("0");
  file.print(GPS.month, DEC);
  file.print(F("/"));
  if (GPS.day < 10) file.print("0");
  file.print(GPS.day, DEC);
  file.print(F(", "));
  if (GPS.hour < 10) file.print("0");
  file.print(GPS.hour, DEC);
  file.print(F(":"));
  if (GPS.minute < 10) file.print("0");
  file.print(GPS.minute, DEC);
  file.print(F(":"));
  if (GPS.seconds < 10) file.print("0");
  file.print(GPS.seconds, DEC);
  file.print(F(", "));
  file.print((float) convertedLatitude, 6);
  file.print(F(", "));
  file.print((float) convertedLongitude, 6);
  file.print(F(", "));
  file.print((int) GPS.fixquality);
  file.print(F(", "));
  file.print(GPS.altitude);
  file.print(F(", "));
  file.print((int)GPS.satellites);
  file.println(F(""));
  file.flush();
}

ISR(TIMER1_OVF_vect)
{
  count++;
  if (count >= 19) // our timer overflow value is 19
  {
    if (myFlags & recording) myFlags |= timeToSave;
    count = 0;
  }
}

void ButtonPressed() {
  // mild debounce:
  // for memory issues, I'm going to use the free flag in myFlags
  volatile unsigned long int stopTime = millis();  // must be volatile since this is called from an ISR
  myFlags |= tempFlag;

  while (stopTime <= stopTime + 500)
  {
    if (digitalRead(2) == HIGH) myFlags &= ~tempFlag;
    stopTime += 30;
  }

  if (myFlags & tempFlag) {
    myFlags |= pressed;
    if ((myFlags & recording) && GPS.fix) {
      myFlags &= ~recording;
      myFlags &= ~recordingMessageSet;
      myFlags &= ~stoppedMessageSet;
    }
    else if (GPS.fix) {
      myFlags |= recording;
      myFlags &= ~recordingMessageSet;
    }

  } 
}

#ifdef VERBOSE
int ShowFreeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
#endif

void FileNameGenerator (char *filename, int offset) {
  // offset is the number of days you want to add/subtract to the name
  filename[0] = '0' + (GPS.year%1000)/10;
  filename[1] = '0' + (GPS.year%1000)%10;
  filename[2] = '-';
  filename[3] = '0' + GPS.month/10;
  filename[4] = '0' + GPS.month%10;
  filename[5] = '-';
  filename[6] = '0' + (GPS.day+offset)/10;
  filename[7] = '0' + (GPS.day+offset)%10;
  filename[8] = '.'; 
  filename[9] = 'c'; 
  filename[10] = 's'; 
  filename[11] = 'v'; 
  filename[12] = '\0';
}

// modified version of the listfiles example from SD library
void printRoot() {
  file = SD.open("/");
  file.rewindDirectory();
  if (!file) Serial.println(F("Couldn't open root."));
  while(true) {
    File entry =  file.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    // Print the 8.3 name
    Serial.print(entry.name());
    Serial.print(F("\t\t"));
    Serial.println(entry.size(), DEC);
    entry.close();
  }
}

