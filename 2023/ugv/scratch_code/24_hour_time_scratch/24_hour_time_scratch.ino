#include <DS3231.h>
#include <Wire.h>

DS3231 myRTC;

byte year;
byte month;
byte date;
byte dOW;
byte hour;
byte minute;
byte second;

void getDateStuff(byte& year, byte& month, byte& date, byte& dOW,
                  byte& hour, byte& minute, byte& second) {
    // Call this if you notice something coming in on
    // the serial port. The stuff coming in should be in
    // the order YYMMDDwHHMMSS, with an 'x' at the end.
    boolean gotString = false;
    char inChar;
    byte temp1, temp2;
    char inString[20];
    
    byte j=0;
    while (!gotString) {
        if (Serial.available()) {
            inChar = Serial.read();
            inString[j] = inChar;
            j += 1;
            if (inChar == 'x') {
                gotString = true;
            }
        }
    }
    Serial.println(inString);
    // Read year first
    temp1 = (byte)inString[0] -48;
    temp2 = (byte)inString[1] -48;
    year = temp1*10 + temp2;
    // now month
    temp1 = (byte)inString[2] -48;
    temp2 = (byte)inString[3] -48;
    month = temp1*10 + temp2;
    // now date
    temp1 = (byte)inString[4] -48;
    temp2 = (byte)inString[5] -48;
    date = temp1*10 + temp2;
    // now Day of Week
    dOW = (byte)inString[6] - 48;
    // now hour
    temp1 = (byte)inString[7] -48;
    temp2 = (byte)inString[8] -48;
    hour = temp1*10 + temp2;
    // now minute
    temp1 = (byte)inString[9] -48;
    temp2 = (byte)inString[10] -48;
    minute = temp1*10 + temp2;
    // now second
    temp1 = (byte)inString[11] -48;
    temp2 = (byte)inString[12] -48;
    second = temp1*10 + temp2;
}

void setup() {
    // Start the serial port
    Serial.begin(9600);
    
    // Start the I2C interface
    Wire.begin();
}

void loop() {
  timeLoop;
}

void timeLoop(){
  // If something is coming in on the serial line, it's
  // a time correction so set the clock accordingly.
  if (Serial.available()) {
    getDateStuff(year, month, date, dOW, hour, minute, second);
        
    myRTC.setClockMode(false);  // set to 24h
    //setClockMode(true); // set to 12h
        
    myRTC.setYear(year);
    myRTC.setMonth(month);
    myRTC.setDate(date);
    myRTC.setDoW(dOW);
    myRTC.setHour(hour);
    myRTC.setMinute(minute);
    myRTC.setSecond(second);      
  }
  delay(1000);
}
