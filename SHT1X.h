/**
 * SHT1x Library
 *
 * Copyright 2009 Jonathan Oxer <jon@oxer.com.au> / <www.practicalarduino.com>
 * Based on previous work by:
 *    Maurice Ribble: <www.glacialwanderer.com/hobbyrobotics/?p=5>
 *    Wayne ?: <ragingreality.blogspot.com/2008/01/ardunio-and-sht15.html>
 *
 * Updated for Arduino 1.6.5 Library Manger by Joel Bartlett 
 * SparkFun Electronics 
 * September 16, 2015
 *
 * Manages communication with SHT1x series (SHT10, SHT11, SHT15)
 * temperature / humidity sensors from Sensirion (www.sensirion.com).
 */

#ifndef SHT1X_h
#define SHT1X_h

class SHT1x
{
  public:
    SHT1x();
    float readHumidity();
    float readTemperatureC();
    float readTemperatureF();
    void setHeater(int on);
  private:
    float readTemperatureRaw();
    int shiftIn();
    void sendCommandSHT(int _command);
    void sendData(int _data);
    void waitForResultSHT();
    int getData16SHT();
    void skipCrcSHT();
};

#endif
