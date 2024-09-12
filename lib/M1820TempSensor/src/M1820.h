#ifndef _H_M1820_
#define _H_M1820_

#include <Arduino.h>
#include <OneWire.h>

class M1820
{
public:
    bool Valid = false;
    static OneWire *oneWire;
    static void SampleNow();

    void BindAddr(byte[8]);
    float receiveTemperature();
    byte Address[8];

private:
    uint8_t _pin;
};

#endif