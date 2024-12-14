#include <Arduino.h>
#include "M1820.h"

OneWire* M1820::oneWire;

void M1820::BindAddr(byte addr[8])
{
    for (int i = 0; i < 8; i++)
    {
        Address[i] = addr[i];
    }
}

float M1820::receiveTemperature()
{
    oneWire->reset();         // Reset the 1-Wire bus
    oneWire->select(Address); // Select the sensor
    oneWire->write(0xBE);     // Read data without using scratchpad

    byte data[2];
    // Read two bytes of data
    data[0] = oneWire->read();
    data[1] = oneWire->read();

    // Combine the two bytes into a 16-bit raw temperature value
    unsigned short rawTemperature = (data[1] << 8) | data[0];

    // Convert raw temperature value to Celsius
    LastTemperature = (((short)rawTemperature) / 256.0 + 40.0);
    return LastTemperature;
}

void M1820::SampleNow(){
    oneWire->reset();
    oneWire->write(0xCC); //SkipRom     广播
    oneWire->write(0x44); //Convert T   现在采集温度
    delay(150);           //            等待转换完成
}