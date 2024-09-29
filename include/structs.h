#ifndef _H_STRUCTS_MAIN_
#define _H_STRUCTS_MAIN_

struct FSDState
{
    /* data */
    uint16_t StateWord, MalfunctionWord, DigitalInState;
    float TargetFrequency, CurrentFrequency,
        OutputFrequency, OutputCurrent,
        OutputTorque, SystemTemperature;
    int RailVotage, OutputVotage, OutputPower,
        MotorRPM, AI1Val, AI2Val, PulseInFreq;
    uint16_t Raw[16];
};

#endif