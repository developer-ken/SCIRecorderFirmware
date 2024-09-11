#ifndef _H_IRDA_
#define _H_IRDA_

#include <Arduino.h>
#include "pinout.h"
#include "soc/uart_struct.h"

void IRDA_Init();

void IRDA_BeginTx();

void IRDA_EndTx();

#endif