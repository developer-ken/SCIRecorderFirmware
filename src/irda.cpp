#include "irda.h"

void IRDA_Init()
{
    UART2.conf0.irda_en = 1;
    digitalWrite(IRDA_nTNOW, LOW);
}

void IRDA_BeginTx()
{
    UART2.conf0.irda_tx_en = 1;
}

void IRDA_EndTx(){
    UART2.conf0.irda_tx_en = 0;
}