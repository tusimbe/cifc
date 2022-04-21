
#include <CrashCatcher.h>
#include <stdio.h>
#include "ch.h"
#include "hal.h"

int CrashCatcher_getc(void)
{
    int b;
    SerialDriver *sdp = &STDIN_SD;
    USART_TypeDef *u = sdp->usart;
    uint16_t sr;
   
    sr = u->SR;
    while (!(sr & USART_SR_RXNE)) {
        sr = u->SR;
    }

    sr = u->SR;
    while (!(sr & USART_SR_RXNE)) {
        sr = u->SR;
    }
    
    b = u->DR & 0xff;
    return b;
}

void CrashCatcher_putc(int c)
{
    SerialDriver *sdp = &STDOUT_SD;
    USART_TypeDef *u = sdp->usart;
    uint16_t sr;
    
    u->DR = c;
    sr = u->SR;

    while (!(sr & USART_SR_TC)) {
        sr = u->SR;
    }
}

const CrashCatcherMemoryRegion* CrashCatcher_GetMemoryRegions(void)
{
    static const CrashCatcherMemoryRegion regions[] = {
        {0x20020000, 0x2002ffff, CRASH_CATCHER_BYTE},
        {0x2001c000, 0x2001ffff, CRASH_CATCHER_BYTE},
        {0x20000000, 0x2001bfff, CRASH_CATCHER_BYTE},
        {0xFFFFFFFF, 0xFFFFFFFF, CRASH_CATCHER_BYTE},
    };

    return regions;
}


