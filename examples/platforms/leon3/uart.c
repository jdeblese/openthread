/*
 *  Copyright (c) 2016, Nest Labs, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file implements the OpenThread platform abstraction for UART communication.
 *
 */

#include <stddef.h>

#include <asm-leon/lambapp_devs.h>
#include <asm-leon/irq.h>

#include <common/code_utils.hpp>
#include <openthread-types.h>
#include <platform/uart.h>
#include "platform-leon3.h"

#define APBUART_STATUS_TXFULL  0x200
#define APBUART_STATUS_DATARDY 0x1

#define APBUART_CTRL_TINTEN    0x8
#define APBUART_CTRL_RINTEN    0x4
#define APBUART_CTRL_TXEN      0x2
#define APBUART_CTRL_RXEN      0x1

//extern void *catch_interrupt(void func(), int irq);

void UARTIntHandler(int irq);

static void processReceive(void);
static void processTransmit(void);

static const uint8_t *sTransmitBuffer = NULL;
static uint16_t sTransmitLength = 0;

static uint8_t sReceiveBuffer[kReceiveBufferSize];
static uint16_t sReceiveHead = 0;
static uint16_t sReceiveLength = 0;

static volatile struct apbuartreg {
    uint32_t data;  // Lower 8 bits are data, access the fifos
    uint32_t status;
    uint32_t control;
    uint32_t scaler;
    uint32_t debug;
} *sUartRegs;

ThreadError otPlatUartEnable(void)
{
    // Get register pointer
    sUartRegs = (struct apbuartreg *)amba_find_apbslv_addr(VENDOR_GAISLER, GAISLER_APBUART, 0);

    // baud rate
    sUartRegs->scaler = kSystemClock / (kBaudRate * 8 + 7);

    // enable rx interrupts
    // enable uart
    sUartRegs->control |= APBUART_CTRL_RINTEN | APBUART_CTRL_TXEN | APBUART_CTRL_RXEN;

    // Enable UART IRQs
    catch_interrupt((int)&UARTIntHandler, kUARTIRQ);
    leon3EnableIrq(kUARTIRQ);

    return kThreadError_None;
}

ThreadError otPlatUartDisable(void)
{
    return kThreadError_None;
}

ThreadError otPlatUartSend(const uint8_t *aBuf, uint16_t aBufLength)
{
    ThreadError error = kThreadError_None;

    VerifyOrExit(sTransmitBuffer == NULL, error = kThreadError_Busy);

    sTransmitBuffer = aBuf;
    sTransmitLength = aBufLength;

exit:
    return error;
}

void processReceive(void)
{
    uint16_t remaining;

    VerifyOrExit(sReceiveLength > 0, ;);

    remaining = kReceiveBufferSize - sReceiveHead;

    if (sReceiveLength >= remaining)
    {
        otPlatUartReceived(sReceiveBuffer + sReceiveHead, remaining);
        sReceiveHead = 0;
        sReceiveLength -= remaining;
    }

    if (sReceiveLength > 0)
    {
        otPlatUartReceived(sReceiveBuffer + sReceiveHead, sReceiveLength);
        sReceiveHead += sReceiveLength;
        sReceiveLength = 0;
    }

exit:
    return;
}

void processTransmit(void)
{
    VerifyOrExit(sTransmitBuffer != NULL, ;);

    // Will queue up bytes as long as the FIFO is not full
    /*
    while (!(sUartRegs->status & APBUART_STATUS_TXFULL))
    {
        if (!sTransmitLength)
        {
            sTransmitBuffer = NULL;
            otPlatUartSendDone();
            break;
        }
        sTransmitLength--;
        sUartRegs->data = *sTransmitBuffer++;
    }
    */

    // Will block until full transmission is queued
    for (; sTransmitLength > 0; sTransmitLength--)
    {
        while (sUartRegs->status & APBUART_STATUS_TXFULL);
        sUartRegs->data = *sTransmitBuffer++;
    }
    sTransmitBuffer = NULL;
    otPlatUartSendDone();

exit:
    return;
}

void leon3UartProcess(void)
{
    processReceive();
    processTransmit();
}

void UARTIntHandler(int irq)
{
    uint16_t tail;
    uint8_t byte;

    while (sUartRegs->status & APBUART_STATUS_DATARDY)
    {
        byte = sUartRegs->data;

       if (sReceiveLength < kReceiveBufferSize)
       {
           tail = (sReceiveHead + sReceiveLength) % kReceiveBufferSize;
           sReceiveBuffer[tail] = byte;
           sReceiveLength++;
       }
    }
}
