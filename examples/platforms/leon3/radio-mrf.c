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
 *   This file implements the OpenThread platform abstraction for radio communication.
 *
 */

#include <openthread-types.h>

#include <common/code_utils.hpp>
#include <platform/radio.h>
#include "platform-leon3.h"
#include "spi.h"
#include "mrf.h"

enum
{
    IEEE802154_MIN_LENGTH = 5,
    IEEE802154_MAX_LENGTH = 127,
    IEEE802154_ACK_LENGTH = 5,
    IEEE802154_FRAME_TYPE_MASK = 0x7,
    IEEE802154_FRAME_TYPE_ACK = 0x2,
    IEEE802154_FRAME_PENDING = 1 << 4,
    IEEE802154_ACK_REQUEST = 1 << 5,
    IEEE802154_DSN_OFFSET = 2,
};

static RadioPacket sTransmitFrame;
static RadioPacket sReceiveFrame;
static ThreadError sTransmitError;
static ThreadError sReceiveError;
static uint8_t sTransmitting;

static uint8_t sTransmitPsdu[IEEE802154_MAX_LENGTH];
static uint8_t sReceivePsdu[IEEE802154_MAX_LENGTH];

static PhyState sState = kStateDisabled;
static bool sIsReceiverEnabled = false;

static uint8_t sLastRss = 0;

uint8_t transaction(uint32_t addr, uint32_t data) {
    uint32_t buf[2];
    buf[0] = addr;
    buf[1] = data;
    leon3SpiTx(2, buf);
    return buf[1]>>16;
}

PhyState getRadioState(void)
{
    return sState;
}

void enableReceiver(void)
{
    sIsReceiverEnabled = true;
}

void disableReceiver(void)
{
    sIsReceiverEnabled = false;
}

void setChannel(uint8_t channel)
{
    // Set channel
    leon3SpiSetWidth(12);
    transaction(longWr(0x200,(((channel - 11) & 0xf)<<4) | 0x3));

    // Reset RF state machine
    leon3SpiSetWidth(8);
    transaction(shortWr(0x36,0x04));
    transaction(shortWr(0x36,0x00));

    // Delay > 192 us

    (void)channel;
}

ThreadError otPlatRadioSetPanId(uint16_t panid)
{
    ThreadError error = kThreadError_Busy;

    if (sState != kStateTransmit)
    {
        leon3SpiSetWidth(8);
        transaction(shortWr(0x01,panid & 0xff));
        transaction(shortWr(0x02,(panid>>8) & 0xff));
        error = kThreadError_None;
    }

    return error;
}

ThreadError otPlatRadioSetExtendedAddress(uint8_t *address)
{
    ThreadError error = kThreadError_Busy;

    if (sState != kStateTransmit)
    {
        int i;

        leon3SpiSetWidth(8);
        for (i = 0; i < 8; i++)
        {
            // Address is given LSB first in array?
            transaction(shortWr(0x0c, address[i]));
            //transaction(shortWr(0x0c - i, address[i]));
            (void)address[i];
        }

        error = kThreadError_None;
    }

    return error;
}

ThreadError otPlatRadioSetShortAddress(uint16_t address)
{
    ThreadError error = kThreadError_Busy;

    if (sState != kStateTransmit)
    {
        leon3SpiSetWidth(8);
        transaction(shortWr(0x03,address & 0xff));
        transaction(shortWr(0x04,(address>>8) & 0xff));
        (void)address;
        error = kThreadError_None;
    }

    return error;
}

static volatile uint32_t *s_gpio;
void leon3RadioInit(void)
{
    s_gpio = (uint32_t *)amba_find_apbslv_addr(VENDOR_GAISLER, GAISLER_GPIO, 0);

    s_gpio[1] = 0x00;
    s_gpio[2] = 0x3f;

    // Enable SPI link to radio
    leon3SpiInit();
    leon3SpiSetBaud(3125000);
    leon3SpiSetWidth(8);
    leon3SpiEnable();

    sTransmitFrame.mLength = 0;
    sTransmitFrame.mPsdu = sTransmitPsdu;
    sReceiveFrame.mLength = 0;
    sReceiveFrame.mPsdu = sReceivePsdu;

    leon3SpiSetWidth(8);
    transaction(shortWr(0x2a,0x07)); // Soft reset
    transaction(shortWr(0x18,0x98));
    transaction(shortWr(0x2e,0x95));
    leon3SpiSetWidth(12);
    transaction(longWr(0x200,0x03));
    transaction(longWr(0x201,0x01));
    transaction(longWr(0x202,0x80));  // Enable PLL
    transaction(longWr(0x206,0x90));
    transaction(longWr(0x207,0x80));  // 100 kHz internal osc
    transaction(longWr(0x208,0x10));
    transaction(longWr(0x220,0x21));
    leon3SpiSetWidth(8);
    transaction(shortWr(0x3a,0x80));  // CCA mode 1 (energy above threshold)
    transaction(shortWr(0x3f,0x60));  // Threshold value (~ -69 dBm)
    //transaction(shortWr(0x3e,0x40));  // RSSI for each packet in RXFIFO
    transaction(shortWr(0x32,~0x09));  // RX, TX interrupts
    transaction(shortWr(0x00,0x08));  // PAN coordinator
    //transaction(shortWr(0x11,0x1c));  // Unslotted (default)
    //transaction(shortWr(0x10,0xFF));  // No beacons, superframes (default)
    leon3SpiSetWidth(12);
    //transaction(longWr(0x200,0xf8));  // Set TX power to min
    transaction(longWr(0x200,0x00));  // Set TX power to max

    // Reset RF state machine
    leon3SpiSetWidth(8);
    transaction(shortWr(0x36,0x04));
    transaction(shortWr(0x36,0x00));

    // Delay > 192 us
    
    sTransmitting = 0;

    leon3SpiSetWidth(12);
    transaction(longWr(0x211,0x02));  // Rising edge interrupt
}

ThreadError otPlatRadioEnable(void)
{
    ThreadError error = kThreadError_Busy;

    if (sState == kStateSleep || sState == kStateDisabled)
    {
        error = kThreadError_None;
        sState = kStateSleep;
    }

    return error;
}

ThreadError otPlatRadioDisable(void)
{
    ThreadError error = kThreadError_Busy;

    if (sState == kStateDisabled || sState == kStateSleep)
    {
        error = kThreadError_None;
        sState = kStateDisabled;
    }

    return error;
}

ThreadError otPlatRadioSleep(void)
{
    ThreadError error = kThreadError_Busy;

    if (sState == kStateSleep || sState == kStateReceive)
    {
        error = kThreadError_None;
        sState = kStateSleep;
        disableReceiver();
    }

    sState = kStateSleep;

    return error;
}

ThreadError otPlatRadioReceive(uint8_t aChannel)
{
    ThreadError error = kThreadError_Busy;

    if (sState != kStateDisabled)
    {
        error = kThreadError_None;
        sState = kStateReceive;
        setChannel(aChannel);
        sReceiveFrame.mChannel = aChannel;
        enableReceiver();
    }

    return error;
}

RadioPacket *otPlatRadioGetTransmitBuffer(void)
{
    return &sTransmitFrame;
}

ThreadError otPlatRadioTransmit(void)
{
    ThreadError error = kThreadError_Busy;

    if (sState == kStateReceive)
    {
        int i;

        error = kThreadError_None;
        sState = kStateTransmit;
        sTransmitError = kThreadError_None;

        s_gpio[1] |= 0x1;

        //while (HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_TX_ACTIVE);

        // flush txfifo

        // Write packet to normal TX FIFO

        // frame length
        leon3SpiSetWidth(12);
        transaction(longWr(0x000, sTransmitFrame.mLength));

        // set frame data
        for (i = 0; i < sTransmitFrame.mLength; i++)
        {
            transaction(longWr(0x001 + i, sTransmitFrame.mPsdu[i]));
        }

        setChannel(sTransmitFrame.mChannel);
        //while ((HWREG(RFCORE_XREG_FSMSTAT1) & 1) == 0);

        // wait for valid rssi

        //VerifyOrExit(HWREG(RFCORE_XREG_FSMSTAT1) & (RFCORE_XREG_FSMSTAT1_CCA | RFCORE_XREG_FSMSTAT1_SFD),
                     //sTransmitError = kThreadError_ChannelAccessFailure);

        // begin transmit
        leon3SpiSetWidth(8);
        transaction(shortWr(0x1B, 0x01));

        //while (HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_TX_ACTIVE);

        sTransmitting = 1;

    }

    return error;
}

int8_t otPlatRadioGetRssi(void)
{
    /*
    uint8_t raw;

    leon3SpiSetWidth(8);
    transaction(shortWr(0x3e,0x80));  // RSSI on request
    while ((transaction(shortRd(0x3e)) & 0x01) == 0);

    leon3SpiSetWidth(12);
    raw = transaction(longRd(0x210));

    return ((int8_t)(raw / 5)) - 90;
    */
    return ((int8_t)(sLastRss / 5)) - 90;
}

otRadioCaps otPlatRadioGetCaps(void)
{
    return kRadioCapsNone;
}

bool otPlatRadioGetPromiscuous(void)
{
    return 0;
}

void otPlatRadioSetPromiscuous(bool aEnable)
{
    (void)aEnable;
}

void leon3RadioProcess(void)
{
    if ((sState == kStateReceive) && (sReceiveFrame.mLength > 0))
    {
        s_gpio[1] &= ~0x4;
#if OPENTHREAD_ENABLE_DIAG
        if (otPlatDiagModeGet())
        {
            otPlatDiagRadioReceiveDone(&sReceiveFrame, sReceiveError);
        }
        else
#endif
        {
            otPlatRadioReceiveDone(&sReceiveFrame, sReceiveError);
        }
    }

    if (sState == kStateTransmit && !sTransmitting)
    {
        if (sTransmitError != kThreadError_None || (sTransmitFrame.mPsdu[0] & IEEE802154_ACK_REQUEST) == 0)
        {
            sState = kStateReceive;
#if OPENTHREAD_ENABLE_DIAG
            if (otPlatDiagModeGet())
            {
                otPlatDiagRadioTransmitDone(false, sTransmitError);
            }
            else
#endif
            {
                otPlatRadioTransmitDone(false, sTransmitError);
            }
        }
        else if (sReceiveFrame.mLength == IEEE802154_ACK_LENGTH &&
                 (sReceiveFrame.mPsdu[0] & IEEE802154_FRAME_TYPE_MASK) == IEEE802154_FRAME_TYPE_ACK &&
                 (sReceiveFrame.mPsdu[IEEE802154_DSN_OFFSET] == sTransmitFrame.mPsdu[IEEE802154_DSN_OFFSET]))
        {
            sState = kStateReceive;
#if OPENTHREAD_ENABLE_DIAG

            if (otPlatDiagModeGet())
            {
                otPlatDiagRadioTransmitDone((sReceiveFrame.mPsdu[0] & IEEE802154_FRAME_PENDING) != 0, sTransmitError);
            }
            else
#endif
            {
                otPlatRadioTransmitDone((sReceiveFrame.mPsdu[0] & IEEE802154_FRAME_PENDING) != 0, sTransmitError);
            }
        }
    }

    sReceiveFrame.mLength = 0;
}

void mrfIntHandler(int irq)
{
    uint8_t intstat;

    leon3SpiSetWidth(8);
    intstat = transaction(shortRd(0x31));

    if (intstat & 0x08)  // RXIF
    {
        uint8_t length, i;

        VerifyOrExit(sState == kStateReceive || sState == kStateTransmit, ;);

        s_gpio[1] |= 0x4;

        transaction(shortWr(0x39,0x04));  // Disable read?

        // Read length
        leon3SpiSetWidth(12);
        length = transaction(longRd(0x300));
        VerifyOrExit(IEEE802154_MIN_LENGTH <= length && length <= IEEE802154_MAX_LENGTH, ;);

        // read psdu
        for (i = 0; i < length - 2; i++)
        {
            sReceiveFrame.mPsdu[i] = transaction(longRd(0x301 + i));
        }

        sReceiveFrame.mLqi    = transaction(longRd(0x301 + length));
        sLastRss              = transaction(longRd(0x301 + length + 1));
        sReceiveFrame.mLength = length;

        leon3SpiSetWidth(8);
        transaction(shortWr(0x39,0x00));  // Enable read?

    }
    if (intstat & 0x01)  // TXIF
    {
        uint8_t txnstat = transaction(shortRd(0x24)) & 0x1;
        sTransmitting = 0;
        sTransmitError = txnstat ? kThreadError_ChannelAccessFailure : kThreadError_None;  // TXNSTAT bit
        s_gpio[1] &= ~0x01;
        if (txnstat)
            s_gpio[1] |= 0x2;
        else
            s_gpio[1] &= ~0x2;
    }

    (void)irq;

exit:
    return;
}
