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
    IEEE802154_MIN_LENGTH         = 5,
    IEEE802154_MAX_LENGTH         = 127,
    IEEE802154_ACK_LENGTH         = 5,

    IEEE802154_BROADCAST          = 0xffff,

    IEEE802154_FRAME_TYPE_ACK     = 2 << 0,
    IEEE802154_FRAME_TYPE_MACCMD  = 3 << 0,
    IEEE802154_FRAME_TYPE_MASK    = 7 << 0,

    IEEE802154_SECURITY_ENABLED   = 1 << 3,
    IEEE802154_FRAME_PENDING      = 1 << 4,
    IEEE802154_ACK_REQUEST        = 1 << 5,
    IEEE802154_PANID_COMPRESSION  = 1 << 6,

    IEEE802154_DST_ADDR_NONE      = 0 << 2,
    IEEE802154_DST_ADDR_SHORT     = 2 << 2,
    IEEE802154_DST_ADDR_EXT       = 3 << 2,
    IEEE802154_DST_ADDR_MASK      = 3 << 2,

    IEEE802154_SRC_ADDR_NONE      = 0 << 6,
    IEEE802154_SRC_ADDR_SHORT     = 2 << 6,
    IEEE802154_SRC_ADDR_EXT       = 3 << 6,
    IEEE802154_SRC_ADDR_MASK      = 3 << 6,

    IEEE802154_DSN_OFFSET         = 2,
    IEEE802154_DSTPAN_OFFSET      = 3,
    IEEE802154_DSTADDR_OFFSET     = 5,

    IEEE802154_SEC_LEVEL_MASK     = 7 << 0,

    IEEE802154_KEY_ID_MODE_0      = 0 << 3,
    IEEE802154_KEY_ID_MODE_1      = 1 << 3,
    IEEE802154_KEY_ID_MODE_2      = 2 << 3,
    IEEE802154_KEY_ID_MODE_3      = 3 << 3,
    IEEE802154_KEY_ID_MODE_MASK   = 3 << 3,

    IEEE802154_MACCMD_DATA_REQ    = 4,
};

static RadioPacket sTransmitFrame;
static RadioPacket sReceiveFrame;
static ThreadError sTransmitError;
//static ThreadError sReceiveError;
static uint8_t sTransmitting;

static uint8_t sTransmitPsdu[IEEE802154_MAX_LENGTH];
static uint8_t sReceivePsdu[IEEE802154_MAX_LENGTH];

static PhyState sState = kStateDisabled;
static bool sIsReceiverEnabled = false;
static bool sAckWait = false;

static uint8_t sLastRss = 0;

// *****************************************************************************
//                     Frame manipulation routines
// *****************************************************************************

static inline bool isFrameTypeAck(const uint8_t *frame)
{
    return (frame[0] & IEEE802154_FRAME_TYPE_MASK) == IEEE802154_FRAME_TYPE_ACK;
}

static inline bool isFrameTypeMacCmd(const uint8_t *frame)
{
    return (frame[0] & IEEE802154_FRAME_TYPE_MASK) == IEEE802154_FRAME_TYPE_MACCMD;
}

static inline bool isSecurityEnabled(const uint8_t *frame)
{
    return (frame[0] & IEEE802154_SECURITY_ENABLED) != 0;
}

static inline bool isFramePending(const uint8_t *frame)
{
    return (frame[0] & IEEE802154_FRAME_PENDING) != 0;
}

static inline bool isAckRequested(const uint8_t *frame)
{
    return (frame[0] & IEEE802154_ACK_REQUEST) != 0;
}

static inline bool isPanIdCompressed(const uint8_t *frame)
{
    return (frame[0] & IEEE802154_PANID_COMPRESSION) != 0;
}

static inline bool isDataRequest(const uint8_t *frame)
{
    const uint8_t *cur = frame;
    uint8_t securityControl;
    bool rval;

    // FCF + DSN
    cur += 2 + 1;

    VerifyOrExit(isFrameTypeMacCmd(frame), rval = false);

    // Destination PAN + Address
    switch (frame[1] & IEEE802154_DST_ADDR_MASK)
    {
    case IEEE802154_DST_ADDR_SHORT:
        cur += sizeof(otPanId) + sizeof(otShortAddress);
        break;

    case IEEE802154_DST_ADDR_EXT:
        cur += sizeof(otPanId) + sizeof(otExtAddress);
        break;

    default:
        ExitNow(rval = false);
    }

    // Source PAN + Address
    switch (frame[1] & IEEE802154_SRC_ADDR_MASK)
    {
    case IEEE802154_SRC_ADDR_SHORT:
        if (!isPanIdCompressed(frame))
        {
            cur += sizeof(otPanId);
        }

        cur += sizeof(otShortAddress);
        break;

    case IEEE802154_SRC_ADDR_EXT:
        if (!isPanIdCompressed(frame))
        {
            cur += sizeof(otPanId);
        }

        cur += sizeof(otExtAddress);
        break;

    default:
        ExitNow(rval = false);
    }

    // Security Control + Frame Counter + Key Identifier
    if (isSecurityEnabled(frame))
    {
        securityControl = *cur;

        if (securityControl & IEEE802154_SEC_LEVEL_MASK)
        {
            cur += 1 + 4;
        }

        switch (securityControl & IEEE802154_KEY_ID_MODE_MASK)
        {
        case IEEE802154_KEY_ID_MODE_0:
            cur += 0;
            break;

        case IEEE802154_KEY_ID_MODE_1:
            cur += 1;
            break;

        case IEEE802154_KEY_ID_MODE_2:
            cur += 5;
            break;

        case IEEE802154_KEY_ID_MODE_3:
            cur += 9;
            break;
        }
    }

    // Command ID
    rval = cur[0] == IEEE802154_MACCMD_DATA_REQ;

exit:
    return rval;
}

static inline uint8_t getDsn(const uint8_t *frame)
{
    return frame[IEEE802154_DSN_OFFSET];
}

static inline otPanId getDstPan(const uint8_t *frame)
{
    return (otPanId)((frame[IEEE802154_DSTPAN_OFFSET + 1] << 8) | frame[IEEE802154_DSTPAN_OFFSET]);
}

static inline otShortAddress getShortAddress(const uint8_t *frame)
{
    return (otShortAddress)((frame[IEEE802154_DSTADDR_OFFSET + 1] << 8) | frame[IEEE802154_DSTADDR_OFFSET]);
}

static inline void getExtAddress(const uint8_t *frame, otExtAddress *address)
{
    size_t i;

    for (i = 0; i < sizeof(otExtAddress); i++)
    {
        address->m8[i] = frame[IEEE802154_DSTADDR_OFFSET + (sizeof(otExtAddress) - 1 - i)];
    }
}

// *****************************************************************************
//                     Low-level MRF communication
// *****************************************************************************


uint8_t transaction(uint32_t addr, uint32_t data) {
    uint32_t buf[2];
    buf[0] = addr;
    buf[1] = data;
    leon3SpiTx(2, buf);
    return buf[1]>>16;
}

// Radio utility functions

PhyState getRadioState(void)
{
    return sState;
}

PhyState getAckWait(void)
{
    return sAckWait;
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

    if ((sState == kStateTransmit && !sAckWait && !sTransmitting) || sState == kStateReceive)
    {
        error = kThreadError_None;
        sState = kStateTransmit;
    }

    return error;
}

int8_t otPlatRadioGetRssi(void)
{
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

// *****************************************************************************
//                       MRF Configuration
// *****************************************************************************

// *****************************************************************************
//                     Radio Functionality
// *****************************************************************************

void radioProcessFrame(void)
{
    ThreadError error = kThreadError_None;
    /* Don't bother verifying frame, MRF should have already
    otPanId dstpan;
    otShortAddress short_address;
    otExtAddress ext_address;

    VerifyOrExit(sPromiscuous == false, error = kThreadError_None);

    switch (sReceiveFrame.mPsdu[1] & IEEE802154_DST_ADDR_MASK)
    {
    case IEEE802154_DST_ADDR_NONE:
        break;

    case IEEE802154_DST_ADDR_SHORT:
        dstpan = getDstPan(sReceiveFrame.mPsdu);
        short_address = getShortAddress(sReceiveFrame.mPsdu);
        VerifyOrExit((dstpan == IEEE802154_BROADCAST || dstpan == sPanid) &&
                     (short_address == IEEE802154_BROADCAST || short_address == sShortAddress),
                     error = kThreadError_Abort);
        break;

    case IEEE802154_DST_ADDR_EXT:
        dstpan = getDstPan(sReceiveFrame.mPsdu);
        getExtAddress(sReceiveFrame.mPsdu, &ext_address);
        VerifyOrExit((dstpan == IEEE802154_BROADCAST || dstpan == sPanid) &&
                     memcmp(&ext_address, sExtendedAddress, sizeof(ext_address)) == 0,
                     error = kThreadError_Abort);
        break;

    default:
        ExitNow(error = kThreadError_Abort);
    }
    */

    // FIXME set power value correctly
    sReceiveFrame.mPower = -20;

    // generate acknowledgment
    // Done automatically by MRF

//exit:

#if OPENTHREAD_ENABLE_DIAG

    if (otPlatDiagModeGet())
    {
        otPlatDiagRadioReceiveDone(error == kThreadError_None ? &sReceiveFrame : NULL, error);
    }
    else
#endif
    {
        otPlatRadioReceiveDone(error == kThreadError_None ? &sReceiveFrame : NULL, error);
    }
}

void radioReceive(void)
{
    if (!sTransmitting && (sState != kStateTransmit || sAckWait))
    {
        uint8_t length, i;

        s_gpio[1] |= 0x4;

        leon3SpiSetWidth(8);
        transaction(shortWr(0x39,0x04));  // Disable read?

        // Read in frame from MRF Rx FIFO

        // read length
        leon3SpiSetWidth(12);
        length = transaction(longRd(0x300));
        //VerifyOrExit(IEEE802154_MIN_LENGTH <= length && length <= IEEE802154_MAX_LENGTH, ;);

        // read psdu
        for (i = 0; i < length - 2; i++)
        {
            sReceiveFrame.mPsdu[i] = transaction(longRd(0x301 + i));
        }

        // Set LQI, RSS, length
        sReceiveFrame.mLqi    = transaction(longRd(0x301 + length));
        sLastRss              = transaction(longRd(0x301 + length + 1));
        sReceiveFrame.mLength = length;

        leon3SpiSetWidth(8);
        transaction(shortWr(0x39,0x00));  // Enable read?

        if (sAckWait &&
            sTransmitFrame.mChannel == sReceiveFrame.mChannel &&
            isFrameTypeAck(sReceiveFrame.mPsdu))
        {
            uint8_t tx_sequence = getDsn(sTransmitFrame.mPsdu);
            uint8_t rx_sequence = getDsn(sReceiveFrame.mPsdu);

            if (tx_sequence == rx_sequence)
            {
                sState = kStateReceive;
                sAckWait = false;

#if OPENTHREAD_ENABLE_DIAG

                if (otPlatDiagModeGet())
                {
                    otPlatDiagRadioTransmitDone(isFramePending(sReceiveFrame.mPsdu), kThreadError_None);
                }
                else
#endif
                {
                    otPlatRadioTransmitDone(isFramePending(sReceiveFrame.mPsdu), kThreadError_None);
                }
            }
        }
        else if (sState == kStateReceive)
        {
            radioProcessFrame();
        }

        s_gpio[1] &= ~0x4;

    }
}

void radioStartTransmit(void)
{
    uint8_t i;

    s_gpio[1] |= 0x1;

    //while (HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_TX_ACTIVE);

    // flush txfifo

    // Write packet to normal TX FIFO

    leon3SpiSetWidth(12);

    // length of MHR
    // FIXME this needs to be completed: properly determine header length
    if (sTransmitFrame.mPsdu[0] & 0xC0)
    {
        transaction(longWr(0x000, 15));
    }
    else
    {
        transaction(longWr(0x000, 7));
    }

    // length of MHR + MSDU
    transaction(longWr(0x001, sTransmitFrame.mLength));

    // MHR + MSDU, without frame check
    for (i = 0; i < sTransmitFrame.mLength; i++)
    {
        transaction(longWr(0x002 + i, sTransmitFrame.mPsdu[i]));
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
    sAckWait = isAckRequested(sTransmitFrame.mPsdu);

    leon3SpiSetWidth(12);
    i = transaction(longRd(0x20F))>>2;
    s_gpio[1] &= ~0x38;
    s_gpio[1] |= i & 0x38;
}

void radioCompleteTransmit(void)
{
    uint8_t txnstat;

    if (sTransmitting)
    {
        s_gpio[1] &= ~0x01;

        txnstat = transaction(shortRd(0x24)) & 0x1;

        sTransmitting = 0;
        sTransmitError = txnstat != 0 ? kThreadError_ChannelAccessFailure : kThreadError_None;  // TXNSTAT bit

        if (txnstat)
            s_gpio[1] |= 0x2;  // Led on for transmission error
        else
            s_gpio[1] &= ~0x2;

        if (!sAckWait)
        {
            sState = kStateReceive;

#if OPENTHREAD_ENABLE_DIAG

            if (otPlatDiagModeGet())
            {
                otPlatDiagRadioTransmitDone(false, kThreadError_None);
            }
            else
#endif
            {
                otPlatRadioTransmitDone(false, kThreadError_None);
            }
        }
    }
}

void leon3RadioProcess(void)
{
    /*
    if (sTransmitting)
        s_gpio[1] &= ~0x10;
    else
        s_gpio[1] |= 0x10;  // Led on for transmitting

    if (sState == kStateReceive)
        s_gpio[1] &= ~0x8;
    else if (sState == kStateTransmit)
        s_gpio[1] |= 0x8;  // Led on for state transmit
        */

    if (sState == kStateTransmit && !sAckWait && !sTransmitting)
    {
        radioStartTransmit();
    }

    /*
    uint8_t i;
    leon3SpiSetWidth(12);
    i = transaction(longRd(0x20F))>>2;
    s_gpio[1] &= ~0x38;
    s_gpio[1] |= i & 0x38;
    */
}

void mrfIntHandler(int irq)
{
    uint8_t intstat;

    leon3SpiSetWidth(8);
    intstat = transaction(shortRd(0x31));

    // Process transmit interrupt first
    if (intstat & 0x01)  // TXIF
    {
        radioCompleteTransmit();
    }

    // Check if there's also a receive interrupt
    if (intstat & 0x08)  // RXIF
    {
        radioReceive();
    }

    (void)irq;

    uint8_t i;
    leon3SpiSetWidth(12);
    i = transaction(longRd(0x20F))>>2;
    s_gpio[1] &= ~0x38;
    s_gpio[1] |= i & 0x38;
//exit:
    return;
}
