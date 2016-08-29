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
 *   This file includes the platform-specific initializers.
 *
 */

#ifndef PLATFORM_LEON3_H_
#define PLATFORM_LEON3_H_

#include <stdint.h>

//#include <asm-leon/amba.h>
#include <asm-leon/lambapp_devs.h>

enum
{
    kSystemClock = 50000000,  ///< MHz
    kBaudRate = 115200,
    kReceiveBufferSize = 128,
    kUARTIRQ = 2,
    kTicksPerSec = 1000,      ///< Alarm ticks per second
};

extern void amba_init(void);
extern unsigned long amba_find_apbslv_addr(unsigned long vendor, unsigned long device, unsigned long *irq);

/**
 * This function initializes the alarm service used by OpenThread.
 *
 */
void leon3AlarmInit(void);

/**
 * This function performs alarm driver processing.
 *
 */
void leon3AlarmProcess(void);

/**
 * This function initializes the radio service used by OpenThread.
 *
 */
void leon3RadioInit(void);

/**
 * This function performs radio driver processing.
 *
 */
void leon3RadioProcess(void);

/**
 * This function initializes the random number service used by OpenThread.
 *
 */
void leon3RandomInit(void);

/**
 * This function performs UART driver processing.
 *
 */
void leon3UartProcess(void);

void leon3EnableIrq (int irq);
void leon3DisableIrq (int irq);

#endif  // PLATFORM_LEON3_H_
