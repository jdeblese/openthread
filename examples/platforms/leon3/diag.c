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

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include <platform/alarm.h>
#include <platform/radio.h>
#include "platform-leon3.h"

#include "mrf.h"
#include "spi.h"

/**
 * diagnostics mode flag.
 *
 */
static bool sDiagMode = false;

void otPlatDiagProcess(int argc, char *argv[], char *aOutput, size_t aOutputMaxLen)
{
    // add more plarform specific diagnostics features here

    if (strcmp(argv[0], "alarm") == 0)
    {
        uint32_t t = otPlatAlarmGetNow();
        sprintf(aOutput, "alarm time is %lu (%lu.%03lu seconds)\r\n", t, t / kTicksPerSec, t%kTicksPerSec);
    }
    else if (strcmp(argv[0], "rssi") == 0)
    {
        sprintf(aOutput, "rssi is %d dBm\r\n", otPlatRadioGetRssi());
    }
    else if (strcmp(argv[0], "radio") == 0)
    {
        if (argc == 1)
        {
            switch (getRadioState())
            {
                case kStateDisabled :
                    sprintf(aOutput, "radio is disabled\r\n");
                    break;
                case kStateTransmit :
                    sprintf(aOutput, "radio is in transmit mode\r\n");
                    break;
                case kStateReceive :
                    sprintf(aOutput, "radio is in transmit mode\r\n");
                    break;
                case kStateSleep :
                    sprintf(aOutput, "radio is sleeping\r\n");
                    break;
                default :
                    sprintf(aOutput, "radio is in an unknown state\r\n");
                    break;
            }
        }
    }
    else if (strcmp(argv[0], "mrf") == 0)
    {
        sprintf(aOutput, "diag MRF access\r\n");
        if (argc > 1 && strcmp(argv[1], "mem") == 0)
        {
            if (argc > 2 && argv[2][0] == '0' && argv[2][1] == 'x')
            {
                uint32_t addr = strtol(argv[2], NULL, 16);
                uint8_t ret;

                if (addr & 0xF00)
                {
                    leon3SpiSetWidth(12);
                    ret = transaction(longRd(addr));
                }
                else
                {
                    leon3SpiSetWidth(8);
                    ret = transaction(shortRd(addr));
                }

                sprintf(aOutput, "0x%03lx : 0x%02hhx\r\n", addr, ret);
            }
        }
    }
    else
    {
        sprintf(aOutput, "diag feature '%s' is not supported\r\n", argv[0]);
    }

    (void)argc;
    (void)aOutputMaxLen;
}

void otPlatDiagModeSet(bool aMode)
{
    sDiagMode = aMode;
}

bool otPlatDiagModeGet()
{
    return sDiagMode;
}
