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
 *   This file implements the OpenThread platform abstraction for the alarm.
 *
 */

#include <stdbool.h>
#include <stdint.h>

#include <platform/alarm.h>
#include "platform-leon3.h"

enum
{
    kTicksPerSec = 1000,      ///< Alarm ticks per second
};

static uint32_t s_alarm_t0 = 0;
static uint32_t s_alarm_dt = 0;
static bool s_is_running = false;

static volatile struct timer_unit {
    uint32_t counter;
    uint32_t reload;
    uint32_t ctrl;
    uint32_t latch;
} *s_timer_units;

static volatile struct timer_regs {
    uint32_t scaler;
    uint32_t reload;
    uint32_t cfg;
    uint32_t latchcfg;
    struct timer_unit t1;
} *s_timer_regs;


void leon3AlarmInit(void)
{
    s_timer_regs = (struct timer_regs *)amba_find_apbslv_addr(VENDOR_GAISLER, GAISLER_GPTIMER, 0);
    s_timer_units = &(s_timer_regs->t1);
}

uint32_t otPlatAlarmGetNow(void)
{
    return 0xFFFFFFFF - s_timer_regs->t1.counter;
}

void otPlatAlarmStartAt(uint32_t t0, uint32_t dt)
{
    s_alarm_t0 = t0;
    s_alarm_dt = dt;
    s_is_running = true;
}

void otPlatAlarmStop(void)
{
    s_is_running = false;
}

void leon3AlarmProcess(void)
{
    uint32_t expires;
    bool fire = false;
    uint32_t time = otPlatAlarmGetNow();

    if (s_is_running)
    {
        expires = s_alarm_t0 + s_alarm_dt;

        if (s_alarm_t0 <= time)
        {
            if (expires >= s_alarm_t0 && expires <= time)
            {
                fire = true;
            }
        }
        else
        {
            if (expires >= s_alarm_t0 || expires <= time)
            {
                fire = true;
            }
        }

        if (fire)
        {
            s_is_running = false;
            otPlatAlarmFired();
        }
    }

}
