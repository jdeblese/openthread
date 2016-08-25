/*
 *  Copyright (c) 2016, Jan-willem De Bleser
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
 *   This file implements a simple SPI driver for Leon3
 *
 */

#include <asm-leon/lambapp_devs.h>
#include "platform-leon3.h"
#include "spi.h"

#define SPI_MODE_ENABLE (1<<24)

static volatile struct spiregs {
    uint32_t capabilities;
    uint32_t r1, r2, r3, r4, r5, r6, r7;
    uint32_t mode;
    uint32_t event;
    uint32_t mask;
    uint32_t command;
    uint32_t transmit;
    uint32_t receive;
    uint32_t slavesel;
} *s_spi_regs;


void leon3SpiInit(void) {
    s_spi_regs = (struct spiregs *)amba_find_apbslv_addr(VENDOR_GAISLER, GAISLER_SPICTRL, 0);
    //s_spi_regs->mode |= (1<<26) | (1<<25) | (1<<2);  // MSb first, Master, Ignore SPISEL
    s_spi_regs->mode |= (1<<26) | (1<<25);  // MSb first, Master
}

void leon3SpiEnable(void) {
    s_spi_regs->mode |= SPI_MODE_ENABLE;
}

void leon3SpiDisable(void) {
    s_spi_regs->mode &= ~SPI_MODE_ENABLE;
}

void leon3SpiSetBaud(uint32_t hz) {
    uint32_t div;
    uint32_t mode = s_spi_regs->mode;

    s_spi_regs->mode = mode & ~SPI_MODE_ENABLE;

    mode &= ~(0xf<<16);

    /*
    if (hz > (kSystemClock>>6)) {
        div = kSystemClock / hz;
        div = (div>>2) + ((div>>1) & 1);  // divide by 4 w/ simple rounding
        mode &= ~(1<<27);  // div16
    } else {
    */
        div = kSystemClock / hz;
        div = (div>>6) + ((div>>5) & 1);  // divide by 64 w/ simple rounding
        mode |= 1<<27;  // div16
    //}
    div--;

    if (div > 0xf)
        div = 0xf;

    mode |= (div & 0xf)<<16;

    s_spi_regs->mode = mode;
}


void leon3SpiSetWidth(uint32_t bw) {
    uint32_t mode = s_spi_regs->mode;
    s_spi_regs->mode = mode & ~SPI_MODE_ENABLE;
    mode &= ~(0xf << 20);
    mode |= (((bw - 1) & 0xf) << 20);
    s_spi_regs->mode = mode;
}

uint8_t leon3SpiTx(uint8_t len, uint32_t *buf) {
    uint8_t maxlen = len;
    uint32_t *start = buf;

    /*
    s_spi_regs->command |= (1<<22);
    s_spi_regs->event   |= (1<<14);
    */

    s_spi_regs->slavesel &= ~1;

    for (; len > 0; len--)
        s_spi_regs->transmit = *(buf++);

    while (s_spi_regs->event & (1<<31));  // block while transfer in progress

    buf = start;
    while (s_spi_regs->event & (1<<9) && len < maxlen) {  // while fifo not empty
        *(buf++) = s_spi_regs->receive;
        len++;
    }

    s_spi_regs->slavesel |= 1;

    return len;
}

