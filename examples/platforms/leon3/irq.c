
#include <stddef.h>
#include <asm-leon/lambapp_devs.h>
#include <asm-leon/irq.h>
#include "platform-leon3.h"

static volatile struct irqmpreg {
    uint32_t level;
    uint32_t pending;
    uint32_t force;
    uint32_t clear;
} *s_irqmp_regs;

static volatile uint32_t *s_mask_reg;

void leon3IntInit(void)
{
    // Get register pointer
    s_irqmp_regs = (struct irqmpreg *)amba_find_apbslv_addr(VENDOR_GAISLER, GAISLER_IRQMP, 0);
    s_mask_reg = ((uint32_t *)s_irqmp_regs) + 0x10;

    // Enable UART IRQs
    catch_interrupt((int)&UARTIntHandler, kUARTIRQ);
    *s_mask_reg |= (1 << kUARTIRQ);

    // Enable MRF interrupts
    catch_interrupt((int)&mrfIntHandler, kMRFIRQ);
    *s_mask_reg |= (1 << kMRFIRQ);
}

