#ifndef SYSTICK_H
#define SYSTICK_H

#include "../core/memorymap_f1.h"
#include "../core/common.h"

#define STK_CSR				MMIO32(SYS_TICK_BASE + 0x00)
#define STK_RVR				MMIO32(SYS_TICK_BASE + 0x04)
#define STK_CVR				MMIO32(SYS_TICK_BASE + 0x08)
#define STK_CALIB			MMIO32(SYS_TICK_BASE + 0x0C)

#define STK_CSR_ENABLE     (1 << 0)
#define STK_CSR_TICKINT    (1 << 1)
#define STK_CSR_CLKSOURCE  (1 << 2)

#define SYSCLK_HZ           (72000000)
void systick_init(uint32_t sysclk_hz);
uint32_t millis(void);

#endif