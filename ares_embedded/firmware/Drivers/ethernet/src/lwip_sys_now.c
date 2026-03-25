#include "lwip/sys.h"
#include "stm32h7xx_hal.h"

u32_t sys_now(void)
{
  return (u32_t)HAL_GetTick();
}

