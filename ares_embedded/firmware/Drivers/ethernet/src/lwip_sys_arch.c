/*
 * Minimal sys_arch hooks for LwIP in NO_SYS mode.
 *
 * LwIP still uses lightweight protection in a few core paths (mem/memp/pbuf).
 * Provide a simple critical section using PRIMASK (global IRQ disable).
 */

#include "lwip/sys.h"

#include "cmsis_gcc.h"

sys_prot_t sys_arch_protect(void)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return (sys_prot_t)primask;
}

void sys_arch_unprotect(sys_prot_t pval)
{
  __set_PRIMASK((uint32_t)pval);
}

