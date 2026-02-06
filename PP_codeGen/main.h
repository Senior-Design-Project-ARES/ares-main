#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"  // Adjust for your STM32 family (f1, f4, f7, h7, etc.)

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
```

**Build Configuration (Important!):**

In your IDE (STM32CubeIDE, Keil, etc.), set these compiler flags:
- **Optimization**: `-O2` or `-O3` (critical for Eigen performance!)
- **C++ Standard**: `-std=c++11` or higher
- **Include Paths**: Add path to Eigen library
- **Linker**: Link with `-lm` (math library)

**Expected Output (via UART at 115200 baud):**
```
=== STM32 Controller Benchmark Test ===
Target: 100Hz (10,000us period)
System Clock: 168 MHz

--- Statistics (Last 1 second) ---
Loop Count:    100
Current Time:  245 us
Min Time:      242 us
Max Time:      251 us
Overruns:      0
Status: PASS ✓ (Max < 10,000us)
Utilization:   2.5%