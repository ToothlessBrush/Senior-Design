// src/system_stm32f4xx.c
#include "stm32f411xe.h"

uint32_t systemcoreclock = 16000000; // hsi frequency

void SystemInit(void) {
/* fpu settings */
#if (__fpu_present == 1) && (__fpu_used == 1)
  scb->cpacr |=
      ((3ul << 10 * 2) | (3ul << 11 * 2)); /* set cp10 and cp11 full access */
#endif

  /* reset rcc clock configuration to default reset state */
  /* set hsion bit */
  RCC->CR |= (uint32_t)0x00000001;

  /* reset cfgr register */
  RCC->CFGR = 0x00000000;

  /* reset hseon, csson and pllon bits */
  RCC->CR &= (uint32_t)0xfef6ffff;

  /* reset pllcfgr register */
  RCC->PLLCFGR = 0x24003010;

  /* reset hsebyp bit */
  RCC->CR &= (uint32_t)0xfffbffff;

  /* disable all interrupts */
  RCC->CIR = 0x00000000;
}
