/**
  * STM32F411xx startup file
  */

.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

/* Memory layout */
.section .isr_vector,"a",%progbits
.type g_pfnVectors, %object
.size g_pfnVectors, .-g_pfnVectors

/* Vector table */
g_pfnVectors:
  .word  _estack                        /* Top of Stack */
  .word  Reset_Handler                  /* Reset Handler */
  .word  NMI_Handler                    /* NMI Handler */
  .word  HardFault_Handler              /* Hard Fault Handler */
  .word  MemManage_Handler              /* MPU Fault Handler */
  .word  BusFault_Handler               /* Bus Fault Handler */
  .word  UsageFault_Handler             /* Usage Fault Handler */
  .word  0                              /* Reserved */
  .word  0                              /* Reserved */
  .word  0                              /* Reserved */
  .word  0                              /* Reserved */
  .word  SVC_Handler                    /* SVCall Handler */
  .word  DebugMon_Handler               /* Debug Monitor Handler */
  .word  0                              /* Reserved */
  .word  PendSV_Handler                 /* PendSV Handler */
  .word  SysTick_Handler                /* SysTick Handler */

  /* External interrupts */
  .word  0                              /* Add your interrupt handlers here */

/* Reset handler */
.section .text.Reset_Handler
.weak Reset_Handler
.type Reset_Handler, %function
Reset_Handler:
  ldr sp, =_estack    /* Set stack pointer */

/* Copy data from flash to RAM */
  movs r1, #0
  b LoopCopyDataInit

CopyDataInit:
  ldr r3, =_sidata
  ldr r3, [r3, r1]
  str r3, [r0, r1]
  adds r1, r1, #4

LoopCopyDataInit:
  ldr r0, =_sdata
  ldr r3, =_edata
  adds r2, r0, r1
  cmp r2, r3
  bcc CopyDataInit
  ldr r2, =_sbss
  b LoopFillZerobss

/* Zero fill the bss segment */
FillZerobss:
  movs r3, #0
  str r3, [r2], #4

LoopFillZerobss:
  ldr r3, = _ebss
  cmp r2, r3
  bcc FillZerobss

/* Call system init and main */
  bl SystemInit
  bl main
  bx lr

/* Default handlers */
.macro def_irq_handler handler_name
.weak \handler_name
.type \handler_name, %function
\handler_name:
  b .
.size \handler_name, . - \handler_name
.endm

def_irq_handler NMI_Handler
def_irq_handler HardFault_Handler
def_irq_handler MemManage_Handler
def_irq_handler BusFault_Handler
def_irq_handler UsageFault_Handler
def_irq_handler SVC_Handler
def_irq_handler DebugMon_Handler
def_irq_handler PendSV_Handler
def_irq_handler SysTick_Handler
