#include "SysTickInts.h"
#include "PLL.h"
#include "tm4c123gh6pm.h"

void PortF_Init(void);
void disable_interrupts(void);
void enable_interrupts(void);
void wait_for_interrupts(void);
//void GPIOPF_Handler(void);

volatile unsigned long count = 0;
volatile unsigned long In, Out;

#define TOGGLE_COUNT 500

/* main */
int main(void){
  PLL_Init();                 // bus clock at 80 MHz
  PortF_Init();
  count = 0;

  SysTick_Init(80000);        // initialize SysTick timer
  enable_interrupts();

  while(1){                   // interrupts every 1ms
      wait_for_interrupts();
  }
}


/* Initialize PortF GPIOs */

void PortF_Init(void) {
    SYSCTL_RCGC2_R |= 0x00000020;           // activate clock for PortF
    while ((SYSCTL_PRGPIO_R & 0x00000020) == 0)
    {};                          // wait until PortF is ready
    GPIO_PORTF_LOCK_R = 0x4C4F434B;         // unlock GPIO PortF
    GPIO_PORTF_CR_R = 0x1F;                 // allow changes to PF4-0
    GPIO_PORTF_AMSEL_R = 0x00;              // disable analog on PortF
    GPIO_PORTF_PCTL_R = 0x00000000;         // use PF4-0 as GPIO
    GPIO_PORTF_DIR_R = 0x0E;                // PF4,PF0 in, PF3-1 out
    GPIO_PORTF_AFSEL_R = 0x00;              // disable alt function on PF
    GPIO_PORTF_PUR_R = 0x11;                // enable pull-up on PF0,PF4
    GPIO_PORTF_DEN_R = 0x1F;                // enable digital I/O on PF4-0
// GPIO initialization
    GPIO_PORTF_IM_R &= 0xEF;            //  disable interrupt from PF4
    GPIO_PORTF_IS_R &= 0xEF;              // PF4 is edge-sensitive
    GPIO_PORTF_IBE_R &= 0xEF;              // PF4 is not both edges
    GPIO_PORTF_IEV_R &= 0xEF;           // PF4 falling edge event
    GPIO_PORTF_ICR_R = 0x10;        /*  Clear flag4 */
    GPIO_PORTF_IM_R |= 0x10;            // enable interrupt on PF4

    NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF1FFFFF) | 0x00A00000; /*  priority 5 */
    NVIC_EN0_R = 0x40000000;        /*  Enable interrupt 30 in NVIC */

 //   enable_interrupts();             /* Enable global Interrupt flag (I) */
}


/* Disable interrupts by setting the I bit in the PRIMASK system register */
void disable_interrupts(void) {
    __asm("    CPSID  I\n"
          "    BX     LR");
}


/* Enable interrupts by clearing the I bit in the PRIMASK system register */
void enable_interrupts(void) {
    __asm("    CPSIE  I\n"
          "    BX     LR");
}


/* Enter low-power mode while waiting for interrupts */
void wait_for_interrupts(void) {
    __asm("    WFI\n"
          "    BX     LR");
}


/* Interrupt service routine for SysTick Interrupt */
// Executed every 12.5ns*(period)
void SysTick_Handler(void){
    count++;
    if (count == TOGGLE_COUNT - 1) {
        count = 0;
        Out = GPIO_PORTF_DATA_R;
        if (Out & 0x04) Out = Out & 0xFB;   // Toggle bit at PF2 position
        else Out = Out | 0x04;
        GPIO_PORTF_DATA_R = Out;            // output
    }
}

void GPIOPF_Handler(void){
    GPIO_PORTF_ICR_R = 0x10;
    Out = GPIO_PORTF_DATA_R;
    if (Out & 0x02) Out = Out & 0xFD;   // Toggle bit at PF2 position
    else Out = Out | 0x02;
    GPIO_PORTF_DATA_R = Out;            // output

//    volatile int m;

//    GPIO_PORTF_ICR_R = 0x10;        /* clear PF4 int */
//    GPIO_PORTF_DATA_R ^= (1<<1);    /* toggle Blue LED */
//    m = GPIO_PORTF_ICR_R;    /* a read to force clearing of interrupt flag */
//    m = m;
}
