#include <stm32f10x.h>

/* STM32F103 errata timer capture.
 * - Board:
 * PC7, timer capture (TIM3 full remap)
 * PD2, GPIO output
 *
 * Test feed PC7: 200 Hz (T=5ms) square wave from Rigol DG4062.
 *
 * Symptom:
 * When time between capture and CCR read is longer than 25.5 us, 
 * the capture is lost.
 *
 * Scope:
 * Each falling edge on PC7 has a pulse of 58 us on PD2.
 * When a capture is missed, the CC2IF interrupt does not trigger 3
 * and PD2 will be high ~5ms.
 *
 */

#define ENABLE_ERRATA   1

volatile uint32_t systicks;

void SystemInit(void){

}

void SysTick_Handler(void){
  systicks++;
}

void TIM3_IRQHandler(void){
  (void)TIM3->CCR1;
  (void)TIM3->CCR2;
  (void)TIM3->CCR3;
  (void)TIM3->CCR4;
  GPIOD->BRR = (1<<2);
  TIM3->SR &= ~TIM_SR_CC2IF;
}

void EXTI9_5_IRQHandler(void){
  GPIOD->BSRR = (1<<2);
  // ~25 us worth of nop at 8 mhz
  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
  __NOP();
  #if ENABLE_ERRATA
  __NOP();
  #endif
  
  EXTI->PR = EXTI_PR_PR7;
}

#define TIM_CCMR1_CC2S_Pos                  (8U)        
#define TIM_CCMR1_IC2PSC_Pos                (10U)
#define TIM_CCMR1_IC2F_Pos                  (12U) 
#define TIM_CCER_CC2P_Pos                   (5U) 

int main(void){
  SysTick->CTRL = 0;
  systicks = 0;
  SysTick->LOAD = 8000000/1000;
  SysTick->VAL = 0; /* Load the SysTick Counter Value */
  SysTick->CTRL = (SysTick_CTRL_TICKINT_Msk   |  /* Enable SysTick exception */
                   SysTick_CTRL_ENABLE_Msk) |    /* Enable SysTick system timer */
                   SysTick_CTRL_CLKSOURCE_Msk;   /* Use processor clock source */
  
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
  RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
  AFIO->MAPR = AFIO_MAPR_TIM3_REMAP_FULLREMAP ;
  GPIOC->CRL = /*07*/ (1u << 30u)| (0u << 28u); // float input
  GPIOD->CRL = /*02*/ (0u << 10u)| (3u << 8u);  // fast out push-pull
  
  // Enable timer
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  TIM3->PSC = 3-1;
  TIM3->ARR = 0xFFFF;                   // Set reload value (max)
  TIM3->CCMR1 = (1<<TIM_CCMR1_CC2S_Pos)     // Inout without swapping
               |(0<<TIM_CCMR1_IC2PSC_Pos)   // No prescaler
               |(15<<TIM_CCMR1_IC2F_Pos);   // Maximum filter
  TIM3->CCER |= TIM_CCER_CC2E | (1<<TIM_CCER_CC2P_Pos);
  TIM3->DIER = TIM_DIER_CC2IE;
  TIM3->CNT = TIM3->ARR;
  TIM3->CR1 |= TIM_CR1_CEN;
  
  // Enable exti
  AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI7_PC;
  EXTI->FTSR = EXTI_FTSR_TR7;
  EXTI->IMR  = EXTI_IMR_MR7;
  
  // Enable nvic
  NVIC_EnableIRQ(EXTI9_5_IRQn);
  NVIC_EnableIRQ(TIM3_IRQn);
  
  while(1){
    __NOP();
  }
}
