#ifndef __STM32_TMER_H
#define __STM32_TMER_H

#include "link.h"
#include "stm32f10x.h"

#define SF_TICK                10  // 10ms
#define TIMER_NUM

//#define TIMER_TEST

typedef struct
{
    Node list;
    void (*func)(void *parameter);
    void *parameter;
    uint32_t init_tick;
    uint32_t timeout_tick;
    uint16_t num;
    uint8_t val;
}st_timer,*st_timer_t;

void st_hw_usdelay(uint16_t num);
void st_hw_msdelay(uint16_t num);
void Delay(volatile int ns);

uint32_t get_timer(void);
st_timer_t cre_sf_timer(st_timer_t t_timer,uint8_t flag);
st_timer_t del_sf_timer(st_timer_t t_imer);
void reload_sf_timer(st_timer_t t_timer);
void update_sf_timer_timeout(st_timer_t t_timer, uint32_t timeout);

void sf_timer_init(void);
void sf_timer_proc(void);

void test_timer(void);

//-- hard  timer----
//void hal_timer_init(TIM_TypeDef* TIMx,TIM_Base_InitTypeDef *TIM_TimeBaseStructure);
//void hal_timer_enable_interrupt (uint8_t TIMx_irq);
//void hal_timer_disable_interrupt(uint8_t TIMx_irq);
//int16_t hal_timer_getvalue(TIM_TypeDef *TIMx);
//void hal_timer_setvalue(uint16_t next_event,TIM_TypeDef *TIMx);
//void hal_benchmarking_timer_init(void);
//uint32_t hal_benchmarking_timer_getvalue(void);
//void hal_benchmarking_timer_start(void);
//void hal_benchmarking_timer_stop(void);

#endif
