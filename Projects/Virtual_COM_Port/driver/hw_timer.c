#include "hw_timer.h"
//#include "log.h"

volatile uint32_t jiff;
pNode timer_list;
st_timer_t pHead_timer;
uint16_t timer_count;

void (*TIM1_func)(void) = 0;
void (*TIM2_func)(void) = 0;
void (*TIM3_func)(void) = 0;
void (*TIM4_func)(void) = 0;

void st_hw_usdelay(uint16_t us)
{
    volatile  uint32_t delta;
    us = us * (SysTick->LOAD/1000) - 60;// 60 is tick offset
    delta = SysTick->VAL;
    while(delta - SysTick->VAL < us);
}


void st_hw_msdelay(uint16_t ms)
{
    volatile  uint32_t delta;
    if( 0xFFFFFFFF - jiff < ms) return;
    delta = jiff;
    while(jiff - delta < ms);
}

void Delay(volatile int ns)
{
    while(ns-->0)
        __ASM("NOP");
}

uint32_t get_timer(void)
{
    return jiff;
}


void sf_timer_init(void)
{
    jiff = 0;
    // init systick 1ms
    SysTick_Config(SystemCoreClock/1000);

    timer_list = 0;
    pHead_timer = 0;
    timer_count = 0;

    #ifdef TIMER_TEST
    test_timer();
    #endif
}

void sf_timer_loop(void)
{
    pNode pHead = timer_list;
    st_timer_t ptimer;
    uint32_t current_tick;

    current_tick = get_timer();
    while(pHead)
    {
        ptimer = list_entry(pHead,st_timer,list);
        if((current_tick - ptimer->init_tick) > ptimer->timeout_tick)
        {
            ptimer->func(ptimer->parameter);
            ptimer->init_tick = get_timer();
        }
        pHead = pHead->next;
    }
}

void reload_sf_timer(st_timer_t t_timer)
{
    pNode pHead = timer_list;
    st_timer_t ptimer;

    while(pHead)
    {
        ptimer = list_entry(pHead,st_timer,list);
        if ( ptimer == t_timer )
        {
            ptimer->init_tick = get_timer();
        }
        pHead = pHead->next;
    }
}

void update_sf_timer_timeout(st_timer_t t_timer, uint32_t timeout)
{
    pNode pHead = timer_list;
    st_timer_t ptimer;

    while(pHead)
    {
        ptimer = list_entry(pHead,st_timer,list);
        if ( ptimer == t_timer )
        {
            ptimer->timeout_tick = timeout;
            ptimer->init_tick = get_timer();
        }
        pHead = pHead->next;
    }
}

void sf_timer_proc(void)
{
    static uint32_t sf_tick=0;
    if(jiff - sf_tick > SF_TICK)
    {
        sf_timer_loop();
        sf_tick = jiff;
    }
}

st_timer_t cre_sf_timer(st_timer_t t_timer,uint8_t flag)
{
    if(timer_list == 0)
    {
        timer_list = &t_timer->list;
        t_timer->list.next = 0;
        t_timer->list.prv = 0;
        t_timer->init_tick = get_timer();
        timer_count = 1;
        return t_timer;
    }
    t_timer->init_tick = get_timer();
    timer_count = get_length_list(timer_list);
    link_add_list(timer_list,timer_count,&t_timer->list);

    return t_timer;
}



st_timer_t del_sf_timer(st_timer_t t_timer)
{
    pNode t_list = timer_list;
    uint16_t timer_num = 0;

    while(t_list)
    {
        timer_num++;
        if(t_list == &t_timer->list)
        {
            link_rm_node(t_list);
            timer_count--;
            if(timer_num == 1) timer_list = timer_list->next;
            return list_entry(t_list,st_timer,list);
        }
        t_list = t_list->next;
    }
    return 0;
}

// interrupt
void SysTick_Handler(void)
{
    jiff++;
}




//+++++++++++++++++++++++test++++++++++++++++++++
#ifdef TIMER_TEST
#include <stdio.h>
st_timer_t tt1,tt2,tt3;
uint16_t c1,c2,c3,c4,c5;

void timer_fun1(void *parameter)
{
    c1++;
    //    log_printf("timer1\r\n");
}

void timer_fun2(void *parameter)
{
    c2++;
    //    log_printf("timer2\r\n");
}

void timer_fun3(void *parameter)
{
    c3++;
    //    log_printf("timer3\r\n");
}

void test_timer(void)
{
static    st_timer timer1 ;
static    st_timer timer2 ;
static    st_timer timer3 ;

    tt1 = &timer1;
    tt1->func = timer_fun1;
    tt1->timeout_tick = 1000;
    cre_sf_timer(&timer1,0);

    tt2 = &timer2;
    tt2->func = timer_fun2;
    tt2->timeout_tick = 200;
    cre_sf_timer(&timer2,0);
 
    tt3 = &timer3;
    tt3->func = timer_fun3;
    tt3->timeout_tick = 2000;
    cre_sf_timer(&timer3,0);
}
#endif
//++++++++++++++++++++test end +++++++++++++++++++





