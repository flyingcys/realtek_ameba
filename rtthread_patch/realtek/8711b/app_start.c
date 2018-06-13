/*
 *  Routines to access hardware
 *
 *  Copyright (c) 2013 Realtek Semiconductor Corp.
 *
 *  This module is a confidential and proprietary property of RealTek and
 *  possession or use of this module requires written permission of RealTek.
 */

#include "ameba_soc.h"
#include "build_info.h"

#include <rtthread.h>
#include <wlan_dev.h>

#ifndef RT_MAIN_THREAD_STACK_SIZE
#define RT_MAIN_THREAD_STACK_SIZE     2048
#endif

#ifndef RT_USING_HEAP
/* if there is not enable heap, we should use static thread and stack. */
ALIGN(8)
static rt_uint8_t main_stack[RT_MAIN_THREAD_STACK_SIZE];
struct rt_thread main_thread;
#endif

extern int amebaz_wifi_init(rt_wlan_mode_t mode);

/* the system main thread */
void main_thread_entry(void *parameter)
{
    extern int main(void);
    extern int $Super$$main(void);

    /* RT-Thread components initialization */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_init();
#endif

    if(amebaz_wifi_init(WIFI_STATION) != RT_EOK)
        rt_kprintf("amebaz_wifi_start failed...\n");

    /* invoke system main function */
#if defined (__CC_ARM)
    $Super$$main(); /* for ARMCC. */
#elif defined(__ICCARM__) || defined(__GNUC__)
    main();
#endif
}

void rt_application_init(void)
{
    rt_thread_t tid;

#ifdef RT_USING_HEAP
    tid = rt_thread_create("main", main_thread_entry, RT_NULL,
                           RT_MAIN_THREAD_STACK_SIZE, RT_THREAD_PRIORITY_MAX / 3, 20);
    RT_ASSERT(tid != RT_NULL);
#else
    rt_err_t result;

    tid = &main_thread;
    result = rt_thread_init(tid, "main", main_thread_entry, RT_NULL,
                            main_stack, sizeof(main_stack), RT_THREAD_PRIORITY_MAX / 3, 20);
    RT_ASSERT(result == RT_EOK);
	
    /* if not define RT_USING_HEAP, using to eliminate the warning */
    (void)result;
#endif

    rt_thread_startup(tid);
}

int rtthread_startup(void)
{
    rt_hw_interrupt_disable();

    /* board level initalization
     * NOTE: please initialize heap inside board initialization.
     */
    rt_hw_board_init();

    /* show RT-Thread version */
    rt_show_version();

    /* timer system initialization */
    rt_system_timer_init();

    /* scheduler system initialization */
    rt_system_scheduler_init();

#ifdef RT_USING_SIGNALS
    /* signal system initialization */
    rt_system_signal_init();
#endif

    /* create init_thread */
    rt_application_init();

    /* timer thread initialization */
    rt_system_timer_thread_init();

    /* idle thread initialization */
    rt_thread_idle_init();

    /* start scheduler */
    rt_system_scheduler_start();

    /* never reach here */
    return 0;
}


void APP_Start(void)
{
#if CONFIG_SOC_PS_MODULE
	SOCPS_InitSYSIRQ();
#endif

    extern void PendSV_Handler(void);
    extern void SysTick_Handler(void);
	InterruptForOSInit((VOID*)NULL,
		(VOID*)PendSV_Handler,
		(VOID*)SysTick_Handler);

#if defined ( __ICCARM__ )
	__iar_cstart_call_ctors(NULL);
#endif
	// force SP align to 8 byte not 4 byte (initial SP is 4 byte align)
	__asm( 
		"mov r0, sp\n"
		"bic r0, r0, #7\n" 
		"mov sp, r0\n"
	);

    /* disable interrupt first */
    rt_hw_interrupt_disable();

    /* startup RT-Thread RTOS */
    rtthread_startup();
}
