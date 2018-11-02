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
#include "rtthread.h"

extern int rtthread_startup(void);

RT_WEAK void SVC_Handler(void)
{
}

void APP_Start(void)
{
#if CONFIG_SOC_PS_MODULE
	SOCPS_InitSYSIRQ();
#endif

    extern void PendSV_Handler(void);
    extern void SysTick_Handler(void);
	InterruptForOSInit((VOID*)SVC_Handler,
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

#ifdef RT_USING_USER_MAIN
    /* startup RT-Thread RTOS */
    rtthread_startup();
#else
	extern int main(void);
    extern int $Super$$main(void);
    /* invoke system main function */
#if defined (__CC_ARM)
    $Super$$main(); /* for ARMCC. */
#elif defined(__ICCARM__) || defined(__GNUC__)
    main();
#endif
#endif
}
