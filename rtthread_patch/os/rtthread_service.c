/* rtthread includes */
#include <rtthread.h>
#include <osdep_service.h>
#include <stdio.h>
#include <freertos_pmu.h>
//#include <tcm_heap.h>
/********************* os depended utilities ********************/

#ifndef USE_MUTEX_FOR_SPINLOCK
#define USE_MUTEX_FOR_SPINLOCK 1
#endif

rt_uint16_t ameba_sem = 0;
rt_uint16_t ameba_mutex = 0;
rt_uint16_t ameba_spin = 0;

//----- ------------------------------------------------------------------
// Misc Function
//----- ------------------------------------------------------------------

void save_and_cli(void)
{
    rt_enter_critical();
}

void restore_flags(void)
{
    rt_exit_critical();
}

void cli(void)
{
    rt_hw_interrupt_disable();
}

/* Not needed on 64bit architectures */
static unsigned int __div64_32(u64 *n, unsigned int base)
{
	u64 rem = *n;
	u64 b = base;
	u64 res, d = 1;
	unsigned int high = rem >> 32;

	/* Reduce the thing a bit first */
	res = 0;
	if (high >= base) {
		high /= base;
		res = (u64) high << 32;
		rem -= (u64) (high * base) << 32;
	}

	while ((u64)b > 0 && b < rem) {
		b = b+b;
		d = d+d;
	}

	do {
		if (rem >= b) {
			rem -= b;
			res += d;
		}
		b >>= 1;
		d >>= 1;
	} while (d);

	*n = res;
	return rem;
}

/********************* os depended service ********************/

u8* _rtthread_malloc(u32 sz)
{
    u8 *pbuf = (u8 *)rt_malloc(sz);

    return pbuf;
}

u8* _rtthread_zmalloc(u32 sz)
{
	u8 *pbuf = rt_malloc(sz);

	if (pbuf != NULL)
		memset(pbuf, 0, sz);

	return pbuf;	
}

void _rtthread_mfree(u8 *pbuf, u32 sz)
{
    rt_free(pbuf);
}

static void _rtthread_memcpy(void* dst, void* src, u32 sz)
{
	memcpy(dst, src, sz);
}

static int _rtthread_memcmp(void *dst, void *src, u32 sz)
{
//under Linux/GNU/GLibc, the return value of memcmp for two same mem. chunk is 0
	if (!(memcmp(dst, src, sz)))
		return 1;

	return 0;
}

static void _rtthread_memset(void *pbuf, int c, u32 sz)
{
	memset(pbuf, c, sz);
}

static void _rtthread_init_sema(_sema *sema, int init_val)
{
    char name[RT_NAME_MAX];

    memset(name, 0, RT_NAME_MAX);
    snprintf(name, RT_NAME_MAX, "sem-%04d", ameba_sem ++);
    
    *sema = rt_sem_create(name, init_val, RT_IPC_FLAG_FIFO);
}

static void _rtthread_free_sema(_sema *sema)
{
	if(*sema != NULL)
        rt_sem_delete(*sema);

	*sema = NULL;
}

static void _rtthread_up_sema(_sema *sema)
{
    rt_sem_release(*sema);
}

static void _rtthread_up_sema_from_isr(_sema *sema)
{
    rt_sem_release(*sema);
}

static u32 _rtthread_down_sema(_sema *sema, u32 timeout)
{
    rt_int32_t tick;
    
	if(timeout >= RT_TICK_MAX / 2) 
    {
		tick = RT_WAITING_FOREVER;
	} 
    else 
    {
		tick = rtw_ms_to_systime(timeout);
	}
    
    if(rt_sem_take(*sema, tick) != RT_EOK)
        return RT_FALSE;

    return RT_TRUE;
}

static void _rtthread_mutex_init(_mutex *pmutex)
{
    char name[RT_NAME_MAX];

    memset(name, 0, RT_NAME_MAX);

    snprintf(name, RT_NAME_MAX, "rmux-%03d", ameba_mutex ++);

    *pmutex = rt_mutex_create(name, RT_IPC_FLAG_FIFO);
}

static void _rtthread_mutex_free(_mutex *pmutex)
{
	if(*pmutex != NULL)
        rt_mutex_delete(*pmutex);   

	*pmutex = NULL;
}

static void _rtthread_mutex_get(_lock *plock)
{
    rt_mutex_take(*plock, RT_WAITING_FOREVER);
}

static int _rtthread_mutex_get_timeout(_lock *plock, u32 timeout_ms)
{
    rt_int32_t tick;
    
	if(timeout_ms >= RT_TICK_MAX / 2) 
    {
		tick = RT_WAITING_FOREVER;
	} 
    else
    {
		tick = rtw_ms_to_systime(timeout_ms);
	}

    return rt_mutex_take(*plock, tick);
}

static void _rtthread_mutex_put(_lock *plock)
{
    rt_mutex_release(*plock);
}

static void _rtthread_enter_critical(_lock *plock, _irqL *pirqL)
{
    rt_enter_critical();
}

static void _rtthread_exit_critical(_lock *plock, _irqL *pirqL)
{
    rt_exit_critical();
}

static u32 uxSavedInterruptStatus = 0;
static void _rtthread_enter_critical_from_isr(_lock *plock, _irqL *pirqL)
{
    rt_enter_critical();
}

static void _rtthread_exit_critical_from_isr(_lock *plock, _irqL *pirqL)
{
    rt_exit_critical();
}

static int _rtthread_enter_critical_mutex(_mutex *pmutex, _irqL *pirqL)
{
	int result = rt_mutex_take(*pmutex, RT_WAITING_FOREVER);

    if(result != RT_EOK)
        return RT_FALSE;

    return RT_TRUE;
}

static void _rtthread_exit_critical_mutex(_mutex *pmutex, _irqL *pirqL)
{
    rt_mutex_release(*pmutex);
}

static void _rtthread_spinlock_init(_lock *plock)
{
#if USE_MUTEX_FOR_SPINLOCK
    char name[RT_NAME_MAX];

    memset(name, 0, RT_NAME_MAX);

    snprintf(name, RT_NAME_MAX, "spin-03d", ameba_spin ++);
    
    *plock = rt_sem_create(name, 1, RT_IPC_FLAG_FIFO);
#endif
}

static void _rtthread_spinlock_free(_lock *plock)
{
#if USE_MUTEX_FOR_SPINLOCK
	if(*plock != NULL)
        rt_sem_delete(*plock);

	*plock = NULL;
#endif
}

static void _rtthread_spinlock(_lock *plock)
{
#if USE_MUTEX_FOR_SPINLOCK
    rt_sem_take(*plock, RT_WAITING_FOREVER);
#endif
}

static void _rtthread_spinunlock(_lock *plock)
{
#if USE_MUTEX_FOR_SPINLOCK
    rt_sem_release(*plock);
#endif
}

static void _rtthread_spinlock_irqsave(_lock *plock, _irqL *irqL)
{
#if USE_MUTEX_FOR_SPINLOCK
    while(rt_sem_take(*plock, 0) != RT_EOK)
    {
        rt_kprintf("spinlock_irqsave failed!\n");
    }
#endif
}

static void _rtthread_spinunlock_irqsave(_lock *plock, _irqL *irqL)
{
#if USE_MUTEX_FOR_SPINLOCK
    rt_sem_release(*plock);
#endif
}

static int _rtthread_init_xqueue( _xqueue* queue, const char* name, u32 message_size, u32 number_of_messages )
{
    return -1;
}

static int _rtthread_push_to_xqueue( _xqueue* queue, void* message, u32 timeout_ms )
{
    return -1;
}

static int _rtthread_pop_from_xqueue( _xqueue* queue, void* message, u32 timeout_ms )
{
    return -1;
}

static int _rtthread_deinit_xqueue( _xqueue* queue )
{
    return -1;
}

static u32 _rtthread_get_current_time(void)
{
    return rt_tick_get();
}

static u32 _rtthread_systime_to_ms(u32 systime)
{
    return systime * 1000 / RT_TICK_PER_SECOND; 
}

static u32 _rtthread_systime_to_sec(u32 systime)
{
    return systime / RT_TICK_PER_SECOND;
}

static u32 _rtthread_ms_to_systime(u32 ms)
{
    return rt_tick_from_millisecond(ms);
}

static u32 _rtthread_sec_to_systime(u32 sec)
{
    return sec * RT_TICK_PER_SECOND;
}

static void _rtthread_msleep_os(int ms)
{
#if defined(CONFIG_PLATFORM_8195A)
	rt_thread_delay(rt_tick_from_millisecond(ms));
#elif defined(CONFIG_PLATFORM_8711B)
	if (pmu_yield_os_check()) {
        rt_thread_delay(rt_tick_from_millisecond(ms));
	} else {
		DelayMs(ms);
	}
#endif
}

static void _rtthread_usleep_os(int us)
{
#if defined(STM32F2XX) || defined(STM32F4XX) || defined(STM32F10X_XL)
	// rtthread does not provide us level delay. Use busy wait
	WLAN_BSP_UsLoop(us);
#elif defined(CONFIG_PLATFORM_8195A)
	//DBG_ERR("%s: Please Implement micro-second delay\n", __FUNCTION__);
#elif defined(CONFIG_PLATFORM_8711B)
	DelayUs(us);
#else
	#error "Please implement hardware dependent micro second level sleep here"
#endif
}

static void _rtthread_mdelay_os(int ms)
{
    rt_thread_delay(rt_tick_from_millisecond(ms));
}

static void _rtthread_udelay_os(int us)
{
#if defined(STM32F2XX)	|| defined(STM32F4XX) || defined(STM32F10X_XL)
	// rtthread does not provide us level delay. Use busy wait
	WLAN_BSP_UsLoop(us);
#elif defined(CONFIG_PLATFORM_8195A)
	HalDelayUs(us);
#elif defined(CONFIG_PLATFORM_8711B)
	DelayUs(us);
#else
	#error "Please implement hardware dependent micro second level sleep here"
#endif
}

static void _rtthread_yield_os(void)
{
#if defined(CONFIG_PLATFORM_8195A)
    rt_thread_yield();
#elif defined(CONFIG_PLATFORM_8711B)
	if (pmu_yield_os_check()) {
        rt_thread_yield();
	} else {
		DelayMs(1);
	}
#endif
}

static void _rtthread_ATOMIC_SET(ATOMIC_T *v, int i)
{
	atomic_set(v,i);
}

static int _rtthread_ATOMIC_READ(ATOMIC_T *v)
{
	return atomic_read(v);
}

static void _rtthread_ATOMIC_ADD(ATOMIC_T *v, int i)
{
	save_and_cli();
	v->counter += i;
	restore_flags();
}

static void _rtthread_ATOMIC_SUB(ATOMIC_T *v, int i)
{
	save_and_cli();
	v->counter -= i;
	restore_flags();
}

static void _rtthread_ATOMIC_INC(ATOMIC_T *v)
{
	_rtthread_ATOMIC_ADD(v, 1);
}

static void _rtthread_ATOMIC_DEC(ATOMIC_T *v)
{
	_rtthread_ATOMIC_SUB(v, 1);
}

static int _rtthread_ATOMIC_ADD_RETURN(ATOMIC_T *v, int i)
{
	int temp;

	save_and_cli();
	temp = v->counter;
	temp += i;
	v->counter = temp;
	restore_flags();

	return temp;
}

static int _rtthread_ATOMIC_SUB_RETURN(ATOMIC_T *v, int i)
{
	int temp;

	save_and_cli();
	temp = v->counter;
	temp -= i;
	v->counter = temp;
	restore_flags();

	return temp;
}

static int _rtthread_ATOMIC_INC_RETURN(ATOMIC_T *v)
{
	return _rtthread_ATOMIC_ADD_RETURN(v, 1);
}

static int _rtthread_ATOMIC_DEC_RETURN(ATOMIC_T *v)
{
	return _rtthread_ATOMIC_SUB_RETURN(v, 1);
}

static u64 _rtthread_modular64(u64 n, u64 base)
{
	unsigned int __base = (base);
	unsigned int __rem;

	if (((n) >> 32) == 0) {
		__rem = (unsigned int)(n) % __base;
		(n) = (unsigned int)(n) / __base;
	}
	else
		__rem = __div64_32(&(n), __base);
	
	return __rem;
}

/* Refer to ecos bsd tcpip codes */
static int _rtthread_arc4random(void)
{
	u32 res = _rtthread_get_current_time();
	static unsigned long seed = 0xDEADB00B;

#if CONFIG_PLATFORM_8711B
	if(random_seed){
		seed = random_seed;
		random_seed = 0;
	}
#endif
	
	seed = ((seed & 0x007F00FF) << 7) ^
	    ((seed & 0x0F80FF00) >> 8) ^ // be sure to stir those low bits
	    (res << 13) ^ (res >> 9);    // using the clock too!
	return (int)seed;
}

static int _rtthread_get_random_bytes(void *buf, size_t len)
{
#if 1 //becuase of 4-byte align, we use the follow code style.
	unsigned int ranbuf;
	unsigned int *lp;
	int i, count;
	count = len / sizeof(unsigned int);
	lp = (unsigned int *) buf;

	for(i = 0; i < count; i ++) {
		lp[i] = _rtthread_arc4random();  
		len -= sizeof(unsigned int);
	}

	if(len > 0) {
		ranbuf = _rtthread_arc4random();
		_rtthread_memcpy(&lp[i], &ranbuf, len);
	}
	return 0;
#else
	unsigned long ranbuf, *lp;
	lp = (unsigned long *)buf;
	while (len > 0) {
		ranbuf = _rtthread_arc4random();
		*lp++ = ranbuf; //this op need the pointer is 4Byte-align!
		len -= sizeof(ranbuf);
	}
	return 0;
#endif
}

static u32 _rtthread_GetFreeHeapSize(void)
{
    return 16 * 1024;
}

#ifndef RT_USING_LWIP
#define     RT_LWIP_TCPTHREAD_PRIORITY       10
#endif
void *tcm_heap_malloc(int size);
static int _rtthread_create_task(struct task_struct *ptask, const char *name,
	u32  stack_size, u32 priority, thread_func_t func, void *thctx)
{
      rt_err_t result;
    rt_thread_t tid;
    char * stack;

    DBG_INFO("create_task %s: ptask 0x%08X, stack %d priority %d \n", name, ptask, stack_size, priority);

    ptask->task_name = name;
    ptask->blocked = 0;
    ptask->callback_running = 0;

    //OSDEP_DBG("&ptask->wakeup_sema: 0x%08X \n", &ptask->wakeup_sema);
    _rtthread_init_sema(&ptask->wakeup_sema, 0);

    //OSDEP_DBG("&ptask->terminate_sema: 0x%08X \n", &ptask->terminate_sema);
    _rtthread_init_sema(&ptask->terminate_sema, 0);

    if(strcmp(name, "rtw_recv_tasklet") == 0)
    {
        stack_size = 3000;
    }
    else if(strcmp(name, "cmd_thread") == 0)
    {
        stack_size = 1800;
    }
    else
    {
        stack_size = stack_size + 512;
    }

    tid = rt_malloc(RT_ALIGN(sizeof(struct rt_thread), 8) + stack_size);
    if(!tid)
    {
        rt_kprintf("no memory NULL for thread:%s\n", name);
        return 0;
    }

    stack = (char *)tid + RT_ALIGN(sizeof(struct rt_thread), 8);

    priority = RT_LWIP_TCPTHREAD_PRIORITY + priority;
    if(priority >= RT_THREAD_PRIORITY_MAX)
    {
        priority = RT_THREAD_PRIORITY_MAX - 1;
    }

    result = rt_thread_init(tid,
                           name,
                           func, thctx,         // fun, parameter
                           stack, stack_size,   // stack, size
                           priority,            //priority
                           10
                          );

    if(result == RT_EOK)
    {
        ptask->task = tid;
        rt_thread_startup(tid);
    }

    return RT_TRUE;
}
	
static void _rtthread_delete_task(struct task_struct *ptask)
{
	if (!ptask->task){
		DBG_ERR("_rtthread_delete_task(): ptask is NULL!\n");
		return;
	}

    rt_kprintf("[func]:%s [line]:%d\n", __FUNCTION__, __LINE__);
	ptask->blocked = 1;

	_rtthread_up_sema(&ptask->wakeup_sema);
	_rtthread_down_sema(&ptask->terminate_sema, TIMER_MAX_DELAY);
	
	//rtw_deinit_queue(&wq->work_queue);
	_rtthread_free_sema(&ptask->wakeup_sema);
	_rtthread_free_sema(&ptask->terminate_sema);

	ptask->task = 0;
	
	DBG_TRACE("Delete Task \"%s\"\n", ptask->task_name);
}

void _rtthread_wakeup_task(struct task_struct *ptask)
{
	_rtthread_up_sema(&ptask->wakeup_sema);
}

static void _rtthread_thread_enter(char *name)
{
//	rt_kprintf("RTKTHREAD %s\n", name);
}

static void _rtthread_thread_exit(void)
{
//	rt_kprintf("RTKTHREAD exit %s\n", __FUNCTION__); 
}

_timerHandle _rtthread_timerCreate( const signed char *pcTimerName, 
							  osdepTickType xTimerPeriodInTicks, 
							  u32 uxAutoReload, 
							  void * pvTimerID, 
							  TIMER_FUN pxCallbackFunction )
{
    rt_timer_t timer;
    rt_tick_t time;
    rt_uint8_t flag;

    if(uxAutoReload == 1)
    {
        flag |= RT_TIMER_FLAG_SOFT_TIMER | RT_TIMER_FLAG_PERIODIC;
    }
    else
    {
        flag |= RT_TIMER_FLAG_SOFT_TIMER | RT_TIMER_FLAG_ONE_SHOT;
    }

    if( xTimerPeriodInTicks >= (RT_TICK_MAX / 2) )
    {
        time = (RT_TICK_MAX / 2) - 1;
    }
    else
    {
        time = xTimerPeriodInTicks;
    }

    timer = rt_timer_create(pcTimerName,
                            pxCallbackFunction,
                            RT_NULL,
                            time,
                            flag);

    //OSDEP_DBG("%s: %s 0x%08X tick:%d uxAutoReload:%d, Callback:0x%08X, pvTimerID:0x%08X, \n", __FUNCTION__, pcTimerName, timer, xTimerPeriodInTicks, uxAutoReload, pxCallbackFunction, pvTimerID);

    if(timer)
    {
        timer->parameter = timer;
    }

    return (_timerHandle)timer;
}

u32 _rtthread_timerDelete( _timerHandle xTimer, 
							   osdepTickType xBlockTime )
{
    if(rt_timer_delete(xTimer) != RT_EOK)
        return RT_FALSE;

    return RT_TRUE;
}

u32 _rtthread_timerIsTimerActive( _timerHandle xTimer )
{
    rt_timer_t pxTimer = (rt_timer_t)xTimer;

    if(!xTimer)
    {
        return 0;
    }
    
    return (pxTimer->parent.flag & RT_TIMER_FLAG_ACTIVATED) ? 1 : 0;
}

u32  _rtthread_timerStop( _timerHandle xTimer, 
							   osdepTickType xBlockTime )
{
    rt_err_t result = rt_timer_stop(xTimer);
    if(result != RT_EOK)
        return RT_FALSE;

    return RT_TRUE;
}

u32  _rtthread_timerChangePeriod( _timerHandle xTimer, 
							   osdepTickType xNewPeriod, 
							   osdepTickType xBlockTime )
{
    rt_err_t result;
    int time;
    
	if(xNewPeriod == 0)
		time = 1;
    else if(xNewPeriod >= (RT_TICK_MAX / 2))
        time = (RT_TICK_MAX / 2) - 1;
    else
        time = xNewPeriod;
    
    result = rt_timer_stop(xTimer);
    result = rt_timer_control(xTimer, RT_TIMER_CTRL_SET_TIME, (void *)&time);

    if(rt_timer_start(xTimer) != RT_EOK)
        return RT_FALSE;

    return RT_TRUE;
}
void *_rtthread_timerGetID( _timerHandle xTimer )
{
    return xTimer;
}

u32  _rtthread_timerStart( _timerHandle xTimer, 
							   osdepTickType xBlockTime )
{
    if(rt_timer_start(xTimer) != RT_EOK)
        return RT_FALSE;

    return RT_TRUE;
}

u32  _rtthread_timerStartFromISR( _timerHandle xTimer, 
							   osdepBASE_TYPE *pxHigherPriorityTaskWoken )
{
    if(rt_timer_start(xTimer) != RT_EOK)
        return RT_FALSE;

    return RT_TRUE;  
}

u32  _rtthread_timerStopFromISR( _timerHandle xTimer, 
							   osdepBASE_TYPE *pxHigherPriorityTaskWoken )
{    
    if(rt_timer_stop(xTimer) != RT_EOK)
        return RT_FALSE;

    return RT_TRUE;
}

u32  _rtthread_timerResetFromISR( _timerHandle xTimer, 
							   osdepBASE_TYPE *pxHigherPriorityTaskWoken )
{
    if(rt_timer_stop(xTimer) != RT_EOK)
        return RT_FALSE;
    
    if(rt_timer_start(xTimer) != RT_EOK)
        return RT_FALSE;
    
    return RT_TRUE;
}

u32  _rtthread_timerChangePeriodFromISR( _timerHandle xTimer, 
							   osdepTickType xNewPeriod, 
							   osdepBASE_TYPE *pxHigherPriorityTaskWoken )
{
    rt_err_t result;
    int time;
    
	if(xNewPeriod == 0)
		time = 1;
    else if(xNewPeriod >= (RT_TICK_MAX / 2))
        time = (RT_TICK_MAX / 2) - 1;
    else
        time = xNewPeriod;
    
    result = rt_timer_stop(xTimer);
    result = rt_timer_control(xTimer, RT_TIMER_CTRL_SET_TIME, (void *)&time);

    if(rt_timer_start(xTimer) != RT_EOK)
        return RT_FALSE;

    return RT_TRUE;
}

u32  _rtthread_timerReset( _timerHandle xTimer, 
							   osdepTickType xBlockTime )
{
    rt_err_t result = rt_timer_stop(xTimer);

    if(result != RT_EOK)
        return RT_FALSE;

    result = rt_timer_start(xTimer);
    if(result != RT_EOK)
        return RT_FALSE;

    return RT_TRUE;
}

void _rtthread_acquire_wakelock(void)
{
#if defined(CONFIG_PLATFORM_8195A)

#if defined(configUSE_WAKELOCK_PMU) && (configUSE_WAKELOCK_PMU == 1)
   	pmu_acquire_wakelock(PMU_WLAN_DEVICE);
#endif
	
#elif defined(CONFIG_PLATFORM_8711B)

#if defined(configUSE_WAKELOCK_PMU) && (configUSE_WAKELOCK_PMU == 1)
	if (pmu_yield_os_check()) 
   		 pmu_acquire_wakelock(PMU_WLAN_DEVICE);
#endif

#endif
}

void _rtthread_release_wakelock(void)
{
#if defined(CONFIG_PLATFORM_8195A)

#if defined(configUSE_WAKELOCK_PMU) && (configUSE_WAKELOCK_PMU == 1)
    pmu_release_wakelock(PMU_WLAN_DEVICE);
#endif
	
#elif defined(CONFIG_PLATFORM_8711B)

#if defined(configUSE_WAKELOCK_PMU) && (configUSE_WAKELOCK_PMU == 1)
	if (pmu_yield_os_check()) 
		pmu_release_wakelock(PMU_WLAN_DEVICE);
#endif

#endif
}

void _rtthread_wakelock_timeout(uint32_t timeout)
{
#if defined(CONFIG_PLATFORM_8195A)
	
#elif defined(CONFIG_PLATFORM_8711B)
	if (pmu_yield_os_check()) 
		pmu_set_sysactive_time(PMU_WLAN_DEVICE, timeout);
	else
		printf("can't aquire wake during suspend flow!!\n");
#endif
}

u8 _rtthread_get_scheduler_state(void)
{
    rt_thread_t thread = rt_thread_self();
	u8 state = thread->stat;
	switch(state){
		case RT_THREAD_INIT:	    state = OS_SCHEDULER_NOT_STARTED;	break;
		case RT_THREAD_RUNNING:		state = OS_SCHEDULER_RUNNING;		break;
		case RT_THREAD_SUSPEND:	    state = OS_SCHEDULER_SUSPENDED;		break;
	}
    rt_kprintf("[func]:%s [line]:%d state:%d\n", __FUNCTION__, __LINE__, state);
	return state;
}


const struct osdep_service_ops osdep_service = {
	_rtthread_malloc,			//rtw_vmalloc
	_rtthread_zmalloc,			//rtw_zvmalloc
	_rtthread_mfree,			//rtw_vmfree
	_rtthread_malloc,			//rtw_malloc
	_rtthread_zmalloc,			//rtw_zmalloc
	_rtthread_mfree,			//rtw_mfree
	_rtthread_memcpy,			//rtw_memcpy
	_rtthread_memcmp,			//rtw_memcmp
	_rtthread_memset,			//rtw_memset
	_rtthread_init_sema,		//rtw_init_sema
	_rtthread_free_sema,		//rtw_free_sema
	_rtthread_up_sema,			//rtw_up_sema
	_rtthread_up_sema_from_isr,	//rtw_up_sema_from_isr
	_rtthread_down_sema,		//rtw_down_sema
	_rtthread_mutex_init,		//rtw_mutex_init
	_rtthread_mutex_free,		//rtw_mutex_free
	_rtthread_mutex_get,		//rtw_mutex_get
	_rtthread_mutex_get_timeout,//rtw_mutex_get_timeout
	_rtthread_mutex_put,		//rtw_mutex_put
	_rtthread_enter_critical,	//rtw_enter_critical
	_rtthread_exit_critical,	//rtw_exit_critical
	_rtthread_enter_critical_from_isr,	//rtw_enter_critical_from_isr
	_rtthread_exit_critical_from_isr,	//rtw_exit_critical_from_isr
	NULL,		//rtw_enter_critical_bh
	NULL,		//rtw_exit_critical_bh
	_rtthread_enter_critical_mutex,	//rtw_enter_critical_mutex
	_rtthread_exit_critical_mutex,	//rtw_exit_critical_mutex
	_rtthread_spinlock_init,		//rtw_spinlock_init
	_rtthread_spinlock_free,		//rtw_spinlock_free
	_rtthread_spinlock,				//rtw_spin_lock
	_rtthread_spinunlock,			//rtw_spin_unlock
	_rtthread_spinlock_irqsave,		//rtw_spinlock_irqsave
	_rtthread_spinunlock_irqsave,	//rtw_spinunlock_irqsave
	_rtthread_init_xqueue,			//rtw_init_xqueue
	_rtthread_push_to_xqueue,		//rtw_push_to_xqueue
	_rtthread_pop_from_xqueue,		//rtw_pop_from_xqueue
	_rtthread_deinit_xqueue,		//rtw_deinit_xqueue
	_rtthread_get_current_time,		//rtw_get_current_time
	_rtthread_systime_to_ms,		//rtw_systime_to_ms
	_rtthread_systime_to_sec,		//rtw_systime_to_sec
	_rtthread_ms_to_systime,		//rtw_ms_to_systime
	_rtthread_sec_to_systime,		//rtw_sec_to_systime
	_rtthread_msleep_os,	//rtw_msleep_os
	_rtthread_usleep_os,	//rtw_usleep_os
	_rtthread_mdelay_os,	//rtw_mdelay_os
	_rtthread_udelay_os,	//rtw_udelay_os
	_rtthread_yield_os,		//rtw_yield_os
	
	_rtthread_ATOMIC_SET,	//ATOMIC_SET
	_rtthread_ATOMIC_READ,	//ATOMIC_READ
	_rtthread_ATOMIC_ADD,	//ATOMIC_ADD
	_rtthread_ATOMIC_SUB,	//ATOMIC_SUB
	_rtthread_ATOMIC_INC,	//ATOMIC_INC
	_rtthread_ATOMIC_DEC,	//ATOMIC_DEC
	_rtthread_ATOMIC_ADD_RETURN,	//ATOMIC_ADD_RETURN
	_rtthread_ATOMIC_SUB_RETURN,	//ATOMIC_SUB_RETURN
	_rtthread_ATOMIC_INC_RETURN,	//ATOMIC_INC_RETURN
	_rtthread_ATOMIC_DEC_RETURN,	//ATOMIC_DEC_RETURN

	_rtthread_modular64,			//rtw_modular64
	_rtthread_get_random_bytes,		//rtw_get_random_bytes
	_rtthread_GetFreeHeapSize,		//rtw_getFreeHeapSize

	_rtthread_create_task,			//rtw_create_task
	_rtthread_delete_task,			//rtw_delete_task
	_rtthread_wakeup_task,			//rtw_wakeup_task

	_rtthread_thread_enter,			//rtw_thread_enter
	_rtthread_thread_exit,			//rtw_thread_exit

	_rtthread_timerCreate,			//rtw_timerCreate,       
	_rtthread_timerDelete,			//rtw_timerDelete,       
	_rtthread_timerIsTimerActive,	//rtw_timerIsTimerActive,
	_rtthread_timerStop,			//rtw_timerStop,         
	_rtthread_timerChangePeriod,	//rtw_timerChangePeriod 
	_rtthread_timerGetID,			//rtw_timerGetID
	_rtthread_timerStart,			//rtw_timerStart
	_rtthread_timerStartFromISR,	//rtw_timerStartFromISR
	_rtthread_timerStopFromISR,		//rtw_timerStopFromISR
	_rtthread_timerResetFromISR,	//rtw_timerResetFromISR
	_rtthread_timerChangePeriodFromISR,	//rtw_timerChangePeriodFromISR
	_rtthread_timerReset,			//rtw_timerReset

	_rtthread_acquire_wakelock,		//rtw_acquire_wakelock
	_rtthread_release_wakelock,		//rtw_release_wakelock
	_rtthread_wakelock_timeout,		//rtw_wakelock_timeout
	_rtthread_get_scheduler_state	//rtw_get_scheduler_state
};

void vTaskDelay( const rt_uint32_t xTicksToDelay )
{
	rt_thread_delay(xTicksToDelay);
}

void *pvPortMalloc( size_t xWantedSize )
{
	return rt_malloc(xWantedSize);
}

void vPortFree( void *pv )
{
    rt_free(pv);
}

uint32_t xTaskGetTickCount( void )
{
    return rt_tick_get();
}


