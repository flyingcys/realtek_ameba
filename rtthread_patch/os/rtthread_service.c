/* rtthread includes */

#include <osdep_service.h>
#include <rtthread_service.h>
#ifdef SECTION
#undef SECTION
#endif
#include <rthw.h>
#include <rtthread.h>
#include <stdio.h>

// #define RTTHREAD_SERVICE_DEBUG
#define RTTHREAD_SERVICE_DEBUG_LEVEL 2
/********************* os depended utilities ********************/
#ifdef RTTHREAD_SERVICE_DEBUG
#define DEBUG_LOG(_level, fmt, args...)  if ((_level) >= RTTHREAD_SERVICE_DEBUG_LEVEL) rt_kprintf(fmt, args)
#else
#define DEBUG_LOG(level, fmt, args...)
#endif

#ifndef USE_MUTEX_FOR_SPINLOCK
#define USE_MUTEX_FOR_SPINLOCK 1
#endif

extern uint32_t pmu_yield_os_check(void);
extern _LONG_CALL_ void DelayUs(u32 us);
extern _LONG_CALL_ void DelayMs(u32 ms);
extern uint32_t pmu_set_sysactive_time(uint32_t timeout_ms);

void save_and_cli(void)
{
    DEBUG_LOG(1, "L:%d fun:%s runing...\n", __LINE__, __FUNCTION__);
    rt_enter_critical();
}

void restore_flags(void)
{
    DEBUG_LOG(1, "L:%d fun:%s runing...\n", __LINE__, __FUNCTION__);
    rt_exit_critical();
}

void cli(void)
{
    DEBUG_LOG(1, "L:%d fun:%s runing...\n", __LINE__, __FUNCTION__);
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
    void *pbuf;

    DEBUG_LOG(2, "L:%d fun:%s sz:%d\n", __LINE__, __FUNCTION__, sz);
    pbuf = rt_malloc(sz);
    return pbuf;
}

u8* _rtthread_zmalloc(u32 sz)
{
    void *pbuf = rt_malloc(sz);
    
    DEBUG_LOG(2, "L:%d fun:%s sz:%d\n", __LINE__, __FUNCTION__, sz);
    if (pbuf != RT_NULL)
        memset(pbuf, 0, sz);

    return pbuf;    
}

void _rtthread_mfree(u8 *pbuf, u32 sz)
{
    DEBUG_LOG(2, "L:%d fun:%s\n", __LINE__, __FUNCTION__);
    rt_free(pbuf);
}

static void _rtthread_memcpy(void* dst, void* src, u32 sz)
{
    DEBUG_LOG(1, "L:%d fun:%s dst:0x%08x src:0x%08x sz:%d\n", __LINE__, __FUNCTION__, dst, src, sz);
    memcpy(dst, src, sz);
}

static int _rtthread_memcmp(void *dst, void *src, u32 sz)
{
//under Linux/GNU/GLibc, the return value of memcmp for two same mem. chunk is 0
    DEBUG_LOG(1, "L:%d fun:%s dst:0x%08x src:0x%08x sz:%d\n", __LINE__, __FUNCTION__, dst, src, sz);
    if (!(memcmp(dst, src, sz)))
        return 1;

    return 0;
}

static void _rtthread_memset(void *pbuf, int c, u32 sz)
{
    DEBUG_LOG(1, "L:%d fun:%s buf:0x%08x c:%c sz:%d\n", __LINE__, __FUNCTION__, pbuf, c, sz);
    memset(pbuf, c, sz);
}

static void _rtthread_init_sema(_sema *sema, int init_val)
{
    char name[RT_NAME_MAX];
    static int ameba_sem = 0;

    DEBUG_LOG(3, "L:%d fun:%s begin val:%d\n", __LINE__, __FUNCTION__, init_val);
    memset(name, 0, RT_NAME_MAX);
    snprintf(name, RT_NAME_MAX, "sem-%03d", ameba_sem);
    *sema = rt_sem_create(name, init_val, RT_IPC_FLAG_FIFO);
    if (*sema != RT_NULL)
        ameba_sem ++;
    DEBUG_LOG(3, "L:%d fun:%s end sema:0x%08x num:%d\n", __LINE__, __FUNCTION__, *sema, ameba_sem);
}

static void _rtthread_free_sema(_sema *sema)
{
    RT_ASSERT(*sema != RT_NULL);

    DEBUG_LOG(2, "L:%d fun:%s sema:0x%08x\n", __LINE__, __FUNCTION__, *sema);
    rt_sem_delete(*sema);
    *sema = RT_NULL;
}

static void _rtthread_up_sema(_sema *sema)
{
    if (*sema == RT_NULL)
    {
        rt_kprintf("err!! up sema is NULL\n");
        return;
    }
    DEBUG_LOG(2, "L:%d fun:%s sema:0x%08x\n", __LINE__, __FUNCTION__, *sema);
    rt_sem_release(*sema);
}

static void _rtthread_up_sema_from_isr(_sema *sema)
{
    if (*sema == RT_NULL)
    {
        rt_kprintf("err!! up sema from isr is NULL\n");
        return;
    }
    DEBUG_LOG(2, "L:%d fun:%s sema:0x%08x\n", __LINE__, __FUNCTION__, *sema);
    rt_sem_release(*sema);
}

static u32 _rtthread_down_sema(_sema *sema, u32 timeout)
{
    rt_int32_t tick;

    RT_ASSERT(*sema != RT_NULL);

    DEBUG_LOG(2, "L:%d fun:%s sema:0x%08x timeout:%d\n", __LINE__, __FUNCTION__, *sema, timeout);

    if(timeout >= RT_TICK_MAX / 2) 
        tick = RT_WAITING_FOREVER;
    else 
        tick = rtw_ms_to_systime(timeout);

    if(rt_sem_take(*sema, tick) != RT_EOK)
        return RT_FALSE;

    return RT_TRUE;
}

static void _rtthread_mutex_init(_mutex *pmutex)
{
    char name[RT_NAME_MAX];
    static int ameba_mutex = 0;

    DEBUG_LOG(3, "L:%d fun:%s begin\n", __LINE__, __FUNCTION__);
    memset(name, 0, RT_NAME_MAX);
    snprintf(name, RT_NAME_MAX, "mux-%03d", ameba_mutex);
    *pmutex = rt_mutex_create(name, RT_IPC_FLAG_FIFO);
    if (*pmutex != RT_NULL)
        ameba_mutex ++;
    DEBUG_LOG(3, "L:%d fun:%s end pmutex:0x%08x\n", __LINE__, __FUNCTION__, *pmutex);
}

static void _rtthread_mutex_free(_mutex *pmutex)
{
    RT_ASSERT(*pmutex != RT_NULL);

    DEBUG_LOG(2, "L:%d fun:%s pmutex:0x%08x\n", __LINE__, __FUNCTION__, *pmutex);
    rt_mutex_delete(*pmutex);
    *pmutex = RT_NULL;
}

static void _rtthread_mutex_get(_lock *plock)
{
    RT_ASSERT(*plock != RT_NULL);

    DEBUG_LOG(2, "L:%d fun:%s pmutex:0x%08x\n", __LINE__, __FUNCTION__, *plock);
    rt_mutex_take(*plock, RT_WAITING_FOREVER);
}

static int _rtthread_mutex_get_timeout(_lock *plock, u32 timeout_ms)
{
    rt_int32_t tick;

    RT_ASSERT(*plock != RT_NULL);

    DEBUG_LOG(2, "L:%d fun:%s plock:0x%08x timeout_ms:%d\n", __LINE__, __FUNCTION__, *plock, timeout_ms);
    if(timeout_ms >= RT_TICK_MAX / 2) 
        tick = RT_WAITING_FOREVER;
    else
        tick = rtw_ms_to_systime(timeout_ms);

    return rt_mutex_take(*plock, tick);
}

static void _rtthread_mutex_put(_lock *plock)
{
    if (*plock == RT_NULL)
    {
        rt_kprintf("err!! mutex put is null\n");
        return;
    }
    DEBUG_LOG(2, "L:%d fun:%s pmutex:0x%08x\n", __LINE__, __FUNCTION__, *plock);
    rt_mutex_release(*plock);
}

static void _rtthread_enter_critical(_lock *plock, _irqL *pirqL)
{
    DEBUG_LOG(1, "L:%d fun:%s *pirqL:0x%08x\n", __LINE__, __FUNCTION__, *pirqL);
    rt_enter_critical();
}

static void _rtthread_exit_critical(_lock *plock, _irqL *pirqL)
{
    DEBUG_LOG(1, "L:%d fun:%s *pirqL:0x%08x\n", __LINE__, __FUNCTION__, *pirqL);
    rt_exit_critical();
}

// static rt_base_t level;
static void _rtthread_enter_critical_from_isr(_lock *plock, _irqL *pirqL)
{
    DEBUG_LOG(1, "L:%d fun:%s *pirqL:0x%08x\n", __LINE__, __FUNCTION__, *pirqL);
    *pirqL = rt_hw_interrupt_disable();
}

static void _rtthread_exit_critical_from_isr(_lock *plock, _irqL *pirqL)
{
    DEBUG_LOG(1, "L:%d fun:%s *pirqL:0x%08x\n", __LINE__, __FUNCTION__, *pirqL);
    rt_hw_interrupt_enable(*pirqL);
}

static int _rtthread_enter_critical_mutex(_mutex *pmutex, _irqL *pirqL)
{
    RT_ASSERT(*pmutex != RT_NULL);

    DEBUG_LOG(1, "L:%d fun:%s *pirqL:0x%08x\n", __LINE__, __FUNCTION__, *pirqL);
    if(rt_mutex_take(*pmutex, RT_WAITING_FOREVER) != RT_EOK)
        return RT_FALSE;

    return RT_TRUE;
}

static void _rtthread_exit_critical_mutex(_mutex *pmutex, _irqL *pirqL)
{
    if (*pmutex == RT_NULL)
    {
        rt_kprintf("err!! critical mutex is null\n");
        return;
    }
    DEBUG_LOG(1, "L:%d fun:%s *pirqL:0x%08x\n", __LINE__, __FUNCTION__, *pirqL);
    rt_mutex_release(*pmutex);
}

static void _rtthread_spinlock_init(_lock *plock)
{
#if USE_MUTEX_FOR_SPINLOCK
    char name[RT_NAME_MAX];
    static int ameba_spin = 0;

    DEBUG_LOG(3, "L:%d fun:%s begin\n", __LINE__, __FUNCTION__);
    memset(name, 0, RT_NAME_MAX);
    snprintf(name, RT_NAME_MAX, "spn-03d", ameba_spin);
    *plock = rt_mutex_create(name, RT_IPC_FLAG_FIFO);
    if (*plock != RT_NULL)
        ameba_spin ++;
    DEBUG_LOG(3, "L:%d fun:%s end plock:0x%08x ameba_spin:%d\n", __LINE__, __FUNCTION__, *plock, ameba_spin);
#endif
}

static void _rtthread_spinlock_free(_lock *plock)
{
#if USE_MUTEX_FOR_SPINLOCK
    // RT_ASSERT(*plock != RT_NULL);
    if (*plock == RT_NULL)
        return;

    DEBUG_LOG(2, "L:%d fun:%s plock:0x%08x\n", __LINE__, __FUNCTION__, *plock);
    rt_mutex_delete(*plock);
    *plock = NULL;
#endif
}

static void _rtthread_spinlock(_lock *plock)
{
#if USE_MUTEX_FOR_SPINLOCK
    RT_ASSERT(*plock != RT_NULL);

    DEBUG_LOG(2, "L:%d fun:%s plock:0x%08x\n", __LINE__, __FUNCTION__, *plock);
    rt_mutex_take(*plock, RT_WAITING_FOREVER);
#endif
}

static void _rtthread_spinunlock(_lock *plock)
{
#if USE_MUTEX_FOR_SPINLOCK
    if (*plock == RT_NULL)
    {
        rt_kprintf("err!! spinunlock is null\n");
        return;
    }
    DEBUG_LOG(2, "L:%d fun:%s plock:0x%08x\n", __LINE__, __FUNCTION__, *plock);
    rt_mutex_release(*plock);
#endif
}

static void _rtthread_spinlock_irqsave(_lock *plock, _irqL *irqL)
{
#if USE_MUTEX_FOR_SPINLOCK
    if (*plock == RT_NULL)
        rt_kprintf("err!! spinlock irqsave null\n");
    
    DEBUG_LOG(2, "L:%d fun:%s plock:0x%08x irqL:0x%08x\n", __LINE__, __FUNCTION__, *plock, *irqL);
    *irqL = 0xdeadbeff;
    while (rt_mutex_take(*plock, 0) != RT_EOK)
    {
        rt_kprintf("spinlock_irqsave failed!\n");
    }
#endif
}

static void _rtthread_spinunlock_irqsave(_lock *plock, _irqL *irqL)
{
#if USE_MUTEX_FOR_SPINLOCK
    if (*plock == RT_NULL)
        rt_kprintf("err!! spinunlock irqsave null\n");

    DEBUG_LOG(2, "L:%d fun:%s plock:0x%08x irqL:0x%08x\n", __LINE__, __FUNCTION__, *plock, *irqL);
    rt_mutex_release(*plock);
#endif
}

static int _rtthread_init_xqueue( _xqueue* queue, const char* name, u32 message_size, u32 number_of_messages )
{
    DEBUG_LOG(3, "L:%d fun:%s begin name:%s size:%d msgs:%d\n", __LINE__, __FUNCTION__, name, message_size, number_of_messages);

    *queue = rt_mq_create(name, message_size, number_of_messages, RT_IPC_FLAG_FIFO);
    if (*queue == RT_NULL)
    {
        rt_kprintf("err!! create xqueue fail\n");
        return -1;
    }
    DEBUG_LOG(3, "L:%d fun:%s end queue:0x%08x\n", __LINE__, __FUNCTION__, *queue);
    return 0;
}

static int _rtthread_push_to_xqueue( _xqueue* queue, void* message, u32 timeout_ms )
{
    RT_ASSERT(*queue != RT_NULL);

    DEBUG_LOG(2, "L:%d fun:%s queue:0x%08x timeout_ms:%d\n", __LINE__, __FUNCTION__, *queue, timeout_ms);
    if (rt_mq_send(*queue, message, ((rt_mq_t)*queue)->msg_size) != RT_EOK)
    {
        return -1;
    }
    return 0;
}

static int _rtthread_pop_from_xqueue( _xqueue* queue, void* message, u32 timeout_ms )
{
    rt_uint32_t tick;

    RT_ASSERT(*queue != RT_NULL);

    DEBUG_LOG(2, "L:%d fun:%s queue:0x%08x msg:0x%08x timeout_ms:%d\n", __LINE__, __FUNCTION__, *queue, message, timeout_ms);
    if(timeout_ms >= RT_TICK_MAX / 2) 
        tick = RT_WAITING_FOREVER;
    else 
        tick = rtw_ms_to_systime(timeout_ms);

    if (rt_mq_recv(*queue, message, ((rt_mq_t)*queue)->msg_size, tick) != RT_EOK)
    {
        return -1;
    }
    return 0;
}

static int _rtthread_deinit_xqueue( _xqueue* queue )
{
    RT_ASSERT(*queue != RT_NULL);

    DEBUG_LOG(2, "L:%d fun:%s queue:0x%08x\n", __LINE__, __FUNCTION__, *queue);
    rt_mq_delete(*queue);
    return 0;
}

static u32 _rtthread_get_current_time(void)
{
    return (u32)rt_tick_get();
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
    DEBUG_LOG(2, "L:%d fun:%s ms:%d\n", __LINE__, __FUNCTION__, ms);
#if defined(CONFIG_PLATFORM_8195A)
    rt_thread_delay(rt_tick_from_millisecond(ms));
#elif defined(CONFIG_PLATFORM_8711B)
    if (pmu_yield_os_check()) 
        rt_thread_delay(rt_tick_from_millisecond(ms));
    else 
        DelayMs(ms);
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
#endif
}

static void _rtthread_mdelay_os(int ms)
{
    rt_thread_delay(rt_tick_from_millisecond(ms));
}

static void _rtthread_udelay_os(int us)
{
#if defined(STM32F2XX)  || defined(STM32F4XX) || defined(STM32F10X_XL)
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
    if (pmu_yield_os_check()) 
        rt_thread_yield();
    else 
        DelayMs(1);
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

static int _rtthread_get_random_bytes(void *buf, u32 len)
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
static int _rtthread_create_task(struct task_struct *ptask, const char *name,
    u32  stack_size, u32 priority, thread_func_t func, void *thctx)
{
    rt_thread_t tid;

    DEBUG_LOG(3, "L:%d fun:%s begin name:%s stack_size:%d priority:%d func:0x%08x thctx:0x%08x\n", \
        __LINE__, __FUNCTION__, name, stack_size, priority, func, thctx);

    rt_memset(ptask, 0, sizeof(struct task_struct));
    ptask->task_name = name;
    ptask->blocked = 0;
    ptask->callback_running = 0;

    _rtthread_init_sema(&ptask->wakeup_sema, 0);
    if (ptask->wakeup_sema == RT_NULL)
    {
        rt_kprintf("L:%d create wakeup sem fail\n");
        goto _thread_err;
    }
    _rtthread_init_sema(&ptask->terminate_sema, 0);
    if (ptask->terminate_sema == RT_NULL)
    {
        rt_kprintf("L:%d terminate sem fail\n");
        goto _thread_err;
    }

    stack_size = (stack_size * 3);

    priority = RT_LWIP_TCPTHREAD_PRIORITY + priority;
    if(priority >= RT_THREAD_PRIORITY_MAX)
        priority = RT_THREAD_PRIORITY_MAX - 1;

    tid = rt_thread_create(name, func, thctx, stack_size, priority, 10);
    if (tid == RT_NULL)
    {
        rt_kprintf("L:%d thread create fail\n");
        goto _thread_err;
    }

    ptask->task = tid;
    rt_thread_startup(tid);

    DEBUG_LOG(3, "L:%d fun:%s end stack_size:%d priority:%d wakeup:0x%08x terminate:0x%08x\n", \
        __LINE__, __FUNCTION__, stack_size, priority, ptask->wakeup_sema, ptask->terminate_sema);

    return RT_TRUE;

_thread_err:
    if (ptask->wakeup_sema)
        _rtthread_free_sema(&ptask->wakeup_sema);
    if (ptask->terminate_sema)
        _rtthread_free_sema(&ptask->terminate_sema);
    rt_memset(ptask, 0, sizeof(struct task_struct));
    return RT_FALSE;
}

static void _rtthread_delete_task(struct task_struct *ptask)
{
    if (!ptask->task)
    {
        rt_kprintf("_rtthread_delete_task(): ptask is NULL!\n");
        return;
    }

    DEBUG_LOG(2, "L:%d fun:%s name:%s\n", __LINE__, __FUNCTION__, ptask->task_name);
    ptask->blocked = 1;

    _rtthread_up_sema(&ptask->wakeup_sema);
    _rtthread_down_sema(&ptask->terminate_sema, TIMER_MAX_DELAY);

    _rtthread_free_sema(&ptask->wakeup_sema);
    _rtthread_free_sema(&ptask->terminate_sema);

    ptask->task = 0;
}

void _rtthread_wakeup_task(struct task_struct *ptask)
{
    DEBUG_LOG(2, "L:%d fun:%s name:%s\n", __LINE__, __FUNCTION__, ptask->task_name);
    _rtthread_up_sema(&ptask->wakeup_sema);
}

static void _rtthread_thread_enter(char *name)
{
    DEBUG_LOG(3, "L:%d fun:%s name:%s\n", __LINE__, __FUNCTION__, name);
}

static void _rtthread_thread_exit(void)
{
    DEBUG_LOG(3, "L:%d fun:%s\n", __LINE__, __FUNCTION__);
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

    DEBUG_LOG(3, "L:%d fun:%s begin name:%s reload:%d tick:%d fun:0x%08x ID:0x%08x\n", 
        __LINE__, __FUNCTION__, pcTimerName, uxAutoReload, xTimerPeriodInTicks, pxCallbackFunction, pvTimerID);

    if(uxAutoReload == 1)
        flag = (RT_TIMER_FLAG_SOFT_TIMER | RT_TIMER_FLAG_PERIODIC);
    else
        flag = (RT_TIMER_FLAG_SOFT_TIMER | RT_TIMER_FLAG_ONE_SHOT);

    if( xTimerPeriodInTicks >= (RT_TICK_MAX / 2) )
        time = (RT_TICK_MAX / 2) - 1;
    else
        time = xTimerPeriodInTicks;

    timer = rt_timer_create(pcTimerName, pxCallbackFunction, RT_NULL, time, flag);
    if(timer == RT_NULL)
    {
        rt_kprintf("timer create fail\n");
        return RT_NULL;
    }
    timer->parameter = timer;
    DEBUG_LOG(3, "L:%d fun:%s end timer:0x%08x\n", __LINE__, __FUNCTION__, timer);

    return (_timerHandle)timer;
}

u32 _rtthread_timerDelete(_timerHandle xTimer, osdepTickType xBlockTime)
{
    DEBUG_LOG(3, "L:%d fun:%s timer:0x%08x\n", __LINE__, __FUNCTION__, xTimer);

    RT_ASSERT(xTimer != RT_NULL);

    rt_timer_delete(xTimer);
    return RT_TRUE;
}

u32 _rtthread_timerIsTimerActive( _timerHandle xTimer )
{
    rt_timer_t pxTimer = (rt_timer_t)xTimer;

    DEBUG_LOG(3, "L:%d fun:%s timer:0x%08x\n", __LINE__, __FUNCTION__, xTimer);
    if(!xTimer)
    {
        rt_kprintf("err!! timer is active null\n");
        return 0;
    }

    return (pxTimer->parent.flag & RT_TIMER_FLAG_ACTIVATED) ? 1 : 0;
}

u32  _rtthread_timerStop(_timerHandle xTimer, osdepTickType xBlockTime)
{
    RT_ASSERT(xTimer != RT_NULL);

    DEBUG_LOG(3, "L:%d fun:%s timer:0x%08x\n", __LINE__, __FUNCTION__, xTimer);
    if (rt_timer_stop(xTimer) != RT_EOK)
    {
        // rt_kprintf("timer stop fail\n");
        return RT_FALSE;
    }
    return RT_TRUE;
}

u32  _rtthread_timerChangePeriod( _timerHandle xTimer, osdepTickType xNewPeriod, osdepTickType xBlockTime )
{
    int time;

    RT_ASSERT(xTimer != RT_NULL);

    DEBUG_LOG(3, "L:%d fun:%s timer:0x%08x new_tick:%d\n", __LINE__, __FUNCTION__, xTimer, xNewPeriod);
    if(xNewPeriod == 0)
        time = 1;
    else if(xNewPeriod >= (RT_TICK_MAX / 2))
        time = (RT_TICK_MAX / 2) - 1;
    else
        time = xNewPeriod;

    rt_timer_stop(xTimer);
    rt_timer_control(xTimer, RT_TIMER_CTRL_SET_TIME, (void *)&time);

    if(rt_timer_start(xTimer) != RT_EOK)
    {
        rt_kprintf("change time and timer start fail\n");
        return RT_FALSE;
    }

    return RT_TRUE;
}
void *_rtthread_timerGetID( _timerHandle xTimer )
{
    return xTimer;
}

u32  _rtthread_timerStart( _timerHandle xTimer, osdepTickType xBlockTime )
{
    RT_ASSERT(xTimer != RT_NULL);

    DEBUG_LOG(3, "L:%d fun:%s timer:0x%08x\n", __LINE__, __FUNCTION__, xTimer);
    if(rt_timer_start(xTimer) != RT_EOK)
    {
        rt_kprintf("change time and timer start fail\n");
        return RT_FALSE;
    }

    return RT_TRUE;
}

u32  _rtthread_timerStartFromISR( _timerHandle xTimer, osdepBASE_TYPE *pxHigherPriorityTaskWoken )
{
    DEBUG_LOG(1, "L:%d fun:%s timer:0x%08x\n", __LINE__, __FUNCTION__, xTimer);

    if (xTimer == RT_NULL)
    {
        rt_kprintf("timer start from isr null\n");
        return RT_FALSE;
    }

    if(rt_timer_start(xTimer) != RT_EOK)
    {
        rt_kprintf("timer start from isr fail\n");
        return RT_FALSE;
    }

    return RT_TRUE;  
}

u32  _rtthread_timerStopFromISR( _timerHandle xTimer, osdepBASE_TYPE *pxHigherPriorityTaskWoken )
{
    DEBUG_LOG(1, "L:%d fun:%s timer:0x%08x\n", __LINE__, __FUNCTION__, xTimer);

    if (xTimer == RT_NULL)
    {
        rt_kprintf("timer start from isr null\n");
        return RT_FALSE;
    }

    if(rt_timer_stop(xTimer) != RT_EOK)
    {
        rt_kprintf("timer stop from isr fail\n");
        return RT_FALSE;
    }

    return RT_TRUE;
}

u32  _rtthread_timerResetFromISR( _timerHandle xTimer, osdepBASE_TYPE *pxHigherPriorityTaskWoken )
{
    DEBUG_LOG(1, "L:%d fun:%s timer:0x%08x\n", __LINE__, __FUNCTION__, xTimer);

    if (xTimer == RT_NULL)
    {
        rt_kprintf("timer Reset from isr null\n");
        return RT_FALSE;
    }

    if(rt_timer_stop(xTimer) != RT_EOK)
    {
        rt_kprintf("timer Reset from isr stop fail\n");
        return RT_FALSE;
    }

    if(rt_timer_start(xTimer) != RT_EOK)
    {
        rt_kprintf("timer Reset from isr start fail\n");
        return RT_FALSE;
    }

    return RT_TRUE;
}

u32  _rtthread_timerChangePeriodFromISR( _timerHandle xTimer, 
                               osdepTickType xNewPeriod, 
                               osdepBASE_TYPE *pxHigherPriorityTaskWoken )
{
    int time;

    DEBUG_LOG(1, "L:%d fun:%s timer:0x%08x\n", __LINE__, __FUNCTION__, xTimer);

    if (xTimer == RT_NULL)
    {
        rt_kprintf("timer Change Period from isr null\n");
        return RT_FALSE;
    }

    if(xNewPeriod == 0)
        time = 1;
    else if(xNewPeriod >= (RT_TICK_MAX / 2))
        time = (RT_TICK_MAX / 2) - 1;
    else
        time = xNewPeriod;

    rt_timer_stop(xTimer);
    rt_timer_control(xTimer, RT_TIMER_CTRL_SET_TIME, (void *)&time);

    if(rt_timer_start(xTimer) != RT_EOK)
    {
        rt_kprintf("timer Change Period from isr start fail\n");
        return RT_FALSE;
    }

    return RT_TRUE;
}

u32  _rtthread_timerReset( _timerHandle xTimer, 
                               osdepTickType xBlockTime )
{
    RT_ASSERT(xTimer != RT_NULL);

    DEBUG_LOG(2, "L:%d fun:%s timer:0x%08x\n", __LINE__, __FUNCTION__, xTimer);

    rt_timer_stop(xTimer);
    if(rt_timer_start(xTimer) != RT_EOK)
    {
        rt_kprintf("timer reset fail\n");
        return RT_FALSE;
    }

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
        pmu_set_sysactive_time(/*PMU_WLAN_DEVICE, */timeout);
    else
        printf("can't aquire wake during suspend flow!!\n");
#endif
}

u8 _rtthread_get_scheduler_state(void)
{
    rt_thread_t thread = rt_thread_self();
    u8 state = thread->stat;
    switch(state){
        case RT_THREAD_INIT:        state = OS_SCHEDULER_NOT_STARTED;   break;
        case RT_THREAD_RUNNING:     state = OS_SCHEDULER_RUNNING;       break;
        case RT_THREAD_SUSPEND:     state = OS_SCHEDULER_SUSPENDED;     break;
    }
    rt_kprintf("[func]:%s [line]:%d state:%d\n", __FUNCTION__, __LINE__, state);
    return state;
}

const struct osdep_service_ops osdep_service = {
    _rtthread_malloc,           //rtw_vmalloc
    _rtthread_zmalloc,          //rtw_zvmalloc
    _rtthread_mfree,            //rtw_vmfree
    _rtthread_malloc,           //rtw_malloc
    _rtthread_zmalloc,          //rtw_zmalloc
    _rtthread_mfree,            //rtw_mfree
    _rtthread_memcpy,           //rtw_memcpy
    _rtthread_memcmp,           //rtw_memcmp
    _rtthread_memset,           //rtw_memset
    _rtthread_init_sema,        //rtw_init_sema
    _rtthread_free_sema,        //rtw_free_sema
    _rtthread_up_sema,          //rtw_up_sema
    _rtthread_up_sema_from_isr, //rtw_up_sema_from_isr
    _rtthread_down_sema,        //rtw_down_sema
    _rtthread_mutex_init,       //rtw_mutex_init
    _rtthread_mutex_free,       //rtw_mutex_free
    _rtthread_mutex_get,        //rtw_mutex_get
    _rtthread_mutex_get_timeout,//rtw_mutex_get_timeout
    _rtthread_mutex_put,        //rtw_mutex_put
    _rtthread_enter_critical,   //rtw_enter_critical
    _rtthread_exit_critical,    //rtw_exit_critical
    _rtthread_enter_critical_from_isr,  //rtw_enter_critical_from_isr
    _rtthread_exit_critical_from_isr,   //rtw_exit_critical_from_isr
    NULL,       //rtw_enter_critical_bh
    NULL,       //rtw_exit_critical_bh
    _rtthread_enter_critical_mutex, //rtw_enter_critical_mutex
    _rtthread_exit_critical_mutex,  //rtw_exit_critical_mutex
    _rtthread_spinlock_init,        //rtw_spinlock_init
    _rtthread_spinlock_free,        //rtw_spinlock_free
    _rtthread_spinlock,             //rtw_spin_lock
    _rtthread_spinunlock,           //rtw_spin_unlock
    _rtthread_spinlock_irqsave,     //rtw_spinlock_irqsave
    _rtthread_spinunlock_irqsave,   //rtw_spinunlock_irqsave
    _rtthread_init_xqueue,          //rtw_init_xqueue
    _rtthread_push_to_xqueue,       //rtw_push_to_xqueue
    _rtthread_pop_from_xqueue,      //rtw_pop_from_xqueue
    _rtthread_deinit_xqueue,        //rtw_deinit_xqueue
    _rtthread_get_current_time,     //rtw_get_current_time
    _rtthread_systime_to_ms,        //rtw_systime_to_ms
    _rtthread_systime_to_sec,       //rtw_systime_to_sec
    _rtthread_ms_to_systime,        //rtw_ms_to_systime
    _rtthread_sec_to_systime,       //rtw_sec_to_systime
    _rtthread_msleep_os,    //rtw_msleep_os
    _rtthread_usleep_os,    //rtw_usleep_os
    _rtthread_mdelay_os,    //rtw_mdelay_os
    _rtthread_udelay_os,    //rtw_udelay_os
    _rtthread_yield_os,     //rtw_yield_os
    
    _rtthread_ATOMIC_SET,   //ATOMIC_SET
    _rtthread_ATOMIC_READ,  //ATOMIC_READ
    _rtthread_ATOMIC_ADD,   //ATOMIC_ADD
    _rtthread_ATOMIC_SUB,   //ATOMIC_SUB
    _rtthread_ATOMIC_INC,   //ATOMIC_INC
    _rtthread_ATOMIC_DEC,   //ATOMIC_DEC
    _rtthread_ATOMIC_ADD_RETURN,    //ATOMIC_ADD_RETURN
    _rtthread_ATOMIC_SUB_RETURN,    //ATOMIC_SUB_RETURN
    _rtthread_ATOMIC_INC_RETURN,    //ATOMIC_INC_RETURN
    _rtthread_ATOMIC_DEC_RETURN,    //ATOMIC_DEC_RETURN

    _rtthread_modular64,            //rtw_modular64
    _rtthread_get_random_bytes,     //rtw_get_random_bytes
    _rtthread_GetFreeHeapSize,      //rtw_getFreeHeapSize

    _rtthread_create_task,          //rtw_create_task
    _rtthread_delete_task,          //rtw_delete_task
    _rtthread_wakeup_task,          //rtw_wakeup_task

    _rtthread_thread_enter,         //rtw_thread_enter
    _rtthread_thread_exit,          //rtw_thread_exit

    _rtthread_timerCreate,          //rtw_timerCreate,       
    _rtthread_timerDelete,          //rtw_timerDelete,       
    _rtthread_timerIsTimerActive,   //rtw_timerIsTimerActive,
    _rtthread_timerStop,            //rtw_timerStop,         
    _rtthread_timerChangePeriod,    //rtw_timerChangePeriod 
    _rtthread_timerGetID,           //rtw_timerGetID
    _rtthread_timerStart,           //rtw_timerStart
    _rtthread_timerStartFromISR,    //rtw_timerStartFromISR
    _rtthread_timerStopFromISR,     //rtw_timerStopFromISR
    _rtthread_timerResetFromISR,    //rtw_timerResetFromISR
    _rtthread_timerChangePeriodFromISR, //rtw_timerChangePeriodFromISR
    _rtthread_timerReset,           //rtw_timerReset

    _rtthread_acquire_wakelock,     //rtw_acquire_wakelock
    _rtthread_release_wakelock,     //rtw_release_wakelock
    _rtthread_wakelock_timeout,     //rtw_wakelock_timeout
    _rtthread_get_scheduler_state   //rtw_get_scheduler_state
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

