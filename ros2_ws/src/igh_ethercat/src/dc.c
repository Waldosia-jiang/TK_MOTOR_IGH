#include <errno.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>

#include "ecrt.h"
#include "igh_example.h"
#include "dc.h"

#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)
#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
    (B).tv_nsec - (A).tv_nsec)

// 全局DC上下文实例
struct dc_context g_dc_ctx = {
    .dc_method = 0,
    .p_master = NULL,
    .cycle_ns = 0,
    .cycle_time_ts = {0, 0},
    .dc_start_time_ns = 0LL,
    .dc_time_ns = 0,
    .dc_started = 0,
    .dc_diff_ns = 0,
    .prev_dc_diff_ns = 0,
    .dc_diff_total_ns = 0LL,
    .dc_delta_total_ns = 0LL,
    .dc_filter_idx = 0,
    .dc_adjust_ns = 0,
    .dc_first_app_time = 0LL,
    .wakeup_time_ns = 0LL,
    .wakeup_time_ns_base = 0LL,
    .sys_time_base = 0LL,
    .ref_time_32 = 0,
    .ref_time_64 = 0,
    .wakeup_time_ts = {0, 0}
};

/**
 * 时间加法运算（私有函数）
 * @param time1 时间1
 * @param time2 时间2
 * @return 相加后的时间
 */
static struct timespec dc_time_add(struct timespec time1, struct timespec time2)
{
    struct timespec result;

    if((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    } else {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }

    return result;
}
/***************************************************/

/* 通过system_time_base将系统时间转换为timespec（私有函数）。
*/
static struct timespec dc_sys_time_2_ts(uint64_t time)
{
    uint64_t time_cnt;
    struct timespec ts;

    time_cnt = time;
    ts.tv_sec = time_cnt / NSEC_PER_SEC;
    ts.tv_nsec = time_cnt % NSEC_PER_SEC;

    return ts;
}

/* 获取当前CPU的时间（纳秒），
     通过system_time_base调整。
     不要直接调用rt_timer_read()，
     而是使用此函数。
*/
uint64_t dc_sys_time_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_TO_USE, &ts);
    return TIMESPEC2NS(ts);
}

/*
     计算第一次睡眠时间以同步参考时钟和主时钟。
*/
void dc_cal_1st_sleep_time(void)
{
    if(g_dc_ctx.dc_method == 0)
    {
        uint64_t dc_remainder = 0LL;
        uint64_t dc_phase_set_time = 0LL;

        dc_phase_set_time = dc_sys_time_ns() + g_dc_ctx.cycle_ns * 10;
        dc_remainder = (dc_phase_set_time - g_dc_ctx.dc_first_app_time) % g_dc_ctx.cycle_ns;
        g_dc_ctx.wakeup_time_ns = dc_phase_set_time - dc_remainder - g_dc_ctx.cycle_ns / 2;
        //g_dc_ctx.wakeup_time_ns = sys_time_ns();
        g_dc_ctx.wakeup_time_ts.tv_sec = g_dc_ctx.wakeup_time_ns / NSEC_PER_SEC;
        g_dc_ctx.wakeup_time_ts.tv_nsec = g_dc_ctx.wakeup_time_ns % NSEC_PER_SEC;
    }
    else if(g_dc_ctx.dc_method == 1)
    {
        clock_gettime(CLOCK_TO_USE, &g_dc_ctx.wakeup_time_ts);
        g_dc_ctx.wakeup_time_ns = TIMESPEC2NS(g_dc_ctx.wakeup_time_ts);
    }
    else
        printf("警告：错误的分布式时钟方法，第一次睡眠时间为0！\n");

    return;
}

/**
 * 分布式时钟初始化
 * @param master EtherCAT主站
 * @param method 分布式时钟方法
 * @param cycletime_ns 周期时间（纳秒）
 * @return 0表示成功，-1表示失败
 */
int dc_init(ec_master_t *master, int method, unsigned int cycletime_ns)
{
    if(master == NULL)
    {
        printf("错误：dc_init中master == NULL\n");
        return -1;
    }
    g_dc_ctx.p_master = master;

    if(method == 0 || method == 1)
        g_dc_ctx.dc_method = method;
    else
    {
        fprintf(stderr, "错误的分布式时钟方法: %d，设置默认值：从站参考时钟\n", 
                method);
        g_dc_ctx.dc_method = 0;
    }
    /* 设置DC补偿周期，与帧周期相同 */
    g_dc_ctx.cycle_ns = cycletime_ns;
    printf("周期时间 %dns (%d Hz)\n", g_dc_ctx.cycle_ns, 1000000000 / g_dc_ctx.cycle_ns);
    if (g_dc_ctx.cycle_ns == 100000 || g_dc_ctx.cycle_ns == 125000 || g_dc_ctx.cycle_ns == 250000 || 
        g_dc_ctx.cycle_ns == 500000 || g_dc_ctx.cycle_ns == 1000000 || g_dc_ctx.cycle_ns == 2000000 || 
        g_dc_ctx.cycle_ns == 4000000 || g_dc_ctx.cycle_ns == 8000000)
    {
        g_dc_ctx.cycle_time_ts.tv_sec = 0;
        g_dc_ctx.cycle_time_ts.tv_nsec = (uint64_t)g_dc_ctx.cycle_ns;
    }
    else
    {
        fprintf(stderr, "错误的周期时间: %u\n", g_dc_ctx.cycle_ns);
        return -1;
    }

    printf("系统时间基准:%lld dc_diff_ns:%d\n", g_dc_ctx.sys_time_base, g_dc_ctx.dc_diff_ns);
    /* 设置初始主站时间 */
    g_dc_ctx.dc_start_time_ns = dc_sys_time_ns();
    g_dc_ctx.dc_time_ns = g_dc_ctx.dc_start_time_ns;
    g_dc_ctx.dc_first_app_time = g_dc_ctx.dc_start_time_ns;
    ecrt_master_application_time(g_dc_ctx.p_master, g_dc_ctx.dc_start_time_ns);

    return 0;
}

/**
 * 等待周期
 */
void dc_wait_period(void)
{
    if(g_dc_ctx.dc_method == 0)
    {
        g_dc_ctx.wakeup_time_ts = dc_time_add(g_dc_ctx.wakeup_time_ts, g_dc_ctx.cycle_time_ts);
        g_dc_ctx.wakeup_time_ns = TIMESPEC2NS(g_dc_ctx.wakeup_time_ts) - g_dc_ctx.sys_time_base;
        g_dc_ctx.wakeup_time_ts.tv_sec = g_dc_ctx.wakeup_time_ns / NSEC_PER_SEC;
        g_dc_ctx.wakeup_time_ts.tv_nsec = g_dc_ctx.wakeup_time_ns % NSEC_PER_SEC;
    }
    else if(g_dc_ctx.dc_method == 1)
    {
        g_dc_ctx.wakeup_time_ts = dc_time_add(g_dc_ctx.wakeup_time_ts, g_dc_ctx.cycle_time_ts);
        g_dc_ctx.wakeup_time_ns = TIMESPEC2NS(g_dc_ctx.wakeup_time_ts);
    }

    clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &g_dc_ctx.wakeup_time_ts, NULL);
    
    return;
}

/**
 * 同步分布式时钟
 */
void dc_sync_distributed_clocks(void)
{
    g_dc_ctx.dc_time_ns = dc_sys_time_ns();
    ecrt_master_application_time(g_dc_ctx.p_master, g_dc_ctx.dc_time_ns);

    if(g_dc_ctx.dc_method == 0)
    {
        int ret = ecrt_master_reference_clock_time(g_dc_ctx.p_master, &g_dc_ctx.ref_time_32);
        if (ret == 0) {
            g_dc_ctx.ref_time_64 = (uint64_t)g_dc_ctx.ref_time_32;
            g_dc_ctx.ref_time_64 = (g_dc_ctx.ref_time_64 - g_dc_ctx.dc_start_time_ns) % g_dc_ctx.cycle_ns;
            g_dc_ctx.dc_diff_ns = (int32_t)(g_dc_ctx.ref_time_64 - 0.5 * g_dc_ctx.cycle_ns);
        } else {
            // 如果没有参考时钟，使用系统时间，但不重复输出错误信息
            static int error_count = 0;
            if (error_count < 5) {
                printf("获取参考时钟时间失败：输入/输出错误（计数：%d）\n", error_count + 1);
                error_count++;
            }
            g_dc_ctx.dc_diff_ns = 0;
        }
    }
    else if(g_dc_ctx.dc_method == 1)
    {
        struct timespec ts;
        clock_gettime(CLOCK_TO_USE, &ts);
        ecrt_master_application_time(g_dc_ctx.p_master, TIMESPEC2NS(ts));
        ecrt_master_sync_reference_clock(g_dc_ctx.p_master);
    }
    else
        printf("错误的分布式时钟方法 %d\n", g_dc_ctx.dc_method);
}

/*
 * 基于参考从站时间差更新主站时间
 * 在ecrt_master_send()之后调用，以避免在
 * dc_sync_distributed_clocks()中出现时间抖动
 */
void dc_update_master_clock(void)
{
    if(g_dc_ctx.dc_method == 1)
        return;

    if (g_dc_ctx.dc_diff_ns != 0) {
        g_dc_ctx.dc_filter_idx++;
        g_dc_ctx.dc_diff_total_ns += g_dc_ctx.dc_diff_ns;
        if (g_dc_ctx.dc_filter_idx >= DC_FILTER_CNT) {
            g_dc_ctx.dc_adjust_ns += g_dc_ctx.dc_diff_total_ns / DC_FILTER_CNT;
            if (g_dc_ctx.dc_adjust_ns < -0.3 * g_dc_ctx.cycle_ns / 1000)
                g_dc_ctx.dc_adjust_ns = -0.3 * g_dc_ctx.cycle_ns / 1000;
            if (g_dc_ctx.dc_adjust_ns > 0.3 * g_dc_ctx.cycle_ns / 1000)
                g_dc_ctx.dc_adjust_ns = 0.3 * g_dc_ctx.cycle_ns / 1000;
            g_dc_ctx.dc_diff_total_ns = 0;
            g_dc_ctx.dc_filter_idx = 0;
        }
        g_dc_ctx.sys_time_base = g_dc_ctx.dc_adjust_ns;
    } else
        g_dc_ctx.sys_time_base = 0;

    return;
}

/**
 * 配置从站的分布式时钟
 * @param slave 从站配置指针
 * @param assign_activate DC分配激活字（通常为0x0300）
 * @param cycletime_ns 周期时间（纳秒）
 * @param sync_shift_percent SYNC Shift Time占周期时间的百分比（如30表示30%）
 * @return 0表示成功，-1表示失败
 */
int dc_config_slave_dc(ec_slave_config_t *slave, uint16_t assign_activate, 
                       unsigned int cycletime_ns, int sync_shift_percent)
{
    if (slave == NULL) {
        printf("错误：dc_config_slave_dc中slave == NULL\n");
        return -1;
    }
    
    // 计算SYNC Shift Time（周期时间的百分比）
    int32_t sync0_shift_time = (int32_t)(cycletime_ns * sync_shift_percent / 100);
    
    // 配置DC信号
    int ret = ecrt_slave_config_dc(slave, assign_activate, cycletime_ns, 
                                    sync0_shift_time, 0, 0);
    if (ret != 0) {
        printf("错误：从站DC配置失败，返回值 = %d\n", ret);
        return -1;
    }
    
    return 0;
}
