#ifndef __DC_H__
#define __DC_H__

#include "ecrt.h"
#include <time.h>

#define CLOCK_TO_USE CLOCK_MONOTONIC
#define NSEC_PER_SEC (1000000000L)
#define DC_FILTER_CNT 16

/**
 * 分布式时钟上下文结构体
 * 包含所有分布式时钟相关的状态变量
 */
struct dc_context {
    // 基本配置
    int dc_method;                      // 0: 从站参考时钟, 1: 主站参考时钟
    ec_master_t *p_master;             // EtherCAT主站指针
    unsigned int cycle_ns;              // 周期时间（纳秒）
    struct timespec cycle_time_ts;      // 周期时间（timespec格式）
    
    // 从站参考时钟方法相关
    uint64_t dc_start_time_ns;          // DC启动时间
    uint64_t dc_time_ns;                // 当前DC时间
    uint8_t dc_started;                  // DC是否已启动
    int32_t dc_diff_ns;                 // DC时间差
    int32_t prev_dc_diff_ns;            // 上一次DC时间差
    int64_t dc_diff_total_ns;           // DC时间差累计
    int64_t dc_delta_total_ns;          // DC增量累计
    int32_t dc_filter_idx;              // DC滤波器索引
    int64_t dc_adjust_ns;                // DC调整值
    uint64_t dc_first_app_time;          // 首次应用时间
    uint64_t wakeup_time_ns;            // 唤醒时间（纳秒）
    unsigned long long int wakeup_time_ns_base;  // 唤醒时间基准
    long long int sys_time_base;         // 系统时间基准
    uint32_t ref_time_32;                // 参考时间（32位）
    uint64_t ref_time_64;                // 参考时间（64位）
    struct timespec wakeup_time_ts;     // 唤醒时间（timespec格式）
};

// 全局DC上下文实例（在dc.c中定义）
extern struct dc_context g_dc_ctx;

// ========== 分布式时钟函数声明（在dc.c中实现）==========
// 初始化
int dc_init(ec_master_t *master, int method, unsigned int cycletime_ns);

// 时钟同步和周期控制
void dc_wait_period(void);
void dc_sync_distributed_clocks(void);
void dc_update_master_clock(void);
void dc_cal_1st_sleep_time(void);

// 系统时间获取
uint64_t dc_sys_time_ns(void);

// 从站DC配置
int dc_config_slave_dc(ec_slave_config_t *slave, uint16_t assign_activate, 
                        unsigned int cycletime_ns, int sync_shift_percent);

// 注意：dc_time_add 和 dc_sys_time_2_ts 是私有函数（static），不在此声明

#endif /* __DC_H__ */

