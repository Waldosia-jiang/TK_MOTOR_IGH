#ifndef __MOTION_H__
#define __MOTION_H__

#include "igh_example.h"

#define MAX_SLAVE_NUMBER 8

/**
 * 增量控制模式上下文结构体
 * 用于管理增量限制模式的状态
 */
struct incremental_limit_context {
    int32_t initial_position_for_limit[MAX_SLAVE_NUMBER];  // 初始位置
    int increment_limit_initialized[MAX_SLAVE_NUMBER];      // 是否已初始化
    int control_cycle_counter_for_limit[MAX_SLAVE_NUMBER];  // 控制周期计数器
    int current_phase[MAX_SLAVE_NUMBER];                    // 当前相位：0=正向，1=反向
};

/**
 * 调试信息管理结构体
 */
struct debug_info {
    int cycle_counter;
    int last_print_cycle[MAX_SLAVE_NUMBER];
    int print_interval;
    int enabled;
};

/**
 * 运动控制状态结构体
 * 用于管理每个从站的运动控制状态
 */
struct motion_state {
    // 故障恢复相关
    int last_fault_state[MAX_SLAVE_NUMBER];
    int fault_recovery_attempts[MAX_SLAVE_NUMBER];
    int fault_recovery_wait[MAX_SLAVE_NUMBER];
    int fault_recovery_interval;
    
    // 上电状态相关
    int last_power_state[MAX_SLAVE_NUMBER];
    
    // 往返运动相关
    int waiting_for_reverse[MAX_SLAVE_NUMBER];
    int wait_counter[MAX_SLAVE_NUMBER];
    int32_t last_target_position[MAX_SLAVE_NUMBER];
    
    // power_state_machine函数中的状态
    int print_counter;
    unsigned short last_status[MAX_SLAVE_NUMBER];
    int last_print_cycle_power[MAX_SLAVE_NUMBER];
    int print_interval_power;
    int zero_status_warning[MAX_SLAVE_NUMBER];
    int initial_target_set[MAX_SLAVE_NUMBER];
};

/**
 * S曲线轨迹规划结构体
 * 用于存储生成的轨迹点和执行状态
 */
#define MAX_TRAJECTORY_POINTS 100000  // 最大轨迹点数（可根据需要调整）
struct s_curve_trajectory {
    int32_t trajectory_points[MAX_TRAJECTORY_POINTS];  // 轨迹点数组
    int total_points;                                   // 总轨迹点数
    int current_index;                                  // 当前执行索引
    int initialized;                                    // 是否已初始化
    int32_t initial_position;                           // 初始位置
    int current_phase;                                  // 当前相位：0=正向，1=反向
};

// 全局上下文实例（在motion.c中定义）
extern struct incremental_limit_context g_inc_limit_ctx;
extern struct debug_info g_debug_mgr;
extern struct motion_state g_motion_state;
extern struct s_curve_trajectory g_s_curve_traj[MAX_SLAVE_NUMBER];

// ========== 运动控制函数声明（在motion.c中实现）==========
// 初始化
void motion_init(int velocity);

// 运行模式控制
void motion_set_operation_mode(int slv_num, unsigned char mode);
void motion_power_state_machine(int slv_num, int is_on);
void motion_fault_recovery(int slv_num);

// 运动控制算法
void motion_process_slave_motion(int slv_num);
void motion_process_slave_motion_csp(int slv_num);
void motion_process_slave_motion_csv(int slv_num);
void motion_process_slave_motion_cst(int slv_num);
void motion_process_slave_motion_csp_vff(int slv_num);
void motion_process_slave_motion_csp_trapezoidal(int slv_num);  // 一阶梯形加减速CSP+前馈模式
void motion_process_slave_motion_csp_2s(int slv_num);  // 二阶S曲线CSP+前馈模式
void motion_process_slave_motion_custom(int slv_num);
// 注意：motion_common_motion_logic 和 motion_calculate_incremental_target 是私有函数（static），不在此声明

// PDO数据处理
void motion_process_pdo_data(unsigned char *p_domain, struct pdo_offset *offset, unsigned int slave_count, ec_domain_t *domain);
void motion_update_pdo_data(unsigned char *p_domain, struct pdo_offset *offset, unsigned int slave_count);

// 调试函数
void motion_debug_print_ordered(int slv_num, const char *format, ...);
void motion_debug_update_cycle(void);
void motion_debug_set_interval(int interval);
void motion_debug_set_enabled(int enabled);

#endif /* __MOTION_H__ */

