#ifndef __IGH_EXAMPLE__
#define __IGH_EXAMPLE__

#include "ecrt.h"
#define MAX_SLAVE_NUMBER 8

/* 注意：域中的过程数据偏移量。
   为每个从站设备声明。 */
struct pdo_offset {
    //接收PDO
    unsigned int control_word; //0x6040,控制字,子索引:0,位长度:16
    unsigned int mode_of_operation; //0x6060,控制模式,子索引:0,位长度:8
    unsigned int target_position;//0x607A,目标位置,子索引:0,位长度:32
    unsigned int target_velocity;//0x60ff,目标速度,子索引:0,位长度:32
    unsigned int target_torque;//0x60ff,目标转矩,子索引:0,位长度:32
    unsigned int velocity_feedforward;//0x60B1,速度前馈,子索引:0,位长度:32
    unsigned int torque_offset;//0x60B2,转矩偏移,子索引:0,位长度:16
    unsigned int digital_outputs;//0x60FE,数字输出,子索引:1,位长度:32

    //发送PDO
    unsigned int status_word;//0x6041,状态字,子索引:0,位长度:16
    unsigned int mode_of_operation_display; //0x6061,子索引:0,位长度:8
    unsigned int position_actual_value;//0x6064,实际位置值,子索引:0,位长度:32
    unsigned int velocity_actual_value;//0x606c,实际速度值,子索引:0,位长度:32
    unsigned int velocity_demand_value;//0x606B,速度需求值,子索引:0,位长度:32
    unsigned int torque_actual_value;//0x6077,实际转矩值,子索引:0,位长度:16
    unsigned int torque_demand_value;//0x6074,转矩需求值,子索引:0,位长度:16
    unsigned int digital_inputs;//0x60fd,数字输入,子索引:0,位长度:32
};

struct motion_control_data {
    //PDO数据
    unsigned short control_word;
    unsigned short status_word;
    int target_position;
    int actual_position;
    int target_velocity;
    int actual_velocity;
    short target_torque;
    short actual_torque;
    unsigned int digital_inputs;
    unsigned char control_mode;
    unsigned char control_mode_acutal;

    //辅助数据
    int counter;
    int cur_velocity;
    int is_reverse;
    int is_first_in;
    int is_power_on;
    
    // 位置限制相关字段
    int32_t desired_target_position;    // 期望的最终目标位置
    int32_t current_target_position;    // 当前实际设置的目标位置
    int32_t position_limit_per_cycle;   // 每周期最大位置增量
    int position_limit_enabled;         // 是否启用位置限制
};

struct slave_id {
    unsigned int vendor_id;
    unsigned int product_code;
};

// 从站数量（在igh_example.c中维护）
extern unsigned int slave_count;

// 控制周期时间配置（外部变量，在igh_example.c中设置）
extern unsigned int g_control_cycle_time_us;  // 控制周期时间（微秒）

// S曲线模式配置（外部变量，在igh_example.c中设置）
extern int s_curve_mode;  // S曲线模式：0=折线往返增量模式（三角形），1=一阶梯形，2=二阶S曲线

// 根据时间（秒）计算需要的控制周期数
static inline int control_cycles_for_time(float time_seconds) {
    if (g_control_cycle_time_us == 0) {
        return 1;  // 防止除零
    }
    return (int)(time_seconds * 1000000.0f / g_control_cycle_time_us + 0.5f);  // 四舍五入
}

//分布式时钟（函数声明在dc.h中）
// 注意：dc.h已包含相关函数声明

// 注意：运动控制相关函数声明在motion.h中（在motion.c中实现）
// 注意：调试函数声明在motion.h中（在motion.c中实现）

// ========== 主程序函数声明（在igh_example.c中实现）==========
// 参数解析和初始化
int example_parse_arguments(int argc, char **argv, int *cpuid, int *dc_method, unsigned int *cycletime_ns);
int example_init_ethercat_master(ec_master_t **master, ec_domain_t **domain, ec_master_info_t *master_info);

// 从站配置
int example_configure_servo_motion_params(int slave_index, ec_slave_config_t *slave);

// 主站激活和线程管理
int example_activate_master_and_start_threads(ec_master_t *master, ec_domain_t *domain, uint8_t **p_domain, unsigned int cycletime_ns, int cpuid);
void example_wait_for_user_exit(void);

// ========== ROS2/库模式辅助API（在igh_example.c中实现）==========
// 启动EtherCAT并创建周期线程（用于ROS2节点集成）
int example_ros2_start(int cpuid, unsigned int cycletime_ns);
// 停止周期线程并释放主站资源
void example_ros2_stop(void);

// 注意：example_is_servo, example_catch_signal, example_cyclic_task, example_init_servo_status_check 是私有函数（static），不在此声明

#endif
