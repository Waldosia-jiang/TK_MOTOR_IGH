#include <stdio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#include "ecrt.h"
#include "igh_example.h"
#include "motion.h"
#include "trajectory_planning.h"
#include "dc.h"

int max_diffPosition;
struct motion_control_data motion_data[MAX_SLAVE_NUMBER];

// 外部变量：控制周期时间（微秒）
extern unsigned int g_control_cycle_time_us;

// 外部变量：EtherCAT周期时间（纳秒）
extern unsigned int g_cycletime_ns;

// 外部变量：每个控制周期最大增量脉冲数
extern int pulses_per_control_cycle;

// 外部变量：PDO模板选择
extern int pdo_choice;

// 外部变量：CSV模式目标速度（cnt）
extern int target_velocity_cnt;

// 外部变量：CST模式位置增量（cnt），内部通过力矩输出实现力位混合控制
extern int target_torque;

// 外部变量：S曲线模式
extern int s_curve_mode;

// 全局上下文实例
struct incremental_limit_context g_inc_limit_ctx = {
    .initial_position_for_limit = {0},
    .increment_limit_initialized = {0},
    .control_cycle_counter_for_limit = {0},
    .current_phase = {0}
};

struct debug_info g_debug_mgr = {0, {0}, 1000, 1};

struct motion_state g_motion_state = {
    .last_fault_state = {0},
    .fault_recovery_attempts = {0},
    .fault_recovery_wait = {0},
    .fault_recovery_interval = 0,
    .last_power_state = {0},
    .waiting_for_reverse = {0},
    .wait_counter = {0},
    .last_target_position = {0},
    .print_counter = 0,
    .last_status = {0},
    .last_print_cycle_power = {0},
    .print_interval_power = 0,
    .zero_status_warning = {0},
    .initial_target_set = {0}
};

// S曲线轨迹规划全局实例
struct s_curve_trajectory g_s_curve_traj[MAX_SLAVE_NUMBER] = {0};

static int32_t g_cst_initial_position[MAX_SLAVE_NUMBER] = {0};
static int     g_cst_initialized[MAX_SLAVE_NUMBER]      = {0};
static int     g_cst_first_printed[MAX_SLAVE_NUMBER]    = {0};

void motion_cst_reset(int slv_num)
{
    if (slv_num < 0 || slv_num >= MAX_SLAVE_NUMBER) return;
    g_cst_initialized[slv_num] = 0;
    g_cst_first_printed[slv_num] = 0;
}

/**
 * 按轴号顺序输出调试信息
 * @param slv_num 从站编号
 * @param format 格式化字符串
 * @param ... 可变参数
 */
void motion_debug_print_ordered(int slv_num, const char *format, ...) {
    if (!g_debug_mgr.enabled) return;
    
    // 检查是否到了打印周期
    if (g_debug_mgr.cycle_counter % g_debug_mgr.print_interval == 0) {
        // 检查这个轴是否已经在这个周期打印过
        if (g_debug_mgr.last_print_cycle[slv_num] != g_debug_mgr.cycle_counter) {
            va_list args;
            va_start(args, format);
            printf("轴 %d: ", slv_num);
            vprintf(format, args);
            printf("\n");
            va_end(args);
            
            // 记录这个轴已经在这个周期打印过
            g_debug_mgr.last_print_cycle[slv_num] = g_debug_mgr.cycle_counter;
        }
    }
}

/**
 * 更新调试周期计数器
 */
void motion_debug_update_cycle() {
    g_debug_mgr.cycle_counter++;
}

/**
 * 设置调试信息打印间隔
 * @param interval 打印间隔（周期数）
 */
void motion_debug_set_interval(int interval) {
    g_debug_mgr.print_interval = interval;
}

/**
 * 启用/禁用调试信息
 * @param enabled 是否启用
 */
void motion_debug_set_enabled(int enabled) {
    g_debug_mgr.enabled = enabled;
}

/**
 * 运动控制初始化
 * @param max_vel 最大速度
 */
void motion_init(int max_vel)
{
    int i = 0;
    // 设置全局变量
    max_diffPosition = max_vel;
    
    for(i = 0; i < MAX_SLAVE_NUMBER; ++i)
    {
        motion_data[i].control_word = 0x0006;   // 准备使能
        motion_data[i].control_mode = 8;        // CSP模式
        motion_data[i].target_position = 0;     // 初始目标位置设为0
        motion_data[i].actual_position = 0;     // 初始实际位置设为0
        motion_data[i].status_word = 0x0000;    // 初始状态字
        motion_data[i].is_power_on = 0;         // 初始未上电
        motion_data[i].is_reverse = 0;          // 初始正向运动
        motion_data[i].is_first_in = 1;         // 首次进入标志
        motion_data[i].counter = 0;             // 计数器
        motion_data[i].cur_velocity = 0;        // 当前速度
    }
}

/**
 * 处理PDO数据
 * @param p_domain 域数据指针
 * @param offset PDO偏移量
 * @param slave_count 从站数量
 * @param domain 域指针（用于检查通信状态，可为NULL）
 */
void motion_process_pdo_data(unsigned char *p_domain, struct pdo_offset *offset, unsigned int slave_count, ec_domain_t *domain)
{
    // 检查通信状态，如果通信异常则不更新状态字，避免读取到错误值
    int comm_ok = 1;
    if (domain) {
        ec_domain_state_t domain_state;
        if (ecrt_domain_state(domain, &domain_state) == 0) {
            if (domain_state.wc_state != EC_WC_COMPLETE) {
                comm_ok = 0;  // 通信异常，不更新状态字
            }
        }
    }
    
    unsigned int i = 0;
    for(i = 0; i < slave_count; ++i)
    {
        // 只有在通信正常时才更新状态字，避免通信异常时读取到错误值导致误下闸
        if(offset[i].status_word < 0x0000ffff && comm_ok)
            motion_data[i].status_word = EC_READ_U16(p_domain + offset[i].status_word);
        if(offset[i].position_actual_value < 0x0000ffff)
            motion_data[i].actual_position = EC_READ_S32(p_domain + offset[i].position_actual_value);
        if(offset[i].mode_of_operation_display < 0x0000ffff)
            motion_data[i].control_mode_acutal = EC_READ_U8(p_domain + offset[i].mode_of_operation_display);
        if(offset[i].velocity_actual_value < 0x0000ffff)
            motion_data[i].actual_velocity = EC_READ_S32(p_domain + offset[i].velocity_actual_value);
        if(offset[i].torque_actual_value < 0x0000ffff)
            motion_data[i].actual_torque = EC_READ_S16(p_domain + offset[i].torque_actual_value);
        
        // 每5秒检查一次PDO数据是否正常（基于时间）
        static uint64_t last_check_time_ns = 0;
        uint64_t current_time_ns = dc_sys_time_ns();
        if (last_check_time_ns == 0) {
            last_check_time_ns = current_time_ns;
        }
        if (current_time_ns - last_check_time_ns >= 5000000000ULL) {  // 5秒 = 5000000000纳秒
            last_check_time_ns = current_time_ns;
            printf("PDO检查 - 从站 %d: 状态=0x%04X, 位置=%d, 状态偏移=%d, 位置偏移=%d\n", 
                   i, motion_data[i].status_word, motion_data[i].actual_position,
                   offset[i].status_word, offset[i].position_actual_value);
            
            // 添加原始PDO数据检查
            if (offset[i].status_word < 0x0000ffff) {
                uint16_t raw_status = EC_READ_U16(p_domain + offset[i].status_word);
                printf("PDO原始数据 - 从站 %d: 原始状态字=0x%04X\n", i, raw_status);
            }
            
            // 检查控制字是否正确写入
            if (offset[i].control_word < 0x0000ffff) {
                uint16_t raw_control = EC_READ_U16(p_domain + offset[i].control_word);
                printf("PDO原始数据 - 从站 %d: 原始控制字=0x%04X, 期望控制字=0x%04X\n", 
                       i, raw_control, motion_data[i].control_word);
            }
        }
    }

    return;
}

/**
 * 计算增量控制模式的目标位置（私有函数）
 * 每个控制周期（比如5ms）都增加increment个脉冲
 * 例如：-p 500000 -i 100，初始位置=25000
 *   周期1：25000 + 100 = 25100
 *   周期2：25100 + 100 = 25200
 *   周期3：25200 + 100 = 25300
 *   ...
 *   到达max_position后反向递减
 * @param slv_num 从站编号
 * @return 计算出的目标位置
 */
static int32_t motion_calculate_incremental_target(int slv_num)
{
    int max_position = max_diffPosition;  // -p参数
    int increment = pulses_per_control_cycle;  // -i参数，每个周期增加的脉冲数
    
    // 初始化应该在process_slave_motion中完成（电机上电后），这里不应该再初始化
    if (!g_inc_limit_ctx.increment_limit_initialized[slv_num]) {
        // 如果还没初始化，返回当前actual_position（避免写入错误值）
        return motion_data[slv_num].actual_position;
    }
    
    int32_t initial_pos = g_inc_limit_ctx.initial_position_for_limit[slv_num];  // 初始化时保存的actual_position
    int32_t current_actual = motion_data[slv_num].actual_position;  // 当前实际位置（用于调试）
    int32_t new_position;
    
    // 计算当前累计的增量：每个周期增加increment
    int32_t total_increment = g_inc_limit_ctx.control_cycle_counter_for_limit[slv_num] * increment;
    
    if (g_inc_limit_ctx.current_phase[slv_num] == 0) {
        // 正向：初始位置 + 累计增量
        new_position = initial_pos + total_increment;
        
        if (total_increment >= max_position) {
            // 到达最大值，切换到反向
            new_position = initial_pos + max_position;
            g_inc_limit_ctx.current_phase[slv_num] = 1;
            g_inc_limit_ctx.control_cycle_counter_for_limit[slv_num] = 0;  // 重置计数器
        }
    } else {
        // 反向：初始位置 + max_position - 累计增量
        int32_t remaining = max_position - total_increment;
        new_position = initial_pos + remaining;
        
        if (remaining <= 0) {
            // 回到初始位置，切换到正向
            new_position = initial_pos;
            g_inc_limit_ctx.current_phase[slv_num] = 0;
            g_inc_limit_ctx.control_cycle_counter_for_limit[slv_num] = 0;  // 重置计数器
        }
    }
    
    g_inc_limit_ctx.control_cycle_counter_for_limit[slv_num]++;
    
    // 每50个周期打印一次（减少打印频率）
    static int32_t last_printed_position[MAX_SLAVE_NUMBER] = {0};
    static int print_counter[MAX_SLAVE_NUMBER] = {0};
    if (++print_counter[slv_num] >= 50 || new_position != last_printed_position[slv_num]) {
        if (print_counter[slv_num] >= 50) {
            print_counter[slv_num] = 0;
        }
        /*
        printf("轴 %d: 目标位置=%d = %d(初始位置) + %d(累计增量) [当前actual_position=%d, 周期计数=%d, phase=%d]\n", 
               slv_num, new_position, initial_pos, total_increment, current_actual, 
               g_inc_limit_ctx.control_cycle_counter_for_limit[slv_num] - 1, g_inc_limit_ctx.current_phase[slv_num]);*/
        last_printed_position[slv_num] = new_position;
    }
    
    return new_position;
}

/**
 * 更新PDO数据
 * 根据operation_mode（CSP/CSV/CST）判断写入哪些参数
 * @param p_domain 域数据指针
 * @param offset PDO偏移量
 * @param slave_count 从站数量
 */
void motion_update_pdo_data(unsigned char *p_domain, struct pdo_offset *offset, unsigned int slave_count)
{
    static int dout_inited[MAX_SLAVE_NUMBER] = {0};
    unsigned int i = 0;
    for(i = 0; i < slave_count; ++i)
    {
        // 所有模式都需要写入：控制字和运行模式
        if(offset[i].control_word < 0x0000ffff)
            EC_WRITE_U16(p_domain + offset[i].control_word, motion_data[i].control_word);
        if(offset[i].mode_of_operation < 0x0000ffff)
            EC_WRITE_U8(p_domain + offset[i].mode_of_operation, motion_data[i].control_mode);

        // 根据operation_mode判断写入哪些参数
        unsigned char operation_mode = motion_data[i].control_mode;
        
        switch(operation_mode) {
            case 8:  // CSP模式 (Cyclic Synchronous Position)
                // CSP模式：写入目标位置
                if(offset[i].target_position < 0x0000ffff)
                    EC_WRITE_S32(p_domain + offset[i].target_position, motion_data[i].target_position);
                break;
                
            case 3:  // CSV模式 (Cyclic Synchronous Velocity)
                // CSV模式：写入目标速度（0x60FF）
                if(offset[i].target_velocity >= 0 && offset[i].target_velocity < 0x0000ffff) {
                    EC_WRITE_S32(p_domain + offset[i].target_velocity, motion_data[i].target_velocity);
                    // 调试：定期按轴打印写入的速度值（每轴每1000个周期）
                    static int csv_debug_counter[MAX_SLAVE_NUMBER] = {0};
                    if (++csv_debug_counter[i] % 1000 == 0) {
                        printf("轴 %d: CSV模式 - 写入目标速度值 = %d (0x%08X) 到偏移量 %d\n",
                               i, motion_data[i].target_velocity, motion_data[i].target_velocity, offset[i].target_velocity);
                    }
                }
                break;
                
            case 4:  // CST模式 (Cyclic Synchronous Torque)
                // CST力位混合控制：根据位置误差计算出的目标转矩写入0x6071
                if(offset[i].target_torque >= 0 && offset[i].target_torque < 0x0000ffff) {
                    EC_WRITE_S16(p_domain + offset[i].target_torque, motion_data[i].target_torque);
                    // 调试：定期按轴打印写入的转矩值（每轴每1000个周期）
                    static int cst_debug_counter[MAX_SLAVE_NUMBER] = {0};
                    if (++cst_debug_counter[i] % 1000 == 0) {
                        printf("轴 %d: CST模式 - 写入目标转矩值 = %d 到偏移量 %d\n",
                               i, motion_data[i].target_torque, offset[i].target_torque);
                    }
                }
                break;
                
            default:
                // 其他模式或未知模式：默认写入目标位置（兼容旧代码）
                if(offset[i].target_position < 0x0000ffff)
                    EC_WRITE_S32(p_domain + offset[i].target_position, motion_data[i].target_position);
                break;
        }

        // 数字输出/通用输出
        // 为了避免对驱动/IO产生潜在耦合，这里只在启动时写一次0（而不是每周期反复写）
        if(offset[i].digital_outputs < 0x0000ffff && !dout_inited[i]) {
            EC_WRITE_U32(p_domain + offset[i].digital_outputs, 0);
            dout_inited[i] = 1;
        }
    }

    return;
}

/**
 * 设置运行模式
 * @param slv_num 从站编号
 * @param mode 控制模式
 */
void motion_set_operation_mode(int slv_num, unsigned char mode)
{
    if(mode > 10)
    {
        printf("0x6060的错误模式 %d\n", mode);
        return;
    }
    motion_data[slv_num].control_mode = mode;
}

/**
 * 伺服上电状态机
 * @param slv_num 从站编号
 * @param is_on 是否上电
 */
void motion_power_state_machine(int slv_num, int is_on)
{
    unsigned short *sw = &motion_data[slv_num].status_word;
    unsigned short *cw = &motion_data[slv_num].control_word;
    
    // 基于时间的打印控制（每1秒打印一次，或状态变化时打印一次）
    static uint64_t last_print_time_ns[MAX_SLAVE_NUMBER] = {0};
    static unsigned short last_printed_status[MAX_SLAVE_NUMBER] = {0xFFFF};
    uint64_t current_time_ns = dc_sys_time_ns();
    int should_print = 0;
    
    if (last_print_time_ns[slv_num] == 0) {
        last_print_time_ns[slv_num] = current_time_ns;
    }
    
    // 状态变化时打印一次
    if (*sw != last_printed_status[slv_num]) {
        should_print = 1;
        last_printed_status[slv_num] = *sw;
    }
    // 或者每1秒打印一次
    else if (current_time_ns - last_print_time_ns[slv_num] >= 1000000000ULL) {  // 1秒
        should_print = 1;
        last_print_time_ns[slv_num] = current_time_ns;
    }

    // 检查状态字是否为0，这可能表示PDO通信问题
    if (*sw == 0x0000) {
        if (!g_motion_state.zero_status_warning[slv_num]) {
            printf("轴 %d: 警告 - 状态字为0x0000，可能存在PDO通信问题！\n", slv_num);
            g_motion_state.zero_status_warning[slv_num] = 1;
        }
    } else {
        g_motion_state.zero_status_warning[slv_num] = 0;
    }

    if(is_on) //上电
    {
        // 检测从正常运行状态（0x07）变成其他状态，可能导致"关闸"
        static unsigned short last_status_low[MAX_SLAVE_NUMBER] = {0xFF};
        unsigned char current_status_low = *sw & 0x0f;
        unsigned char last_status_low_val = last_status_low[slv_num] & 0x0f;
        
        // 如果之前是正常运行状态（0x07），现在变成其他状态，打印调试信息
        if (last_status_low_val == 0x07 && current_status_low != 0x07 && motion_data[slv_num].is_power_on) {
            printf("轴 %d: 🔍 调试 - 状态字从0x07变成0x%02X，target_position=%d, actual_position=%d, 控制字=0x%04X\n", 
                   slv_num, current_status_low, motion_data[slv_num].target_position, 
                   motion_data[slv_num].actual_position, motion_data[slv_num].control_word);
        }
        last_status_low[slv_num] = *sw;
        
        // 正确的上电逻辑：0x00, 0x01, 0x03, 0x07 都是正常的上电过程状态，不下闸
        // 只有在真正的故障状态（其他值）时才下闸
        if ((*sw & 0x0f) == 0x00)
        {
            // 0x00: 低4位为0（包含 0x0000 与 0x0250 等）
            // 0x0000：可能为 PDO 未更新/通信异常误读，若此时发 0x06 会对本在 0x1237 的驱动器下闸，导致振荡
            // 0x0250：CiA402 合法状态“电压使能、开启禁用”，应发 0x06 继续上电流程
            if (motion_data[slv_num].is_power_on && last_status_low_val == 0x07) {
                if (*sw == 0x0000) {
                    // 全 0 可能是通信误读，保持 0x0F 避免误下闸，不按“通信异常”发 0x06
                    *cw = 0x0f;
                    if (should_print) {
                        printf("轴 %d: 状态字0x0000（可能PDO误读），保持0x0F不发送0x06 (状态=0x%04X)\n", slv_num, *sw);
                    }
                } else {
                    // 非 0x0000（如 0x0250），为合法“开启禁用”等状态，发 0x06 重新上电
                    *cw = 0x06;
                    if (should_print) {
                        printf("轴 %d: 状态字从0x07变为0x%04X，发送0x06重新上电 (状态=0x%04X)\n", slv_num, *sw, *sw);
                    }
                }
            } else if (motion_data[slv_num].is_power_on) {
                // 已经认为轴处于上电运行阶段，但当前状态低4位为0。
                // 这里继续区分两类情况：
                // 1) 0x0000：仍按可能PDO误读处理，保持0x0F避免误下闸；
                // 2) 非0x0000（如0x0250 电压使能/开启禁用）：发送0x06，按正常CiA402流程重新上电，
                //    避免长时间停留在0x0250而无法回到0x37。
                if (*sw == 0x0000) {
                    *cw = 0x0f;
                    if (should_print) {
                        printf("轴 %d: 已上电阶段检测到状态字0x0000，保持控制字0x0F (状态=0x%04X)\n", slv_num, *sw);
                    }
                } else {
                    *cw = 0x06;
                    if (should_print) {
                        printf("轴 %d: 已上电阶段检测到状态字0x%04X（如0x0250），发送0x06尝试重新上电\n", slv_num, *sw);
                    }
                }
            } else {
                // 初始上电：0x0000 发 0x06；0x0250 等也发 0x06 继续流程
                *cw = 0x06;
                if (should_print) {
                    printf("轴 %d: 🔍 初始上电 - 状态字0x%04X，发送控制字0x06 (状态=0x%04X)\n", slv_num, *sw, *sw);
                }
            }
            motion_data[slv_num].target_position = motion_data[slv_num].actual_position;
        }
        else if ((*sw & 0x0f) == 0x01)
        {
            // 0x01: 使能禁用 - 准备状态
            // 如果之前是正常运行状态（0x07），现在变成0x01，说明可能是通信异常导致的下电
            // 直接发送0x07重新上电，但不修改is_power_on标志，避免误判
            if (motion_data[slv_num].is_power_on && last_status_low_val == 0x07) {
                // 从正常运行状态变成0x01，说明驱动器可能因为通信异常而下电了
                // 直接发送0x07重新上电，不修改is_power_on
                *cw = 0x07;
                if (should_print) {
                    printf("轴 %d: ⚠️ 状态字从0x07变为0x01（通信异常导致下电？），发送0x07重新上电 (状态=0x%04X)\n", slv_num, *sw);
                }
            } else if (motion_data[slv_num].is_power_on) {
                // 已经上电运行，但之前不是0x07，可能是短暂的状态变化，保持0x0F
                *cw = 0x0f;
                if (should_print && last_status_low_val == 0x07) {
                    printf("轴 %d: 🔍 状态字从0x07变为0x01（通信抖动？），保持控制字0x0F (状态=0x%04X)\n", slv_num, *sw);
                }
            } else {
                // 初始上电状态，发送0x07继续上电流程
                *cw = 0x07;
                if (should_print) {
                    printf("轴 %d: 🔍 初始上电 - 状态字0x01，发送控制字0x07 (状态=0x%04X)\n", slv_num, *sw);
                }
            }
            motion_data[slv_num].target_position = motion_data[slv_num].actual_position;
        }
        else if ((*sw & 0x0f) == 0x03)
        {
            // 0x03: 操作启用 - 已启用但未运行使能
            // 无论是否已上电，都发送0x0f（这是正常运行的控制字）
            *cw = 0x0f;
            motion_data[slv_num].target_position = motion_data[slv_num].actual_position;
            // 如果已经上电运行，状态字变成0x03可能是通信抖动，保持0x0F让驱动器恢复
            if (should_print && last_status_low_val == 0x07) {
                printf("轴 %d: 🔍 状态字从0x07变为0x03（通信抖动？），保持控制字0x0F，不下闸 (状态=0x%04X)\n", slv_num, *sw);
            }
        }
        else if ((*sw & 0x0f) == 0x07)
        {
            // 0x07: 运行使能 - 正常运行状态
            *cw = 0x0f;
            motion_data[slv_num].is_power_on = 1;
            
            // 设置初始目标位置（使用绝对位置，避免相对位置跳跃）
            if (!g_motion_state.initial_target_set[slv_num]) {
                motion_data[slv_num].target_position = motion_data[slv_num].actual_position;  // 先保持当前位置
                g_motion_state.initial_target_set[slv_num] = 1;
                if (should_print) {
                    printf("轴 %d: 上电并就绪，保持当前位置 = %d\n", slv_num, motion_data[slv_num].actual_position);
                }
            }
        }
        else 
        {
            // 遇到故障状态（如0x08），发送清除故障命令，但不关闸
            // 让故障恢复函数来处理故障清除，保持上电状态
            *cw = 0x80;
            motion_data[slv_num].target_position = motion_data[slv_num].actual_position;
            if (should_print || (last_status_low_val == 0x07)) {
                printf("轴 %d: ⚠️ 故障检测 - 状态字0x%02X（故障），发送清除故障命令0x80，保持上电状态 (状态=0x%04X)\n", 
                       slv_num, current_status_low, *sw);
            }
        }
    }
    else //power off
    {
        *cw = 0x00;
        motion_data[slv_num].is_power_on = 0;
        if (should_print) {
            printf("轴 %d: 断电\n", slv_num);
        }
    }
}

void motion_fault_recovery(int slv_num)
{
    // 智能故障处理：不直接返回，而是尝试恢复
    if (motion_data[slv_num].status_word & 0x0008) {
        if (!g_motion_state.last_fault_state[slv_num]) {
            g_motion_state.last_fault_state[slv_num] = 1;
            g_motion_state.fault_recovery_attempts[slv_num] = 0;
            g_motion_state.fault_recovery_wait[slv_num] = 0;
        }
        
        // 尝试自动恢复故障
        g_motion_state.fault_recovery_wait[slv_num]++;
        
        // 每1秒尝试一次故障清除
        // 注意：fault_recovery_wait是基于控制周期的，不是EtherCAT周期
        if (g_motion_state.fault_recovery_interval == 0) {
            g_motion_state.fault_recovery_interval = control_cycles_for_time(1.0f);  // 1秒
            if (g_motion_state.fault_recovery_interval < 1) g_motion_state.fault_recovery_interval = 1;
        }
        if (g_motion_state.fault_recovery_wait[slv_num] >= g_motion_state.fault_recovery_interval) {
            g_motion_state.fault_recovery_attempts[slv_num]++;
            g_motion_state.fault_recovery_wait[slv_num] = 0;
            
            if (g_motion_state.fault_recovery_attempts[slv_num] <= 5) {  // 最多尝试5次
                printf("轴 %d: 故障恢复尝试 %d/5 - 发送清除故障命令\n", slv_num, g_motion_state.fault_recovery_attempts[slv_num]);
                // 发送清除故障命令
                motion_data[slv_num].control_word = 0x0080;
            } else {
                // 超过5次尝试，暂停一段时间再重试
                if (g_motion_state.fault_recovery_attempts[slv_num] == 6) {
                    printf("轴 %d: 故障恢复失败，暂停30秒后重试\n", slv_num);
                }
                
                // 暂停30秒
                static int fault_pause_interval = 0;  // 故障暂停间隔（控制周期数），首次调用时计算
                if (fault_pause_interval == 0) {
                    fault_pause_interval = control_cycles_for_time(30.0f);  // 30秒
                    if (fault_pause_interval < 1) fault_pause_interval = 1;
                }
                if (g_motion_state.fault_recovery_wait[slv_num] >= fault_pause_interval) {
                    g_motion_state.fault_recovery_attempts[slv_num] = 0;  // 重置尝试次数
                    printf("轴 %d: 重新开始故障恢复尝试\n", slv_num);
                }
            }
        }
        
        // 即使有故障，也继续执行上电逻辑，让power_state_machine处理故障清除
        // 不直接返回，给系统恢复的机会
    } else {
        // 故障已清除
        if (g_motion_state.last_fault_state[slv_num]) {
            printf("轴 %d: 故障已清除，恢复正常运行 (状态=0x%04X)\n", slv_num, motion_data[slv_num].status_word);
            g_motion_state.last_fault_state[slv_num] = 0;
        }
    }
    
    if(!motion_data[slv_num].is_power_on) {
        if (!g_motion_state.last_power_state[slv_num]) {
            printf("轴 %d: 伺服未上电，尝试上电 (状态=0x%04X)\n", slv_num, motion_data[slv_num].status_word);
            g_motion_state.last_power_state[slv_num] = 1;
        }
        // 不直接返回，让power_state_machine继续尝试上电
    } else {
        g_motion_state.last_power_state[slv_num] = 0;
    }

    return;
}

/**
 * 通用的往返运动逻辑（增量限制模式或往返模式）（私有函数）
 * @param slv_num 从站编号
 */
static void motion_common_motion_logic(int slv_num)
{
    // 如果启用了增量限制模式，使用增量控制逻辑
    if (pulses_per_control_cycle > 0) {
        // 在电机上电完成且准备开始运动时，保存初始位置
        if (!g_inc_limit_ctx.increment_limit_initialized[slv_num] && motion_data[slv_num].is_power_on) {
            g_inc_limit_ctx.initial_position_for_limit[slv_num] = motion_data[slv_num].actual_position;
            g_inc_limit_ctx.control_cycle_counter_for_limit[slv_num] = 0;
            g_inc_limit_ctx.current_phase[slv_num] = 0;
            g_inc_limit_ctx.increment_limit_initialized[slv_num] = 1;
            printf("轴 %d: 增量模式初始化 - 电机已上电，保存初始位置=%d\n", slv_num, g_inc_limit_ctx.initial_position_for_limit[slv_num]);
        }
        
        // 如果电机未上电，不计算目标位置
        if (!motion_data[slv_num].is_power_on) {
            return;
        }
        
        motion_data[slv_num].target_position = motion_calculate_incremental_target(slv_num);
        return;
    }
    
    // 原有的往返运动逻辑
    int32_t position_diff = motion_data[slv_num].target_position - motion_data[slv_num].actual_position;
    int32_t move_distance = max_diffPosition;
    
    if (motion_data[slv_num].is_first_in && abs(position_diff) < 10) {
        motion_data[slv_num].target_position = motion_data[slv_num].actual_position + move_distance;
        motion_data[slv_num].is_reverse = 0;
        motion_data[slv_num].is_first_in = 0;
        printf("轴 %d: 初始目标位置 = %d\n", slv_num, motion_data[slv_num].target_position);
        return;
    }
    
    if (abs(position_diff) < move_distance/100 && abs(position_diff) < 100) {
        if (!g_motion_state.waiting_for_reverse[slv_num]) {
            g_motion_state.waiting_for_reverse[slv_num] = 1;
            g_motion_state.wait_counter[slv_num] = 0;
            printf("轴 %d: 到达目标点，等待3秒后反向运动\n", slv_num);
        } else {
            g_motion_state.wait_counter[slv_num]++;
            static int reverse_wait_interval = 0;
            if (reverse_wait_interval == 0) {
                reverse_wait_interval = control_cycles_for_time(3.0f);
                if (reverse_wait_interval < 1) reverse_wait_interval = 1;
            }
            if (g_motion_state.wait_counter[slv_num] >= reverse_wait_interval) {
                int32_t new_target = motion_data[slv_num].is_reverse 
                    ? motion_data[slv_num].actual_position - move_distance
                    : motion_data[slv_num].actual_position + move_distance;
                motion_data[slv_num].target_position = new_target;
                motion_data[slv_num].is_reverse = !motion_data[slv_num].is_reverse;
                g_motion_state.waiting_for_reverse[slv_num] = 0;
                g_motion_state.wait_counter[slv_num] = 0;
                printf("轴 %d: 设置新目标 = %d\n", slv_num, new_target);
            }
        }
    } else {
        if (g_motion_state.waiting_for_reverse[slv_num]) {
            g_motion_state.waiting_for_reverse[slv_num] = 0;
            g_motion_state.wait_counter[slv_num] = 0;
        }
    }
}

/**
 * CSP模式运动控制 (-o 0)
 * @param slv_num 从站编号
 */
void motion_process_slave_motion_csp(int slv_num)
{
    // CSP模式：位置控制模式
    // 使用通用的往返运动逻辑
    motion_common_motion_logic(slv_num);
}

/**
 * CSV模式运动控制 (-o 1)
 * CSV模式使用速度控制，通过target_velocity (0x60FF) 设置目标速度
 * @param slv_num 从站编号
 */
void motion_process_slave_motion_csv(int slv_num)
{
    // CSV模式：速度控制模式
    // 如果电机未上电，不设置速度
    if (!motion_data[slv_num].is_power_on) {
        return;
    }
    
    // 如果未通过-v参数设置速度，使用默认值0（停止）
    if (target_velocity_cnt == 0) {
        motion_data[slv_num].target_velocity = 0;
        return;
    }
    
    // 直接使用输入的cnt值作为速度值（单位：cnt）
    // 不进行编码器分辨率转换，直接写入PDO
    motion_data[slv_num].target_velocity = target_velocity_cnt;
    
    // 首次设置速度时打印信息
    static int csv_velocity_initialized[MAX_SLAVE_NUMBER] = {0};
    if (!csv_velocity_initialized[slv_num]) {
        printf("轴 %d: CSV模式 - 设置目标速度 = %d cnt\n", 
               slv_num, target_velocity_cnt);
        csv_velocity_initialized[slv_num] = 1;
    }
}

/**
 * CST模式运动控制 (-o 2 或 -o 7/8 -t)
 * @param slv_num 从站编号
 */
void motion_process_slave_motion_cst(int slv_num)
{
    // CST模式：力位混合控制（位置闭环 + 力矩输出）
    // 如果电机未上电，不设置转矩
    if (!motion_data[slv_num].is_power_on) {
        return;
    }
    
    // 这里约定：motion_data[slv_num].target_position 由 q 字段映射而来，
    // 直接表示“绝对期望位置”（单位：编码器计数），不再作为相对增量使用。
    int32_t pos_desired_cnt = motion_data[slv_num].target_position;
    // 如果期望位置等于当前实际位置，则目标转矩为0（保持当前位置，无力矩输出）
    if (pos_desired_cnt == motion_data[slv_num].actual_position) {
        motion_data[slv_num].target_torque = 0;
        return;
    }
    
    // 首次进入CST模式时，记录初始位置，主要用于调试打印
    if (!g_cst_initialized[slv_num]) {
        g_cst_initial_position[slv_num] = motion_data[slv_num].actual_position;
        g_cst_initialized[slv_num] = 1;
    }
    
    // 当前位置（编码器cnt）
    int32_t pos_current_cnt = motion_data[slv_num].actual_position;
    
    // 当前位置、期望位置转换为角度（度）：cnt / 131072 * 360
    double pos_current_deg = (double)pos_current_cnt / 131072.0 * 360.0;
    double pos_desired_deg = (double)pos_desired_cnt / 131072.0 * 360.0;
    
    // 角度转弧度
    double pos_current_rad = pos_current_deg * (M_PI / 180.0);
    double pos_desired_rad = pos_desired_deg * (M_PI / 180.0);
    
    // 速度：实际速度为编码器cnt/s，简单近似直接使用 actual_velocity（cnt/s）
    double spd_current = (double)motion_data[slv_num].actual_velocity;
    // 期望速度这里简单取0（静止到目标位置），可根据需要扩展
    double spd_desired = 0.0;
    
    // 控制参数
    const double KP = 10.0;
    const double KD = 0.0;
    const double Torque = 0.0;
    const double Kt = 10.0;
    
    // motor_current = (KP*(pos_desired - pos_current) + KD*(spd_desired - spd_current) + Torque) / Kt
    double pos_error_rad = (pos_desired_rad - pos_current_rad);
    double motor_current = (KP * pos_error_rad
                           + KD * (spd_desired - spd_current)
                           + Torque) / Kt;
    
    // 将计算结果映射到驱动器的目标转矩内部单位
    // motor_current 目前量级较小，这里放大一个系数以避免被截断为0
    double torque_cmd = motor_current * 2000.0;  // 缩放系数，可根据现场需要调整
    
    // 当位置误差足够小(接近目标)时，将目标转矩强制置0，避免到点仍有残余力矩
    if (fabs(pos_error_rad) < 0.001) { // 约等于 <0.057 度
        torque_cmd = 0.0;
    }
    if (torque_cmd > 200.0) torque_cmd = 200.0;
    if (torque_cmd < -200.0) torque_cmd = -200.0;
    motion_data[slv_num].target_torque = (short)torque_cmd;
    
    // 首次设置转矩时打印信息（调试用），体现绝对位置 + 力矩输出的混合控制逻辑
    if (!g_cst_first_printed[slv_num]) {
        printf("轴 %d: CST模式 - 初始位置=%d, 期望绝对位置=%d, motor_current=%.3f, 计算目标转矩 = %d\n", 
               slv_num,
               g_cst_initial_position[slv_num],
               pos_desired_cnt,
               motor_current,
               motion_data[slv_num].target_torque);
        g_cst_first_printed[slv_num] = 1;
    }
}

// 轨迹生成函数已移至 trajectory_planning 模块

/**
 * CSP+前馈模式运动控制 (-o 3)
 * @param slv_num 从站编号
 */
void motion_process_slave_motion_csp_vff(int slv_num)
{
    // CSP+前馈模式：位置控制模式 + 速度前馈
    // 使用通用的往返运动逻辑
    // 注意：速度前馈值在motion_update_pdo_data中通过velocity_feedforward写入
    motion_common_motion_logic(slv_num);
}

/**
 * 一阶梯形加减速CSP+前馈模式运动控制 (-o 7/8, -s 1)
 * 使用一阶梯形加减速算法生成平滑轨迹（梯形速度曲线）
 * @param slv_num 从站编号
 */
void motion_process_slave_motion_csp_trapezoidal(int slv_num)
{
    // 如果电机未上电，不执行运动
    if (!motion_data[slv_num].is_power_on) {
        return;
    }
    
    // 初始化轨迹（仅在首次上电时）
    if (!g_s_curve_traj[slv_num].initialized) {
        g_s_curve_traj[slv_num].initial_position = motion_data[slv_num].actual_position;
        g_s_curve_traj[slv_num].current_phase = 0;  // 0=正向，1=反向
        
        // 生成正向轨迹
        int total_distance = max_diffPosition;  // -p参数
        int avg_increment = pulses_per_control_cycle;  // -i参数，作为平均基础阶跃参考值
        
        if (trajectory_planning_generate_1s_curve(slv_num, total_distance, avg_increment) < 0) {
            printf("轴 %d: 一阶梯形加减速轨迹生成失败，使用默认逻辑\n", slv_num);
            motion_common_motion_logic(slv_num);
            return;
        }
        
        g_s_curve_traj[slv_num].initialized = 1;
        g_s_curve_traj[slv_num].current_index = 0;
        printf("轴 %d: 一阶梯形加减速轨迹初始化完成，初始位置=%d，总位移=%d\n",
               slv_num, g_s_curve_traj[slv_num].initial_position, total_distance);
    }
    
    // 从轨迹数组中读取当前周期的目标位置
    if (g_s_curve_traj[slv_num].current_index < g_s_curve_traj[slv_num].total_points) {
        motion_data[slv_num].target_position = g_s_curve_traj[slv_num].trajectory_points[g_s_curve_traj[slv_num].current_index];
        g_s_curve_traj[slv_num].current_index++;
    } else {
        // 轨迹执行完成，等待实际位置到达目标位置后再切换
        int32_t final_target = g_s_curve_traj[slv_num].trajectory_points[g_s_curve_traj[slv_num].total_points - 1];
        int32_t position_error = abs(motion_data[slv_num].actual_position - final_target);
        
        // 如果位置误差小于阈值（例如10个脉冲），则认为到达目标，可以切换
        // 注意：不要设置target_position = final_target，避免换向时target_position突变导致驱动器状态字变化
        // 直接使用actual_position作为新轨迹的起点，确保target_position不会突变
        if (position_error < 10) {
            // 轨迹执行完成，切换到反向或重新开始
            // 使用actual_position作为新轨迹的起点，避免target_position突变导致驱动器状态字变化
            // 这样可以确保换向时target_position不会突变，避免"关闸/开闸"的声音
            int32_t switch_position = motion_data[slv_num].actual_position;
            
            if (g_s_curve_traj[slv_num].current_phase == 0) {
                // 正向完成，切换到反向
                g_s_curve_traj[slv_num].current_phase = 1;
                g_s_curve_traj[slv_num].initial_position = switch_position;
                
                // 生成反向轨迹
                int total_distance = max_diffPosition;
                int avg_increment = pulses_per_control_cycle;
                
                if (trajectory_planning_generate_1s_curve(slv_num, -total_distance, avg_increment) < 0) {
                    printf("轴 %d: 反向轨迹生成失败\n", slv_num);
                    return;
                }
                
                g_s_curve_traj[slv_num].current_index = 0;
                // 确保新轨迹的第一个点等于switch_position，避免位置突变
                g_s_curve_traj[slv_num].trajectory_points[0] = switch_position;
                printf("轴 %d: 切换到反向轨迹，初始位置=%d，第一个目标位置=%d\n", 
                       slv_num, switch_position, 
                       g_s_curve_traj[slv_num].trajectory_points[0]);
            } else {
                // 反向完成，重新开始正向
                g_s_curve_traj[slv_num].current_phase = 0;
                g_s_curve_traj[slv_num].initial_position = switch_position;
                
                // 生成正向轨迹
                int total_distance = max_diffPosition;
                int avg_increment = pulses_per_control_cycle;
                
                if (trajectory_planning_generate_1s_curve(slv_num, total_distance, avg_increment) < 0) {
                    printf("轴 %d: 正向轨迹生成失败\n", slv_num);
                    return;
                }
                
                g_s_curve_traj[slv_num].current_index = 0;
                // 确保新轨迹的第一个点等于switch_position，避免位置突变
                g_s_curve_traj[slv_num].trajectory_points[0] = switch_position;
                printf("轴 %d: 重新开始正向轨迹，初始位置=%d，第一个目标位置=%d\n", 
                       slv_num, switch_position,
                       g_s_curve_traj[slv_num].trajectory_points[0]);
            }
            
            // 设置当前周期的目标位置为actual_position，避免target_position突变导致驱动器状态字变化
            // 这样可以确保换向时target_position不会突变，避免"关闸/开闸"的声音
            motion_data[slv_num].target_position = motion_data[slv_num].actual_position;
            if (g_s_curve_traj[slv_num].current_index < g_s_curve_traj[slv_num].total_points) {
                g_s_curve_traj[slv_num].current_index++;
            }
        }
        // 如果还未到达目标位置，继续保持在最后一个轨迹点，等待到达
    }
}

/**
 * 二阶S曲线CSP+前馈模式运动控制 (-o 7/8, -s 2)
 * 使用二阶S曲线加减速算法生成平滑轨迹（加速度线性变化）
 * @param slv_num 从站编号
 */
void motion_process_slave_motion_csp_2s(int slv_num)
{
    // 如果电机未上电，不执行运动
    if (!motion_data[slv_num].is_power_on) {
        return;
    }
    
    // 初始化轨迹（仅在首次上电时）
    if (!g_s_curve_traj[slv_num].initialized) {
        g_s_curve_traj[slv_num].initial_position = motion_data[slv_num].actual_position;
        g_s_curve_traj[slv_num].current_phase = 0;  // 0=正向，1=反向
        
        // 生成正向轨迹
        int total_distance = max_diffPosition;  // -p参数
        int avg_increment = pulses_per_control_cycle;  // -i参数，作为平均基础阶跃参考值
        
        if (trajectory_planning_generate_2s_curve(slv_num, total_distance, avg_increment) < 0) {
            printf("轴 %d: 二阶S曲线轨迹生成失败，使用默认逻辑\n", slv_num);
            motion_common_motion_logic(slv_num);
            return;
        }
        
        g_s_curve_traj[slv_num].initialized = 1;
        g_s_curve_traj[slv_num].current_index = 0;
        printf("轴 %d: 二阶S曲线轨迹初始化完成，初始位置=%d，总位移=%d\n",
               slv_num, g_s_curve_traj[slv_num].initial_position, total_distance);
    }
    
    // 从轨迹数组中读取当前周期的目标位置
    if (g_s_curve_traj[slv_num].current_index < g_s_curve_traj[slv_num].total_points) {
        motion_data[slv_num].target_position = g_s_curve_traj[slv_num].trajectory_points[g_s_curve_traj[slv_num].current_index];
        g_s_curve_traj[slv_num].current_index++;
    } else {
        // 轨迹执行完成，等待实际位置到达目标位置后再切换
        int32_t final_target = g_s_curve_traj[slv_num].trajectory_points[g_s_curve_traj[slv_num].total_points - 1];
        int32_t position_error = abs(motion_data[slv_num].actual_position - final_target);
        
        // 如果位置误差小于阈值（例如10个脉冲），则认为到达目标，可以切换
        // 注意：不要设置target_position = final_target，避免换向时target_position突变导致驱动器状态字变化
        // 直接使用actual_position作为新轨迹的起点，确保target_position不会突变
        if (position_error < 10) {
            // 轨迹执行完成，切换到反向或重新开始
            // 使用actual_position作为新轨迹的起点，避免target_position突变导致驱动器状态字变化
            // 这样可以确保换向时target_position不会突变，避免"关闸/开闸"的声音
            int32_t switch_position = motion_data[slv_num].actual_position;
            
            if (g_s_curve_traj[slv_num].current_phase == 0) {
                // 正向完成，切换到反向
                g_s_curve_traj[slv_num].current_phase = 1;
                g_s_curve_traj[slv_num].initial_position = switch_position;
                
                // 生成反向轨迹
                int total_distance = max_diffPosition;
                int avg_increment = pulses_per_control_cycle;
                
                if (trajectory_planning_generate_2s_curve(slv_num, -total_distance, avg_increment) < 0) {
                    printf("轴 %d: 反向轨迹生成失败\n", slv_num);
                    return;
                }
                
                g_s_curve_traj[slv_num].current_index = 0;
                // 确保新轨迹的第一个点等于switch_position，避免位置突变
                g_s_curve_traj[slv_num].trajectory_points[0] = switch_position;
                printf("轴 %d: 切换到反向轨迹，初始位置=%d，第一个目标位置=%d\n", 
                       slv_num, switch_position, 
                       g_s_curve_traj[slv_num].trajectory_points[0]);
            } else {
                // 反向完成，重新开始正向
                g_s_curve_traj[slv_num].current_phase = 0;
                g_s_curve_traj[slv_num].initial_position = switch_position;
                
                // 生成正向轨迹
                int total_distance = max_diffPosition;
                int avg_increment = pulses_per_control_cycle;
                
                if (trajectory_planning_generate_2s_curve(slv_num, total_distance, avg_increment) < 0) {
                    printf("轴 %d: 正向轨迹生成失败\n", slv_num);
                    return;
                }
                
                g_s_curve_traj[slv_num].current_index = 0;
                // 确保新轨迹的第一个点等于switch_position，避免位置突变
                g_s_curve_traj[slv_num].trajectory_points[0] = switch_position;
                printf("轴 %d: 重新开始正向轨迹，初始位置=%d，第一个目标位置=%d\n", 
                       slv_num, switch_position,
                       g_s_curve_traj[slv_num].trajectory_points[0]);
            }
            
            // 设置当前周期的目标位置为actual_position，避免target_position突变导致驱动器状态字变化
            // 这样可以确保换向时target_position不会突变，避免"关闸/开闸"的声音
            motion_data[slv_num].target_position = motion_data[slv_num].actual_position;
            if (g_s_curve_traj[slv_num].current_index < g_s_curve_traj[slv_num].total_points) {
                g_s_curve_traj[slv_num].current_index++;
            }
        }
        // 如果还未到达目标位置，继续保持在最后一个轨迹点，等待到达
    }
}

/**
 * 自定义模式运动控制 (-o 7/8)
 * @param slv_num 从站编号
 */
void motion_process_slave_motion_custom(int slv_num)
{
    // 自定义模式：使用RW模板，可以自定义PDO映射
    // 使用通用的往返运动逻辑
    motion_common_motion_logic(slv_num);
}

/**
 * 处理单个从站的位置检测和运动逻辑（路由函数）
 * 先通过operation_mode判断CSP/CSV/CST，PDO模板固定为0x1607/0x1A07
 * @param slv_num 从站编号
 */
void motion_process_slave_motion(int slv_num)
{
    // 先通过operation_mode判断运行模式
    unsigned char operation_mode = motion_data[slv_num].control_mode;
    
    switch (operation_mode) {
        case 3:  // CSV模式 (Cyclic Synchronous Velocity)
            motion_process_slave_motion_csv(slv_num);
            break;
            
        case 4:  // CST模式 (Cyclic Synchronous Torque)
            motion_process_slave_motion_cst(slv_num);
            break;
            
        case 8:  // CSP模式 (Cyclic Synchronous Position)
            // PDO模板固定为0x1607/0x1A07，根据s_curve_mode选择不同的轨迹规划算法
            switch (s_curve_mode) {
                case 0:
                    // 折线往返增量模式（三角形）- 原来的模式
                    motion_process_slave_motion_custom(slv_num);
                    break;
                case 1:
                    // 一阶梯形加减速（梯形速度曲线）
                    motion_process_slave_motion_csp_trapezoidal(slv_num);
                    break;
                case 2:
                    // 二阶S曲线CSP+前馈模式（加速度线性变化）
                    motion_process_slave_motion_csp_2s(slv_num);
                    break;
                default:
                    // 默认使用折线模式
                    motion_process_slave_motion_custom(slv_num);
                    break;
            }
            break;
            
        default:
            // 其他运行模式或未知模式，使用自定义模式
            printf("轴 %d: 警告 - 未知运行模式 %d，使用自定义模式\n", slv_num, operation_mode);
            motion_process_slave_motion_custom(slv_num);
            break;
    }
}
