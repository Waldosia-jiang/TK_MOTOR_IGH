#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <getopt.h>
#if USE_GNU_THREAD
#define __USE_GNU
#endif
#include <sched.h>
#include <pthread.h>

#include "ecrt.h"
#include "igh_example.h"
#include "dc.h"
#include "motion.h"
#include "pdo_set.h"

static unsigned int vendor_id[MAX_SLAVE_NUMBER];        // 从站厂商ID
static unsigned int product_code[MAX_SLAVE_NUMBER];     // 从站产品代码
static int is_servo_slave[MAX_SLAVE_NUMBER];            // 是否为伺服从站
static ec_slave_config_t *slave_configs[MAX_SLAVE_NUMBER] = {NULL};  // 从站配置指针数组
static ec_master_t *master = NULL;                      // EtherCAT主站
static ec_domain_t *domain = NULL;                      // 域
static uint8_t *p_domain = NULL;                        // 域数据指针
static pthread_t cyclic_thread;                         // 周期任务线程
static int run = 0;                                     // 运行标志
unsigned int slave_count;                               // 从站数量
struct pdo_offset offset[MAX_SLAVE_NUMBER];             // PDO偏移量
static int is_move = 0;                                 // 是否运动
static int max_diffPosition = 0;                        // 最大往返距离
static int g_cpuid = -1;                                // CPU ID（-1表示使用默认：最后一个CPU）
static unsigned int g_cycletime_ns = 1000 * 1000;     // EtherCAT周期时间(ns)，通信层周期（决定通信频率），1000 * 1000(ns) = 1000us = 1ms
unsigned int g_control_cycle_time_us = 1000;          // 控制周期时间(us)，控制层周期（决定控制频率，必须>=EtherCAT周期）
int pulses_per_control_cycle = 0;                        // 每个控制周期最大增量脉冲数，0表示不限制（外部变量，供motion.c使用）
int pdo_choice = 7;                                      // PDO模板选择：固定为7（0x1607/0x1A07，RW模板）
int target_velocity_cnt = 0;                             // CSV模式目标速度（cnt），通过-v参数设置（外部变量，供motion.c使用）
int target_torque = 65536;                               // CST模式目标转矩，ROS2默认等效于 demo 的 -t 65536
int s_curve_mode = 0;                                    // S曲线模式：0=折线往返增量模式（三角形），1=一阶梯形，2=二阶S曲线（外部变量，供motion.c使用）
extern struct motion_control_data motion_data[MAX_SLAVE_NUMBER];  // 运动控制数据

const struct slave_id servo_id[] = {
    //泰科伺服
    {0x00000c48, 0x00030924},
    {0x00000c19, 0x00030924},
    {0x0000009A, 0x00030924}, 
    
};

/**
 * 判断是否为伺服从站（私有函数）
 * @param v_id 厂商ID
 * @param p_code 产品代码
 * @return 1表示是伺服从站，0表示不是
 */
static int example_is_servo(unsigned int v_id, unsigned p_code)
{
    int slave_number = sizeof(servo_id) / sizeof(struct slave_id);
    int i = 0;
    for(i = 0; i < slave_number; ++i)
    {
        if(v_id == servo_id[i].vendor_id && 
            p_code == servo_id[i].product_code)
        return 1;
    }

    return 0;
}

/**
 * 解析命令行参数
 * @param argc 参数个数
 * @param argv 参数数组
 * @param cpuid CPU ID输出参数
 * @param dc_method 分布式时钟方法输出参数
 * @param cycletime_ns 周期时间输出参数
 * @return 0表示成功，-1表示失败（需要退出）
 */
int example_parse_arguments(int argc, char **argv, int *cpuid, int *dc_method, unsigned int *cycletime_ns)
{
    int ch = 0;
    
    // 周期时间固定为1ms，直接使用全局变量
    *cycletime_ns = g_cycletime_ns;
    
    is_move = 0;
    
    while((ch = getopt(argc, argv, "c:d:p:i:v:t:s:h")) != -1)
    {
                switch(ch)
        {
        case 'c':
            *cpuid = atoi(optarg);
            printf("设置任务到cpu%d\n", *cpuid);
            break;
        case 'd':
            *dc_method = atoi(optarg);
            printf("分布式时钟方法 %d\n", *dc_method);
            if(*dc_method == 0)
                printf("设置从站时钟作为参考时钟\n");
            else if(*dc_method == 1)
                printf("设置主时钟作为参考时钟\n");
            else {
                printf("错误的分布式时钟方法，设置默认值0！\n");
                *dc_method = 0;
            }
            break;
        case 'p':
            is_move = 1;  // 使用-p参数时也要启用运动
            max_diffPosition = atoi(optarg);
            if(max_diffPosition < 1000)
                max_diffPosition = 1000;
            else if(max_diffPosition > 10000000)
                max_diffPosition = 10000000;
            printf("设置CSP模式位移距离: %d\n", max_diffPosition);
            break;
        case 'i':
            pulses_per_control_cycle = atoi(optarg);
            if(pulses_per_control_cycle < 0)
                pulses_per_control_cycle = 0;
            else if(pulses_per_control_cycle > 100000)
                pulses_per_control_cycle = 100000;
            if(pulses_per_control_cycle > 0) {
                printf("启用增量限制模式: 每个控制周期最大增量 %d 个脉冲 (控制周期时间根据control_frequency_hz动态计算)\n", 
                       pulses_per_control_cycle);
            } else {
                printf("禁用增量限制模式\n");
            }
            break;
        case 'v':
            target_velocity_cnt = atoi(optarg);
            if (target_velocity_cnt < -10000000)
                target_velocity_cnt = -10000000;
            else if (target_velocity_cnt > 10000000)
                target_velocity_cnt = 10000000;
            printf("设置CSV模式目标速度: %d cnt\n", target_velocity_cnt);
            is_move = 1;  // 使用-v参数时自动启用运动控制
            break;
        case 't':
            target_torque = atoi(optarg);
            printf("设置CST模式位置增量: %d (cnt)\n", target_torque);
            is_move = 1;  // 使用-t参数时自动启用运动控制
            break;
        case 's':
            s_curve_mode = atoi(optarg);
            if (s_curve_mode < 0)
                s_curve_mode = 0;
            else if (s_curve_mode > 2)
                s_curve_mode = 2;
            if (s_curve_mode == 0) {
                printf("S曲线模式: 0 (折线往返增量模式-三角形)\n");
            } else if (s_curve_mode == 1) {
                printf("S曲线模式: 1 (一阶梯形加减速)\n");
            } else if (s_curve_mode == 2) {
                printf("S曲线模式: 2 (二阶S曲线)\n");
            }
            break;
        case 'h':
            printf("    -c 设置任务的CPU ID\n");
            printf("    -d 分布式时钟方法，0:从站参考时钟 1:主站参考时钟\n");
            printf("    -p 开始运动并设置CSP模式位移距离\n");
            printf("    -i 设置每个控制周期最大增量脉冲数 (0=禁用增量限制)\n");
            printf("    -v 设置CSV模式目标速度 (cnt, -10000000到10000000)\n");
            printf("    -t 设置CST模式位置增量 (cnt)\n");
            printf("    -s S曲线模式(0-2): 0=折线往返增量(三角形), 1=一阶梯形, 2=二阶S曲线 (默认0)\n");
            printf("    注意: PDO模板固定为0x1607/0x1A07 (RW模板CSP模式)\n");
            return -1;  // 需要退出
        default:
            printf("错误参数:%c\n", ch);
            return -1;
        }
    }
    
    memset(offset, 0xff, sizeof(offset));
    motion_init(max_diffPosition);
    
    return 0;
}

/**
 * 初始化EtherCAT主站和域
 * @param master 主站指针输出参数
 * @param domain 域指针输出参数
 * @param master_info 主站信息输出参数
 * @return 0表示成功，-1表示失败
 */
int example_init_ethercat_master(ec_master_t **master, ec_domain_t **domain, ec_master_info_t *master_info)
{
    // 检查EtherCAT主站状态，如果异常则尝试重新初始化
    printf("检查EtherCAT主站状态...\n");
    ec_master_t *temp_master = ecrt_request_master(0);
    if (temp_master) {
        ec_master_info_t temp_info;
        if (ecrt_master(temp_master, &temp_info) == 0) {
            printf("检测到EtherCAT主站处于活动状态，尝试释放并重新初始化...\n");
            ecrt_release_master(temp_master);
            usleep(500000);  // 等待500ms确保完全释放
        }
    }

    /* 请求EtherCAT主站 */
    *master = ecrt_request_master(0);
    if (!*master) {
        printf("ecrt_request_master失败！\n");
        return -1;
    }

    /* 获取从站在线状态 */
    ecrt_master(*master, master_info);
    slave_count = master_info->slave_count;
    if(!slave_count)
    {
        printf("没有扫描到从站，退出。\n");
        return -1;
    }

    /* 为PDO交换创建域。 */
    *domain = ecrt_master_create_domain(*master);
    if (!*domain) {
        printf("ecrt_master_create_domain失败！\n");
        return -1;
    }

    return 0;
}

/**
 * 信号处理函数，处理Ctrl+C信号（私有函数）
 * @param sig 信号编号
 */
static void example_catch_signal(int sig) {
    printf("接收到信号 %d，开始安全退出...\n", sig);
    
    // 1. 停止运行标志
    run = 0;
    
    // 2. 等待周期任务线程结束
    printf("等待周期任务线程结束...\n");
    if (pthread_join(cyclic_thread, NULL) != 0) {
        printf("警告：周期任务线程等待失败\n");
    }
    
    // 3. 安全复位所有伺服状态
    printf("复位伺服状态...\n");
    if (master && domain && p_domain) {
        // 发送一次通信，确保能写入复位命令
        ecrt_master_receive(master);
        ecrt_domain_process(domain);
        
        // 复位所有伺服从站
        for (int i = 0; i < slave_count; i++) {
            if (is_servo_slave[i]) {
                printf("复位从站 %d 状态...\n", i);
                
                // 发送断电命令
                if (offset[i].control_word < 0x0000ffff) {
                    EC_WRITE_U16(p_domain + offset[i].control_word, 0x0000);  // 断电
                }
            }
        }
        
        // 更新PDO数据（退出清理）
        motion_update_pdo_data(p_domain, offset, slave_count);
        ecrt_domain_queue(domain);
        
        // 发送一次通信
        ecrt_master_send(master);
        
        // 等待一段时间确保命令发送完成
        usleep(100000);  // 等待100ms
        
        // 再次发送断电命令确保执行
        ecrt_master_receive(master);
        ecrt_domain_process(domain);
        ecrt_domain_queue(domain);
        ecrt_master_send(master);
    }
    
    // 4. 释放EtherCAT主站
    printf("释放EtherCAT主站...\n");
    if (master) {
        ecrt_release_master(master);
    }
    
    printf("安全退出完成\n");
    exit(0);
}

/**
 * 周期任务函数
 * @param arg 线程参数
 * @return NULL
 */
static void* example_cyclic_task(void *arg)
{
    int index = 0, counter = 0;
    uint64_t t_send_start, t_send_end;
    uint64_t t_receive_start, t_receive_end;

    // 设置CPU亲和性：使用-c参数指定的CPU，如果未指定则使用最后一个CPU
    cpu_set_t mask;
    CPU_ZERO(&mask);
    int target_cpu = g_cpuid;
    if (target_cpu < 0) {
        // 未指定CPU，使用最后一个CPU
        target_cpu = sysconf(_SC_NPROCESSORS_CONF) - 1;
    }
    // 检查CPU ID是否有效
    int max_cpu = sysconf(_SC_NPROCESSORS_CONF);
    if (target_cpu >= max_cpu) {
        printf("警告：CPU ID %d 无效，系统只有 %d 个CPU核心，使用最后一个CPU\n", target_cpu, max_cpu);
        target_cpu = max_cpu - 1;
    }
    CPU_SET(target_cpu, &mask);
    if (sched_setaffinity(0, sizeof(mask), &mask)) {
        printf("设置CPU亲和性失败，错误：%s\n", strerror(errno));
    } else {
        printf("周期任务线程已绑定到CPU %d\n", target_cpu);
    }

    struct sched_param schedp;
    memset(&schedp, 0, sizeof(schedp));
    schedp.sched_priority = 90;
    if(sched_setscheduler(0, SCHED_FIFO, &schedp))
        printf("设置调度器失败\n");
    
    dc_cal_1st_sleep_time();
    
    // 控制周期配置：根据g_cycletime_ns和g_control_cycle_time_us计算每个控制周期需要多少个EtherCAT周期
    unsigned int ethercat_frequency_hz = 1000000000 / g_cycletime_ns;  // EtherCAT频率(Hz)
    unsigned int control_cycle_time_ns = g_control_cycle_time_us * 1000;  // 控制周期时间(纳秒)
    
    // 计算每个控制周期需要多少个EtherCAT周期
    int ethercat_cycles_per_control = control_cycle_time_ns / g_cycletime_ns;
    if (ethercat_cycles_per_control < 1) {
        ethercat_cycles_per_control = 1;
    }
    
    // 根据实际计算值更新控制周期时间（确保是EtherCAT周期的整数倍）
    g_control_cycle_time_us = (g_cycletime_ns * ethercat_cycles_per_control) / 1000;
    
    // 计算实际控制频率
    unsigned int control_frequency_hz = 1000000 / g_control_cycle_time_us;
    
    printf("控制周期配置: EtherCAT周期=%u ns (%d Hz), 控制周期=%u us (%d Hz), 每 %d 个EtherCAT周期执行一次控制\n",
           g_cycletime_ns, ethercat_frequency_hz, g_control_cycle_time_us, control_frequency_hz, ethercat_cycles_per_control);
    
    // 控制周期累积计数器
    static int control_cycle_counter = 0;
    // CST(-t)预备阶段：先模式8保持当前位置，待所有轴就绪(0x37)后再切到模式4
    static int cst_prestage_active = 0;
    static int cst_prestage_done = 0;
    // ROS2 预备阶段：必须先模式8、所有电机状态低8位=0x37 后，才响应 /low_cmd 中的 mode/q/tau 等
    static int ros2_prep_done = 0;

    while(run) {
        dc_wait_period();

        t_receive_start = dc_sys_time_ns();
        ecrt_master_receive(master);
        t_receive_end = dc_sys_time_ns();

        /********************************************************/
        // ========== 通信层：每个EtherCAT周期都必须执行 ==========
        //处理PDO数据
        ecrt_domain_process(domain);
        dc_sync_distributed_clocks();

        motion_process_pdo_data(p_domain, offset, slave_count, domain);
        
        // 更新调试周期计数器
        motion_debug_update_cycle();
        
        // ========== 控制层：累积执行，根据control_frequency_hz频率 ==========
        // 状态机逻辑和运动控制算法都在控制周期执行，降低CPU占用
        int is_control_cycle = 0;  // 标记是否为控制周期
        control_cycle_counter++;
        if (control_cycle_counter >= ethercat_cycles_per_control) {
            control_cycle_counter = 0;  // 重置计数器
            is_control_cycle = 1;       // 标记为控制周期
            
            // 检查域状态，如果通信异常则不更新状态机
            ec_domain_state_t domain_state;
            int comm_ok = 1;  // 通信是否正常
            if (ecrt_domain_state(domain, &domain_state) == 0) {
                // 如果 working counter 状态不是完整，说明通信异常
                if (domain_state.wc_state != EC_WC_COMPLETE) {
                    comm_ok = 0;
                    // 只在首次检测到通信异常时打印一次
                    static int last_comm_error_printed = 0;
                    if (!last_comm_error_printed) {
                        printf("⚠️ 通信异常 - Working counter状态=%d, 值=%u，暂停状态机更新\n", 
                               domain_state.wc_state, domain_state.working_counter);
                        last_comm_error_printed = 1;
                    }
                } else {
                    static int last_comm_error_printed = 0;
                    if (last_comm_error_printed) {
                        printf("✅ 通信恢复 - Working counter正常\n");
                        last_comm_error_printed = 0;
                    }
                }
            }
            
            // 控制逻辑（按控制周期执行）
            // 若未处于-t模式，则复位预备阶段状态
            if (!is_move || target_torque == 0) {
                cst_prestage_active = 0;
                cst_prestage_done = 0;
            }

            // 如果使用 -t（CST）且预备阶段未完成，则启动预备阶段（只启动一次）
            if (is_move && target_torque != 0 && !cst_prestage_done && !cst_prestage_active) {
                cst_prestage_active = 1;
                printf("CST预备阶段: 先切换所有轴到模式8并保持当前位置，等待所有轴状态低8位=0x37后进入CST正常控制\n");
            }

            // 计算是否所有伺服轴已就绪(状态字低8位=0x37)（用于 standalone CST 或 ROS2 预备阶段）
            int all_servo_ready_0x37 = 1;
            if (is_move && target_torque != 0 && cst_prestage_active && !cst_prestage_done) {
                for (int i = 0; i < slave_count; ++i) {
                    if (is_servo_slave[i]) {
                        if ( ((uint8_t)(motion_data[i].status_word & 0x00FF)) != 0x37 ) {
                            all_servo_ready_0x37 = 0;
                            break;
                        }
                    }
                }
            } else if (!ros2_prep_done) {
                for (int i = 0; i < slave_count; ++i) {
                    if (is_servo_slave[i] && ((uint8_t)(motion_data[i].status_word & 0x00FF)) != 0x37) {
                        all_servo_ready_0x37 = 0;
                        break;
                    }
                }
            } else {
                all_servo_ready_0x37 = 0;
            }

            if (!ros2_prep_done) {
                // ROS2 预备阶段：强制模式8、目标位置=当前位置，不响应 /low_cmd 的 mode/q/tau
                static int ros2_prep_once = 0;
                if (!ros2_prep_once) {
                    ros2_prep_once = 1;
                    printf("ROS2 预备阶段: 模式8保持当前位置，等待所有电机状态低8位=0x37 后再响应 /low_cmd。\n");
                }
                for (index = 0; index < slave_count; ++index) {
                    if (is_servo_slave[index]) {
                        motion_data[index].target_position = motion_data[index].actual_position;
                        motion_set_operation_mode(index, 8);
                        if (comm_ok) {
                            motion_power_state_machine(index, 1);
                            motion_fault_recovery(index);
                        }
                        motion_process_slave_motion(index);
                    }
                }
                if (comm_ok && all_servo_ready_0x37) {
                    ros2_prep_done = 1;
                    printf("ROS2 预备阶段完成: 所有电机 0x37，开始响应 /low_cmd 中的 mode/q/tau。\n");
                    for (int i = 0; i < slave_count; ++i) {
                        if (is_servo_slave[i]) {
                            motion_cst_reset(i);
                        }
                    }
                }
            } else {
                // 预备阶段完成后：运行模式与目标由 /low_cmd（如 cst_toggle.sh）中的 mode/q/tau 决定
                for (index = 0; index < slave_count; ++index) {
                    if (is_servo_slave[index]) {
                        unsigned char operation_mode = motion_data[index].control_mode;
                        motion_set_operation_mode(index, operation_mode);
                        if (comm_ok) {
                            motion_power_state_machine(index, 1);
                            motion_fault_recovery(index);
                        }
                        motion_process_slave_motion(index);
                    }
                }
            }

            // 若全部轴就绪且通信正常，则结束 standalone CST 预备阶段
            if (is_move && target_torque != 0 && !cst_prestage_done && cst_prestage_active && comm_ok && all_servo_ready_0x37) {
                printf("CST预备阶段完成: 所有轴状态低8位=0x37，切换到CST正常控制\n");
                for (int i = 0; i < slave_count; ++i) {
                    if (is_servo_slave[i]) {
                        motion_cst_reset(i);
                    }
                }
                cst_prestage_active = 0;
                cst_prestage_done = 1;
            }
        }
        
        // 按轴号顺序输出统一的调试信息（每2秒打印一次）
        static uint64_t last_debug_time_ns = 0;
        uint64_t current_time_ns = dc_sys_time_ns();
        if (last_debug_time_ns == 0) {
            last_debug_time_ns = current_time_ns;
        }
        if (current_time_ns - last_debug_time_ns >= 2000000000ULL) {  // 2秒 = 2000000000纳秒
            last_debug_time_ns = current_time_ns;
            printf("\n=== 统一调试信息 ===\n");
            for(int i = 0; i < slave_count; ++i) {
                if(is_servo_slave[i]) {
                    // 解析状态字
                    char status_str[100] = "";
                    if (motion_data[i].status_word & 0x0001) strcat(status_str, "就绪 ");
                    if (motion_data[i].status_word & 0x0002) strcat(status_str, "已开启 ");
                    if (motion_data[i].status_word & 0x0004) strcat(status_str, "运行使能 ");
                    if (motion_data[i].status_word & 0x0008) strcat(status_str, "故障 ");
                    if (motion_data[i].status_word & 0x0010) strcat(status_str, "电压使能 ");
                    if (motion_data[i].status_word & 0x0020) strcat(status_str, "快速停止 ");
                    if (motion_data[i].status_word & 0x0040) strcat(status_str, "开启禁用 ");
                    if (motion_data[i].status_word & 0x0200) strcat(status_str, "目标到达 ");
                    
                    printf("轴 %d: 状态=0x%04X(%s) 控制=0x%04X 模式=%d/%d 实际=%d 目标=%d 差值=%d 实际转矩=%d 上电=%s\n",
                        i, 
                        motion_data[i].status_word,
                        status_str,
                        motion_data[i].control_word,
                        motion_data[i].control_mode,
                        motion_data[i].control_mode_acutal,
                        motion_data[i].actual_position,
                        motion_data[i].target_position,
                        motion_data[i].target_position - motion_data[i].actual_position,
                        motion_data[i].actual_torque,
                        motion_data[i].is_power_on ? "是" : "否"
                    );
                }
            }
            printf("================================\n\n");
        }
        
        //更新PDO数据
        motion_update_pdo_data(p_domain, offset, slave_count);
        ecrt_domain_queue(domain);
        /*********************************************************/

        t_send_start = dc_sys_time_ns();
        ecrt_master_send(master);
        t_send_end = dc_sys_time_ns();
        dc_update_master_clock();
    }
}

/**
 * 初始化时检查和复位伺服状态
 * @param master EtherCAT主站
 * @param domain 域
 * @param p_domain 域数据指针
 * @param slave_count 从站数量
 * @param offset PDO偏移量数组
 * @param is_servo_slave 伺服标志数组
 */
static void example_init_servo_status_check(ec_master_t *master, ec_domain_t *domain, 
                           uint8_t *p_domain, int slave_count,
                           struct pdo_offset *offset, int *is_servo_slave) {
    printf("检查并复位伺服状态...\n");
    
    // 检查p_domain是否有效
    if (!p_domain) {
        printf("警告：域数据指针无效\n");
        return;
    }
    
    // 发送几次通信来检查状态
    for (int cycle = 0; cycle < 5; cycle++) {
        ecrt_master_receive(master);
        ecrt_domain_process(domain);
        
        // 检查所有伺服从站状态
        for (int i = 0; i < slave_count; i++) {
            if (is_servo_slave[i] && offset[i].status_word < 0x0000ffff) {
                uint16_t status = EC_READ_U16(p_domain + offset[i].status_word);
                printf("从站 %d 状态字: 0x%04X\n", i, status);
                
                // 如果伺服处于运行状态，发送断电命令
                if (status & 0x0004) {  // 检查是否上电
                    printf("从站 %d 处于上电状态，发送断电命令...\n", i);
                    if (offset[i].control_word < 0x0000ffff) {
                        EC_WRITE_U16(p_domain + offset[i].control_word, 0x0000);
                    }
                }
            }
        }
        
        // 更新PDO数据（初始化检查）
        motion_update_pdo_data(p_domain, offset, slave_count);
        ecrt_domain_queue(domain);
        ecrt_master_send(master);
        
        usleep(100000);  // 等待100ms
    }
    
    printf("伺服状态检查和复位完成\n");
}

/**
 * 配置伺服运动参数
 * @param slave_index 从站索引
 * @param slave 从站配置指针
 * @return 0表示成功
 */
int example_configure_servo_motion_params(int slave_index, ec_slave_config_t *slave)
{
    // 配置基本运动参数（只配置必要的参数）
    printf("为伺服从站 %d 配置基本运动参数\n", slave_index);
    
    // 定义参数数组：0x6080(最大速度), 0x6083(加速度), 0x6084(减速度), 0x6085(急停减速度)
    // 小参数数组：用于只有-p参数的情况，防止距离过大跑飞
    const int32_t small_params[4] = {5, 1, 1, 1};
    // 大参数数组：用于其他情况（有-i, -v, -t, -s参数）
    const int32_t large_params[4] = {1000000, 100000, 100000, 100000};
    
    // 判断是否只有-p参数（没有-i, -v, -t, -s参数）
    int use_small_params = (pulses_per_control_cycle == 0 && 
                           target_velocity_cnt == 0 && 
                           target_torque == 0 && 
                           s_curve_mode == 0);
    
    const int32_t *params = use_small_params ? small_params : large_params;
    const char *param_type = use_small_params ? "小参数" : "大参数";
    
    printf("从站 %d: 使用%s配置（最大速度=%d, 加速度=%d, 减速度=%d, 急停减速度=%d）\n",
           slave_index, param_type, params[0], params[1], params[2], params[3]);
    
    // 设置最大速度 (0x6080)
    if (ecrt_slave_config_sdo32(slave, 0x6080, 0x00, params[0]) == 0) {
        printf("从站 %d: 设置轮廓最大速度成功 (%d)\n", slave_index, params[0]);
    } else {
        printf("从站 %d: 设置轮廓最大速度失败\n", slave_index);
    }
    
    // 设置加速度 (0x6083)
    if (ecrt_slave_config_sdo32(slave, 0x6083, 0x00, params[1]) == 0) {
        printf("从站 %d: 设置轮廓加速度成功 (%d)\n", slave_index, params[1]);
    } else {
        printf("从站 %d: 设置轮廓加速度失败\n", slave_index);
    }
    
    // 设置减速度 (0x6084)
    if (ecrt_slave_config_sdo32(slave, 0x6084, 0x00, params[2]) == 0) {
        printf("从站 %d: 设置轮廓减速度成功 (%d)\n", slave_index, params[2]);
    } else {
        printf("从站 %d: 设置轮廓减速度失败\n", slave_index);
    }
    
    // 设置急停减速度 (0x6085)
    if (ecrt_slave_config_sdo32(slave, 0x6085, 0x00, params[3]) == 0) {
        printf("从站 %d: 设置急停减速度成功 (%d)\n", slave_index, params[3]);
    } else {
        printf("从站 %d: 设置急停减速度失败\n", slave_index);
    }
    
    return 0;
}

/**
 * 激活主站并启动周期任务线程
 * @param master EtherCAT主站
 * @param domain 域
 * @param p_domain 域数据指针输出参数
 * @param cycletime_ns 周期时间（纳秒）
 * @return 0表示成功，-1表示失败
 */
int example_activate_master_and_start_threads(ec_master_t *master, ec_domain_t *domain, uint8_t **p_domain, unsigned int cycletime_ns, int cpuid)
{
    int ret = 0;

    /* 设置EtherCAT主站发送间隔(us)，
    通常与控制任务的周期相同。 */
    ecrt_master_set_send_interval(master, cycletime_ns / 1000);
    
    /* 设置初始主站时间并选择从站作为DC
    * 参考时钟，否则传递NULL以自动选择第一个有能力的
    * 从站。注意：无论主站还是参考从站将
    * 用作系统主DC时钟，都可以使用此功能。
    */
    // 尝试使用主时钟作为参考时钟，避免参考时钟获取失败
    dc_init(master, 1, cycletime_ns);

    /* 在激活主站之前，尝试重置从站状态以清除PDO映射 */
    printf("尝试在激活前重置从站状态...\n");
    // 注意：从站状态重置需要在激活前通过其他方式处理
    // 这里暂时跳过，让EtherCAT主站自动处理

    /* 激活主站并创建域 */
    if (ecrt_master_activate(master)) {
        fprintf(stderr, "ecrt_master_activate失败！\n");
        return -1;
    }
    
    if(!(*p_domain = ecrt_domain_data(domain))) {
        fprintf(stderr, "ecrt_domain_data失败！\n");
        return -1;
    }

    /* 初始化时检查和复位伺服状态 */
    example_init_servo_status_check(master, domain, *p_domain, slave_count, offset, is_servo_slave);

    /* 保存CPU ID供周期任务线程使用 */
    g_cpuid = cpuid;
    
    /* 创建周期实时线程 */
    run = 1;

    ret = pthread_create(&cyclic_thread, NULL, &example_cyclic_task, NULL);
    if (ret) {
        printf("创建周期任务线程失败:%d\n", ret);
        return -1;
    }

    return 0;
}

/**
 * 等待用户输入退出
 */
void example_wait_for_user_exit(void)
{
    printf("\n*****************************************\n");
    printf("*****************************************\n");
    printf("按 'q' 或 'Q' 然后按 [回车] 退出！\n");
    
    /* 等待从站完全进入OP状态并稳定运行 */
    printf("等待从站稳定运行...\n");
    
    // 检查从站状态，确保所有从站都在OP状态
    int retry_count = 0;
    const int max_retries = 10;
    ec_master_state_t master_state;
    
    while (retry_count < max_retries) {
        if (ecrt_master_state(master, &master_state) == 0) {
            // 检查是否所有从站都在OP状态 (al_states的第3位)
            if (master_state.al_states & 0x08) { // 0x08 = OP状态位
                printf("所有从站已进入OP状态\n");
                break;
            }
        }
        printf("等待从站进入OP状态... (尝试 %d/%d)\n", retry_count + 1, max_retries);
        usleep(500000); // 等待0.5秒
        retry_count++;
    }
    
    if (retry_count >= max_retries) {
        printf("警告：从站可能未完全进入OP状态\n");
    }
    
    while (1) {
#if 1
        char c = getchar();
        printf("ch=%c\n", c);
        if(c == 'q' || c == 'Q')
            break;
#endif
    }
    run = 0;
    pthread_join(cyclic_thread, NULL);
}

#ifndef IGH_EXAMPLE_NO_MAIN
/**
 * 主函数
 * 注意：虽然C语言不要求main函数必须在最后（因为函数已在头文件中声明），
 * 但将main函数放在最后是常见做法，提高代码可读性
 */
int main(int argc, char **argv)
{
    int ret = 0, i = 0, cpuid = 0, dc_method = 0;
    unsigned int cycletime_ns = 1000000;
    ec_master_info_t master_info;

    //注册Ctrl+C信号处理
    signal(SIGINT, example_catch_signal);

    /* 锁定所有当前映射的页面以防止页面交换 */
    mlockall(MCL_CURRENT | MCL_FUTURE);

    // 解析命令行参数
    ret = example_parse_arguments(argc, argv, &cpuid, &dc_method, &cycletime_ns);
    if (ret != 0) {
        return ret;  // -1表示需要退出（如-h参数）
    }

    // 初始化EtherCAT主站和域
    ret = example_init_ethercat_master(&master, &domain, &master_info);
    if (ret != 0) {
        goto out;
    }

    // 配置所有从站
    for(i = 0; i < slave_count; ++i)
    {
        ec_slave_info_t slave_info;
        ret = ecrt_master_get_slave(master, i, &slave_info);
        if(ret != 0)
        {
            printf("ecrt_master_get_slave %d失败，返回值 = %d！\n", i, ret);
            goto out;
        }

        vendor_id[i] = slave_info.vendor_id;
        product_code[i] = slave_info.product_code;

        ec_slave_config_t *slave = NULL;
        slave = ecrt_master_slave_config(master, 0, i, vendor_id[i], product_code[i]);
        if (!slave)
        {
            fprintf(stderr, "获取从站0和位置0失败。\n");
            return -1;
        }
        slave_configs[i] = slave;  // 保存slave配置指针

        is_servo_slave[i] = example_is_servo(vendor_id[i], product_code[i]);
        if(is_servo_slave[i])
        {
            printf("**********从站 %d 是伺服！**************\n", i);
            printf("从站 %d 厂商ID: 0x%08X 产品代码: 0x%08X\n",
                    i, slave_info.vendor_id, slave_info.product_code);

            // 配置伺服从站的PDO映射
            printf("为伺服从站 %d 配置PDO映射\n", i);
            ret = pdo_set_configure_slave_pdo(i, slave, domain, offset, pdo_choice, target_velocity_cnt, target_torque);
            if (ret != 0) {
                printf("从站 %d: PDO配置失败\n", i);
                goto out;
            }

            // 配置伺服运动参数
            example_configure_servo_motion_params(i, slave);
        }
        // 配置DC信号（所有从站都配置）
        // SYNC Shift Time设置为周期时间的30%
        if (dc_config_slave_dc(slave, 0x0300, cycletime_ns, 30) != 0) {
            printf("从站 %d DC配置失败！\n", i);
            goto out;
        }
        printf("从站 %d DC配置完成！\n", i);
    }

    // 激活主站并启动周期任务线程
    ret = example_activate_master_and_start_threads(master, domain, &p_domain, cycletime_ns, cpuid);
    if (ret != 0) {
        goto out;
    }

    // 等待用户输入退出
    example_wait_for_user_exit();

out:
    printf("程序结束\n");
    if (master) {
        ecrt_release_master(master); /* 释放请求的EtherCAT主站 */
    }

    return 0;
}
#endif

int example_ros2_start(int cpuid, unsigned int cycletime_ns)
{
    int ret = 0;
    int i = 0;
    ec_master_info_t master_info;

    // 控制周期时间默认与EtherCAT周期一致（微秒）
    if (cycletime_ns > 0) {
        g_control_cycle_time_us = cycletime_ns / 1000;
        if (g_control_cycle_time_us == 0) g_control_cycle_time_us = 1;
    }

    // 需要offset为0xFF初始值
    memset(offset, 0xff, sizeof(offset));
    motion_init(0);

    // 初始化EtherCAT主站和域
    ret = example_init_ethercat_master(&master, &domain, &master_info);
    if (ret != 0) {
        return ret;
    }

    // 配置所有从站（复用main中的逻辑，按 master_info->slave_count 遍历）
    for (i = 0; i < slave_count; ++i) {
        ec_slave_info_t slave_info;
        ret = ecrt_master_get_slave(master, i, &slave_info);
        if (ret != 0) {
            printf("ecrt_master_get_slave %d失败，返回值 = %d！跳过该从站，继续配置其他从站。\n", i, ret);
            // 不修改 slave_count，保持对当前扫描到的所有从站的整体支持
            continue;
        }

        vendor_id[i] = slave_info.vendor_id;
        product_code[i] = slave_info.product_code;

        ec_slave_config_t *slave = NULL;
        slave = ecrt_master_slave_config(master, 0, i, vendor_id[i], product_code[i]);
        if (!slave) {
            fprintf(stderr, "获取从站0和位置0失败。\n");
            example_ros2_stop();
            return -1;
        }
        slave_configs[i] = slave;

        is_servo_slave[i] = example_is_servo(vendor_id[i], product_code[i]);
        if (is_servo_slave[i]) {
            printf("**********从站 %d 是伺服！**************\n", i);
            printf("从站 %d 厂商ID: 0x%08X 产品代码: 0x%08X\n",
                   i, slave_info.vendor_id, slave_info.product_code);

            printf("为伺服从站 %d 配置PDO映射\n", i);
            ret = pdo_set_configure_slave_pdo(i, slave, domain, offset, pdo_choice, target_velocity_cnt, target_torque);
            if (ret != 0) {
                printf("从站 %d: PDO配置失败\n", i);
                example_ros2_stop();
                return -1;
            }

            example_configure_servo_motion_params(i, slave);
        }

        if (dc_config_slave_dc(slave, 0x0300, cycletime_ns, 30) != 0) {
            printf("从站 %d DC配置失败！\n", i);
            example_ros2_stop();
            return -1;
        }
        printf("从站 %d DC配置完成！\n", i);
    }

    // 激活主站并启动周期任务线程
    ret = example_activate_master_and_start_threads(master, domain, &p_domain, cycletime_ns, cpuid);
    if (ret != 0) {
        example_ros2_stop();
        return ret;
    }

    // ROS2：等待从站进入 OP 且 wkc 稳定后再返回，确保后续 topic 控制生效前通信已就绪
    {
        int retry = 0;
        const int max_retries = 20;  // 最多 20 * 0.5s = 10s
        ec_domain_state_t domain_state;
        ec_master_state_t master_state;
        printf("ROS2: 等待从站 OP 与 wkc 稳定...\n");
        while (retry < max_retries) {
            usleep(500000);  // 0.5s
            if (ecrt_domain_state(domain, &domain_state) == 0 && domain_state.wc_state == EC_WC_COMPLETE &&
                ecrt_master_state(master, &master_state) == 0 && (master_state.al_states & 0x08)) {
                printf("ROS2: 从站已进入 OP，wkc 正常，可响应 /low_cmd 控制。\n");
                break;
            }
            retry++;
            if (retry < max_retries) {
                printf("ROS2: 等待 OP/wkc... (尝试 %d/%d)\n", retry, max_retries);
            }
        }
        if (retry >= max_retries) {
            printf("ROS2: 警告 - 等待 OP/wkc 超时，将继续运行，/low_cmd 将在预备阶段完成后生效。\n");
        }
    }

    return 0;
}

void example_ros2_stop(void)
{
    if (run) {
        run = 0;
        pthread_join(cyclic_thread, NULL);
    }

    if (master) {
        ecrt_release_master(master);
        master = NULL;
    }
    domain = NULL;
    p_domain = NULL;
}
