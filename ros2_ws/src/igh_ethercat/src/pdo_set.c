#include <stdio.h>
#include <unistd.h>
#include "ecrt.h"
#include "igh_example.h"
#include "pdo_set.h"

// 外部变量声明（用于判断是否只有-p参数）
extern int pulses_per_control_cycle;
extern int s_curve_mode;

/**
 * 清除并重新分配PDO（用于RW模板）（私有函数）
 * @param slave 从站配置指针
 * @param slave_index 从站索引
 * @param rx_pdo RxPDO索引
 * @param tx_pdo TxPDO索引
 */
static void pdo_set_clear_pdo_assign(ec_slave_config_t *slave, int slave_index, uint16_t rx_pdo, uint16_t tx_pdo)
{
    printf("清除从站 %d 的默认PDO分配（RW模板）...\n", slave_index);
    ecrt_slave_config_pdo_assign_clear(slave, 2);  // 清除SM2的输出PDO
    ecrt_slave_config_pdo_assign_clear(slave, 3);  // 清除SM3的输入PDO
    
    if (ecrt_slave_config_pdo_assign_add(slave, 2, rx_pdo) != 0) {
        printf("从站 %d: 分配SM2 RxPDO失败 (0x%04X)，可能导致PDO配置错误\n", slave_index, rx_pdo);
    }
    if (ecrt_slave_config_pdo_assign_add(slave, 3, tx_pdo) != 0) {
        printf("从站 %d: 分配SM3 TxPDO失败 (0x%04X)，可能导致PDO配置错误\n", slave_index, tx_pdo);
    }
}


/**
 * 添加PDO映射条目（私有函数）
 * @return 0表示成功，非0表示失败
 */
static int pdo_set_add_pdo_mapping(ec_slave_config_t *slave, uint16_t pdo_index, 
                                    uint16_t index, uint8_t subindex, uint8_t bit_length,
                                    const char *entry_name)
{
    if (ecrt_slave_config_pdo_mapping_add(slave, pdo_index, index, subindex, bit_length) == 0) {
        printf("将%s(0x%04X)添加到PDO 0x%04X\n", entry_name, index, pdo_index);
        return 0;
    } else {
        printf("向PDO 0x%04X添加%s失败\n", pdo_index, entry_name);
        return -1;
    }
}

/**
 * 配置RW模板CSV模式（私有函数）
 */
static int pdo_set_configure_rw_template_csv(ec_slave_config_t *slave, ec_domain_t *domain,
                                             int slave_index, struct pdo_offset *offset,
                                             uint16_t rx_pdo, uint16_t tx_pdo)
{
    printf("从站 %d: RW模板配置CSV模式PDO映射 (RX=0x%04X, TX=0x%04X)\n", slave_index, rx_pdo, tx_pdo);
    
    // 先通过SDO设置运行模式为CSV模式（3）
    ecrt_slave_config_sdo8(slave, 0x6060, 0x00, 3);  // CSV模式
    
    // RxPDO: 控制字 + 运行模式 + 目标速度
    pdo_set_add_pdo_mapping(slave, rx_pdo, 0x6040, 0x00, 16, "控制字");
    pdo_set_add_pdo_mapping(slave, rx_pdo, 0x6060, 0x00, 8, "运行模式");
    pdo_set_add_pdo_mapping(slave, rx_pdo, 0x60ff, 0x00, 32, "目标速度");
    
    // TxPDO: 状态字 + 运行模式显示 + 实际位置
    pdo_set_add_pdo_mapping(slave, tx_pdo, 0x6041, 0x00, 16, "状态字");
    pdo_set_add_pdo_mapping(slave, tx_pdo, 0x6061, 0x00, 8, "运行模式显示");
    pdo_set_add_pdo_mapping(slave, tx_pdo, 0x6064, 0x00, 32, "实际位置值");
    
    // 注册CSV模式的PDO条目到域
    offset[slave_index].control_word = ecrt_slave_config_reg_pdo_entry(slave, 0x6040, 0x00, domain, NULL);
    offset[slave_index].mode_of_operation = ecrt_slave_config_reg_pdo_entry(slave, 0x6060, 0x00, domain, NULL);
    offset[slave_index].target_velocity = ecrt_slave_config_reg_pdo_entry(slave, 0x60ff, 0x00, domain, NULL);
    
    // CSV模式不使用target_position和target_torque，明确设置为-1
    offset[slave_index].target_position = -1;  // CSV模式不使用target_position
    offset[slave_index].target_torque = -1;    // CSV模式不使用target_torque
    
    offset[slave_index].status_word = ecrt_slave_config_reg_pdo_entry(slave, 0x6041, 0x00, domain, NULL);
    offset[slave_index].mode_of_operation_display = ecrt_slave_config_reg_pdo_entry(slave, 0x6061, 0x00, domain, NULL);
    offset[slave_index].position_actual_value = ecrt_slave_config_reg_pdo_entry(slave, 0x6064, 0x00, domain, NULL);
    
    // CSV模式必要项：6040/60FF/6041
    if ((int)offset[slave_index].control_word < 0 || (int)offset[slave_index].target_velocity < 0 ||
        (int)offset[slave_index].status_word < 0) {
        printf("从站 %d: RW模板CSV模式PDO条目注册失败（offset为负）\n", slave_index);
        return -1;
    }
    
    printf("为伺服从站 %d 配置的PDO映射(CSV模式):\n", slave_index);
    printf("  控制字偏移量: %d\n", offset[slave_index].control_word);
    printf("  运行模式偏移量: %d\n", offset[slave_index].mode_of_operation);
    printf("  目标速度(0x60FF)偏移量: %d\n", offset[slave_index].target_velocity);
    printf("  状态字偏移量: %d\n", offset[slave_index].status_word);
    printf("  运行模式显示偏移量: %d\n", offset[slave_index].mode_of_operation_display);
    if ((int)offset[slave_index].position_actual_value >= 0) {
        printf("  实际位置偏移量: %d\n", offset[slave_index].position_actual_value);
    }
    return 0;
}

/**
 * 配置RW模板CST模式（私有函数）
 */
static int pdo_set_configure_rw_template_cst(ec_slave_config_t *slave, ec_domain_t *domain,
                                             int slave_index, struct pdo_offset *offset,
                                             uint16_t rx_pdo, uint16_t tx_pdo)
{
    printf("从站 %d: RW模板配置CST模式PDO映射 (RX=0x%04X, TX=0x%04X)\n", slave_index, rx_pdo, tx_pdo);
    
    // 先通过SDO设置运行模式为CST模式（4）
    ecrt_slave_config_sdo8(slave, 0x6060, 0x00, 4);  // CST模式
    
    // RxPDO: 控制字 + 运行模式 + 目标位置(用于CSP预备保持) + 目标转矩
    pdo_set_add_pdo_mapping(slave, rx_pdo, 0x6040, 0x00, 16, "控制字");
    pdo_set_add_pdo_mapping(slave, rx_pdo, 0x6060, 0x00, 8, "运行模式");
    // 预备阶段需要在模式8下写目标位置=当前位置，因此在CST模板中也加入0x607A
    pdo_set_add_pdo_mapping(slave, rx_pdo, 0x607a, 0x00, 32, "目标位置");
    pdo_set_add_pdo_mapping(slave, rx_pdo, 0x6071, 0x00, 16, "目标转矩");
    
    // TxPDO: 状态字 + 运行模式显示 + 实际位置 + 实际转矩
    pdo_set_add_pdo_mapping(slave, tx_pdo, 0x6041, 0x00, 16, "状态字");
    pdo_set_add_pdo_mapping(slave, tx_pdo, 0x6061, 0x00, 8, "运行模式显示");
    pdo_set_add_pdo_mapping(slave, tx_pdo, 0x6064, 0x00, 32, "实际位置值");
    pdo_set_add_pdo_mapping(slave, tx_pdo, 0x6077, 0x00, 16, "实际转矩值");
    
    // 注册CST模式的PDO条目到域
    offset[slave_index].control_word = ecrt_slave_config_reg_pdo_entry(slave, 0x6040, 0x00, domain, NULL);
    offset[slave_index].mode_of_operation = ecrt_slave_config_reg_pdo_entry(slave, 0x6060, 0x00, domain, NULL);
    offset[slave_index].target_position = ecrt_slave_config_reg_pdo_entry(slave, 0x607a, 0x00, domain, NULL);
    offset[slave_index].target_torque = ecrt_slave_config_reg_pdo_entry(slave, 0x6071, 0x00, domain, NULL);
    
    // CST模式不使用target_velocity，明确设置为-1（target_position用于预备阶段）
    offset[slave_index].target_velocity = -1;   // CST模式不使用target_velocity
    
    offset[slave_index].status_word = ecrt_slave_config_reg_pdo_entry(slave, 0x6041, 0x00, domain, NULL);
    offset[slave_index].mode_of_operation_display = ecrt_slave_config_reg_pdo_entry(slave, 0x6061, 0x00, domain, NULL);
    offset[slave_index].position_actual_value = ecrt_slave_config_reg_pdo_entry(slave, 0x6064, 0x00, domain, NULL);
    offset[slave_index].torque_actual_value = ecrt_slave_config_reg_pdo_entry(slave, 0x6077, 0x00, domain, NULL);
    
    // CST模式必要项：6040/6071/6041
    if ((int)offset[slave_index].control_word < 0 || (int)offset[slave_index].target_torque < 0 ||
        (int)offset[slave_index].status_word < 0) {
        printf("从站 %d: RW模板CST模式PDO条目注册失败（offset为负）\n", slave_index);
        return -1;
    }
    
    printf("为伺服从站 %d 配置的PDO映射(CST模式):\n", slave_index);
    printf("  控制字偏移量: %d\n", offset[slave_index].control_word);
    printf("  运行模式偏移量: %d\n", offset[slave_index].mode_of_operation);
    printf("  目标位置(0x607A)偏移量: %d\n", offset[slave_index].target_position);
    printf("  目标转矩(0x6071)偏移量: %d\n", offset[slave_index].target_torque);
    printf("  状态字偏移量: %d\n", offset[slave_index].status_word);
    printf("  运行模式显示偏移量: %d\n", offset[slave_index].mode_of_operation_display);
    if ((int)offset[slave_index].position_actual_value >= 0) {
        printf("  实际位置偏移量: %d\n", offset[slave_index].position_actual_value);
    }
    if ((int)offset[slave_index].torque_actual_value >= 0) {
        printf("  实际转矩偏移量: %d\n", offset[slave_index].torque_actual_value);
    }
    return 0;
}

/**
 * 配置RW模板基础CSP模式（私有函数）
 * 参考RO模板0x1600，只配置最基本的PDO映射
 */
static int pdo_set_configure_rw_template_csp_base(ec_slave_config_t *slave, ec_domain_t *domain,
                                                  int slave_index, struct pdo_offset *offset,
                                                  uint16_t rx_pdo, uint16_t tx_pdo)
{
    printf("从站 %d: RW模板配置基础CSP模式PDO映射 (RX=0x%04X, TX=0x%04X)\n", slave_index, rx_pdo, tx_pdo);
    
    // 先通过SDO设置运行模式为CSP模式（8）
    ecrt_slave_config_sdo8(slave, 0x6060, 0x00, 8);  // CSP模式
    
    // RxPDO条目（参考RO模板0x1600）：6040 + 607A + 60FE:01
    pdo_set_add_pdo_mapping(slave, rx_pdo, 0x6040, 0x00, 16, "控制字");
    pdo_set_add_pdo_mapping(slave, rx_pdo, 0x607a, 0x00, 32, "目标位置");
    pdo_set_add_pdo_mapping(slave, rx_pdo, 0x60FE, 0x01, 32, "数字输出");
    
    // TxPDO条目（参考RO模板0x1A00）：6041 + 6064 + 60FD
    pdo_set_add_pdo_mapping(slave, tx_pdo, 0x6041, 0x00, 16, "状态字");
    pdo_set_add_pdo_mapping(slave, tx_pdo, 0x6064, 0x00, 32, "实际位置值");
    pdo_set_add_pdo_mapping(slave, tx_pdo, 0x60FD, 0x00, 32, "数字输入");
    
    // 注册基础CSP模式的PDO条目到域
    offset[slave_index].control_word = ecrt_slave_config_reg_pdo_entry(slave, 0x6040, 0x00, domain, NULL);
    offset[slave_index].target_position = ecrt_slave_config_reg_pdo_entry(slave, 0x607a, 0x00, domain, NULL);
    offset[slave_index].digital_outputs = ecrt_slave_config_reg_pdo_entry(slave, 0x60fe, 0x01, domain, NULL);
    
    // 基础CSP模式不包含运行模式、速度前馈等高级功能
    offset[slave_index].mode_of_operation = -1;  // 基础模式不包含0x6060
    offset[slave_index].velocity_feedforward = -1;  // 基础模式不包含0x60B1
    offset[slave_index].target_velocity = -1;  // CSP模式不使用target_velocity
    offset[slave_index].target_torque = -1;    // CSP模式不使用target_torque
    
    offset[slave_index].status_word = ecrt_slave_config_reg_pdo_entry(slave, 0x6041, 0x00, domain, NULL);
    offset[slave_index].position_actual_value = ecrt_slave_config_reg_pdo_entry(slave, 0x6064, 0x00, domain, NULL);
    offset[slave_index].digital_inputs = ecrt_slave_config_reg_pdo_entry(slave, 0x60fd, 0x00, domain, NULL);
    
    // 基础模式不包含运行模式显示和实际速度
    offset[slave_index].mode_of_operation_display = -1;  // 基础模式不包含0x6061
    offset[slave_index].velocity_actual_value = -1;  // 基础模式不包含0x606C
    
    // 必要项检查：6040/607A/6041/6064
    if ((int)offset[slave_index].control_word < 0 || (int)offset[slave_index].target_position < 0 ||
        (int)offset[slave_index].status_word < 0 || (int)offset[slave_index].position_actual_value < 0) {
        printf("从站 %d: RW模板基础CSP模式PDO条目注册失败（offset为负）\n", slave_index);
        return -1;
    }
    
    printf("为伺服从站 %d 配置的PDO映射(基础CSP模式):\n", slave_index);
    printf("  RxPDO:\n");
    printf("    控制字偏移量: %d\n", offset[slave_index].control_word);
    printf("    目标位置偏移量: %d\n", offset[slave_index].target_position);
    if ((int)offset[slave_index].digital_outputs >= 0) {
        printf("    数字输出(0x60FE:01)偏移量: %d\n", offset[slave_index].digital_outputs);
    }
    printf("  TxPDO:\n");
    printf("    状态字偏移量: %d\n", offset[slave_index].status_word);
    printf("    实际位置偏移量: %d\n", offset[slave_index].position_actual_value);
    if ((int)offset[slave_index].digital_inputs >= 0) {
        printf("    数字输入(0x60FD)偏移量: %d\n", offset[slave_index].digital_inputs);
    }
    return 0;
}

/**
 * 配置RW模板CSP模式（带前馈的高级模式）（私有函数）
 */
static int pdo_set_configure_rw_template_csp(ec_slave_config_t *slave, ec_domain_t *domain,
                                            int slave_index, struct pdo_offset *offset,
                                            uint16_t rx_pdo, uint16_t tx_pdo)
{
    printf("从站 %d: RW模板配置CSP模式PDO映射 (RX=0x%04X, TX=0x%04X)\n", slave_index, rx_pdo, tx_pdo);
    
    // RxPDO条目：6040 + 6060 + 607A + 60FE:01 + 60B1
    pdo_set_add_pdo_mapping(slave, rx_pdo, 0x6040, 0x00, 16, "控制字");
    pdo_set_add_pdo_mapping(slave, rx_pdo, 0x6060, 0x00, 8, "运行模式");
    pdo_set_add_pdo_mapping(slave, rx_pdo, 0x607a, 0x00, 32, "目标位置");
    pdo_set_add_pdo_mapping(slave, rx_pdo, 0x60FE, 0x01, 32, "数字输出");
    pdo_set_add_pdo_mapping(slave, rx_pdo, 0x60B1, 0x00, 32, "速度前馈");
    
    // TxPDO条目：6041 + 6061 + 6064 + 60FD + 606C
    pdo_set_add_pdo_mapping(slave, tx_pdo, 0x6041, 0x00, 16, "状态字");
    pdo_set_add_pdo_mapping(slave, tx_pdo, 0x6061, 0x00, 8, "运行模式显示");
    pdo_set_add_pdo_mapping(slave, tx_pdo, 0x6064, 0x00, 32, "实际位置值");
    pdo_set_add_pdo_mapping(slave, tx_pdo, 0x60FD, 0x00, 32, "数字输入");
    pdo_set_add_pdo_mapping(slave, tx_pdo, 0x606C, 0x00, 32, "实际速度值");
    
    // 注册RW模板的PDO条目到域
    offset[slave_index].control_word = ecrt_slave_config_reg_pdo_entry(slave, 0x6040, 0x00, domain, NULL);
    offset[slave_index].mode_of_operation = ecrt_slave_config_reg_pdo_entry(slave, 0x6060, 0x00, domain, NULL);
    offset[slave_index].target_position = ecrt_slave_config_reg_pdo_entry(slave, 0x607a, 0x00, domain, NULL);
    offset[slave_index].digital_outputs = ecrt_slave_config_reg_pdo_entry(slave, 0x60fe, 0x01, domain, NULL);
    offset[slave_index].velocity_feedforward = ecrt_slave_config_reg_pdo_entry(slave, 0x60b1, 0x00, domain, NULL);
    
    // CSP模式不使用target_velocity和target_torque，明确设置为-1
    offset[slave_index].target_velocity = -1;  // CSP模式不使用target_velocity
    offset[slave_index].target_torque = -1;     // CSP模式不使用target_torque
    
    offset[slave_index].status_word = ecrt_slave_config_reg_pdo_entry(slave, 0x6041, 0x00, domain, NULL);
    offset[slave_index].mode_of_operation_display = ecrt_slave_config_reg_pdo_entry(slave, 0x6061, 0x00, domain, NULL);
    offset[slave_index].position_actual_value = ecrt_slave_config_reg_pdo_entry(slave, 0x6064, 0x00, domain, NULL);
    offset[slave_index].digital_inputs = ecrt_slave_config_reg_pdo_entry(slave, 0x60fd, 0x00, domain, NULL);
    offset[slave_index].velocity_actual_value = ecrt_slave_config_reg_pdo_entry(slave, 0x606c, 0x00, domain, NULL);
    
    // 必要项至少要：6040/607A/6041/6064
    if ((int)offset[slave_index].control_word < 0 || (int)offset[slave_index].target_position < 0 ||
        (int)offset[slave_index].status_word < 0 || (int)offset[slave_index].position_actual_value < 0) {
        printf("从站 %d: RW模板PDO条目注册失败（offset为负）\n", slave_index);
        return -1;
    }
    
    printf("为伺服从站 %d 配置的PDO映射(CSP模式):\n", slave_index);
    printf("  RxPDO:\n");
    printf("    控制字偏移量: %d\n", offset[slave_index].control_word);
    printf("    运行模式偏移量: %d\n", offset[slave_index].mode_of_operation);
    printf("    目标位置偏移量: %d\n", offset[slave_index].target_position);
    if ((int)offset[slave_index].digital_outputs >= 0) {
        printf("    数字输出(0x60FE:01)偏移量: %d\n", offset[slave_index].digital_outputs);
    }
    if ((int)offset[slave_index].velocity_feedforward >= 0) {
        printf("    速度前馈(0x60B1)偏移量: %d\n", offset[slave_index].velocity_feedforward);
    }
    printf("  TxPDO:\n");
    printf("    状态字偏移量: %d\n", offset[slave_index].status_word);
    printf("    运行模式显示偏移量: %d\n", offset[slave_index].mode_of_operation_display);
    printf("    实际位置偏移量: %d\n", offset[slave_index].position_actual_value);
    if ((int)offset[slave_index].digital_inputs >= 0) {
        printf("    数字输入(0x60FD)偏移量: %d\n", offset[slave_index].digital_inputs);
    }
    if ((int)offset[slave_index].velocity_actual_value >= 0) {
        printf("    实际速度(0x606C)偏移量: %d\n", offset[slave_index].velocity_actual_value);
    }
    return 0;
}

/**
 * 配置从站PDO（RW模板0x1607/0x1A07）
 * @param slave_index 从站索引
 * @param slave 从站配置指针
 * @param domain 域指针
 * @param offset PDO偏移量数组
 * @param pdo_choice PDO模板选择：固定为7（0x1607/0x1A07）
 * @param target_velocity_cnt CSV模式目标速度（cnt），如果非0则表示使用CSV模式
 * @param target_torque CST模式目标转矩，如果非0则表示使用CST模式
 * @return 0表示成功，-1表示失败
 */
int pdo_set_configure_slave_pdo(int slave_index, ec_slave_config_t *slave, ec_domain_t *domain,
                                 struct pdo_offset *offset, int pdo_choice, int target_velocity_cnt, int target_torque)
{
    // PDO模板固定为0x1607/0x1A07（RW模板）
    uint16_t rx_pdo = (uint16_t)(0x1600 + pdo_choice);
    uint16_t tx_pdo = (uint16_t)(0x1a00 + pdo_choice);
    
    // 判断运行模式：优先级 CST > CSV > CSP
    int is_cst_mode = (target_torque != 0);
    int is_csv_mode = (target_velocity_cnt != 0 && !is_cst_mode);
    
    // RW模板：清除默认的PDO分配并重新分配
    pdo_set_clear_pdo_assign(slave, slave_index, rx_pdo, tx_pdo);
    
    printf("为从站 %d 使用PDO: RX=0x%04X, TX=0x%04X\n", slave_index, rx_pdo, tx_pdo);
    
    // 清除默认的PDO映射
    ecrt_slave_config_pdo_mapping_clear(slave, rx_pdo);
    ecrt_slave_config_pdo_mapping_clear(slave, tx_pdo);
    
    // 根据参数决定配置CST/CSV/CSP模式（优先级：CST > CSV > CSP）
    if (is_cst_mode) {
        return pdo_set_configure_rw_template_cst(slave, domain, slave_index, offset, rx_pdo, tx_pdo);
    } else if (is_csv_mode) {
        return pdo_set_configure_rw_template_csv(slave, domain, slave_index, offset, rx_pdo, tx_pdo);
    } else {
        // CSP模式：判断是否只有-p参数（没有-i, -v, -t, -s参数）
        int is_base_csp = (pulses_per_control_cycle == 0 && 
                           target_velocity_cnt == 0 && 
                           target_torque == 0 && 
                           s_curve_mode == 0);
        
        if (is_base_csp) {
            // 只有-p参数，使用基础CSP模式（参考RO模板0x1600）
            return pdo_set_configure_rw_template_csp_base(slave, domain, slave_index, offset, rx_pdo, tx_pdo);
        } else {
            // 有其他参数，使用带前馈的高级CSP模式
            return pdo_set_configure_rw_template_csp(slave, domain, slave_index, offset, rx_pdo, tx_pdo);
        }
    }
}

