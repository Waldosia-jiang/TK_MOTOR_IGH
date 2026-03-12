#ifndef __PDO_SET_H__
#define __PDO_SET_H__

#include "ecrt.h"
#include "igh_example.h"

// ========== PDO配置函数声明（在pdo_set.c中实现）==========
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
                                 struct pdo_offset *offset, int pdo_choice, int target_velocity_cnt, int target_torque);

#endif /* __PDO_SET_H__ */

