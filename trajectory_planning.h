#ifndef __TRAJECTORY_PLANNING_H__
#define __TRAJECTORY_PLANNING_H__

#include "motion.h"

/**
 * 生成一阶梯形加减速轨迹（梯形速度曲线）
 * @param slv_num 从站编号
 * @param total_distance 总位移（脉冲数，可为负数表示反向）
 * @param avg_increment 平均阶跃值（脉冲数/周期）
 * @return 轨迹点数，失败返回-1
 */
int trajectory_planning_generate_1s_curve(int slv_num, int32_t total_distance, int avg_increment);

/**
 * 生成二阶S曲线轨迹（二次S曲线，梯形加速度曲线）
 * @param slv_num 从站编号
 * @param total_distance 总位移（脉冲数，可为负数表示反向）
 * @param avg_increment 平均阶跃值（脉冲数/周期）
 * @return 轨迹点数，失败返回-1
 */
int trajectory_planning_generate_2s_curve(int slv_num, int32_t total_distance, int avg_increment);


#endif /* __TRAJECTORY_PLANNING_H__ */

