#include <stdio.h>
#include <math.h>
#include "trajectory_planning.h"

// 外部变量：控制周期时间（毫秒）
extern unsigned int g_control_cycle_time_us;

/**
 * 生成一阶梯形加减速轨迹（梯形速度曲线）
 * @param slv_num 从站编号
 * @param total_distance 总位移（脉冲数，可为负数表示反向）
 * @param avg_increment 平均阶跃值（脉冲数/周期）
 * @return 轨迹点数，失败返回-1
 */
int trajectory_planning_generate_1s_curve(int slv_num, int32_t total_distance, int avg_increment)
{
    if (avg_increment <= 0 || total_distance == 0) {
        printf("轴 %d: 错误 - 无效的轨迹参数（总位移=%d, 平均阶跃=%d）\n", slv_num, total_distance, avg_increment);
        return -1;
    }
    
    // 处理负距离（反向运动）
    int is_reverse = (total_distance < 0);
    int32_t abs_distance = is_reverse ? -total_distance : total_distance;
    int32_t signed_distance = is_reverse ? -abs_distance : abs_distance;
    
    // 1. 基础参数计算
    // T_s: 插补周期（秒）
    double T_s = (double)g_control_cycle_time_us / 1000000.0;  // 控制周期时间（秒）
    
    // 从 avg_increment 计算速度 v_avg (cnt/s)
    double v_avg = (double)avg_increment / T_s;
    
    // 直接将 v_avg 作为最大速度 v_max（不再乘以倍数）
    double v_max = v_avg;
    
    // 计算默认最大加速度 a_max
    // 策略：假设加速段和减速段各占总时间的15-25%，匀速段占50-70%
    // 先估算总时间：t_total ≈ S / v_avg
    double t_total_est = (double)abs_distance / v_avg;
    
    // 假设加速时间为总时间的20%（可根据需要调整，范围10%-30%）
    double t_a_est = t_total_est * 0.2;
    
    // 确保加速时间至少为1个控制周期
    if (t_a_est < T_s) {
        t_a_est = T_s;
    }
    
    // 计算加速度：a_max = v_max / t_a
    double a_max = v_max / t_a_est;
    
    // 限制加速度范围：避免过大或过小
    // 最小加速度：至少能在总时间的50%内加速到最大速度
    double a_min = v_max / (t_total_est * 0.5);
    if (a_max < a_min) {
        a_max = a_min;
    }
    
    // 最大加速度：限制为平均速度的10倍（避免加速度过大导致冲击）
    double a_max_limit = v_avg * 10.0;
    if (a_max > a_max_limit) {
        a_max = a_max_limit;
    }
    
    // 2. 轨迹可行性判断：是否有匀速段
    double t_a = v_max / a_max;          // 理论加速时间
    double S_a = 0.5 * a_max * t_a * t_a; // 加速段理论位移
    double S_d = S_a;                    // 减速段位移（对称）
    double v_m = v_max;                  // 实际最大速度
    double t_v = 0;                      // 匀速时间
    
    if ((double)abs_distance < S_a + S_d) {
        // 无匀速段，重新计算实际最大速度和加速时间
        v_m = sqrt((double)abs_distance * a_max);
        t_a = v_m / a_max;
        S_a = 0.5 * a_max * t_a * t_a;
        S_d = S_a;
        t_v = 0;
    } else {
        // 有匀速段，计算匀速时间
        t_v = ((double)abs_distance - S_a - S_d) / v_max;
    }
    
    // 3. 总运动时间
    double t_total = t_a + t_v + t_a; // t_d = t_a（减速时间等于加速时间）
    // 总插补次数（向上取整）
    int N = (int)(t_total / T_s) + 1;
    
    // 限制最大轨迹点数
    if (N > MAX_TRAJECTORY_POINTS) {
        printf("轴 %d: 警告 - 轨迹点数(%d)超过最大值(%d)，限制为最大值\n", slv_num, N, MAX_TRAJECTORY_POINTS);
        N = MAX_TRAJECTORY_POINTS;
    }
    
    // 4. 逐周期计算目标位置
    int32_t initial_pos = g_s_curve_traj[slv_num].initial_position;
    int32_t *traj = g_s_curve_traj[slv_num].trajectory_points;
    
    for (int k = 0; k < N; k++) {
        double t_k = k * T_s;       // 当前时刻
        double s_k = 0;             // 当前目标位置（相对于初始位置的位移）
        
        if (t_k <= t_a) {
            // 加速段
            s_k = 0.5 * a_max * t_k * t_k;
        } else if (t_k <= t_a + t_v) {
            // 匀速段
            s_k = S_a + v_m * (t_k - t_a);
        } else {
            // 减速段
            double t_dec = t_k - t_a - t_v; // 减速段已运行时间
            s_k = S_a + v_m * t_v + v_m * t_dec - 0.5 * a_max * t_dec * t_dec;
            // 防止超程（位移略超总位移）
            if (s_k > (double)abs_distance) {
                s_k = (double)abs_distance;
            }
        }
        
        // 根据方向调整符号
        int32_t position = initial_pos + (int32_t)(s_k * (signed_distance >= 0 ? 1 : -1));
        traj[k] = position;
    }
    
    // 确保最后一个点精确到达目标位置
    traj[N - 1] = initial_pos + signed_distance;
    
    // 确保 total_points 不超过数组大小，防止越界
    g_s_curve_traj[slv_num].total_points = (N > MAX_TRAJECTORY_POINTS) ? MAX_TRAJECTORY_POINTS : N;
    g_s_curve_traj[slv_num].current_index = 0;
    
    printf("轴 %d: 一阶梯形加减速轨迹生成完成 - 总位移=%d, 总周期数=%d\n", slv_num, signed_distance, g_s_curve_traj[slv_num].total_points);
    printf("  参数: v_max=%.2f cnt/s, a_max=%.2f cnt/s², T_s=%.3f s\n", v_max, a_max, T_s);
    printf("  阶段: 加速时间=%.3f s (位移=%.2f), 匀速时间=%.3f s, 减速时间=%.3f s (位移=%.2f)\n",
           t_a, S_a, t_v, t_a, S_d);
    
    return N;
}

/**
 * 生成二阶S曲线轨迹（二次S曲线，梯形加速度曲线）
 * @param slv_num 从站编号
 * @param total_distance 总位移（脉冲数，可为负数表示反向）
 * @param avg_increment 平均阶跃值（脉冲数/周期）
 * @return 轨迹点数，失败返回-1
 */
int trajectory_planning_generate_2s_curve(int slv_num, int32_t total_distance, int avg_increment)
{
    if (avg_increment <= 0 || total_distance == 0) {
        printf("轴 %d: 错误 - 无效的轨迹参数（总位移=%d, 平均阶跃=%d）\n", slv_num, total_distance, avg_increment);
        return -1;
    }
    
    // 处理负距离（反向运动）
    // 直接使用 total_distance 作为 signed_distance，保持原始符号
    int32_t signed_distance = total_distance;
    int32_t abs_distance = (total_distance < 0) ? -total_distance : total_distance;
    
    // 1. 基础参数计算
    // T_s: 插补周期（秒）
    double T_s = (double)g_control_cycle_time_us / 1000000.0;  // 控制周期时间（秒）
    
    // 从 avg_increment 计算速度 v_avg (cnt/s)
    double v_avg = (double)avg_increment / T_s;
    
    // 直接将 v_avg 作为最大速度 v_max
    double v_max = v_avg;
    
    // 计算默认最大加速度 a_max
    // 先估算总时间：t_total ≈ S / v_avg
    double t_total_est = (double)abs_distance / v_avg;
    
    // 假设加速时间为总时间的25%（二阶S曲线需要更长的加速时间）
    double t_a_est = t_total_est * 0.25;
    
    // 确保加速时间至少为2个控制周期（因为需要两个子阶段）
    if (t_a_est < 2.0 * T_s) {
        t_a_est = 2.0 * T_s;
    }
    
    // 计算最大加速度：a_max = v_max / (t_a / 2) = 2 * v_max / t_a
    // 因为平均加速度是 a_max/2，所以 a_max = 2 * v_max / t_a
    double a_max = 2.0 * v_max / t_a_est;
    
    // 限制加速度范围：避免过大或过小
    double a_min = 2.0 * v_max / (t_total_est * 0.5);
    if (a_max < a_min) {
        a_max = a_min;
    }
    
    // 最大加速度：限制为平均速度的8倍
    double a_max_limit = v_avg * 8.0;
    if (a_max > a_max_limit) {
        a_max = a_max_limit;
    }
    
    // 计算最大加加速度 J_max（根据加速度和时间计算）
    // 对于二阶S曲线，t_j = a_max / J_max，t_a = 2 * t_j
    // 所以 J_max = a_max / t_j = 2 * a_max / t_a
    double J_max = 2.0 * a_max / t_a_est;
    
    // 限制加加速度范围
    double J_min = 2.0 * a_max / (t_total_est * 0.5);
    if (J_max < J_min) {
        J_max = J_min;
    }
    
    // 2. 基础参数计算
    double t_j = a_max / J_max;          // 加加速/减加速子阶段时长
    double t_a = 2.0 * t_j;             // 加速段总时长
    double S_a = v_max * t_a / 2.0;     // 加速段总位移
    double S_d = S_a;                   // 减速段总位移
    double v_m = v_max;                 // 实际最大速度
    double t_v = 0;                     // 匀速时间
    
    // 3. 轨迹可行性判断：是否有匀速段
    if ((double)abs_distance < S_a + S_d) {
        // 无匀速段，重新计算实际最大速度和子阶段时长
        v_m = sqrt((double)abs_distance * J_max / 2.0);
        t_j = v_m / a_max;
        t_a = 2.0 * t_j;
        S_a = v_m * t_a / 2.0;
        S_d = S_a;
        t_v = 0;
    } else {
        // 有匀速段，计算匀速时间
        t_v = ((double)abs_distance - S_a - S_d) / v_max;
    }
    
    // 4. 总运动时间
    double t_total = t_a + t_v + t_a; // t_d = t_a（减速时间等于加速时间）
    // 总插补次数（向上取整）
    int N = (int)(t_total / T_s) + 1;
    
    // 限制最大轨迹点数
    if (N > MAX_TRAJECTORY_POINTS) {
        printf("轴 %d: 警告 - 轨迹点数(%d)超过最大值(%d)，限制为最大值\n", slv_num, N, MAX_TRAJECTORY_POINTS);
        N = MAX_TRAJECTORY_POINTS;
    }
    
    // 5. 逐周期插补计算
    int32_t initial_pos = g_s_curve_traj[slv_num].initial_position;
    int32_t *traj = g_s_curve_traj[slv_num].trajectory_points;
    
    // 确保第一个点从初始位置开始
    traj[0] = initial_pos;
    
    for (int k = 0; k < N; k++) {
        double t_k = k * T_s;       // 当前时刻
        double s_k = 0;             // 当前目标位置（相对于初始位置的位移）
        
        if (t_k <= t_j) {
            // 加速段-加加速阶段
            // J_k = J_max
            // a_k = J_max * t_k
            // v_k = 0.5 * J_max * t_k^2
            // s_k = (1/6) * J_max * t_k^3
            s_k = (1.0/6.0) * J_max * t_k * t_k * t_k;
        } else if (t_k <= t_a) {
            // 加速段-减加速阶段
            double t_sub = t_k - t_j;
            // J_k = -J_max
            // a_k = a_max - J_max * t_sub (加速度从a_max线性减少到0)
            // 加加速阶段结束时的速度：v_end = 0.5 * J_max * t_j^2
            // 减加速阶段的速度：v_k = v_end + ∫(a_max - J_max * t_sub)dt
            //                   = 0.5 * J_max * t_j^2 + a_max * t_sub - 0.5 * J_max * t_sub^2
            //                   = 0.5 * J_max * t_j^2 + J_max * t_j * t_sub - 0.5 * J_max * t_sub^2
            //                   = 0.5 * J_max * (t_j^2 + 2*t_j*t_sub - t_sub^2)
            // 位置积分：s_k = s_a1 + v_end * t_sub + 0.5 * a_max * t_sub^2 - (1/6) * J_max * t_sub^3
            double s_a1 = (1.0/6.0) * J_max * t_j * t_j * t_j; // 加加速段末位置
            double v_end = 0.5 * J_max * t_j * t_j; // 加加速阶段结束时的速度
            s_k = s_a1 + v_end * t_sub + 0.5 * a_max * t_sub * t_sub - (1.0/6.0) * J_max * t_sub * t_sub * t_sub;
        } else if (t_k <= t_a + t_v) {
            // 匀速段
            s_k = S_a + v_m * (t_k - t_a);
        } else {
            // 减速段
            double t_dec = t_k - t_a - t_v; // 减速段已运行时间
            if (t_dec <= t_j) {
                // 减速段-加减速阶段
                // J_k = -J_max
                // a_k = -J_max * t_dec
                // v_k = v_m - 0.5 * J_max * t_dec^2
                double s_av = S_a + v_m * t_v; // 加速+匀速段末位置
                s_k = s_av + v_m * t_dec - (1.0/6.0) * J_max * t_dec * t_dec * t_dec;
            } else {
                // 减速段-减减速阶段
                double t_sub = t_dec - t_j;
                // J_k = J_max
                // a_k = -a_max + J_max * t_sub (加速度从-a_max线性增加到0)
                // v_k = v_end - a_max * t_sub + 0.5 * J_max * t_sub^2
                // s_k = s_avd1 + v_end * t_sub - 0.5 * a_max * t_sub^2 + (1/6) * J_max * t_sub^3
                double s_avd1 = S_a + v_m * t_v + v_m * t_j - (1.0/6.0) * J_max * t_j * t_j * t_j; // 加减速段末位置
                double v_end = v_m - 0.5 * J_max * t_j * t_j; // 加减速阶段结束时的速度
                // 位置积分：s_k = s_avd1 + v_end * t_sub - 0.5 * a_max * t_sub^2 + (1/6) * J_max * t_sub^3
                s_k = s_avd1 + v_end * t_sub - 0.5 * a_max * t_sub * t_sub + (1.0/6.0) * J_max * t_sub * t_sub * t_sub;
                // 防止超程
                if (s_k > (double)abs_distance) {
                    s_k = (double)abs_distance;
                }
            }
        }
        
        // 根据方向调整符号
        int32_t position = initial_pos + (int32_t)(s_k * (signed_distance >= 0 ? 1 : -1));
        traj[k] = position;
    }
    
    // 确保最后一个点精确到达目标位置
    traj[N - 1] = initial_pos + signed_distance;
    
    // 确保 total_points 不超过数组大小，防止越界
    g_s_curve_traj[slv_num].total_points = (N > MAX_TRAJECTORY_POINTS) ? MAX_TRAJECTORY_POINTS : N;
    g_s_curve_traj[slv_num].current_index = 0;
    
    // 验证轨迹方向：第一个点和最后一个点
    int32_t first_point = traj[0];
    int32_t last_point = traj[g_s_curve_traj[slv_num].total_points - 1];
    int32_t actual_displacement = last_point - first_point;
    
    printf("轴 %d: 二阶S曲线轨迹生成完成 - 总位移=%d, 总周期数=%d\n", slv_num, signed_distance, g_s_curve_traj[slv_num].total_points);
    printf("  初始位置=%d, 第一个点=%d, 最后一个点=%d, 实际位移=%d\n", 
           initial_pos, first_point, last_point, actual_displacement);
    printf("  参数: v_max=%.2f cnt/s, a_max=%.2f cnt/s², J_max=%.2f cnt/s³, T_s=%.3f s\n", v_max, a_max, J_max, T_s);
    printf("  阶段: 加速时间=%.3f s (t_j=%.3f, 位移=%.2f), 匀速时间=%.3f s, 减速时间=%.3f s (位移=%.2f)\n",
           t_a, t_j, S_a, t_v, t_a, S_d);
    
    // 检查方向一致性
    if ((signed_distance > 0 && actual_displacement < 0) || (signed_distance < 0 && actual_displacement > 0)) {
        printf("轴 %d: 警告 - 轨迹方向不一致！期望位移=%d, 实际位移=%d\n", slv_num, signed_distance, actual_displacement);
    }
    
    return N;
}

