# Standalone (-t) 与 ROS2 (mode 4) 启动方式差异说明

## 1. 两种进入“模式 4”（CST）的方式

### 1.1 Standalone：`./example -t -65536`

- **入口**：命令行参数 `-t` 设置 `target_torque`（CST 位置增量，单位 cnt），并置 `is_move = 1`。
- **模式与预备阶段**：
  - 周期任务里根据 `target_torque != 0` 走 **CST 预备阶段**：先让所有轴处于 **模式 8（CSP）**、目标位置=当前位置，等所有轴状态字低 8 位为 **0x37** 后，再切到 **模式 4（CST）**。
  - `operation_mode` 在代码里由 `target_torque` / `target_velocity_cnt` 决定（8/4/3），**不**从外部 topic 来。
- **主流程**：`main()` → 解析参数（含 `-t`）→ 初始化 EtherCAT → **激活主站并启动周期线程** → **等待从站进入 OP**（在 `example_wait_for_user_exit()` 里循环检查 `al_states`，最多 10 次 × 0.5s）→ 然后等待用户按 `q` 退出。
- **特点**：有“等待从站进入 OP”的步骤（虽与周期线程并行），周期线程在 1 ms 下发控制，CST 预备阶段保证先 0x37 再切模式 4。

### 1.2 ROS2：`ros2 run igh_ethercat igh_ethercat_node` + `cst_toggle.sh`（mode 4）

- **入口**：无命令行 `-t`，`example_ros2_start()` 里 **不** 调用 `example_parse_arguments()`，因此：
  - `is_move` 保持为 **0**，
  - `target_torque` 在 ROS2 的 igh_example.c 里被设为 **65536**（非 0，仅影响 PDO/配置侧，不参与“预备阶段”判定）。
- **模式来源**：运行模式由 **topic** `/low_cmd` 的 `motor_cmd[i].mode` 提供，写入 `motion_data[i].control_mode`。`cst_toggle.sh` 里发布 `mode: 4`，即 **直接** 让周期任务里使用 **模式 4**。
- **周期任务（ROS2 版 igh_example.c）**：
  - **没有** CST 预备阶段逻辑：不检查“所有轴 0x37 再切模式 4”，而是 **每周期直接用** `operation_mode = motion_data[index].control_mode`（即 4）并写 PDO。
  - 仍会做上电状态机、故障恢复和 `motion_process_slave_motion(index)`。
- **主流程**：`example_ros2_start()` → 初始化 EtherCAT → **仅** 调用 `example_activate_master_and_start_threads()`（激活 + 启动周期线程），**没有** 调用 `example_wait_for_user_exit()`，因此 **没有** “等待从站进入 OP” 的循环，直接返回后进入 `rclcpp::spin()`。
- **特点**：节点一起动周期线程就开始跑，无 OP 等待；模式 4 由 topic 立即生效，无预备阶段，可能出现通信或状态尚未稳定就切到 CST。

## 2. 主要差异汇总

| 项目 | Standalone (`-t`) | ROS2 (mode 4 + cst_toggle.sh) |
|------|-------------------|--------------------------------|
| 是否解析 `-t` / `is_move` | 是，`is_move=1` | 否，`is_move=0` |
| 运行模式来源 | 内部根据 `target_torque`/`target_velocity_cnt` 定为 8→4 或 3 | topic `motor_cmd.mode`（如 4） |
| CST 预备阶段 | 有：先模式 8、等 0x37 再切模式 4 | 无：直接模式 4 |
| “等待从站进入 OP” | 有（在 `example_wait_for_user_exit()` 中） | 无 |
| 周期线程启动时机 | 与“等待 OP”并行，但主线程会先跑完等待循环 | 激活后立即启动，无额外等待 |

## 3. ROS2 下出现“状态字 0x07→0x00”与 0x1237↔0x0250 振荡的原因分析

1. **启动即跑、无 OP 等待**  
   周期线程在 **wkc 尚未稳定** 时就开始运行，首周期可能出现 “Working counter 状态=0”，`comm_ok=0`，状态机暂停更新；之后 wkc 恢复，状态机才按 PDO 更新。若中间出现偶发 0 或错误 PDO，容易触发“从 0x07 变 0x00”的恢复逻辑。

2. **状态字低 4 位为 0 时的统一处理**  
   当前逻辑对所有 `(status_word & 0x0f) == 0x00` 一视同仁：
   - **0x0000**：可能是 PDO 未更新/通信异常导致的误读；
   - **0x0250**：CiA402 合法状态“电压使能、开启禁用”（bit4=1），低 4 位也是 0。  
   若在驱动器实际为 **0x1237（运行使能）** 时，因偶发读到 **0x0000** 或 **0x0250**，代码都按“从 0x07 变 0x00”发送 **0x06**。对真实 0x1237 发 0x06 会令驱动器进入“准备/关闭”状态（如 0x0231/0x0250），从而出现 0x1237 ↔ 0x0250 的反复。

3. **无 CST 预备阶段**  
   没有“先模式 8、等 0x37 再切 4”的步骤，在 EtherCAT/从站尚未完全就绪时就切到模式 4，可能加剧状态或 PDO 的短暂异常。

## 4. 已做修改与可选改进

### 4.1 已做：状态机区分 0x0000 与 0x0250（motion.c）

- 对“低 4 位为 0”做了区分：
  - **当 `status_word == 0x0000`** 且“上一状态为 0x07、已上电”时：视为可能 PDO 误读，**保持控制字 0x0F，不发送 0x06**，避免单次误读导致关使能、引发 0x1237↔0x0250 振荡。
  - **当 `status_word != 0x0000`**（如 0x0250）且低 4 位为 0：按正常 CiA402 流程发送 0x06 重新上电。
- 已同时改动 **根目录 motion.c** 与 **ros2_ws 下 igh_ethercat 的 motion.c**，standalone 与 ROS2 行为一致。

### 4.2 可选：ROS2 启动时等待 OP / wkc 稳定

- 在 `example_ros2_start()` 中，在 `example_activate_master_and_start_threads()` 之后增加短循环：等待从站 OP 或 domain wkc 稳定后再 return，可减少首周期“通信异常 - Working counter 状态=0”及后续误判。

### 4.3 可选：ROS2 下 mode 4 增加 CST 预备阶段

- 若希望与 standalone `-t` 完全一致，可在 ROS2 周期任务中，当 `control_mode == 4` 时增加“CST 预备阶段”：先以模式 8 保持位置，等待所有轴状态低 8 位为 0x37 后再切到模式 4。

以上修改可减少“状态字从 0x07 变为 0x00”的误报和 0x1237↔0x0250 的振荡。
