#include <chrono>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "arm_control/msg/low_cmd.hpp"
#include "arm_control/msg/low_state.hpp"

extern "C" {
#include "igh_example.h"
#include "motion.h"
extern struct motion_control_data motion_data[MAX_SLAVE_NUMBER];
extern unsigned int slave_count;

int example_ros2_start(int cpuid, unsigned int cycletime_ns);
void example_ros2_stop(void);
}

using namespace std::chrono_literals;

namespace {
constexpr int kMaxMotors = MAX_SLAVE_NUMBER;
}

class IghEthercatNode final : public rclcpp::Node {
public:
  IghEthercatNode() : Node("igh_ethercat_node") {
    // 默认参数：cpuid=6, cycletime_ns=1000000, counts_per_rad=131072.0
    cpuid_ = this->declare_parameter<int>("cpuid", 6);
    cycletime_ns_ = this->declare_parameter<int>("cycletime_ns", 1000000);  // 1 ms
    counts_per_rad_ = this->declare_parameter<double>("counts_per_rad", 131072.0);
    counts_per_rad_per_sec_ = this->declare_parameter<double>("counts_per_rad_per_sec", 1.0);

    state_pub_ = this->create_publisher<arm_control::msg::LowState>("low_state", 10);
    cmd_sub_ = this->create_subscription<arm_control::msg::LowCmd>(
      "low_cmd", 10,
      std::bind(&IghEthercatNode::on_cmd, this, std::placeholders::_1));

    const int ret = example_ros2_start(cpuid_, static_cast<unsigned int>(cycletime_ns_));
    if (ret != 0) {
      throw std::runtime_error("example_ros2_start failed");
    }

    timer_ = this->create_wall_timer(10ms, std::bind(&IghEthercatNode::publish_state, this));
  }

  ~IghEthercatNode() override {
    example_ros2_stop();
  }

private:
  void on_cmd(const arm_control::msg::LowCmd::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(cmd_mtx_);

    const int begin = static_cast<int>(msg->begin);
    const int end = static_cast<int>(msg->end);
    // 使用实际从站数量限制索引范围，避免越界访问和上层"Exceeded upper bound"错误
    const int max_index = static_cast<int>(std::min<unsigned int>(slave_count, kMaxMotors));
    if (begin < 0 || begin >= max_index || end < 0 || end >= max_index || begin > end) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "invalid begin/end: %d..%d", begin, end);
      return;
    }

    for (int i = begin; i <= end; ++i) {
      if (i >= static_cast<int>(msg->motor_cmd.size())) {
        break;
      }
      const auto & c = msg->motor_cmd[static_cast<size_t>(i)];

      // mode 对齐现有 CiA402：8=CSP, 3=CSV, 4/10=CST 等
      motion_data[i].control_mode = static_cast<unsigned char>(c.mode);

      // 将 rad / rad/s 映射为 cnt / cnt/s
      motion_data[i].target_position =
        static_cast<int>(c.q);
      motion_data[i].target_velocity =
        static_cast<int>(c.dq);
      motion_data[i].target_torque =
        static_cast<short>(c.tau);

      (void)c.kp;
      (void)c.kd;
    }
  }

  void publish_state() {
    arm_control::msg::LowState st;
    st.header.stamp = this->now();
    st.slave_num = static_cast<uint8_t>(slave_count);
    // 仅根据实际从站数量发布状态，防止超过消息定义的上限
    const int motor_count = static_cast<int>(std::min<unsigned int>(slave_count, kMaxMotors));
    st.motor_state.resize(static_cast<size_t>(motor_count));

    for (int i = 0; i < motor_count; ++i) {
      auto & ms = st.motor_state[static_cast<size_t>(i)];
      ms.mode = motion_data[i].control_mode_acutal;
      ms.q = static_cast<float>(
        static_cast<double>(motion_data[i].actual_position) / counts_per_rad_);
      ms.dq = static_cast<float>(
        static_cast<double>(motion_data[i].actual_velocity) / counts_per_rad_per_sec_);
      ms.current = static_cast<float>(motion_data[i].actual_torque);
      ms.motorstate = static_cast<uint32_t>(motion_data[i].status_word);
    }

    state_pub_->publish(st);
  }

  int cpuid_{6};
  int cycletime_ns_{1000000};
  double counts_per_rad_{131072.0};
  double counts_per_rad_per_sec_{1.0};

  std::mutex cmd_mtx_;
  rclcpp::Subscription<arm_control::msg::LowCmd>::SharedPtr cmd_sub_;
  rclcpp::Publisher<arm_control::msg::LowState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<IghEthercatNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    fprintf(stderr, "igh_ethercat_node error: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}

