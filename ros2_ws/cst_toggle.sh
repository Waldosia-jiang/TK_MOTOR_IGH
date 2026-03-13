#!/usr/bin/env bash

# 循环次数，设为 1000 基本等于无限，可按需调整
LOOP=1000

while true; do
  echo "==== cycle $((i+1)): q=0 ===="
  timeout 2s ros2 topic pub /low_cmd arm_control/msg/LowCmd "
{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'},
  begin: 0,
  end: 6,
  motor_cmd: [
    {mode: 4, q: 0.0, dq: 0.0, tau: 0.0, kp: 10.0, kd: 0.0},
    {mode: 4, q: 0.0, dq: 0.0, tau: 0.0, kp: 10.0, kd: 0.0},
    {mode: 4, q: 0.0, dq: 0.0, tau: 0.0, kp: 10.0, kd: 0.0},
    {mode: 4, q: 0.0, dq: 0.0, tau: 0.0, kp: 10.0, kd: 0.0},
    {mode: 4, q: 0.0, dq: 0.0, tau: 0.0, kp: 10.0, kd: 0.0},
    {mode: 4, q: 0.0, dq: 0.0, tau: 0.0, kp: 10.0, kd: 0.0},
    {mode: 4, q: 0.0, dq: 0.0, tau: 0.0, kp: 10.0, kd: 0.0}
  ]
}" -r 50

  echo "==== cycle $((i+1)): q=32768 ===="
  timeout 2s ros2 topic pub /low_cmd arm_control/msg/LowCmd "
{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'},
  begin: 0,
  end: 6,
  motor_cmd: [
    {mode: 4, q: 32768.0, dq: 0.0, tau: 0.0, kp: 10.0, kd: 0.0},
    {mode: 4, q: 32768.0, dq: 0.0, tau: 0.0, kp: 10.0, kd: 0.0},
    {mode: 4, q: 32768.0, dq: 0.0, tau: 0.0, kp: 10.0, kd: 0.0},
    {mode: 4, q: 32768.0, dq: 0.0, tau: 0.0, kp: 10.0, kd: 0.0},
    {mode: 4, q: 32768.0, dq: 0.0, tau: 0.0, kp: 10.0, kd: 0.0},
    {mode: 4, q: 32768.0, dq: 0.0, tau: 0.0, kp: 10.0, kd: 0.0},
    {mode: 4, q: 32768.0, dq: 0.0, tau: 0.0, kp: 10.0, kd: 0.0}
  ]
}" -r 50
done