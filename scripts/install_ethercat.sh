#!/bin/bash
# 从 ethercat-stable-1.6 源码安装 IgH EtherCAT（与 README 一致）
# 用法: sudo ./install_ethercat.sh
# 安装前请先运行 uninstall_ethercat.sh 完全卸载旧版本

set -e

ETHERCAT_SOURCE="${ETHERCAT_SOURCE:-/home/p30039115276/ethercat-stable-1.6}"

if [ "$(id -u)" -ne 0 ]; then
  echo "请使用 sudo 运行此脚本"
  exit 1
fi

if [ ! -d "$ETHERCAT_SOURCE" ]; then
  echo "错误: 源码目录不存在: $ETHERCAT_SOURCE"
  echo "可设置环境变量: export ETHERCAT_SOURCE=/path/to/ethercat-stable-1.6"
  exit 1
fi

cd "$ETHERCAT_SOURCE"

echo "========== 1. bootstrap =========="
./bootstrap

echo "========== 2. configure =========="
./configure --enable-8139too=no --disable-eoe

echo "========== 3. 编译 (make all modules) =========="
make all modules

echo "========== 4. 安装 (make modules_install install) =========="
make modules_install install

echo "========== 5. depmod =========="
depmod -a

echo "========== 6. 链接 init 脚本 =========="
ln -sf /usr/local/etc/init.d/ethercat /etc/init.d/ethercat

echo "========== 7. sysconfig =========="
mkdir -p /etc/sysconfig
cp -f /usr/local/etc/sysconfig/ethercat /etc/sysconfig/ethercat 2>/dev/null || true

echo "========== 安装完成 =========="
echo ""
echo "请手动完成："
echo "  1. 编辑 /etc/sysconfig/ethercat："
echo "     设置 MASTER0_DEVICE=\"你的网卡MAC\"，DEVICE_MODULES=\"generic\""
echo "  2. 启动: sudo /etc/init.d/ethercat start"
echo "  3. （可选）开机自启: sudo update-rc.d ethercat defaults"
echo ""
echo "验证: sudo ethercat version && sudo ethercat slaves"
