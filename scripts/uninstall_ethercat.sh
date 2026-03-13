#!/bin/bash
# 完全卸载当前 EtherCAT (IgH) 安装
# 用法: sudo ./uninstall_ethercat.sh

set -e

if [ "$(id -u)" -ne 0 ]; then
  echo "请使用 sudo 运行此脚本"
  exit 1
fi

ETHERCAT_SOURCE="${ETHERCAT_SOURCE:-/home/p30039115276/ethercat-stable-1.6}"
KVER="$(uname -r)"

echo "========== 1. 停止 EtherCAT 服务 =========="
if [ -x /etc/init.d/ethercat ]; then
  /etc/init.d/ethercat stop 2>/dev/null || true
fi
systemctl stop ethercat 2>/dev/null || true

echo "========== 2. 卸载内核模块 =========="
for m in ec_master ec_generic; do
  if lsmod | grep -q "^${m} "; then
    modprobe -r "$m" 2>/dev/null || true
  fi
done

echo "========== 3. 移除开机自启 =========="
update-rc.d -f ethercat remove 2>/dev/null || true

echo "========== 4. 删除 /etc 下的配置与脚本 =========="
rm -f /etc/init.d/ethercat
rm -f /etc/sysconfig/ethercat

echo "========== 5. 卸载 apt 安装的 ethercat 相关包（如有） =========="
apt list --installed 2>/dev/null | grep -i ethercat || true
for pkg in ethercat-master etherlab-master ethercat-tools; do
  dpkg -l "$pkg" 2>/dev/null | grep -q ^ii && apt-get remove -y "$pkg" || true
done

echo "========== 6. 从源码目录执行 make uninstall（用户态） =========="
if [ -d "$ETHERCAT_SOURCE" ] && [ -f "$ETHERCAT_SOURCE/Makefile" ]; then
  cd "$ETHERCAT_SOURCE"
  make uninstall 2>/dev/null || true
  cd - >/dev/null
else
  echo "源码目录不存在或无 Makefile，跳过 make uninstall"
fi

echo "========== 7. 删除 /usr/local 下的 EtherCAT 文件（兜底） =========="
rm -f /usr/local/bin/ethercat
rm -f /usr/local/lib/libethercat.so /usr/local/lib/libethercat.so.1
rm -f /usr/local/lib/libethercat.la
rmdir /usr/local/lib/ethercat 2>/dev/null || true
rm -rf /usr/local/include/ethercat
rm -f /usr/local/etc/init.d/ethercat
rm -rf /usr/local/etc/sysconfig
rm -f /usr/local/etc/sysconfig/ethercat 2>/dev/null || true

echo "========== 8. 删除内核模块文件（apt 或自定义路径） =========="
for base in "/lib/modules/${KVER}/ethercat" "/lib/modules/${KVER}/extra"; do
  if [ -d "$base" ]; then
    find "$base" -name 'ec_*.ko*' -exec rm -f {} \;
    find "$base" -type d -empty -delete 2>/dev/null || true
  fi
done

echo "========== 9. 更新模块依赖 =========="
depmod -a "$KVER"

echo "========== 卸载完成 =========="
echo "可执行以下命令确认已无 ethercat："
echo "  which ethercat     # 应无输出"
echo "  ldconfig -p | grep ethercat   # 应无输出"
echo "  modinfo ec_master  # 应提示模块不存在"
