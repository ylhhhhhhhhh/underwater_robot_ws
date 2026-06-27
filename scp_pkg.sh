#!/bin/bash
set -e

# ===================== 【请手动修改这里的配置】=====================
ROBOT_IP="192.168.2.101"
ROBOT_USER="elf"
ROBOT_TARGET_PATH="/home/elf/underwater_robot_ws/src"
IGNORE_PKG_LIST="robot_description serial_ros2 cabin_gui"
# =================================================================

LOCAL_SRC_PATH="$(pwd)/src"
echo "============================================="
echo "本地src路径: $LOCAL_SRC_PATH"
echo "机器人地址: $ROBOT_USER@$ROBOT_IP:$ROBOT_TARGET_PATH"
echo "忽略的功能包: [$IGNORE_PKG_LIST]"
echo "============================================="

if [ ! -d "$LOCAL_SRC_PATH" ]; then
    echo "错误：当前目录没有src文件夹，请在ROS工作空间根目录执行脚本！"
    exit 1
fi

cd "$LOCAL_SRC_PATH"
pkg_count=0
skip_count=0

# 建立SSH持久连接，全程复用连接，只握手一次
SSH_CONTROL_PATH="$HOME/.ssh/ssh_socket_robot"
ssh -M -S "$SSH_CONTROL_PATH" -fnN ${ROBOT_USER}@${ROBOT_IP}

# 遍历传输
for pkg_dir in */; do
    pkg_name="${pkg_dir%/}"
    skip_flag=0
    for ignore_name in $IGNORE_PKG_LIST; do
        if [ "$pkg_name" = "$ignore_name" ]; then
            skip_flag=1
            break
        fi
    done

    if [ $skip_flag -eq 1 ]; then
        echo "[跳过忽略] $pkg_name"
        skip_count=$((skip_count+1))
        continue
    fi

    echo "[开始传输] $pkg_name ..."
    # 复用已有SSH通道
    scp -o ControlPath="$SSH_CONTROL_PATH" -r "$pkg_name" "${ROBOT_USER}@${ROBOT_IP}:${ROBOT_TARGET_PATH}/"
    pkg_count=$((pkg_count+1))
done

# 关闭持久SSH连接
ssh -S "$SSH_CONTROL_PATH" -O exit ${ROBOT_USER}@${ROBOT_IP}

echo "============================================="
echo "执行完成！"
echo "成功传输包数量: $pkg_count"
echo "忽略跳过包数量: $skip_count"
echo "============================================="

