#!/bin/bash
set -e

# ===================== 【请手动修改这里的配置】=====================
# 机器人板卡IP（你网线直连的静态IP）
ROBOT_IP="192.168.2.101"
# 板卡用户名
ROBOT_USER="elf"
# 机器人上存放源码的目标路径（一般放 ~/ros2_ws/src）
ROBOT_TARGET_PATH="/home/elf/underwater_robot_ws/src"
# 需要【忽略不传输】的包名，空格隔开，精准匹配文件夹名
IGNORE_PKG_LIST="robot_description serial_ros2 cabin_gui"
# =================================================================

# 获取本机当前ws的src路径
LOCAL_SRC_PATH="$(pwd)/src"
echo "============================================="
echo "本地src路径: $LOCAL_SRC_PATH"
echo "机器人地址: $ROBOT_USER@$ROBOT_IP:$ROBOT_TARGET_PATH"
echo "忽略的功能包: [$IGNORE_PKG_LIST]"
echo "============================================="

# 判断本地src是否存在
if [ ! -d "$LOCAL_SRC_PATH" ]; then
    echo "错误：当前目录没有src文件夹，请在ROS工作空间根目录执行脚本！"
    exit 1
fi

# 进入src遍历一级文件夹（ROS功能包）
cd "$LOCAL_SRC_PATH"
pkg_count=0
skip_count=0

for pkg_dir in */; do
    # 去掉末尾斜杠，得到包名
    pkg_name="${pkg_dir%/}"

    # 判断是否在忽略列表
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

    # scp传输整个功能包文件夹到机器人
    echo "[开始传输] $pkg_name ..."
    scp -r "$pkg_name" "${ROBOT_USER}@${ROBOT_IP}:${ROBOT_TARGET_PATH}/"
    pkg_count=$((pkg_count+1))
done

echo "============================================="
echo "执行完成！"
echo "成功传输包数量: $pkg_count"
echo "忽略跳过包数量: $skip_count"
echo "============================================="
