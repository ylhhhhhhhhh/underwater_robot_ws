#!/bin/bash

# 针对你的 CP2102 设备（serial=0001）设置别名，避免冲突
RULE_FILE="/etc/udev/rules.d/99-wheeltec-imu-gnss.rules"

# 写入规则（仅适配你的设备，其他芯片规则保留但注释，需用时再启用）
sudo tee $RULE_FILE <<-'EOF'
# 你的 CP2102 设备（idVendor=10c4, idProduct=ea60, serial=0001）
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", MODE:="0660", GROUP:="dialout", SYMLINK+="wheeltec_FDI_IMU_GNSS"

# 以下为其他芯片备用规则（如需使用，删除 # 并修改 serial 为实际值）
# CH9102（已装驱动，需修改 serial）
# KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="你的设备serial", MODE:="0660", GROUP:="dialout", SYMLINK+="wheeltec_FDI_IMU_GNSS_CH9102"
# CH340（无 serial 限制，所有同型号设备）
# KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0660", GROUP:="dialout", SYMLINK+="wheeltec_FDI_IMU_GNSS_CH340"
EOF

# 重新加载规则并触发生效
sudo udevadm control --reload-rules
sudo udevadm trigger

# 添加当前用户到 dialout 组（避免权限问题）
sudo usermod -aG dialout $USER

echo "配置完成！"
echo "1. 请注销当前用户后重新登录（dialout 组权限生效）"
echo "2. 设备接入后，别名对应：/dev/wheeltec_FDI_IMU_GNSS"

