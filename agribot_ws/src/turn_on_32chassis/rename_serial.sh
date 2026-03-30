#如何使用：
#sudo chmod 777 rename_serial.sh
#sudo sh rename_serial.sh 
# 如何删除对应的规则：
# sudo rm -f /etc/udev/rules.d/agribot_serial.rules

# 重命名串口 #CH343，将其命名为 agribot_serial
echo 'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="agribot_serial"' > /etc/udev/rules.d/agribot_serial.rules

# 添加对 ttyACM 设备的支持
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="agribot_serial"' >> /etc/udev/rules.d/agribot_serial.rules

service udev reload
sleep 2
service udev restart