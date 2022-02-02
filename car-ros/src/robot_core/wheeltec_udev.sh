echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="stm32_port"' >/etc/udev/rules.d/robot_controller.rules

service udev reload
sleep 2
service udev restart


