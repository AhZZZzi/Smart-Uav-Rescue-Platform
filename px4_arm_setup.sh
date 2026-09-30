#!/bin/bash
# Đợi PX4 khởi động xong rồi tự động set params
sleep 15
echo "Setting PX4 arming params..."
cd ~/PX4-Autopilot

./build/px4_sitl_default/bin/px4-commander param set COM_RCL_EXCEPT 7
./build/px4_sitl_default/bin/px4-commander param set COM_RC_IN_MODE 4
./build/px4_sitl_default/bin/px4-commander param set CBRK_FLIGHTTERM 121212
./build/px4_sitl_default/bin/px4-commander param set CBRK_SUPPLY_CHK 894281
./build/px4_sitl_default/bin/px4-commander param set CBRK_IO_SAFETY 22027
./build/px4_sitl_default/bin/px4-commander param set COM_ARM_WO_GPS 1
./build/px4_sitl_default/bin/px4-commander param set MAV_0_BROADCAST 1
./build/px4_sitl_default/bin/px4-commander param set COM_DISARM_PRFLT 0
./build/px4_sitl_default/bin/px4-commander param save
echo "Params saved. Ready to arm."
