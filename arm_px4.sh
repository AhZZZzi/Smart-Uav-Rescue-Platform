#!/bin/bash
# Đợi PX4 khởi động xong rồi tự động set params qua mavlink
sleep 20
source /opt/ros/humble/setup.bash

ros2 param set /mavros COM_RC_IN_MODE 4 2>/dev/null || true

# Set params qua MAVROS2
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 \
  "{param_id: 'COM_RC_IN_MODE', value: {integer_value: 4}}" 2>/dev/null

ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 \
  "{param_id: 'COM_RCL_EXCEPT', value: {integer_value: 7}}"

ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 \
  "{param_id: 'NAV_RCL_ACT', value: {integer_value: 0}}"

ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 \
  "{param_id: 'COM_DISARM_PRFLT', value: {integer_value: 0}}"

ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 \
  "{param_id: 'CBRK_SUPPLY_CHK', value: {integer_value: 894281}}"

ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 \
  "{param_id: 'CBRK_FLIGHTTERM', value: {integer_value: 121212}}"

ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 \
  "{param_id: 'CBRK_USB_CHK', value: {integer_value: 197848}}"

echo "All params set! Saving..."
ros2 service call /mavros/param/pull mavros_msgs/srv/ParamPull "{force_pull: false}"

echo "Done — ready to arm!"
