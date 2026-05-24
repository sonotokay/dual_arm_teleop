#!/usr/bin/env bash
set -e
CM="--controller-manager /controller_manager"

echo "Spawning position controllers (inactive)..."
ros2 run controller_manager spawner left_arm_position_controller  $CM --inactive
ros2 run controller_manager spawner right_arm_position_controller $CM --inactive

echo "Switching both arms to position controllers..."
ros2 control switch_controllers $CM \
  --deactivate left_arm_controller  --activate left_arm_position_controller
ros2 control switch_controllers $CM \
  --deactivate right_arm_controller --activate right_arm_position_controller

echo "Starting + unpausing both servo nodes..."
for side in left right; do
  ros2 service call /${side}_servo_node/start_servo   std_srvs/srv/Trigger {}
  ros2 service call /${side}_servo_node/unpause_servo std_srvs/srv/Trigger {}
done

echo "Done. Verifying:"
ros2 control list_controllers $CM