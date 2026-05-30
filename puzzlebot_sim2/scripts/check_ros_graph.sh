#!/usr/bin/env bash
set -e

echo "=== Topics ==="
ros2 topic list

echo ""
echo "=== Required topics ==="
for t in /cmd_vel /odom /wr /wl /joint_states /tf /tf_static; do
  if ros2 topic list | grep -qx "$t"; then
    echo "OK   $t"
  else
    echo "MISS $t"
  fi
done

if ros2 topic list | grep -qx "/scan"; then
  echo "OK   /scan"
else
  echo "MISS /scan  <-- final_bug_nav needs this for obstacle avoidance"
fi

echo ""
echo "=== One-shot odom test ==="
ros2 topic echo /odom --once || true
