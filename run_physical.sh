#!/bin/bash
# Wrapper para el launch fisico.
# Cuando se presiona Ctrl+C:
#   1. Publica Twist cero a /cmd_vel (el robot para)
#   2. Luego manda SIGINT al launch para apagado limpio
#
# Uso:  bash run_physical.sh [args]
# Ej:   bash run_physical.sh nav:=false use_rviz:=false

_stop_robot() {
    echo ""
    echo ">>> Ctrl+C: enviando stop a /cmd_vel..."
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    echo ">>> Stop enviado. Cerrando launch..."
    kill -INT "$_LAUNCH_PID" 2>/dev/null
    wait "$_LAUNCH_PID" 2>/dev/null
    exit 0
}

trap _stop_robot INT TERM

# El subshell ignora INT para que el Ctrl+C llegue solo al trap de arriba,
# no al launch directamente — asi el robot recibe el stop antes de morir.
(trap '' INT; exec ros2 launch final_challenge physical_challenge.launch.py "$@") &
_LAUNCH_PID=$!
wait "$_LAUNCH_PID"
