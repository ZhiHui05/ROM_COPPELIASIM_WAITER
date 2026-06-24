#!/bin/bash
###########################################################################
# Script de prueba de modelos DQN - Camarero 2 mesas (sin orden fijo)
# Escena: turtlebot3_burger_ROS2_dqn_waiter.ttt (scene_num:=5)
#
# Uso:
#   ./test_models.sh model131.h5                  # probar infinitos episodios
#   ./test_models.sh model131.h5 false false 10   # probar 10 episodios con estadisticas
#   ./test_models.sh model131.h5 gpu              # con GPU
#
# Requiere:
#   - CoppeliaSim instalado y en el PATH
#   - ROS2 con el workspace compilado
#   - export TURTLEBOT3_MODEL=burger
###########################################################################

set -e

MODEL_FILE="${1:-}"
USE_GPU="${2:-false}"
VERBOSE="${3:-false}"
MAX_EPISODES="${4:-0}"

if [ -z "$MODEL_FILE" ]; then
    echo "Uso: ./test_models.sh <model_file> [use_gpu] [verbose] [max_episodes]"
    echo "  model_file:   nombre del modelo (ej: model131.h5)"
    echo "  use_gpu:      true/false (default: false)"
    echo "  verbose:      true/false (default: false)"
    echo "  max_episodes: numero de episodios (default: 0 = infinito)"
    exit 1
fi

SCENE_NUM=5
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "============================================"
echo " Prueba de Modelo DQN - Escena Waiter"
echo "============================================"
echo " Workspace  : $SCRIPT_DIR"
echo " Escena     : $SCENE_NUM (waiter)"
echo " Modelo     : $MODEL_FILE"
echo " GPU        : $USE_GPU"
echo " Verbose    : $VERBOSE"
if [ "$MAX_EPISODES" -gt 0 ] 2>/dev/null; then
    echo " Episodios  : $MAX_EPISODES"
else
    echo " Episodios  : ilimitados (Ctrl+C para salir)"
fi
echo "============================================"

# 0. Limpiar procesos previos
echo ""
echo "[0] Limpiando procesos previos..."
pkill -f coppeliaSim 2>/dev/null || true
pkill -f dqn_coppeliasim 2>/dev/null || true
pkill -f dqn_environment 2>/dev/null || true
pkill -f dqn_test 2>/dev/null || true
sleep 1

# 1. Source del entorno
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash" 2>/dev/null || source "$SCRIPT_DIR/install/local_setup.bash"
export TURTLEBOT3_MODEL=burger

# 2. Lanzar CoppeliaSim con GUI
echo ""
echo "[1] Lanzando CoppeliaSim (scene_num:=$SCENE_NUM)..."
ros2 launch turtlebot3_coppeliasim turtlebot3_coppeliasim_dqn.launch.py scene_num:=$SCENE_NUM &
COPPELIA_PID=$!

echo "    Esperando a que CoppeliaSim cargue la escena (30s)..."
sleep 30

if ! kill -0 $COPPELIA_PID 2>/dev/null; then
    echo "ERROR: CoppeliaSim no se inicio correctamente"
    exit 1
fi
echo "    CoppeliaSim iniciado (PID: $COPPELIA_PID)"

# 3. Interfaz CoppeliaSim
echo ""
echo "[2] Iniciando dqn_coppeliasim..."
ros2 run turtlebot3_dqn dqn_coppeliasim &
COPPELIA_IFACE_PID=$!
sleep 2

# 4. Entorno RL
echo ""
echo "[3] Iniciando dqn_environment..."
ros2 run turtlebot3_dqn dqn_environment --ros-args -p lidar_samples:=25 &
ENV_PID=$!
sleep 2

# 5. Test del modelo (foreground)
echo ""
echo "============================================"
echo " INICIANDO PRUEBA DEL MODELO"
echo "============================================"
echo ""

ros2 run turtlebot3_dqn dqn_test --ros-args \
    -p model_file:="$MODEL_FILE" \
    -p use_gpu:="$USE_GPU" \
    -p verbose:="$VERBOSE" \
    -p max_episodes:="$MAX_EPISODES"

# 6. Limpiar al terminar
echo ""
echo "============================================"
echo " Prueba finalizada"
echo "============================================"
echo ""
echo "[6] Cerrando procesos..."
kill $COPPELIA_IFACE_PID 2>/dev/null || true
kill $ENV_PID 2>/dev/null || true
kill $COPPELIA_PID 2>/dev/null || true
sleep 2

pkill -f coppeliaSim 2>/dev/null || true
pkill -f dqn_coppeliasim 2>/dev/null || true
pkill -f dqn_environment 2>/dev/null || true
pkill -f dqn_test 2>/dev/null || true
