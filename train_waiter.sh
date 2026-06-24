#!/bin/bash
###########################################################################
# Script de entrenamiento DQN para la escena waiter (3 mesas)
# Escena: turtlebot3_burger_ROS2_dqn_waiter.ttt (scene_num:=5)
#
# Uso:
#   ./train_waiter.sh                              # entrenar desde cero (2000 episodios, GPU)
#   ./train_waiter.sh 1000                         # entrenar desde cero con N episodios
#   ./train_waiter.sh 1000 "" cpu                  # entrenar sin GPU
#   ./train_waiter.sh 1000 model42.h5              # reanudar desde checkpoint
#   ./train_waiter.sh 1000 model42.h5 cpu          # reanudar sin GPU
#   ./train_waiter.sh 1000 model42.h5 gpu viz      # con visualizacion
#
# Requiere:
#   - CoppeliaSim instalado y en el PATH
#   - ROS2 Humble con el workspace compilado
#   - export TURTLEBOT3_MODEL=burger
###########################################################################

set -e

MAX_EPISODES="${1:-2000}"
MODEL_FILE="${2:-}"
USE_GPU="${3:-gpu}"        # por defecto usa GPU, pasar "cpu" para desactivar
WITH_VIZ="${4:-}"       # pasar "viz" para abrir graficas

SCENE_NUM=5
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "============================================"
echo " Entrenamiento DQN - Camarero 2 mesas (sin orden)"
echo "============================================"
echo " Workspace  : $SCRIPT_DIR"
echo " Escena     : $SCENE_NUM (waiter)"
echo " Episodios  : $MAX_EPISODES"
echo " Epsilon decay: 20000"
echo " max_step   : 600"
echo " action_dur : 0.3s"
echo " Red neuronal: [512,256,128]"
if [ -n "$MODEL_FILE" ]; then
    echo " Checkpoint : $MODEL_FILE"
else
    echo " Checkpoint : ninguno (desde cero)"
fi
if [ "$USE_GPU" = "cpu" ]; then
    echo " GPU        : no (CPU)"
else
    echo " GPU        : activada"
fi
if [ "$WITH_VIZ" = "viz" ]; then
    echo " Graficas   : activadas"
fi
echo "============================================"

# 0. Limpiar procesos previos
echo ""
echo "[0] Limpiando procesos previos..."
pkill -f coppeliaSim 2>/dev/null || true
pkill -f dqn_coppeliasim 2>/dev/null || true
pkill -f dqn_environment 2>/dev/null || true
pkill -f dqn_agent 2>/dev/null || true
pkill -f action_graph 2>/dev/null || true
pkill -f result_graph 2>/dev/null || true
sleep 1

# 1. Source del entorno
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash" 2>/dev/null || source "$SCRIPT_DIR/install/local_setup.bash"
export TURTLEBOT3_MODEL=burger

# 2. Lanzar CoppeliaSim en modo headless
echo ""
echo "[1] Lanzando CoppeliaSim headless (scene_num:=$SCENE_NUM)..."
echo "    $ ros2 launch turtlebot3_coppeliasim turtlebot3_coppeliasim_dqn_headless.launch.py scene_num:=$SCENE_NUM"

ros2 launch turtlebot3_coppeliasim turtlebot3_coppeliasim_dqn_headless.launch.py scene_num:=$SCENE_NUM &
COPPELIA_PID=$!

echo "[2] Esperando a que CoppeliaSim cargue la escena (30s)..."
sleep 30

if ! kill -0 $COPPELIA_PID 2>/dev/null; then
    echo "ERROR: CoppeliaSim no se inicio correctamente"
    exit 1
fi
echo "    CoppeliaSim iniciado (PID: $COPPELIA_PID)"

# 3. Interfaz CoppeliaSim
echo ""
echo "[3] Iniciando dqn_coppeliasim..."
echo "    $ ros2 run turtlebot3_dqn dqn_coppeliasim"
ros2 run turtlebot3_dqn dqn_coppeliasim &
COPPELIA_IFACE_PID=$!
sleep 2

# 4. Entorno RL
echo ""
echo "[4] Iniciando dqn_environment..."
echo "    $ ros2 run turtlebot3_dqn dqn_environment --ros-args -p max_step:=600"
ros2 run turtlebot3_dqn dqn_environment --ros-args -p max_step:=600 &
ENV_PID=$!
sleep 2

# 5. Visualizacion (opcional)
if [ "$WITH_VIZ" = "viz" ]; then
    echo ""
    echo "[5a] Iniciando graficas de progreso..."
    echo "     $ ros2 run turtlebot3_dqn result_graph"
    ros2 run turtlebot3_dqn result_graph &
    RESULT_VIZ_PID=$!
    echo "     $ ros2 run turtlebot3_dqn action_graph"
    ros2 run turtlebot3_dqn action_graph &
    ACTION_VIZ_PID=$!
    sleep 1
fi

# 6. Agente DQN (foreground)
echo ""
echo "============================================"
echo " INICIANDO ENTRENAMIENTO"
echo "============================================"
echo ""

# Construir argumentos del agente
AGENT_ARGS=()
AGENT_ARGS+=(-p "epsilon_decay:=20000")
AGENT_ARGS+=(-p "max_training_episodes:=$MAX_EPISODES")
AGENT_ARGS+=(-p "verbose:=false")
AGENT_ARGS+=(-p "lidar_samples:=25")

if [ "$USE_GPU" != "cpu" ]; then
    AGENT_ARGS+=(-p "use_gpu:=true")
fi

if [ -n "$MODEL_FILE" ]; then
    AGENT_ARGS+=(-p "model_file:=$MODEL_FILE")
fi

echo "    $ ros2 run turtlebot3_dqn dqn_agent --ros-args ${AGENT_ARGS[*]}"
echo ""

ros2 run turtlebot3_dqn dqn_agent --ros-args "${AGENT_ARGS[@]}"

# 7. Limpiar al terminar
echo ""
echo "============================================"
echo " Entrenamiento finalizado"
echo "============================================"
echo ""
echo "[7] Cerrando procesos..."
kill $COPPELIA_IFACE_PID 2>/dev/null || true
kill $ENV_PID 2>/dev/null || true
kill $COPPELIA_PID 2>/dev/null || true
if [ "$WITH_VIZ" = "viz" ]; then
    kill $RESULT_VIZ_PID 2>/dev/null || true
    kill $ACTION_VIZ_PID 2>/dev/null || true
fi
sleep 2

pkill -f coppeliaSim 2>/dev/null || true
pkill -f dqn_coppeliasim 2>/dev/null || true
pkill -f dqn_environment 2>/dev/null || true
pkill -f dqn_agent 2>/dev/null || true
pkill -f action_graph 2>/dev/null || true
pkill -f result_graph 2>/dev/null || true

echo ""
echo "Modelos guardados en:"
echo "  $SCRIPT_DIR/src/turtlebot3_dqn/saved_model/"
echo ""
echo "Para ver resultados con TensorBoard:"
echo "  tensorboard --logdir ~/turtlebot3_dqn_logs/gradient_tape"
echo ""
echo "Para probar un modelo:"
echo "  ros2 run turtlebot3_dqn dqn_test --ros-args -p model_file:=modelN.h5 -p verbose:=true"
echo "  (con GPU: añadir -p use_gpu:=true)"
