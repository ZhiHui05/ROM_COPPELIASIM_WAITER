# Parrubot 2 — Robot Camarero Autónomo con Deep Q-Learning

Robot autónomo TurtleBot3 Burger que aprende mediante **Deep Q-Network (DQN)** a servir 2 mesas cualesquiera (sin orden fijo) en un entorno simulado con CoppeliaSim, evitando obstáculos. El robot **no recibe un goal explícito**: aprende a dirigirse a las mesas guiado exclusivamente por la recompensa.

---

## Índice

1. [Conceptos de Reinforcement Learning](#1-conceptos-de-reinforcement-learning)
2. [Requisitos e instalación](#2-requisitos-e-instalación)
3. [Entrenamiento](#3-entrenamiento)
4. [Prueba de modelos](#4-prueba-de-modelos)
5. [Arquitectura de la red neuronal](#5-arquitectura-de-la-red-neuronal)
6. [El estado (State)](#6-el-estado-state)
7. [Las acciones (Actions)](#7-las-acciones-actions)
8. [Sistema de recompensa (Reward)](#8-sistema-de-recompensa-reward)
9. [La tarea: 2 mesas sin orden fijo](#9-la-tarea-2-mesas-sin-orden-fijo)
10. [Flujo de datos entre nodos ROS2](#10-flujo-de-datos-entre-nodos-ros2)
11. [La escena CoppeliaSim](#11-la-escena-coppeliasim)
12. [Estructura del proyecto](#12-estructura-del-proyecto)
13. [Compatibilidad de versiones](#13-compatibilidad-de-versiones)
14. [Visualización de resultados](#14-visualización-de-resultados)

---

## 1. Conceptos de Reinforcement Learning

### ¿Qué es DQN?

Deep Q-Network (DQN) es un algoritmo de aprendizaje por refuerzo que entrena una red neuronal para aproximar la **función Q(s, a)** — el valor esperado de tomar la acción `a` en el estado `s` y seguir la política óptima después. Se usa **Double DQN** para reducir la sobreestimación de valores Q.

### Componentes del sistema RL

| Componente | Implementación | Descripción |
|-----------|---------------|-------------|
| **Agente** | `dqn_agent.py` | El robot (TurtleBot3) que observa el estado y ejecuta acciones |
| **Entorno** | `dqn_environment.py` + CoppeliaSim | Proporciona el estado, ejecuta las acciones y devuelve la recompensa |
| **Política** | Red neuronal | Aproxima Q(s,a) para decidir qué acción tomar |
| **Recompensa** | `calculate_reward()` | Señal numérica que guía el aprendizaje — sin goal fijo |
| **Experiencia** | Replay Memory | Buffer de 500k transiciones (s, a, r, s', done) |

### Ciclo de entrenamiento

```
1. Observar estado s (36 entradas: distancias y ángulos a las 3 mesas, flags, LIDAR...)
2. Elegir acción a (ε-greedy: explorar o explotar)
3. Ejecutar a en el entorno (el robot se mueve)
4. Recibir recompensa r y nuevo estado s'
5. Guardar (s, a, r, s') en Replay Memory
6. Entrenar la red con un mini-batch aleatorio de la memoria (Double DQN)
7. Repetir hasta que el episodio termine (éxito, colisión o timeout)
```

### Double DQN (actualización Q)

```
a* = argmax Q_online(s')
Q(s, a) ← r + γ × Q_target(s')[a*]
```

- La red **online** elige la mejor acción → `argmax Q_online(s')`
- La red **target** evalúa el valor de esa acción → `Q_target(s')[a*]`
- **r**: recompensa inmediata
- **γ** (gamma): factor de descuento = 0.99

### ε-greedy exploration

- ε inicial = 1.0 (100% exploración)
- ε decae exponencialmente en 20.000 pasos
- ε mínimo = 0.05 (5% exploración residual)

### Target Network

Una copia de la red principal que se actualiza cada **3.000 pasos** para estabilizar el entrenamiento.

### Experience Replay

Mini-batches aleatorios de una memoria de **500.000 transiciones**. Rompe la correlación temporal entre muestras.

---

## 2. Requisitos e instalación

### Software necesario

| Componente | Versión |
|-----------|---------|
| Ubuntu | 22.04 |
| ROS2 | Humble (o Jazzy) |
| CoppeliaSim | Edu 4.9.0 rev6 |
| Python | 3.10 |
| TensorFlow | 2.19.0 |
| Keras | 3.9.2 |
| NumPy | 1.26.4 |

### Instalación de dependencias

```bash
pip install --upgrade numpy==1.26.4 scipy==1.10.1 tensorflow==2.19.0 keras==3.9.2 pyqtgraph
```

### Clonar y compilar

```bash
cd ~
git clone https://github.com/larmesto/RM_prac.git
cd RM_prac/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
cd ~/RM_prac
colcon build --symlink-install --allow-overriding turtlebot3_msgs
```

### Configurar el entorno

Añadir a `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=0
source ~/RM_prac/install/setup.bash
```

---

## 3. Entrenamiento

### Script: `train_waiter.sh`

Orquesta 4 procesos ROS2:

```
train_waiter.sh
  ├─ [1] CoppeliaSim (headless, scene_num=5)
  ├─ [2] dqn_coppeliasim (interfaz con el simulador)
  ├─ [3] dqn_environment (entorno RL: estado, recompensa)
  └─ [4] dqn_agent (agente DQN: red neuronal, entrenamiento)
```

### Comandos

```bash
# Desde cero (2000 episodios, GPU por defecto)
./train_waiter.sh

# N episodios
./train_waiter.sh 5000

# Continuar desde checkpoint
./train_waiter.sh 4000 modelN.h5

# Sin GPU
./train_waiter.sh 2000 "" cpu

# Con gráficas de progreso
./train_waiter.sh 2000 "" gpu viz
```

### Guardado automático

- **Cada 100 episodios** + guardado **final**
- Ruta: `src/turtlebot3_dqn/saved_model/modelN.h5` + `.json`
- JSON guarda: `epsilon`, `step_counter`, `trained_episodes`, `state_version`

### Progreso esperado con 2000 episodios

| Fase | Episodios | Qué aprende |
|------|-----------|------------|
| Descubrimiento | 0–200 | Asociar "acercarse a mesas = bueno" |
| Navegación básica | 200–800 | Girar + avanzar hacia la mesa más cercana, esquivar obstáculos |
| Transición | 800–1500 | Tras 1ª mesa, buscar la siguiente no visitada. 6 caminos posibles |
| Consolidación | 1500–2000 | Completar 2 mesas en 40–70% de episodios |

---

## 4. Prueba de modelos

### Script: `test_models.sh`

```bash
# 10 episodios con estadísticas
./test_models.sh modelN.h5 false false 10

# Sin límite (Ctrl+C para salir)
./test_models.sh modelN.h5
```

### Output del test

```
Ep    1 | EXITO | Score:   512.3 | Pasos:  280 | Mesas: 2/2 | Exito: 100.0% | Score prom:  512.3
Ep    2 | FALLO | Score:  -138.5 | Pasos:  600 | Mesas: 1/2 | Exito:  50.0% | Score prom:  186.9
...
=================================================================
 RESUMEN FINAL
=================================================================
  Episodios totales:     10
  Exitos (2 mesas):      7  (70.0%)
  Fallos:                3
  Score promedio:        245.3
  Score max:             612.5
  Score min:             -138.5
  Tasa 1 mesa:           90.0%
  Tasa 2 mesas:          70.0%
=================================================================
```

---

## 5. Arquitectura de la red neuronal

### Estructura

```
Input(36) → Dense(1024, ReLU) → Dense(512, ReLU) → Dense(256, ReLU) → Dense(5, linear)
```

### Hiperparámetros

| Parámetro | Valor | Justificación |
|-----------|-------|---------------|
| Capas ocultas | 1024, 512, 256 | Capacidad para 36 entradas |
| Activación | ReLU | Evita vanishing gradient |
| Salida | Linear | Los valores Q pueden ser cualquier número real |
| Optimizador | Adam (`clipnorm=1.0`, LR decay ×0.96/10k pasos) | Gradientes estables, afinamiento progresivo |
| Learning rate inicial | 0.0007 | Convergencia estable |
| Batch size | 128 | Balance estabilidad/velocidad |
| Pérdida | MSE | Estándar para DQN |
| Descuento γ | 0.99 | Valora recompensas futuras |
| Replay memory | 500.000 | ~500 episodios de experiencia |
| ε inicial → mínimo | 1.0 → 0.05 | Exploración → explotación |
| ε decay | 20.000 pasos | Más exploración por estado complejo |
| Target update | Cada 3.000 pasos | Estabilización más frecuente |
| Algoritmo | Double DQN | Reduce sobreestimación |
| Eager execution | True | Compatibilidad TF 2.19 |

---

## 6. El estado (State)

La red neuronal recibe **36 valores normalizados** que representan el panorama completo de las 3 mesas, sin un goal fijo:

| Índices | Variable | Descripción | Rango crudo | Normalización | Rango |
|---------|----------|-------------|-------------|---------------|-------|
| 0 | `dist_m1` | Distancia a Mesa 1 | 0 ~ 5 m | `/5.0` | [0, 1] |
| 1 | `dist_m2` | Distancia a Mesa 2 | 0 ~ 5 m | `/5.0` | [0, 1] |
| 2 | `dist_m3` | Distancia a Mesa 3 | 0 ~ 5 m | `/5.0` | [0, 1] |
| 3 | `angle_m1` | Ángulo hacia Mesa 1 | −π ~ +π | `/π` | [−1, 1] |
| 4 | `angle_m2` | Ángulo hacia Mesa 2 | −π ~ +π | `/π` | [−1, 1] |
| 5 | `angle_m3` | Ángulo hacia Mesa 3 | −π ~ +π | `/π` | [−1, 1] |
| 6 | `visited_1` | ¿Mesa 1 visitada? | 0/1 | — | 0.0 o 1.0 |
| 7 | `visited_2` | ¿Mesa 2 visitada? | 0/1 | — | 0.0 o 1.0 |
| 8 | `visited_3` | ¿Mesa 3 visitada? | 0/1 | — | 0.0 o 1.0 |
| 9 | `robot_x` | Posición X del robot | −3.5 ~ +3.5 | `(+3.5)/7.0` | [0, 1] |
| 10 | `robot_y` | Posición Y del robot | −3.5 ~ +3.5 | `(+3.5)/7.0` | [0, 1] |
| 11–35 | `lidar` | LIDAR frontal (25 muestras) | 0 ~ 3.5 m | `/3.5` | [0, 1] |

### ¿Por qué este diseño?

- **Sin goal fijo**: el robot ve todas las mesas simultáneamente. La recompensa lo guía hacia cualquiera.
- **Distancias y ángulos a las 3 mesas**: la red sabe exactamente dónde está cada mesa respecto al robot.
- **Flags de visitadas**: la red sabe qué mesas ya fueron servidas y dejan de dar recompensa.
- **Posición (x,y)**: el robot sabe dónde está en la habitación para planificar rutas.
- **LIDAR frontal**: detecta obstáculos delanteros a 0–90° y 270–360°, remuestreado a 25 valores.

---

## 7. Las acciones (Actions)

5 velocidades angulares discretas con velocidad lineal constante (0.2 m/s):

| Acción | ω (rad/s) | Movimiento resultante |
|--------|-----------|----------------------|
| 0 | +1.50 | Giro fuerte a la izquierda |
| 1 | +0.75 | Giro suave a la izquierda |
| 2 | 0.00 | Avanzar recto |
| 3 | −0.75 | Giro suave a la derecha |
| 4 | −1.50 | Giro fuerte a la derecha |

Cada acción se ejecuta durante **0.8 segundos**, tras lo cual el robot frena y espera la siguiente orden.

---

## 8. Sistema de recompensa (Reward)

**Principio fundamental**: no hay goal fijo. La recompensa por paso premia acercarse y orientarse hacia **cualquier** mesa no visitada. El robot aprende solo a qué mesa conviene ir.

### Recompensa por paso (~0.8s)

```python
reward = table_reward + obstacle_reward + step_penalty + side_penalty + survival_bonus
```

#### a) `table_reward` — Premio por mirar + acercarse a mesas no visitadas

```python
for cada mesa NO visitada:
    orient_factor = 0.2 + 0.8 × (1 − |ángulo| / π)   # nunca baja de 0.2
    dist_factor   = 1 / √(1 + distancia)
    table_reward += orient_factor × dist_factor × 3.0

    if distancia < récord_personal (en este episodio):
        table_reward += 2.0
```

| Situación | `table_reward` por paso |
|-----------|------------------------|
| Lejos (3m) y desorientado | `0.2 × 0.5 × 3.0 = 0.30` |
| Lejos (3m) y bien orientado | `1.0 × 0.5 × 3.0 = 1.50` |
| Cerca (1m) y bien orientado | `1.0 × 0.71 × 3.0 = 2.12` |
| Cerca y batiendo récord | `2.12 + 2.0 = 4.12` |

**Clave**: el `orient_factor` nunca baja de 0.2, así que incluso completamente desorientado (180°) hay señal de gradiente. Sin zonas muertas.

#### b) `obstacle_reward` — Obstáculos frontales (< 0.8m)

```python
obstacle_reward = −(0.3 + 1.5 × weighted_decay)   # [−0.3, −1.8]
```

Pondera los obstáculos según su dirección angular usando pesos direccionales.

#### c) `side_penalty` — Alerta de colisión inminente

```python
side_penalty = −2.0   si min_obstáculo < 0.35m (cualquier dirección, 360°)
side_penalty =  0.0   si no
```

#### d) `step_penalty` — Penalización fija por ineficiencia

```python
step_penalty = −0.02
```

#### e) `survival_bonus` — Premio por no chocar

```python
survival_bonus = +0.1 cada 25 pasos sin colisionar ni alcanzar mesa
```

Incentiva evitar obstáculos y mantener trayectorias seguras.

### Eventos terminales

| Evento | Condición | Recompensa |
|--------|-----------|------------|
| Alcanzar 1ª mesa | Dist < 0.6m a cualquier mesa no visitada | **+100** |
| Alcanzar 2ª mesa | Dist < 0.6m a la segunda mesa | **+300** |
| Éxito (2 mesas) | Bonus final | **+200** extra |
| Colisión | Obstáculo < 0.15m | **−50** |
| Timeout | 600 pasos sin completar | **−50** |

**Total máximo por episodio: ~700**

### Balance de incentivos

- +100 por 1ª mesa >> −50 por colisión → siempre vale la pena intentarlo
- +300 por 2ª mesa + 200 éxito >> penalizaciones acumuladas → completar la tarea siempre es rentable
- `survival_bonus` +0.1/25 pasos → recompensa por trayectorias seguras

---

## 9. La tarea: 2 mesas sin orden fijo

### Cómo funciona

El robot **no recibe instrucciones** de a qué mesa ir. El estado incluye las 3 mesas y la recompensa premia estar cerca y orientado hacia **cualquier** mesa no visitada:

```
Estado: [dist_m1, dist_m2, dist_m3, angle_m1, angle_m2, angle_m3, visited_1/2/3, ...]
         ↓
Recompensa: +por mirar a M1, +por mirar a M2, +por mirar a M3
            (más recompensa cuanto más cerca estés de cada una)
         ↓
Robot: "M2 está más cerca y mejor orientada → voy a M2"
         ↓
Alcanza M2 → +100 → M2 se marca visitada → deja de dar recompensa
         ↓
Robot: "Ahora M1 y M3 dan recompensa → M1 está más cerca → voy a M1"
         ↓
Alcanza M1 → +300 + 200 → Tarea completada
```

### Caminos posibles

Cualquier combinación de 2 mesas en cualquier orden es válida:

```
M1 → M2    M1 → M3
M2 → M1    M2 → M3
M3 → M1    M3 → M2
```

En cada episodio, el robot descubre por sí mismo qué camino tomar. La recompensa lo guía naturalmente.

### Detección de mesa alcanzada

Se considera que el robot ha llegado a una mesa cuando la distancia euclídea entre el robot y el dummy de la mesa es < **0.6 metros**. La detección se hace para **todas** las mesas no visitadas en cada paso — no solo para una "meta" fija.

### Coordenadas de las mesas

Cargadas dinámicamente desde la escena CoppeliaSim mediante el servicio `table_goals`. Los dummies en la escena son `Goal_Mesa1`, `Goal_Mesa2`, `Goal_Mesa3`. Si el servicio no está disponible, se usan las coordenadas por defecto:

| Mesa | Hardcoded |
|------|-----------|
| Mesa 1 | (3.0, 3.0) |
| Mesa 2 | (0.0, −3.0) |
| Mesa 3 | (−3.0, 0.0) |

### Control de episodios

- **Éxito**: `tables_visited_count >= 2` → episodio termina con bonus +200
- **Colisión**: `min_obstacle_distance < 0.15m` → episodio termina con −50
- **Timeout**: `local_step >= max_step` (600 por defecto) → episodio termina con −50
- **Reinicio**: al empezar nuevo episodio → `visited_tables = [False, False, False]`, `tables_visited_count = 0`

---

## 10. Flujo de datos entre nodos ROS2

```
┌──────────────────────────────────────────────────────────┐
│                     CoppeliaSim                          │
│  ┌─────────────────────────────────────────────────┐    │
│  │  waiter_goals.lua                                │    │
│  │  Servicios: /new_goal, /reset_simulation,        │    │
│  │             table_goals                          │    │
│  │  Tópicos: /odom, /scan, /cmd_vel                │    │
│  └─────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────────┘
         ↑ Servicios              ↑ Tópicos (odom/scan)
         │                        │
┌────────┴────────────┐  ┌───────┴──────────────────┐
│  dqn_coppeliasim    │  │  dqn_environment          │
│  - task_succeed      │  │  - calculate_state()     │
│  - task_failed       │  │  - calculate_reward()    │
│  - initialize_env    │  │  - rl_agent_interface    │
└─────────────────────┘  └──────────┬───────────────┘
                                    │ Dqn.Request(action)
                                    │ Dqn.Response(state, reward, done)
                            ┌───────┴───────────────┐
                            │  dqn_agent             │
                            │  - process() loop      │
                            │  - get_action()        │
                            │  - train_model()       │
                            │  - Double DQN          │
                            │  - replay_memory       │
                            └───────────────────────┘
```

### Ciclo de un paso

```
1. dqn_agent:     elige acción con ε-greedy
2. dqn_agent:     envía Dqn.Request(action) → rl_agent_interface
3. dqn_environment:  publica /cmd_vel
4. CoppeliaSim:   mueve el robot, publica /odom y /scan
5. dqn_environment:  calcula estado (36 entradas) y recompensa
6. dqn_environment:  devuelve Dqn.Response(state, reward, done)
7. dqn_agent:     guarda transición en replay memory
8. dqn_agent:     entrena con Double DQN (mini-batch aleatorio)
9. Repetir o terminar episodio
```

---

## 11. La escena CoppeliaSim

### Elementos

| Objeto | Nombre | Función |
|--------|--------|---------|
| Robot | TurtleBot3 Burger | Agente móvil con LIDAR y odometría |
| Mesa 1 | `Goal_Mesa1` | Dummy en (3, 3) |
| Mesa 2 | `Goal_Mesa2` | Dummy en (0, −3) |
| Mesa 3 | `Goal_Mesa3` | Dummy en (−3, 0) |
| Flag | `/goal` | Indicador visual decorativo |
| Obstáculos | Varios | Paredes y objetos entre las mesas |

### Lua Script: `waiter_goals.lua`

Servicios ROS2 expuestos desde la escena:

| Servicio | Tipo | Función |
|----------|------|---------|
| `/new_goal` | `std_srvs/Trigger` | Avanza el flag visual a la siguiente mesa |
| `/reset_simulation` | `std_srvs/Empty` | Reinicia el robot al origen |
| `table_goals` | `std_srvs/Trigger` | Devuelve coordenadas reales de las 3 mesas |

### Headless mode

```bash
ros2 launch turtlebot3_coppeliasim turtlebot3_coppeliasim_dqn_headless.launch.py scene_num:=5
```

---

## 12. Estructura del proyecto

```
RM_prac/
├── README.md                       # Este documento
├── train_waiter.sh                 # Entrenamiento completo
├── test_models.sh                  # Prueba de modelos
│
├── src/
│   ├── turtlebot3_dqn/             # Paquete DQN
│   │   ├── turtlebot3_dqn/
│   │   │   ├── dqn_agent.py        # Agente DQN (red, Double DQN, memoria)
│   │   │   ├── dqn_environment.py  # Entorno RL (estado, recompensa, sensores)
│   │   │   ├── dqn_test.py         # Prueba con estadísticas por episodio
│   │   │   ├── dqn_coppeliasim.py  # Interfaz ROS2 ↔ CoppeliaSim
│   │   │   ├── result_graph.py     # Gráfica de recompensa (PyQt5)
│   │   │   └── action_graph.py     # Gráfica de acciones (PyQt5)
│   │   ├── saved_model/            # Checkpoints (.h5 + .json)
│   │   └── setup.py
│   │
│   └── turtlebot3_coppeliasim/
│       ├── launch/
│       │   ├── turtlebot3_coppeliasim_dqn.launch.py
│       │   └── turtlebot3_coppeliasim_dqn_headless.launch.py
│       ├── scenes/
│       │   └── turtlebot3_burger_ROS2_dqn_waiter.ttt
│       └── scripts/
│           └── waiter_goals.lua     # Control de mesas en CoppeliaSim
```

---

## 13. Compatibilidad de versiones

### Metadato `state_version`

Cada checkpoint guarda en su `.json`:

```json
{
  "epsilon": 0.05,
  "step_counter": 154868,
  "trained_episodes": 2000,
  "state_version": 2
}
```

| Versión | Formato | Entradas |
|---------|---------|----------|
| **v1** | Estado crudo (distancia, ángulo, LIDAR) — single goal | 27–29 |
| **v2** | Estado normalizado (3 mesas, flags, posición, LIDAR) — multi-goal | 36 |

### `adapt_state()` — conversión automática

- **v1 → v2**: desnormaliza (×5, ×π, ×3.5) y elimina campos nuevos (flags, posición)
- **v2 → v2**: solo ajusta longitud (pad/trim)
- Se aplica al cargar modelos viejos para seguir entrenando o probando

---

## 14. Visualización de resultados

### Gráficas en tiempo real

```bash
# Recompensa por episodio
ros2 run turtlebot3_dqn result_graph

# Acciones tomadas (heatmap)
ros2 run turtlebot3_dqn action_graph
```

También se lanzan con `./train_waiter.sh ... viz`.

### TensorBoard

```bash
tensorboard --logdir ~/turtlebot3_dqn_logs/gradient_tape
```

### Output durante entrenamiento

```
Step: 50,  Goal: M2, Dist: 1.42, Angle: -0.62, Mesas: 0/2, Total: 1.01, Obst: 0.00, Tables: 1.03, Surv: 50
Step: 100, Goal: M1, Dist: 1.12, Angle: 1.18, Mesas: 0/2, Total: 2.04, Obst: 0.00, Tables: 2.06, Surv: 100
...
Episode: 42 score: 487.3 memory length: 3824 epsilon: 0.71
```

| Campo | Significado |
|-------|------------|
| `Goal: M2` | Mesa no visitada con mejor combinación de cercanía + orientación (informativo, no fuerza al robot) |
| `Dist / Angle` | Distancia y ángulo hacia esa mesa |
| `Mesas: 1/2` | Mesas visitadas / necesarias |
| `Total` | Recompensa total del paso |
| `Obst` | Penalización por obstáculos |
| `Tables` | Recompensa acumulada de mesas no visitadas (sin bonuses) |
| `Surv` | Pasos consecutivos sin colisionar |

### Interpretación

| Observación | Significado |
|-------------|-------------|
| `Tables` > 1.5 consistentemente | Robot bien orientado hacia alguna mesa |
| `Obst` distinto de 0 | Hay obstáculos cercanos (< 0.8m) |
| `Surv` alto | Buena evasión de obstáculos |
| `Total` negativo recurrente | Muchas colisiones o mala orientación |
| Score > 400 | Probablemente completó las 2 mesas |

---

## Referencias

- [ROBOTIS-GIT/turtlebot3_machine_learning](https://github.com/ROBOTIS-GIT/turtlebot3_machine_learning)
- [TurtleBot3 Machine Learning Tutorial](https://emanual.robotis.com/docs/en/platform/turtlebot3/machine_learning/)
- [Playing Atari with Deep Reinforcement Learning](https://arxiv.org/abs/1312.5602) — DQN (Mnih et al., 2013)
- [Deep Reinforcement Learning with Double Q-learning](https://arxiv.org/abs/1509.06461) — Double DQN (van Hasselt et al., 2015)
- [CoppeliaSim ROS2 Interface](https://manual.coppeliarobotics.com/en/simROS2.htm)
