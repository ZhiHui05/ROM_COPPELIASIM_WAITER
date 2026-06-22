# Parrubot 2 — Robot Camarero Autónomo con Deep Q-Learning

## Resumen

**Parrubot 2** es un robot autónomo TurtleBot3 Burger que, mediante **Deep Q-Network (DQN)** — un algoritmo de Aprendizaje por Refuerzo — aprende a navegar un entorno con obstáculos y servir 3 mesas en secuencia (1 → 2 → 3). El entrenamiento se realiza en el simulador **CoppeliaSim**, utilizando **ROS2** como middleware y **TensorFlow/Keras** para la red neuronal.

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
9. [Secuencia de servicio](#9-secuencia-de-servicio)
10. [Flujo de datos entre nodos ROS2](#10-flujo-de-datos-entre-nodos-ros2)
11. [La escena CoppeliaSim](#11-la-escena-coppeliasim)
12. [Estructura del proyecto](#12-estructura-del-proyecto)
13. [Compatibilidad de versiones](#13-compatibilidad-de-versiones)
14. [Visualización de resultados](#14-visualización-de-resultados)

---

## 1. Conceptos de Reinforcement Learning

### ¿Qué es DQN?

Deep Q-Network (DQN) es un algoritmo de aprendizaje por refuerzo que entrena una red neuronal para aproximar la **función Q(s, a)** — el valor esperado de tomar la acción `a` en el estado `s` y seguir la política óptima después.

### Componentes del sistema RL

| Componente | Implementación | Descripción |
|-----------|---------------|-------------|
| **Agente** | `dqn_agent.py` | El robot (TurtleBot3) que observa el estado y ejecuta acciones |
| **Entorno** | `dqn_environment.py` + CoppeliaSim | Proporciona el estado, ejecuta las acciones y devuelve la recompensa |
| **Política** | Red neuronal | Aproxima Q(s,a) para decidir qué acción tomar |
| **Recompensa** | `calculate_reward()` | Señal numérica que guía el aprendizaje |
| **Experiencia** | Replay Memory | Buffer de 500k transiciones (s, a, r, s', done) |

### Ciclo de entrenamiento

```
1. Observar estado s (distancia, ángulo, LIDAR, posición)
2. Elegir acción a (ε-greedy: explorar o explotar)
3. Ejecutar a en el entorno (el robot se mueve)
4. Recibir recompensa r y nuevo estado s'
5. Guardar (s, a, r, s') en Replay Memory
6. Entrenar la red con un mini-batch aleatorio de la memoria
7. Repetir hasta que el episodio termine (éxito, colisión o timeout)
```

### Ecuación de Bellman (actualización Q)

```
Q(s, a) ← r + γ · max Q(s', a')
```

- **r**: recompensa inmediata
- **γ** (gamma): factor de descuento = 0.99 — valora más las recompensas cercanas que las lejanas
- **max Q(s', a')**: mejor valor esperado desde el siguiente estado (según la target network)

### ε-greedy exploration

El robot explora (acciones aleatorias) con probabilidad ε, y explota (mejor acción según la red) con probabilidad 1−ε.

- ε inicial = 1.0 (100% exploración)
- ε decae exponencialmente
- ε mínimo = 0.05 (5% exploración residual)
- ε decae en ~12.000 pasos (≈ 120 episodios)

### Target Network

Para estabilizar el entrenamiento, se usa una **red objetivo** (target network) que es una copia de la red principal. Se actualiza cada 5.000 pasos. Esto evita que la red persiga un objetivo móvil (moving target problem).

### Experience Replay

En lugar de entrenar con transiciones consecutivas (que están correlacionadas), se entrena con mini-batches aleatorios de una memoria de 500.000 transiciones. Esto rompe la correlación temporal y mejora la estabilidad.

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

### Instalación de dependencias Python

```bash
pip install --upgrade numpy==1.26.4 scipy==1.10.1 tensorflow==2.19.0 keras==3.9.2 pyqtgraph
```

### Clonar y compilar el workspace

```bash
cd ~
git clone https://github.com/larmesto/RM_prac.git
cd RM_prac/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
cd ~/RM_prac
colcon build --symlink-install --allow-overriding turtlebot3_msgs
```

La opción `--symlink-install` crea enlaces simbólicos en `install/` que apuntan a `src/`. Así los cambios en el código Python se reflejan sin recompilar. La opción `--allow-overriding` usa la versión local de `turtlebot3_msgs`.

### Configurar el entorno

Añadir al final de `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=0
source ~/RM_prac/install/setup.bash
```

Recargar: `source ~/.bashrc`

---

## 3. Entrenamiento

### Script principal: `train_waiter.sh`

El script orquesta 4 procesos ROS2:

```
train_waiter.sh
  ├─ [1] CoppeliaSim (headless, scene_num=5)
  ├─ [2] dqn_coppeliasim (interfaz con el simulador)
  ├─ [3] dqn_environment (entorno RL: estado, recompensa)
  └─ [4] dqn_agent (agente DQN: red neuronal, entrenamiento)
```

### Comandos

```bash
# Entrenar desde cero (2000 episodios, GPU por defecto)
./train_waiter.sh

# Entrenar N episodios
./train_waiter.sh 5000

# Continuar desde un checkpoint
./train_waiter.sh 8000 model131.h5

# Sin GPU (CPU)
./train_waiter.sh 2000 "" cpu

# Con gráficas de progreso en vivo
./train_waiter.sh 2000 "" gpu viz
```

### Parámetros del script

| Argumento | Posición | Descripción | Default |
|-----------|----------|-------------|---------|
| `MAX_EPISODES` | 1 | Episodios totales | `2000` |
| `MODEL_FILE` | 2 | Checkpoint a cargar (vacío = desde cero) | `""` |
| `GPU` | 3 | `cpu` para desactivar GPU | `gpu` |
| `VIZ` | 4 | `viz` para abrir gráficas | `""` |

### Guardado automático

- **Cada 100 episodios**: checkpoint `modelN.h5` + `modelN.json` en `src/turtlebot3_dqn/saved_model/`
- **Al finalizar el entrenamiento**: checkpoint final con el número máximo de episodios
- El índice N auto-incrementa (model1, model2, ...)
- El JSON guarda: `epsilon`, `step_counter`, `trained_episodes`, `state_version`

---

## 4. Prueba de modelos

### Script: `test_models.sh`

Ejecuta el modelo entrenado en CoppeliaSim (con GUI) para ver su comportamiento:

```bash
./test_models.sh model131.h5
```

El robot ejecuta la política aprendida (ε = 0, solo explotación) e intenta completar la secuencia de 3 mesas.

---

## 5. Arquitectura de la red neuronal

### Estructura

```
Input(30) → Dense(512, ReLU) → Dense(256, ReLU) → Dense(128, ReLU) → Dense(5, linear)
```

La salida son 5 valores Q(s,a), uno por cada acción posible. La red aprende a predecir el valor esperado de cada acción dado el estado actual.

### Hiperparámetros

| Parámetro | Valor | Justificación |
|-----------|-------|---------------|
| Capas ocultas | 512, 256, 128 | Capacidad suficiente para mapear 30 entradas a políticas complejas |
| Activación | ReLU | Evita vanishing gradient, entrenamiento rápido |
| Salida | Linear | Los valores Q pueden ser cualquier número real |
| Optimizador | Adam | Adaptativo, buen rendimiento por defecto |
| Learning rate | 0.0007 | Suficientemente bajo para convergencia estable |
| LR schedule | ExponentialDecay (×0.96 cada 10k pasos) | Afinamiento progresivo |
| Gradient clipping | clipnorm=1.0 | Evita gradientes explosivos |
| Batch size | 128 | Balance entre estabilidad y velocidad |
| Pérdida | Mean Squared Error | Estándar para DQN |
| Descuento γ | 0.99 | Valora recompensas futuras casi tanto como las inmediatas |
| Replay memory | 500.000 transiciones | Memoria suficiente para ~500 episodios de experiencia |
| Min replay size | 5.000 transiciones | No entrena hasta tener suficiente experiencia variada |
| Target update | Cada 5.000 pasos | Estabiliza el aprendizaje |
| ε inicial | 1.0 | Exploración máxima al principio |
| ε mínimo | 0.05 | 5% de exploración residual |
| ε decay | 12.000 pasos | Transición suave de exploración a explotación |
| Eager execution | True | Requerido por TF 2.19 / Keras 3.9.2 |

---

## 6. El estado (State)

La red neuronal recibe **30 valores numéricos normalizados**. La normalización es crítica: valores en rangos similares ([0,1] o [−1,1]) permiten que el optimizador converja más rápido y de forma más estable.

### Tabla de entradas

| Índice | Variable | Descripción | Rango crudo | Normalización | Rango normalizado |
|--------|----------|-------------|-------------|---------------|-------------------|
| 0 | `goal_distance` | Distancia euclídea a la mesa objetivo | 0 ~ 7 m | `/5.0` | [0, ~1.4] |
| 1 | `goal_angle` | Ángulo entre orientación del robot y la mesa | −π ~ +π | `/π` | [−1, 1] |
| 2 | `table_index` | Qué mesa es el objetivo actual | 0, 1, 2 | `/2` | 0.0, 0.5, 1.0 |
| 3 | `robot_pose_x` | Posición X del robot en el mundo | −3.5 ~ +3.5 | `(+3.5)/7.0` | [0, 1] |
| 4 | `robot_pose_y` | Posición Y del robot en el mundo | −3.5 ~ +3.5 | `(+3.5)/7.0` | [0, 1] |
| 5–29 | `front_lidar` | 25 muestras del LIDAR frontal | 0 ~ 3.5 m | `/3.5` | [0, 1] |

### ¿Por qué estos 30 valores?

- **Distancia y ángulo**: información mínima para navegar hacia un punto
- **Índice de mesa**: la red necesita saber a qué mesa va (cada mesa está en una ubicación distinta y requiere una estrategia diferente)
- **Posición (x,y)**: el robot necesita saber dónde está en la habitación para planificar rutas entre mesas
- **LIDAR frontal**: detecta obstáculos en el hemisferio delantero (0° a 90° y 270° a 360°). Se remuestrea a 25 valores equidistantes para reducir dimensionalidad

### Lectura dinámica de coordenadas

Al iniciar el entorno, se llama al servicio Lua `table_goals` que devuelve las posiciones reales de los dummies `Goal_Mesa1/2/3` en la escena CoppeliaSim. Esto evita coordenadas hardcodeadas y permite modificar la escena sin cambiar el código.

---

## 7. Las acciones (Actions)

El espacio de acciones es **discreto** con 5 posibles velocidades angulares. La velocidad lineal es constante (0.2 m/s).

| Acción | ω (rad/s) | Movimiento resultante |
|--------|-----------|----------------------|
| 0 | +1.50 | Giro fuerte a la izquierda |
| 1 | +0.75 | Giro suave a la izquierda |
| 2 | 0.00 | Avanzar recto |
| 3 | −0.75 | Giro suave a la derecha |
| 4 | −1.50 | Giro fuerte a la derecha |

### ¿Por qué solo velocidad angular?

- Simplifica el espacio de acciones (5 en vez de 15 si tuviéramos 3 velocidades lineales × 5 angulares)
- La velocidad lineal constante (0.2 m/s) es segura para un robot TurtleBot3 en interiores
- El robot aprende a encadenar giros y avances para alcanzar cualquier posición
- Reducir el espacio de acciones acelera el aprendizaje

### Control de velocidad

Cada acción se ejecuta durante **0.8 segundos** (timer `stop_cmd_vel_timer`), tras lo cual el robot se detiene y espera la siguiente acción. Esto da tiempo suficiente para que el efecto de la acción sea observable en el siguiente estado.

---

## 8. Sistema de recompensa (Reward)

La recompensa es la señal más crítica del sistema: **lo que no está en la recompensa, el robot no lo aprende**.

### Principios de diseño

1. **Densa pero no abrumadora**: recompensa en cada paso para guiar continuamente
2. **Balanceada**: ningún componente debe dominar a los demás
3. **Sin castigos innecesarios**: orientarse mal no penaliza (solo se premia orientarse bien)
4. **Progresiva**: llegar más lejos da más recompensa
5. **Premios grandes por hitos**: alcanzar una mesa da una recompensa que supera cualquier penalización acumulada

### Recompensa por paso (step reward)

Cada ~0.8 segundos, el robot recibe una recompensa compuesta por 6 términos:

#### a) Orientación (yaw_reward)

```
yaw_reward = 1.0 − |goal_angle| / π
```

- **Rango**: [0, 1]
- **Sin castigo**: si el robot mira en dirección opuesta a la mesa (ángulo = π), recibe 0
- **Máximo**: si mira directamente a la mesa (ángulo = 0), recibe +1
- **Propósito**: guiar al robot para que se oriente hacia la mesa objetivo

#### b) Progreso hacia la mesa (distance_reward)

```
Δ = distancia_anterior − distancia_actual

Si Δ > 0 (acercándose):  distance_reward = +Δ × 15.0
Si Δ ≤ 0 (alejándose):   distance_reward = +Δ × 5.0
```

- **Acercarse premia 3× más** que lo que penaliza alejarse
- La asimetría es intencionada: alejarse a veces es necesario para esquivar obstáculos
- **Propósito**: incentivar trayectorias que reducen la distancia a la mesa

#### c) Obstáculos frontales (obstacle_reward)

Se activa cuando hay objetos a menos de **0.8 m** en el LIDAR frontal:

```
obstacle_reward = −(0.3 + 1.5 × weighted_decay)
```

- `weighted_decay` pondera los obstáculos según su dirección (más peso a los que están justo delante)
- **Rango**: [−0.3, −1.8]
- **Propósito**: enseñar al robot a mantener distancia de seguridad con obstáculos

#### d) Proximidad peligrosa (side_penalty)

```
side_penalty = −2.0  si min_obstacle_distance < 0.35 m (cualquier dirección)
side_penalty = 0.0   en caso contrario
```

- Usa **todos** los rayos LIDAR (360°), no solo los frontales
- Captura colisiones laterales que el obstacle_reward frontal podría no detectar
- **Propósito**: alertar de colisión inminente en cualquier dirección

#### e) Progreso récord (best_progress)

```
Si distancia_actual < mejor_distancia_del_episodio:
    reward += 0.5
    mejor_distancia = distancia_actual
```

- Solo se otorga una vez por cada nuevo récord de cercanía
- Se reinicia al cambiar de mesa objetivo
- **Propósito**: dar retroalimentación positiva frecuente al inicio, cuando el robot está lejos

#### f) Penalización por paso (step_penalty)

```
step_penalty = −0.02
```

- Penalización fija por cada acción tomada
- **Propósito**: incentivar eficiencia (llegar en menos pasos da más recompensa total)

### Eventos terminales

| Evento | Condición | Recompensa | Propósito |
|--------|-----------|------------|-----------|
| Alcanzar Mesa 1 | `dist < 0.5m` y `current_table = 0` | **+100** | Premiar el primer hito |
| Alcanzar Mesa 2 | `dist < 0.5m` y `current_table = 1` | **+100 + 50 extra** | Bonus progresivo por avanzar |
| Alcanzar Mesa 3 | `dist < 0.5m` y `current_table = 2` | **+100 + 300 éxito** | Recompensa máxima por completar |
| Colisión | `min_obstacle < 0.15m` | **−50** | Penalizar chocar |
| Timeout | 800 pasos sin completar | **−50** | Penalizar ineficiencia extrema |

### Estructura progresiva de bonus

```
Mesa 1:  +100
Mesa 2:  +100 + 50  = +150  (acumulado: +250)
Mesa 3:  +100 + 300 = +400  (acumulado: +650)
```

Cada mesa alcanzada da **el doble** que la penalización por fallo (−50). Esto garantiza que:
- Llegar a una mesa siempre es mejor que no intentarlo
- Avanzar en la secuencia da cada vez más recompensa
- El robot no aprende a "saltarse" mesas (no puede, el entorno lo impide)

### Ejemplo de recompensa en un episodio exitoso

```
Paso 1-100:   ~100 × (+0.5) = +50   (aproximándose a Mesa 1)
Alcanzar M1:  +100                   (bonus por mesa)
Paso 101-250: ~150 × (+0.3) = +45   (navegando a Mesa 2)
Alcanzar M2:  +150                   (bonus + extra)
Paso 251-400: ~150 × (+0.2) = +30   (navegando a Mesa 3)
Alcanzar M3:  +400                   (bonus + éxito)
──────────────────────────────────
Total:        ~+775
```

---

## 9. Secuencia de servicio

El robot debe visitar las mesas en orden estricto 1 → 2 → 3:

```
Inicio → Mesa 1 → Mesa 2 → Mesa 3 → Éxito
```

### Control de secuencia

La secuencia está **forzada por el entorno**, no por la recompensa:

1. El objetivo inicial es siempre Mesa 1 (`self.current_table = 0`)
2. Cuando `goal_distance < 0.5 m`, se incrementa `current_table`
3. Si `current_table < 3`, el nuevo objetivo es la siguiente mesa
4. Si `current_table >= 3`, el episodio termina con éxito
5. Si ocurre colisión o timeout, el episodio termina con fallo

### Reinicio entre episodios

Al comenzar un nuevo episodio, se resetea:
- `current_table = 0` (objetivo vuelve a Mesa 1)
- `goal_pose_x/y = coordenadas de Mesa 1`
- `best_goal_distance = ∞`
- Posición del robot (vía `reset_simulation` en CoppeliaSim)

### Coordenadas de las mesas

Las posiciones se leen **dinámicamente** de la escena CoppeliaSim mediante el servicio `table_goals`. En la escena por defecto:

| Mesa | Posición (x, y) |
|------|----------------|
| Mesa 1 | (3.0, 3.0) |
| Mesa 2 | (0.0, −3.0) |
| Mesa 3 | (−3.0, 0.0) |

---

## 10. Flujo de datos entre nodos ROS2

El sistema se compone de 4 nodos ROS2 que se comunican mediante servicios y tópicos:

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
│  - new_goal client   │  │  - cmd_vel publisher     │
└─────────────────────┘  └──────────┬───────────────┘
                                    │ Dqn.Request(action)
                                    │ Dqn.Response(state, reward, done)
                            ┌───────┴───────────────┐
                            │  dqn_agent             │
                            │  - process() loop      │
                            │  - get_action()        │
                            │  - train_model()       │
                            │  - replay_memory       │
                            │  - neural network      │
                            └───────────────────────┘
```

### Ciclo de un paso

```
1. dqn_agent:     elige acción con ε-greedy
2. dqn_agent:     envía Dqn.Request(action) → rl_agent_interface
3. dqn_environment:  recibe action, publica /cmd_vel
4. CoppeliaSim:   mueve el robot, publica /odom y /scan
5. dqn_environment:  lee /odom y /scan, calcula estado y recompensa
6. dqn_environment:  devuelve Dqn.Response(state, reward, done)
7. dqn_agent:     guarda (s, a, r, s') en replay memory
8. dqn_agent:     entrena la red con mini-batch
9. Repetir o terminar episodio
```

---

## 11. La escena CoppeliaSim

### Archivo

- **Nombre**: `turtlebot3_burger_ROS2_dqn_waiter.ttt`
- **Número de escena**: 5 (`scene_num:=5`)
- **Ubicación**: `src/turtlebot3_coppeliasim/scenes/`

### Elementos de la escena

| Objeto | Nombre | Función |
|--------|--------|---------|
| Robot | TurtleBot3 Burger | Agente móvil con LIDAR y odometría |
| Mesa 1 | `Goal_Mesa1` | Dummy en (3, 3) — primera mesa a visitar |
| Mesa 2 | `Goal_Mesa2` | Dummy en (0, −3) — segunda mesa |
| Mesa 3 | `Goal_Mesa3` | Dummy en (−3, 0) — tercera mesa |
| Flag | `/goal` | Indicador visual que sigue la mesa objetivo actual |
| Obstáculos | Varios | Paredes y objetos entre las mesas |

### Lua Script: `waiter_goals.lua`

Script hijo de la escena que expone servicios ROS2:

| Servicio | Tipo | Función |
|----------|------|---------|
| `/new_goal` | `std_srvs/Trigger` | Avanza a la siguiente mesa (1→2→3→1) y devuelve sus coordenadas |
| `/reset_simulation` | `std_srvs/Empty` | Reinicia el robot y el goal a Mesa 1 |
| `table_goals` | `std_srvs/Trigger` | Devuelve las coordenadas reales de las 3 mesas desde la escena |

### Lógica del flag `justReset`

Tras un `/reset_simulation`, la siguiente llamada a `/new_goal` **no avanza** de mesa. Esto evita que al reiniciar un episodio (reset + new_goal), el objetivo salte incorrectamente a Mesa 2.

### Headless mode

Para entrenamiento prolongado sin interfaz gráfica:

```bash
ros2 launch turtlebot3_coppeliasim turtlebot3_coppeliasim_dqn_headless.launch.py scene_num:=5
```

---

## 12. Estructura del proyecto

```
RM_prac/
├── README.md                       # Este documento
├── train_waiter.sh                 # Script de entrenamiento completo
├── test_models.sh                  # Script de prueba de modelos
│
├── src/
│   ├── turtlebot3_dqn/             # Paquete principal DQN
│   │   ├── turtlebot3_dqn/
│   │   │   ├── dqn_agent.py        # Agente DQN (red, entrenamiento, memoria)
│   │   │   ├── dqn_environment.py  # Entorno RL (estado, recompensa, sensores)
│   │   │   ├── dqn_test.py         # Prueba/validación de modelos
│   │   │   ├── dqn_coppeliasim.py  # Interfaz ROS2 ↔ servicios CoppeliaSim
│   │   │   ├── dqn_gazebo.py       # Interfaz alternativa para Gazebo
│   │   │   ├── result_graph.py     # Visualización PyQt5 de recompensa
│   │   │   └── action_graph.py     # Visualización PyQt5 de acciones
│   │   ├── saved_model/            # Checkpoints guardados (.h5 + .json)
│   │   ├── setup.py                # Configuración del paquete ROS2
│   │   └── package.xml
│   │
│   └── turtlebot3_coppeliasim/     # Integración con CoppeliaSim
│       ├── launch/
│       │   ├── turtlebot3_coppeliasim_dqn.launch.py        # Con GUI
│       │   └── turtlebot3_coppeliasim_dqn_headless.launch.py # Sin GUI
│       ├── scenes/
│       │   └── turtlebot3_burger_ROS2_dqn_waiter.ttt       # Escena camarero
│       └── scripts/
│           ├── waiter_goals.lua     # Control de mesas (script hijo)
│           └── waiter_table_goals.lua # Servicio de coordenadas
│
├── install/                        # Build instalado (colcon)
├── build/                          # Archivos de compilación
└── log/                            # Logs de compilación
```

---

## 13. Compatibilidad de versiones

El sistema ha evolucionado a través de 2 versiones del formato de estado. Para no perder modelos entrenados, se implementa **conversión automática**.

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

| Versión | Estado | Entradas | Época |
|---------|--------|----------|-------|
| **v1** (sin `state_version`) | Crudo: dist (0–7), ángulo (−π,π), LIDAR (0–3.5) | 27–29 | Pre-junio 2026 |
| **v2** (`state_version: 2`) | Normalizado: todo en [0,1] o [−1,1], +posición robot | 30 | Actual |

### Función `adapt_state()`

Al cargar un modelo, se lee `state_version`:

- **v2** (actual): el estado del entorno ya es compatible, solo se ajusta la longitud (pad/trim)
- **v1** (antiguo): desnormaliza dist×5, ángulo×π, LIDAR×3.5 y elimina los campos `table_index`, `robot_x`, `robot_y`

Esto permite **seguir entrenando desde modelos antiguos** sin perder el progreso. Los nuevos checkpoints se guardan como v2.

---

## 14. Visualización de resultados

### Gráficas en tiempo real

```bash
# Evolución de la recompensa por episodio
ros2 run turtlebot3_dqn result_graph

# Acciones tomadas por el robot (heatmap)
ros2 run turtlebot3_dqn action_graph
```

Estas gráficas usan PyQtGraph y se abren como ventanas independientes. También se lanzan automáticamente con `./train_waiter.sh ... viz`.

### TensorBoard

Los logs de entrenamiento se guardan en `~/turtlebot3_dqn_logs/gradient_tape/`:

```bash
tensorboard --logdir ~/turtlebot3_dqn_logs/gradient_tape
```

Métricas disponibles:
- `dqn_reward`: recompensa total por episodio
- La recompensa debería tender a aumentar conforme avanza el entrenamiento

### Interpretación de resultados

| Observación | Significado |
|-------------|-------------|
| Recompensa negativa persistente | El robot no está aprendiendo; revisar balance de recompensa |
| Recompensa sube lentamente | Aprendizaje en progreso; continuar entrenamiento |
| Recompensa se estanca en ~200–300 | El robot alcanza 1–2 mesas pero no completa las 3 |
| Recompensa > 400 consistente | El robot completa las 3 mesas regularmente |
| Score muy negativo (> −300) | El robot colisiona mucho; necesita más entrenamiento |

---

## Referencias

- [ROBOTIS-GIT/turtlebot3_machine_learning](https://github.com/ROBOTIS-GIT/turtlebot3_machine_learning) — Paquete original
- [TurtleBot3 Machine Learning Tutorial](https://emanual.robotis.com/docs/en/platform/turtlebot3/machine_learning/)
- [Playing Atari with Deep Reinforcement Learning](https://arxiv.org/abs/1312.5602) — Paper original de DQN (Mnih et al., 2013)
- [CoppeliaSim ROS2 Interface](https://manual.coppeliarobotics.com/en/simROS2.htm)
