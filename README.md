# Parrubot 2 — Robot Camarero Autónomo con Deep Q-Learning

Robot autónomo TurtleBot3 Burger que aprende mediante **Deep Q-Network (DQN)** a servir 2 mesas cualesquiera (sin orden fijo) en un entorno simulado con CoppeliaSim, evitando obstáculos. El robot **no recibe un goal explícito**: aprende a dirigirse a las mesas guiado exclusivamente por la recompensa.

---

## Índice

1. [Conceptos de Reinforcement Learning](#1-conceptos-de-reinforcement-learning)
2. [Formulación matemática](#2-formulación-matemática)
3. [Requisitos e instalación](#3-requisitos-e-instalación)
4. [Entrenamiento](#4-entrenamiento)
5. [Prueba de modelos](#5-prueba-de-modelos)
6. [Arquitectura de la red neuronal](#6-arquitectura-de-la-red-neuronal)
7. [El estado (State)](#7-el-estado-state)
8. [Las acciones (Actions)](#8-las-acciones-actions)
9. [Sistema de recompensa (Reward)](#9-sistema-de-recompensa-reward)
10. [La tarea: 2 mesas sin orden fijo](#10-la-tarea-2-mesas-sin-orden-fijo)
11. [Flujo de datos entre nodos ROS2](#11-flujo-de-datos-entre-nodos-ros2)
12. [La escena CoppeliaSim](#12-la-escena-coppeliasim)
13. [Estructura del proyecto](#13-estructura-del-proyecto)
14. [Compatibilidad de versiones](#14-compatibilidad-de-versiones)
15. [Visualización de resultados](#15-visualización-de-resultados)

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

## 2. Formulación matemática

### 2.1 Algoritmo DQN

#### Ecuación de Bellman (Q-learning)

El objetivo del agente es aprender la **función de valor óptima** Q\*(s, a), que representa la recompensa esperada acumulada al tomar la acción `a` en el estado `s` y seguir la política óptima después:

$$Q^\ast(s, a) = r + \gamma \cdot \max_{a'} Q^\ast(s', a')$$

Donde:
- **r** : recompensa inmediata al tomar la acción `a`
- **γ** = 0.99 : factor de descuento — pondera más el presente que el futuro
- **s'** : estado resultante tras ejecutar `a`
- **max Q\*(s', a')** : máximo valor esperado desde `s'`

#### Double DQN (reducción de sobreestimación)

El DQN clásico usa la misma red para elegir y evaluar la acción, lo que produce **sobreestimación sistemática** de los valores Q. Double DQN desacopla ambas operaciones:

$$a^\ast = \underset{a'}{\arg\max}\; Q_{online}(s', a')$$

$$Q(s, a) \leftarrow r + \gamma \cdot Q_{target}(s', a^\ast)$$

La red **online** elige la mejor acción (`argmax`), y la red **target** evalúa su valor. Esto reduce el sesgo de sobreestimación y acelera la convergencia.

#### ε-greedy exploration

El agente explora (acción aleatoria) con probabilidad **ε** y explota (mejor acción) con probabilidad **1−ε**:

$$a = \begin{cases} \text{random}(0, 4) & \text{con probabilidad } \varepsilon \\ \underset{a}{\arg\max}\; Q(s, a) & \text{con probabilidad } 1 - \varepsilon \end{cases}$$

ε decae exponencialmente:

$$\varepsilon = \varepsilon_{min} + (1 - \varepsilon_{min}) \cdot e^{-N / \tau}$$

Donde:
- **εₘᵢₙ** = 0.05
- **N** = `step_counter` (pasos totales ejecutados)
- **τ** = `epsilon_decay` = 20.000 (constante de decaimiento)

#### Función de pérdida (MSE)

La red neuronal se entrena minimizando el error cuadrático medio entre el valor Q predicho y el valor objetivo calculado con la ecuación de Bellman:

$$\mathcal{L} = \frac{1}{B} \sum_{i=1}^{B} \left( Q(s_i, a_i) - y_i \right)^2$$

$$y_i = \begin{cases} r_i & \text{si } done_i = \text{True} \\ r_i + \gamma \cdot Q_{target}(s'_i, a^\ast_i) & \text{si } done_i = \text{False} \end{cases}$$

Donde **B** = 128 (batch size).

#### Learning Rate con decaimiento exponencial

$$\eta_t = \eta_0 \cdot d^{\lfloor t / s \rfloor}$$

- **η₀** = 0.0007 (learning rate inicial)
- **d** = 0.96 (decay rate)
- **s** = 10.000 (decay steps)
- **t** = pasos de entrenamiento

#### Gradient Clipping

Para evitar gradientes explosivos durante la retropropagación:

$$\| \nabla \mathcal{L} \| \leq 1.0$$

Si la norma del gradiente supera 1.0, se reescala proporcionalmente.

---

### 2.2 Geometría del robot

#### Distancia a una mesa

Distancia euclídea entre el robot `(rx, ry)` y la mesa `(tx, ty)`:

$$d_i = \sqrt{(x_{mesa_i} - x_{robot})^2 + (y_{mesa_i} - y_{robot})^2}$$

#### Ángulo hacia una mesa

Dirección del vector robot → mesa:

$$\theta_{path} = \text{atan2}(y_{mesa} - y_{robot},\; x_{mesa} - x_{robot})$$

Ángulo relativo entre la orientación del robot y la dirección a la mesa:

$$\phi_i = \theta_{path} - \theta_{robot}$$

Normalizado al rango **[−π, π]** mediante:

$$\phi_i = \begin{cases} \phi_i - 2\pi & \text{si } \phi_i > \pi \\ \phi_i + 2\pi & \text{si } \phi_i < -\pi \\ \phi_i & \text{en otro caso} \end{cases}$$

---

### 2.3 Normalización del estado

Todos los valores del estado se normalizan para que estén en rangos comparables [0, 1] o [−1, 1]. Esto mejora la convergencia del optimizador Adam:

| Variable | Fórmula | Rango |
|----------|---------|-------|
| Distancia | `min(d / 5.0, 1.0)` | [0, 1] |
| Ángulo | `φ / π` | [−1, 1] |
| Posición X | `(rx + 3.5) / 7.0` | [0, 1] |
| Posición Y | `(ry + 3.5) / 7.0` | [0, 1] |
| LIDAR | `min_d / 3.5` | [0, 1] |

#### Remuestreo LIDAR

El LIDAR produce N lecturas en el rango frontal (0°–90° y 270°–360°). Se remuestrea a exactamente 25 valores equidistantes mediante interpolación lineal:

$$\mathrm{lidar}[k] = \mathrm{interp}\!\left(\frac{k}{24} \cdot (N-1),\ \mathrm{frontRanges}\right), \quad k = 0, 1, \dots, 24$$

---

### 2.4 Sistema de recompensa

#### Recompensa por orientación hacia una mesa

Factor de orientación con **mínimo garantizado** de 0.2 para evitar zonas sin gradiente:

$$f_{orient}(\phi_i) = 0.2 + 0.8 \cdot \left(1 - \frac{|\phi_i|}{\pi}\right)$$

- **φᵢ = 0** (perfectamente alineado): `f = 0.2 + 0.8 × 1.0 = 1.0`
- **φᵢ = π** (dirección opuesta): `f = 0.2 + 0.8 × 0.0 = 0.2`

#### Factor de distancia

$$f_{dist}(d_i) = \frac{1}{\sqrt{1 + d_i}}$$

- **d = 0**: `f = 1.0`
- **d = 3m**: `f = 0.5`
- **d = 8m**: `f ≈ 0.33`

#### Recompensa total por mesas

Para cada mesa `i` **no visitada**:

$$R_{mesas} = \sum_{i \notin \text{visitadas}} \left[ f_{orient}(\phi_i) \cdot f_{dist}(d_i) \cdot 3.0 + \delta_i \cdot 2.0 \right]$$

Donde **δᵢ = 1** si `dᵢ < best_dist[i]` (nuevo récord de cercanía), y **δᵢ = 0** en caso contrario.

#### Penalización por obstáculos frontales

Solo se activa si hay obstáculos a menos de 0.8m en el LIDAR frontal:

$$R_{obs} = -\left(0.3 + 1.5 \cdot w_{decay}\right)$$

Donde `w_decay` es una suma ponderada por la dirección del obstáculo respecto al robot:

$$w_{decay} = \sum_{j \in \text{cercanos}} w_j \cdot \exp\left(-2.0 \cdot \max(r_j - 0.25,\; 0.01)\right)$$

Los pesos direccionales `wⱼ` favorecen obstáculos frontales sobre los laterales:

$$w_j = \frac{\cos^6(\theta_j) + 0.1}{\sum_k (\cos^6(\theta_k) + 0.1)}$$

#### Recompensa total por paso

$$R_{paso} = R_{mesas} + R_{obs} + R_{side} + R_{step} + R_{surv}$$

| Término | Fórmula | Valor típico |
|---------|---------|-------------|
| `R_mesas` | Ecuación arriba | +0.3 a +4.1 |
| `R_obs` | `−(0.3 + 1.5 × w_decay)` | 0 a −1.8 |
| `R_side` | `−2.0` si min_obst < 0.35m | 0 o −2.0 |
| `R_step` | `−0.02` | −0.02 |
| `R_surv` | `+0.1` cada 25 pasos sin colisión | 0 o +0.1 |

---

### 2.5 Actualización del optimizador (Adam)

El optimizador Adam actualiza los pesos **θ** de la red usando momentos de primer y segundo orden:

$$m_t = \beta_1 \cdot m_{t-1} + (1 - \beta_1) \cdot \nabla \mathcal{L}$$

$$v_t = \beta_2 \cdot v_{t-1} + (1 - \beta_2) \cdot (\nabla \mathcal{L})^2$$

$$\hat{m}_t = \frac{m_t}{1 - \beta_1^t}, \quad \hat{v}_t = \frac{v_t}{1 - \beta_2^t}$$

$$\theta_t = \theta_{t-1} - \eta_t \cdot \frac{\hat{m}_t}{\sqrt{\hat{v}_t} + 10^{-7}}$$

Con parámetros por defecto: β₁ = 0.9, β₂ = 0.999.

---

## 3. Requisitos e instalación

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

## 4. Entrenamiento

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

## 5. Prueba de modelos

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

## 6. Arquitectura de la red neuronal

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

## 7. El estado (State)

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

## 8. Las acciones (Actions)

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

## 9. Sistema de recompensa (Reward)

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

## 10. La tarea: 2 mesas sin orden fijo

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

## 11. Flujo de datos entre nodos ROS2

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

## 12. La escena CoppeliaSim

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

## 13. Estructura del proyecto

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

## 14. Compatibilidad de versiones

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

## 15. Visualización de resultados

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
