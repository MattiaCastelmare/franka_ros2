# franka_simulation — Analisi Architetturale e Piano di Refactoring

> Data analisi: 2026-05-09  
> Branch: `humble-mattia`  
> Autore analisi: Claude (reverse engineering da codice sorgente)

---

## 1. Architettura Attuale

### 1.1 Overview Completa

Il sistema è composto da 6 package strettamente integrati:

| Package | Ruolo |
|---------|-------|
| `franka_simulation` | Launch + CBF avoidance pipeline in simulazione Gazebo |
| `franka_experiments` | Nodi sperimentali: velocity/torque commanders, CBF/OSCBF filters |
| `franka_description` | URDF/Xacro robot FR3, kinematics, meshes |
| `franka_bringup` | Launch robot reale + controller_manager |
| `franka_rt_controllers` | Controller C++ RT: RtVelocityExecutorController, RtTorqueController, CBFTorqueController |
| `franka_gazebo` | Plugin Gazebo Ignition (`franka_ign_ros2_control`) + launch examples |

### 1.2 Dependency Graph

```
franka_simulation ──────────────────────────────────────────────────────────┐
  depends on:                                                                │
    franka_description  (URDF/Xacro)                                        │
    franka_fr3_moveit_config  (kinematics, OMPL, RViz MoveIt config)        │
    ros_gz_sim  (Gazebo Ignition launch)                                     │
    ros_gz_bridge  (clock bridge /clock)                                     │
    franka_ign_ros2_control  (Gazebo hardware plugin)  ←── franka_gazebo    │
    franka_gazebo_bringup  (franka_gazebo_controllers.yaml hardcoded!)      │
    controller_manager, joint_state_broadcaster                              │
    velocity_controllers, joint_trajectory_controller                        │
    moveit_ros_move_group  (move_group node)                                 │
    pymoveit2  (Python action clients per motion server)                     │
    pinocchio  (FK/IK per avoidance controller)                              │
    realsense2_camera  (optional)                                            │

franka_experiments ──────────────────────────────────────────────────────────┤
  depends on:                                                                │
    franka_bringup  (franka.launch.py)                                       │
    franka_rt_controllers  (rt_velocity_executor_controller, rt_torque_controller) │
    franka_description  (URDF per Pinocchio)                                 │
    franka_msgs  (MultiLinkDistance, per CBF obstacle)                       │
    pinocchio, qpsolvers, osqp/cvxopt                                        │
```

### 1.3 Flow dei Dati — Pipeline CBF Corrente (move_group.launch.py)

```
                    ┌──────────────────────────────────────────────┐
                    │              Gazebo Ignition                 │
                    │  IgnitionROS2ControlPlugin (1000 Hz)         │
                    │    ├── joint_state_broadcaster               │
                    │    ├── fr3_arm_controller (INACTIVE)         │
                    │    └── fr3_velocity_controller (ACTIVE)      │
                    └────────────┬─────────────────────────────────┘
                                 │ /joint_states
                    ┌────────────▼─────────────────────────────────┐
                    │       robot_state_publisher                   │
                    │       (TF tree: fr3_link0..fr3_link8)        │
                    └────────────┬─────────────────────────────────┘
                                 │
          ┌──────────────────────▼──────────────────────┐
          │       franka_motion_server (MoveIt2)         │
          │  Action servers: move_to_pose, move_to_joint │
          │  ─ IK (avoid_collisions=False)               │
          │  ─ OMPL planning                             │
          │  → /velocity_blender/trajectory              │
          └─────────────────────────────────────────────┘

          ┌──────────────────────────────────────────────┐
          │       obstacle_synchronizer                   │
          │  reads URDF Xacro → parses obstacles         │
          │  → /obstacle_scene (avoidance)               │
          │  → /planning_scene (RViz)                    │
          │  → /collision_object (MoveIt)                │
          └─────────────────────────────────────────────┘
                          │
          ┌───────────────▼──────────────────────────────┐
          │       online_avoidance_controller (100 Hz)    │
          │  Pinocchio FK → capsule distances             │
          │  → /avoidance/min_distance                   │
          │  → /avoidance/closest_constraint (Jacobian)  │
          │  → /robot_capsules_markers (RViz)            │
          └─────────────────────────────────────────────┘
                          │
          ┌───────────────▼──────────────────────────────┐
          │       velocity_control_blender (100 Hz)       │
          │  riceve: /velocity_blender/trajectory         │
          │          /avoidance/closest_constraint        │
          │          /avoidance/min_distance              │
          │  algoritmo: Kp=6.0 tracking + CBF-QP safety  │
          │  → /fr3_velocity_controller/commands          │
          └─────────────────────────────────────────────┘
                          │
          ┌───────────────▼──────────────────────────────┐
          │  fr3_velocity_controller                      │
          │  (JointGroupVelocityController)               │
          │  command_interface: velocity[7]               │
          │  → Gazebo joint velocities                    │
          └─────────────────────────────────────────────┘
```

### 1.4 Integrazione Gazebo / RViz

**Gazebo Ignition** è avviato con `empty.sdf -r` (real-time mode).  
Il robot è spawnato via topic `/robot_description`.  
La variabile `GZ_SIM_RESOURCE_PATH` punta alla cartella parent di `franka_description` per trovare le mesh.

**Hardware plugin** in Gazebo:
```xml
<!-- In franka_arm.ros2_control.xacro, quando gazebo=true -->
<plugin filename="franka_ign_ros2_control-system"
        name="ign_ros2_control::IgnitionROS2ControlPlugin">
  <parameters>$(find franka_gazebo_bringup)/config/franka_gazebo_controllers.yaml</parameters>
</plugin>
```

**PROBLEMA CRITICO**: Il plugin Gazebo carica il file YAML di `franka_gazebo_bringup` hardcoded nel xacro, non il file yaml di `franka_simulation`. Questo file definisce solo i controller example di Franka. I controller reali (`fr3_velocity_controller`, ecc.) vengono aggiunti successivamente dagli spawner, che è corretto — ma l'`update_rate` e il `thread_priority` del controller_manager sono quelli del file di `franka_gazebo_bringup`.

**RViz2**: avviato con la config di `franka_fr3_moveit_config` se MoveIt è abilitato, oppure con `franka_simulation.rviz`. Fixed frame: `fr3_link0`.

**Clock Bridge**: `/clock` bridged da Gazebo a ROS2 via `ros_gz_bridge`.  
I nodi con `use_sim_time: True` usano il clock di Gazebo.

### 1.5 TF Tree

```
world
  └── base
        └── fr3_link0
              └── fr3_link1 → fr3_link2 → ... → fr3_link8
                                                    └── fr3_hand (se load_gripper=true)
                                                          └── fr3_hand_tcp

obstacle/world  (se spawn_obstacles=true)
  └── [TF obstacles]
```

I TF `world→base` e `base→fr3_link0` sono pubblicati da `static_transform_publisher` (identità).  
`fr3_link0` → ... → `fr3_link8` sono pubblicati da `robot_state_publisher` dai `/joint_states`.

---

## 2. Analisi Controller

### 2.1 `joint_state_broadcaster`

| Attributo | Valore |
|-----------|--------|
| Tipo | `joint_state_broadcaster/JointStateBroadcaster` |
| Ruolo | Pubblica `/joint_states` (position, velocity, effort per ogni joint) |
| Command interface | nessuna |
| State interface | position, velocity, effort (tutti i joint) |
| Status | **ATTIVO** — sempre necessario |
| Launch che lo usano | `franka_simulation.launch.py`, `move_group.launch.py` |
| Compatibilità velocity | ✅ necessario |
| Compatibilità acceleration | ✅ necessario |
| Compatibilità torque | ✅ necessario |

### 2.2 `fr3_arm_controller`

| Attributo | Valore |
|-----------|--------|
| Tipo | `joint_trajectory_controller/JointTrajectoryController` |
| Ruolo | Segue traiettorie JointTrajectory da MoveIt (action `follow_joint_trajectory`) |
| Command interface | **position** |
| State interface | position, velocity |
| Status | Spawnato **INATTIVO** in `move_group.launch.py` (con `--inactive`), non spawnato in `franka_simulation.launch.py` |
| Launch che lo usano | `franka_simulation.launch.py` (spawned ACTIVE), `move_group.launch.py` (spawned INACTIVE) |
| Compatibilità velocity | ❌ usa position |
| Compatibilità acceleration | ❌ usa position |
| Compatibilità torque | ❌ usa position |
| Note | Usato solo se si vuole MoveIt trajectory execution via position. Conflitto con `fr3_velocity_controller` (stesso hardware) |

**OSSERVAZIONE**: In `move_group.launch.py` il `delayed_arm` è commentato nella LaunchDescription finale. Quindi `fr3_arm_controller` NON viene mai spawnato nel launch corrente principale. È del codice di configurazione vestigiale.

### 2.3 `fr3_velocity_controller`

| Attributo | Valore |
|-----------|--------|
| Tipo | `velocity_controllers/JointGroupVelocityController` |
| Ruolo | Invia comandi di velocità ai 7 giunti FR3 |
| Command interface | **velocity** (7 joint) |
| State interface | position, velocity |
| Command topic | `/fr3_velocity_controller/commands` (Float64MultiArray, size=7) |
| Status | **ATTIVO** — il controller principale della pipeline CBF |
| Launch che lo usano | `move_group.launch.py`, `franka_simulation.launch.py` |
| Compatibilità velocity | ✅ **NATIVO** |
| Compatibilità acceleration | ⚠️ tramite nodo integratore esterno |
| Compatibilità torque | ❌ non applicabile |
| Config | `config/fr3_velocity_controller.yaml` |

### 2.4 `fr3_gripper`

| Attributo | Valore |
|-----------|--------|
| Tipo | `position_controllers/GripperActionController` |
| Ruolo | Controlla il gripper Franka |
| Command interface | position (fr3_finger_joint1) |
| Status | Definito in `controllers.yaml` ma **MAI SPAWNATO** nei launch correnti |
| Launch che lo usano | nessuno (legacy) |
| Compatibilità velocity/acceleration/torque | ❌ irrelevante |

### 2.5 `rt_velocity_executor_controller` (in `franka_rt_controllers`)

| Attributo | Valore |
|-----------|--------|
| Tipo | `franka_rt_controllers/RtVelocityExecutorController` |
| Ruolo | Controller C++ RT che riceve Float64MultiArray e applica rate-limiting + clamp |
| Command interface | **velocity** (7 joint) |
| Command topic | `qdot_cmd` (relativo al namespace) |
| Interpolazione | Opzionale (lineare tra campioni, elimina jitter sample-and-hold) |
| Rate limiter | `max_accel` [rad/s²] — previene salti di velocità |
| Clamp | `qdot_max` [rad/s] |
| Status | Usato nel robot **REALE** (via `franka_experiments/minimal.launch.py`) |
| Gazebo support | Ha flag `is_gazebo_` (salta `set_full_collision_behavior`) — **compatibile con Gazebo** |
| Compatibilità velocity | ✅ **NATIVO** — progettato per questo |
| Compatibilità acceleration | ❌ |
| Compatibilità torque | ❌ |
| Note | Superiore a `fr3_velocity_controller` per smoothness; da preferire per velocity simulation |

### 2.6 `rt_torque_controller` (in `franka_rt_controllers`)

| Attributo | Valore |
|-----------|--------|
| Tipo | C++ controller, `RtTorqueController` |
| Ruolo | Riceve torque (τ senza gravity), aggiunge g(q) via Pinocchio, applica effort |
| Command interface | **effort** (7 joint) |
| Command topic | Configurabile (default: `torque_cmd`, OSCBF: `torque_safe`) |
| LPF | alpha opzionale per filtraggio |
| Status | Usato nel robot **REALE** (via `franka_experiments/minimal.launch.py`) |
| Gazebo support | Ha flag `is_gazebo_` — **ATTENZIONE**: in Gazebo aggiunge g(q) che è già gestita dalla fisica |
| Compatibilità velocity | ❌ |
| Compatibilità acceleration | ⚠️ tramite upstream CBF filter (τ = M·q̈ + C·q̇) |
| Compatibilità torque | ✅ **NATIVO** ma con gravity compensation issue in Gazebo |
| Limiti Gazebo | L'effort interface in Gazebo è disabilitata per default (bug #343 in gz_ros2_control). Richiede `gazebo_effort:=true` nel xacro |

### 2.7 `cbf_torque_controller` (in `franka_rt_controllers`)

| Attributo | Valore |
|-----------|--------|
| Tipo | C++ controller, `CBFTorqueController` |
| Ruolo | Riceve qddot_safe, fa inverse dynamics (τ = M·q̈ + C·q̇ + g), applica effort |
| Command interface | **effort** (7 joint) |
| Input topic | `/NS_1/qddot_safe` |
| Status | **LEGACY** — rimpiazzato da `rt_torque_controller` + `cbf_safety_filter` |
| Note | Quasi identico a rt_torque_controller ma riceve accelerazioni invece di torchi |

### 2.8 Controller Franka Example (in `franka_gazebo_bringup`)

| Controller | Tipo | Command interface |
|-----------|------|------------------|
| `joint_position_example_controller` | franka_example_controllers | position |
| `joint_velocity_example_controller` | franka_example_controllers | velocity |
| `joint_impedance_example_controller` | franka_example_controllers | effort (impedance) |

Questi sono **già disponibili in Gazebo** tramite il plugin YAML di `franka_gazebo_bringup`. Il `joint_velocity_example_controller` è un riferimento funzionante per la velocity pipeline. Il suo topic è configurabile.

---

## 3. Analisi Launch Files

### 3.1 `franka_simulation/launch/franka_simulation.launch.py`

| Attributo | Valore |
|-----------|--------|
| Status | **USATO** — launch "minimale" di simulazione |
| Funzione | Gazebo + RSP + joint_state_broadcaster + fr3_arm_controller |
| Controller spawnati | `joint_state_broadcaster` (ACTIVE), `fr3_arm_controller` (ACTIVE) |
| Nodi avviati | Gazebo, RSP, JSB spawner, arm spawner, RViz |
| MoveIt | ❌ nessuno |
| CBF pipeline | ❌ nessuna |
| Note | Avvia solo posizione. Non usa `fr3_velocity_controller`. Confusamente nominato rispetto a `move_group.launch.py` che fa di più. |
| Legacy? | ⚠️ Parzialmente — utile come base per sim_position pipeline ma non allineato con l'architettura CBF |
| Eliminabile? | No, ma da rinominare: `sim_position.launch.py` |

### 3.2 `franka_simulation/launch/move_group.launch.py`

| Attributo | Valore |
|-----------|--------|
| Status | **ATTIVO — PRINCIPALE** |
| Funzione | Pipeline completa CBF: Gazebo + MoveIt + obstacle + avoidance + velocity blender |
| Controller spawnati | `joint_state_broadcaster` (ACTIVE), `fr3_arm_controller` (INACTIVE, commentato), `fr3_velocity_controller` (ACTIVE) |
| Nodi avviati | Gazebo, RSP, move_group, RViz (MoveIt), static TFs, clock_bridge, obstacle_rsp, spawn_obstacle, obstacle_synchronizer, franka_motion_server, online_avoidance_controller, velocity_control_blender, RealSense (optional), safe_avoidance_test (optional) |
| MoveIt | ✅ se enable_moveit:=true (default) |
| CBF pipeline | ✅ COMPLETA |
| BUG | `velocity_control_blender` istanziato 2 volte: nodo standalone senza parametri (linee 580–585) + `delayed_blender` con parametri corretti (linee 607–627). Il primo è dead code. |
| BUG 2 | `delayed_arm` commentato nella LaunchDescription finale (linea 668) — `fr3_arm_controller` non viene mai spawnato |
| BUG 3 | `motion_server_action` (linea 598) è un OpaqueFunction non usata |
| Eliminabile? | No — è il launch principale. Da tenere e fixare. |

### 3.3 `franka_experiments/launch/minimal.launch.py`

| Attributo | Valore |
|-----------|--------|
| Status | **ATTIVO** — bringup robot reale |
| Funzione | franka_bringup + RT controller (velocity o torque) |
| Controller | `rt_velocity_executor_controller` OPPURE `rt_torque_controller` OPPURE `cbf_torque_controller` |
| Supporto Gazebo | ❌ pensato per robot reale (ha flag `gazebo` ma non avvia Gazebo) |
| Dipendenze | `franka_bringup/franka.launch.py`, `franka_experiments/utils/ros.py` |
| Usato da | `cbf_experiment.launch.py`, `oscbf_experiment.launch.py` |

### 3.4 `franka_experiments/launch/cbf_experiment.launch.py`

| Attributo | Valore |
|-----------|--------|
| Status | **ATTIVO** — pipeline CBF su robot reale |
| Funzione | pentagon_qddot_commander → cbf_safety_filter → rt_torque_controller |
| Pipeline | acceleration-level HOCBF QP |
| Fasi | Phase 1 (bypass_cbf:=true) / Phase 2 (bypass_cbf:=false) |
| Dipendenze | `minimal.launch.py`, real_time_distance (Phase 2) |
| Eliminabile? | No |

### 3.5 `franka_experiments/launch/oscbf_experiment.launch.py`

| Attributo | Valore |
|-----------|--------|
| Status | **ATTIVO** — pipeline OSCBF su robot reale |
| Funzione | pentagon_torque_commander → oscbf_filter → rt_torque_controller |
| Pipeline | torque-level Dynamic-OSCBF QP (Morton & Pavone 2025) |
| Fasi | Phase 1 (bypass_filter) / Phase 2 (joint CBF) / Phase 3 (obstacle CBF) |
| Dipendenze | `minimal.launch.py`, real_time_distance (Phase 3) |
| Eliminabile? | No |

### 3.6 `franka_experiments/launch/wrapper_forward_velocity.launch.py`

| Attributo | Valore |
|-----------|--------|
| Status | **AMBIGUO** — file .pyc in __pycache__ ma sorgente presente |
| Funzione | Wrapper per velocity forward |
| Eliminabile? | Da verificare |

### 3.7 `franka_experiments/launch/thales.launch.py`

| Attributo | Valore |
|-----------|--------|
| Status | **SPECIFICO** — launch sperimentale per setup Thales |
| Eliminabile? | No, ma non è una pipeline generale |

### 3.8 `franka_experiments/launch/handeye_calibration_bringup.launch.py`

| Attributo | Valore |
|-----------|--------|
| Status | **UTILITY** — calibrazione hand-eye |
| Eliminabile? | No — utility necessaria |

### 3.9 `franka_gazebo/franka_gazebo_bringup/launch/gazebo_joint_velocity_controller_example.launch.py`

| Attributo | Valore |
|-----------|--------|
| Status | **ESEMPIO FUNZIONANTE** — reference per velocity in Gazebo |
| Funzione | Gazebo + RSP + joint_velocity_example_controller |
| Note | Usa `joint_velocity_example_controller` (Franka C++ controller, command_interface=velocity) |
| Eliminabile? | No (appartiene a franka_gazebo) |

---

## 4. Analisi ros2_control

### 4.1 Hardware Interface in Gazebo

**Plugin**: `franka_ign_ros2_control/IgnitionSystem`  
**Dichiarazione** nel xacro (quando `gazebo=true`):

```xml
<plugin filename="franka_ign_ros2_control-system"
        name="ign_ros2_control::IgnitionROS2ControlPlugin">
  <parameters>$(find franka_gazebo_bringup)/config/franka_gazebo_controllers.yaml</parameters>
</plugin>
```

### 4.2 Command Interfaces per Giunto (Gazebo)

```xml
<!-- Nel configure_joint macro, quando gazebo=true -->
<command_interface name="position"/>
<command_interface name="velocity"/>
<!-- effort è CONDIZIONALE: -->
<xacro:if value="${gazebo == 0 or gazebo_effort == 1}">
  <command_interface name="effort"/>
</xacro:if>
```

**Per default (`gazebo_effort=false`):**
- ✅ `position` command interface — disponibile
- ✅ `velocity` command interface — disponibile  
- ❌ `effort` command interface — **DISABILITATA** (bug gz_ros2_control #343)

**Con `gazebo_effort:=true`:**
- ✅ `position` command interface — disponibile
- ✅ `velocity` command interface — disponibile
- ✅ `effort` command interface — disponibile (se il bug è risolto nella versione installata)

### 4.3 State Interfaces per Giunto

```xml
<state_interface name="position"/>  <!-- sempre disponibile -->
<state_interface name="velocity"/>  <!-- sempre disponibile -->
<state_interface name="effort"/>    <!-- sempre disponibile (solo lettura) -->
```

### 4.4 GPIO Interfaces (solo robot reale)

Le interfacce `cartesian_velocity`, `cartesian_pose_command`, `elbow_command` sono dichiarate nel xacro ma gestite solo da `franka_hardware/FrankaHardwareInterface`. In Gazebo sono presenti nel descrittore ma non funzionali.

### 4.5 Controller Manager in Gazebo

Il `IgnitionROS2ControlPlugin` istanzia un controller_manager interno con `update_rate: 1000 Hz` (da `franka_gazebo_controllers.yaml`). Lo stesso plugin gestisce la sincronia tra il loop di simulazione e il loop di controllo.

**ATTENZIONE**: Il file YAML del plugin (`franka_gazebo_controllers.yaml`) definisce solo i controller example di Franka. I controller custom di `franka_simulation` (es. `fr3_velocity_controller`) vengono aggiunti dopo via spawner. Questo è corretto e funziona.

### 4.6 Compatibilità velocity/acceleration/torque in Gazebo

| Interface | Disponibile per default | Note |
|-----------|------------------------|------|
| position | ✅ | JointTrajectoryController funzionante |
| velocity | ✅ | JointGroupVelocityController funzionante |
| effort | ❌ | Richiede `gazebo_effort:=true` in xacro + verifica bug |

---

## 5. Nuova Architettura Proposta

### 5.1 Pipeline 1 — Velocity Simulation

**Obiettivo**: Nodo esterno pubblica Float64MultiArray (7 joint velocities) → robot si muove in Gazebo.

**Architettura**:
```
[Nodo esterno]
    │ Float64MultiArray (7) → /fr3_velocity_controller/commands
    ▼
fr3_velocity_controller (JointGroupVelocityController)
    │ velocity command interfaces
    ▼
Gazebo IgnitionSystem
    │ joint states
    ▼
joint_state_broadcaster → /joint_states
    │
robot_state_publisher → TF tree
```

**Launch richiesto**: `franka_simulation/launch/sim_velocity.launch.py`

**Componenti**:
- Gazebo Ignition (empty world)
- robot_state_publisher (ros2_control=true, gazebo=true, gazebo_effort=false)
- Static TF: world→fr3_link0
- clock_bridge
- joint_state_broadcaster (spawner)
- fr3_velocity_controller (spawner, ACTIVE)
- RViz2

**Topic esterno**: `/fr3_velocity_controller/commands` — `std_msgs/Float64MultiArray` (size=7)

**Nota**: Questo controller esiste già e funziona. Il launch è semplicemente una versione pulita e minimale di quello esistente.

### 5.2 Pipeline 2 — Acceleration Simulation

**Obiettivo**: Nodo esterno pubblica Float64MultiArray (7 joint accelerations) → robot si muove in Gazebo.

**Due opzioni**:

#### Opzione A — Integrazione esterna (RACCOMANDATA per semplicità)

```
[Nodo esterno]
    │ Float64MultiArray (7) → /acceleration_cmd
    ▼
sim_acceleration_bridge (nuovo nodo Python, in franka_simulation/scripts/)
    │ integra: q̇(t) = q̇(t-1) + q̈·dt (con clamping a joint velocity limits)
    │ Float64MultiArray (7) → /fr3_velocity_controller/commands
    ▼
fr3_velocity_controller
    ▼
Gazebo
```

**Pro**: Semplice, nessun controller nuovo.  
**Con**: Drift di integrazione; non fisicamente accurato per torque dynamics.  
**Topic esterno**: `/acceleration_cmd` — `std_msgs/Float64MultiArray` (size=7)

#### Opzione B — Effort con dynamics esterna (per accuratezza fisica)

```
[Nodo esterno] pubblica τ = M(q)q̈ + C(q,q̇)q̇  [NO gravity — Gazebo la gestisce]
    │ Float64MultiArray (7) → /fr3_effort_controller/commands
    ▼
fr3_effort_controller (JointGroupEffortController)
    ▼
Gazebo (effort interface) — richiede gazebo_effort:=true
```

**Pro**: Fisicamente corretto.  
**Con**: Richiede abilitazione effort interface in Gazebo (possibile bug), il nodo esterno deve calcolare full dynamics.

**Launch richiesto**: `franka_simulation/launch/sim_acceleration.launch.py`

**Componenti Opzione A**:
- Tutti i componenti di sim_velocity.launch.py +
- `sim_acceleration_bridge` (nuovo, da creare)

### 5.3 Pipeline 3 — Torque Simulation

**Obiettivo**: Nodo esterno pubblica Float64MultiArray (7 joint torques) → robot si muove in Gazebo.

**Architettura**:
```
[Nodo esterno]
    │ Float64MultiArray (7) → /fr3_effort_controller/commands
    │ (torque SENZA gravity: Gazebo gestisce la fisica)
    ▼
fr3_effort_controller (JointGroupEffortController)
    │ effort command interfaces
    ▼
Gazebo IgnitionSystem (con gazebo_effort:=true)
```

**Differenza critica vs robot reale**: In Gazebo la gravità è gestita dalla fisica. Il nodo esterno NON deve aggiungere g(q). Questo è diverso dalla pipeline reale dove `rt_torque_controller` aggiunge g(q).

**Launch richiesto**: `franka_simulation/launch/sim_torque.launch.py`

**Componenti**:
- Gazebo con `gazebo_effort:=true` nel xacro
- joint_state_broadcaster (spawner)
- fr3_effort_controller (spawner, ACTIVE)
- RViz2
- Static TF, clock_bridge

**Topic esterno**: `/fr3_effort_controller/commands` — `std_msgs/Float64MultiArray` (size=7)

### 5.4 Pipeline 4 — CBF/Capsules (esistente, da mantenere)

La pipeline attuale in `move_group.launch.py` va mantenuta e corretta:

```
franka_motion_server → /velocity_blender/trajectory
obstacle_synchronizer → /obstacle_scene
online_avoidance_controller → /avoidance/closest_constraint
velocity_control_blender → /fr3_velocity_controller/commands
fr3_velocity_controller → Gazebo
```

**Modifiche necessarie** (fix only, no architecture change):
- Rimuovere il nodo `velocity_control_blender` duplicato (linee 580–585)
- Sbloccare/documentare lo stato di `fr3_arm_controller` (commentato)
- Rimuovere `motion_server_action` OpaqueFunction non usata (linea 598)

---

## 6. File da Creare

### 6.1 Nuovi Launch Files

| File | Contenuto |
|------|-----------|
| `franka_simulation/launch/sim_velocity.launch.py` | Gazebo + fr3_velocity_controller minimale; robot riceve velocity da nodo esterno |
| `franka_simulation/launch/sim_acceleration.launch.py` | Come velocity + sim_acceleration_bridge; robot riceve acceleration da nodo esterno |
| `franka_simulation/launch/sim_torque.launch.py` | Gazebo con gazebo_effort:=true + fr3_effort_controller; robot riceve torque da nodo esterno |

### 6.2 Nuovi Config Files

| File | Contenuto |
|------|-----------|
| `franka_simulation/config/fr3_effort_controller.yaml` | JointGroupEffortController per 7 giunti FR3 |
| `franka_simulation/config/sim_controllers.yaml` | Unico controller_manager config per tutte le sim pipelines (velocity + effort) |

### 6.3 Nuovi Nodi

| File | Ruolo |
|------|-------|
| `franka_simulation/scripts/sim_acceleration_bridge.py` | Integra acceleration commands → velocity; pubblica su `/fr3_velocity_controller/commands`; clamping ai velocity limits FR3 |

### 6.4 Nuovi Topic/Interfacce

| Topic | Tipo | Pipeline |
|-------|------|----------|
| `/fr3_velocity_controller/commands` | `std_msgs/Float64MultiArray` (7) | ✅ già esistente (Velocity) |
| `/acceleration_cmd` | `std_msgs/Float64MultiArray` (7) | Nuovo (Acceleration bridge input) |
| `/fr3_effort_controller/commands` | `std_msgs/Float64MultiArray` (7) | Nuovo (Torque) |

---

## 7. File da Modificare

### 7.1 `franka_simulation/launch/move_group.launch.py`

**Cosa modificare**:
1. **Rimuovere nodo `velocity_control_blender` duplicato** (righe 580–585): istanziato senza parametri, overridden da `delayed_blender`. Causa: potenzialmente due nodi con lo stesso nome che si sovrascrivono o creano conflitti.
2. **Rimuovere `motion_server_action`** (riga 598): OpaqueFunction mai usata nella LaunchDescription finale.
3. **Documentare o sbloccare `delayed_arm`** (commentato riga 668): se `fr3_arm_controller` è necessario per MoveIt trajectory execution, sbloccarlo; altrimenti rimuovere il codice.
4. **Parametro esplicito per `avoidance_params.yaml`** in `velocity_control_blender`: al momento i parametri di avoidance sono caricati separatamente e poi sovritti con override. Verificare che la fusione di params funzioni correttamente.

**Perché**: Ridurre ambiguità e dead code; ridurre rischio di conflitti tra due istanze di `velocity_control_blender`.

**Impatto**: Nessun impatto sull'architettura. Solo pulizia.

### 7.2 `franka_description/robots/common/franka_arm.ros2_control.xacro`

**Cosa modificare** (OPZIONALE, solo per pipeline torque):
- Il parametro `gazebo_effort` è già gestito nel xacro. Non occorre modificare il file.
- Ma la **dipendenza hardcoded** su `franka_gazebo_bringup/config/franka_gazebo_controllers.yaml` nel plugin Gazebo è problematica: se `franka_simulation` vuole usare un yaml diverso, non può sovrascriverlo facilmente.

**Soluzione alternativa (senza modificare il xacro)**:
- Assicurarsi che il yaml di `franka_gazebo_bringup` includa `fr3_effort_controller` come tipo da registrare, oppure
- Passare il file custom come override nei parametri del plugin.

**Impatto architetturale**: BASSO se si evita di modificare il xacro.

### 7.3 `franka_simulation/config/controllers.yaml`

**Cosa modificare**:
- Aggiungere `fr3_effort_controller` nella sezione `controller_manager`:
  ```yaml
  fr3_effort_controller:
    type: effort_controllers/JointGroupEffortController
  ```
- Aggiungere la sezione di configurazione per `fr3_effort_controller`

**Perché**: Il controller manager deve conoscere il tipo del controller prima che lo spawner possa crearlo.

**Impatto**: BASSO — solo aggiunta, nessuna rimozione.

### 7.4 `franka_simulation/CMakeLists.txt`

**Cosa modificare**:
- Aggiungere `sim_acceleration_bridge.py` alla lista degli script installati
- Aggiungere i nuovi launch files all'installazione

**Impatto**: BASSO — solo aggiunta.

### 7.5 `franka_simulation/package.xml`

**Cosa modificare**:
- Aggiungere dipendenza su `effort_controllers` (per JointGroupEffortController)
  - Package ROS2: `ros2_controllers` (già incluso) — verificare che `effort_controllers` sia nel set installato

**Impatto**: BASSO.

---

## 8. File Eliminabili

### 8.1 In `franka_simulation`

| File | Motivo |
|------|--------|
| `src/real_time_distance.py` | Funzionalità duplicata (esiste in `franka_experiments/nodes/real_time_distance.py`). Non referenziato da nessun launch. Mai installato da CMakeLists. |
| `src/utils.py` | Supporto per `src/real_time_distance.py` — stesso motivo. |
| `test/prova.py` | File di scratch/test senza contenuto utile. |
| `scripts/__pycache__/` (intera dir) | Bytecode compilato, va in .gitignore |
| `config/.gitkeep`, `launch/.gitkeep`, `rviz/.gitkeep` | Placeholder, non necessari se le dir contengono già file |

### 8.2 In `franka_simulation/config`

| File | Motivo |
|------|--------|
| `config/fr3_gripper.yaml` | Il gripper non è mai spawnato nei launch attivi. Posizione attuale: referenziata solo da `controllers.yaml` che definisce il tipo, ma il gripper spawner non esiste nel launch. |
| `config/safe_avoidance_test_params.yaml` | Il nodo `safe_avoidance_test` esiste solo come .pyc (non come .py sorgente). Il nodo è condizionale (`run_safe_test:=false` di default). Rimuovere se il nodo non è più mantenuto. |

**NOTA CRITICA**: Il file `scripts/__pycache__/safe_avoidance_test.cpython-313.pyc` esiste ma il sorgente `.py` NON esiste in `scripts/`. Questo significa che il nodo `safe_avoidance_test` non è eseguibile con l'installazione corrente. Il launch lo referenzia come eseguibile, causando errore se `run_safe_test:=true`.

### 8.3 In `franka_experiments`

| File | Motivo |
|------|--------|
| `legacy/nodes/avoidance_controller.py` | Esplicitamente marcato come legacy nel README.md |
| `legacy/nodes/cbf_avoidance_controller.py` | Stesso motivo |
| `legacy/nodes/distance_estimator.py` | Stesso motivo |
| `franka_experiments/nodes/experiment_logs/` | Dati sperimentali passati, non codice. Da gestire con .gitignore o spostarsi fuori dal package. |
| `launch/__pycache__/` | Bytecode compilato |

---

## 9. Piano di Refactoring

### 9.1 Fase 0 — Prerequisiti e Analisi (COMPLETATA con questo documento)

- [x] Reverse engineering completo del package
- [x] Identificazione controller compatibili
- [x] Identificazione dipendenze critiche
- [x] Verifica bug effort interface Gazebo

### 9.2 Fase 1 — Fix Immediati (SICURI, nessun rischio)

**Priorità: ALTA — correttezza**

1. **Fix `move_group.launch.py`** (eliminare dead code):
   - Rimuovere nodo `velocity_control_blender` standalone (righe 580–585)
   - Rimuovere `motion_server_action` OpaqueFunction non usata (riga 598)
   - Documentare perché `delayed_arm` è commentato

2. **Fix `safe_avoidance_test`** (sorgente mancante):
   - Verificare se il sorgente esiste in un altro branch
   - Se non disponibile: rimuovere il riferimento dal launch e i file di config

3. **Pulizia .gitignore**:
   - Aggiungere `**/__pycache__/`, `**/*.pyc` al .gitignore

### 9.3 Fase 2 — Velocity Simulation Pipeline (SICURA)

**Priorità: ALTA — obiettivo principale**

**Dipendenze critiche**: Nessuna nuova — `fr3_velocity_controller` esiste già.

**Rischi**: BASSI — operazione solo additive.

**Step**:
1. Creare `franka_simulation/launch/sim_velocity.launch.py`
   - Copia/adattamento di `franka_simulation.launch.py` semplificato
   - Avvia solo: Gazebo, RSP, JSB, fr3_velocity_controller, RViz, static TF, clock_bridge
   - NO MoveIt, NO avoidance, NO obstacles, NO camera
   - Launch arg: `arm_id`, `load_gripper`, `rviz:=true/false`

2. Testare il launch in isolamento:
   - `ros2 launch franka_simulation sim_velocity.launch.py`
   - Verificare che `/fr3_velocity_controller/commands` sia disponibile
   - Testare con: `ros2 topic pub /fr3_velocity_controller/commands std_msgs/Float64MultiArray "{data: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"`

3. Verificare funzionamento con nodo esterno (es. `ee_pentagon_velocity_commander`)

### 9.4 Fase 3 — Torque Simulation Pipeline (MEDIA COMPLESSITÀ)

**Priorità: MEDIA**

**Dipendenze critiche**:
- Verifica che `gazebo_effort:=true` funzioni con la versione installata di `franka_ign_ros2_control`
- Verifica che il bug #343 (gz_ros2_control) sia risolto

**Rischi**:
- MEDIO: Il bug dell'effort interface potrebbe non essere risolto nella versione installata
- MEDIO: Double-counting della gravità se non gestito correttamente

**Step**:
1. **Verifica preliminare**: avviare Gazebo con `gazebo_effort:=true` e verificare che `/fr3_effort_controller` si attivi correttamente
   ```bash
   # Test: spawna il robot con effort interface
   ros2 launch franka_simulation sim_velocity.launch.py
   # In un altro terminale, spawna manualmente effort controller:
   ros2 control list_hardware_interfaces  # verifica effort[i]/command
   ```

2. Creare `franka_simulation/config/fr3_effort_controller.yaml`:
   ```yaml
   fr3_effort_controller:
     ros__parameters:
       joints: [fr3_joint1, ..., fr3_joint7]
       command_interfaces: [effort]
       state_interfaces: [position, velocity, effort]
   ```

3. Modificare `franka_simulation/config/controllers.yaml` per aggiungere `fr3_effort_controller`

4. Creare `franka_simulation/launch/sim_torque.launch.py`:
   - Come `sim_velocity.launch.py` ma:
   - RSP processa il xacro con `gazebo_effort:=true`
   - Spawna `fr3_effort_controller` invece di `fr3_velocity_controller`

5. **DOCUMENTARE** chiaramente: nodo esterno deve inviare torque SENZA gravity (Gazebo gestisce la fisica)

6. Testare con comando manuale:
   ```bash
   ros2 topic pub /fr3_effort_controller/commands std_msgs/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
   ```

### 9.5 Fase 4 — Acceleration Simulation Pipeline (DIPENDE DA FASE 3)

**Priorità: MEDIA**

**Approccio raccomandato**: Opzione A (bridge di integrazione) per la semplicità.

**Step (Opzione A)**:
1. Creare `franka_simulation/scripts/sim_acceleration_bridge.py`:
   - Subscriber: `/acceleration_cmd` (Float64MultiArray, 7)
   - Subscriber: `/joint_states` (per q̇ corrente)
   - Publisher: `/fr3_velocity_controller/commands` (Float64MultiArray, 7)
   - Logic: q̇(t+1) = clamp(q̇(t) + q̈·dt, -qdot_max, +qdot_max)
   - Parametri: `dt`, `qdot_max` per ciascun giunto

2. Creare `franka_simulation/launch/sim_acceleration.launch.py`:
   - Come `sim_velocity.launch.py` +
   - Avvia `sim_acceleration_bridge`
   - Espone topic `/acceleration_cmd`

3. Registrare `sim_acceleration_bridge` in CMakeLists.txt

**Step (Opzione B — torque accurate)**:
- Dipende dalla Fase 3 (torque pipeline funzionante)
- Il nodo esterno deve calcolare τ = M(q)q̈ + C(q,q̇)q̇ (NON g(q) — Gazebo gestisce gravity)
- Non richiede nuovo bridge — l'utente usa direttamente `sim_torque.launch.py`

### 9.6 Fase 5 — Pipeline CBF Legacy (Fix & Stabilizzazione)

**Priorità: BASSA** (già funzionante)

**Step**:
1. Applicare i fix della Fase 1 a `move_group.launch.py`
2. Rinominare `franka_simulation.launch.py` → `sim_position.launch.py` per chiarezza
3. Aggiungere README.md alla dir `launch/` con documentazione delle pipeline
4. Verificare compatibilità con eventuale nuovo `controllers.yaml` (aggiunta effort controller)

---

## 10. Dipendenze Critiche e Rischi Architetturali

### 10.1 Bug gazebo_effort (CRITICO per Pipeline Torque)

**Issue**: Il commento nel xacro `franka_arm.ros2_control.xacro` cita il bug https://github.com/ros-controls/gz_ros2_control/issues/343 come motivo per cui l'effort command interface è disabilitata in Gazebo per default.

**Stato**: Sconosciuto se il bug è risolto nella versione installata di `franka_ign_ros2_control`.

**Azione richiesta PRIMA di Fase 3**:
```bash
# Verificare versione installata:
ros2 pkg xml franka_ign_ros2_control | grep version
# Verificare se l'issue è chiusa nella versione installata
# Testare manualmente con gazebo_effort:=true
```

**Fallback**: Se il bug non è risolto, implementare Pipeline Torque via Opzione B di Accelerazione (la dinamica inversa avviene esternamente e viene inviata come velocity correction).

### 10.2 Gravity in rt_torque_controller — ANALISI CODICE SORGENTE

**TROVATO LEGGENDO `franka_rt_controllers/src/rt_torque_controller.cpp`:**

```cpp
// update() — RT path
// 4. Gravity torques → data_.g (pre-allocated)
pinocchio::computeGeneralizedGravity(model_, data_, q_pin_);

// 5. tau_hw = clip(tau_filtered + g(q), -tau_max, tau_max)  ← COMMENTO
for (size_t i = 0; i < kNumJoints; ++i) {
    const double tau_raw = tau_filtered_[i];   // ← NON aggiunge data_.g !
    command_interfaces_[i].set_value(
        std::clamp(tau_raw, -tau_max_[i], tau_max_[i]));
}
```

**CHIARIMENTO**: La gravità viene calcolata (step 4) ma non sommata al comando di uscita. Questo è **corretto per design**: il firmware Franka bilancia automaticamente la gravità a livello hardware. I nodi upstream (cbf_safety_filter, OSCBF_filter) inviano quindi solo la componente di moto τ = M(q)·q̈ + C(q,q̇)·q̇, senza dover calcolare g(q). Il commento nel codice C++ ("+ g(q)") è fuorviante e probabilmente residuo di una versione precedente — il codice è corretto.

**Implicazione per la simulazione**:
- In Gazebo la gravità è gestita dal motore fisico, non dal firmware.
- Il `rt_torque_controller` invia τ_raw senza aggiungere g(q): su robot reale questo è corretto (firmware compensa). In Gazebo idem — non c'è double-counting.
- La singola differenza rimane: richiede l'effort command interface (`gazebo_effort:=true`).

**Conclusione per la torque simulation pipeline**:
- `rt_torque_controller` **è compatibile con Gazebo** a patto di abilitare l'effort interface.
- Il nodo esterno (es. `pentagon_torque_commander`, `cbf_OSCBF_filter`) può essere usato identicamente in sim e su hardware reale: invia τ senza gravity, il firmware / la fisica se ne occupano.
- Alternativa più semplice senza port del `rt_torque_controller`: usare `fr3_effort_controller` (JointGroupEffortController) direttamente in Gazebo, con lo stesso topic convention.

### 10.3 Controller Manager Config Hardcoded nel Xacro (ARCHITETTURALE)

Il plugin Gazebo carica `$(find franka_gazebo_bringup)/config/franka_gazebo_controllers.yaml` hardcoded nel xacro. Questo file non include i controller custom di `franka_simulation`.

**Impatto attuale**: Basso — i controller custom vengono aggiunti via spawner dopo l'avvio.

**Rischio**: Se si vuole cambiare `update_rate` o `thread_priority` del controller_manager, bisogna modificare il file di `franka_gazebo_bringup` (package separato).

**Soluzione a lungo termine**: Parametrizzare il percorso del YAML nel xacro, oppure creare un file YAML di `franka_simulation` che estende quello di `franka_gazebo_bringup`.

### 10.4 Dipendenza da `franka_fr3_moveit_config` (NON-STANDARD)

`move_group.launch.py` carica la configurazione OMPL da `franka_fr3_moveit_config`. Questo package potrebbe non essere installato in tutti gli ambienti.

**Impatto**: Se `franka_fr3_moveit_config` non è presente, il launch fallisce anche con `enable_moveit:=false` — il codice Python cerca comunque il package per costruire la LaunchDescription.

**Fix**: Aggiungere gestione dell'eccezione nel `load_yaml()`, oppure condizionare il caricamento a `enable_moveit`.

### 10.5 Safe Avoidance Test — Sorgente Mancante

Il file `scripts/safe_avoidance_test.py` NON esiste, ma esiste solo `scripts/__pycache__/safe_avoidance_test.cpython-313.pyc`.

**Impatto**: Se si lancia con `run_safe_test:=true`, il nodo fallisce all'avvio con `ModuleNotFoundError` o `FileNotFoundError`.

**Azione**: Verificare se il sorgente è in altro branch. Se non recuperabile, rimuovere il riferimento dal launch e il file di config `safe_avoidance_test_params.yaml`.

### 10.6 Limite Gazebo — Sincronizzazione Real-Time

Gazebo Ignition in modalità `-r` (real-time) tenta di sincronizzare la simulazione con il tempo reale. Se la macchina è lenta, il controller manager a 1000 Hz potrebbe non riuscire a mantenere il ritmo, causando jitter nei comandi.

**Impatto**: La velocity pipeline è meno sensibile (JointGroupVelocityController è semplice). La torque/effort pipeline è più sensibile al jitter.

**Soluzione**: Ridurre `update_rate` a 500 Hz per simulazione se necessario. Aggiungere un launch arg `controller_update_rate`.

### 10.7 Incompatibilità ROS2 `use_sim_time`

In `move_group.launch.py`, `use_sim_time: True` è impostato per RSP e MoveIt. Ma `online_avoidance_controller` e `velocity_control_blender` non hanno `use_sim_time` impostato esplicitamente.

**Impatto potenziale**: Timestamp mismatch tra nodi con `use_sim_time` e nodi senza. Potenziali problemi di stale data detection (throttle_duration_sec usa wall clock vs sim clock).

**Fix**: Aggiungere `{'use_sim_time': True}` a tutti i nodi avviati in simulazione.

---

## 11. Riepilogo Tabellare — Cosa Esiste, Cosa Manca

### 11.1 Controller

| Controller | Esiste? | Funziona in Gazebo? | Pipeline |
|-----------|---------|---------------------|----------|
| `fr3_velocity_controller` | ✅ | ✅ | Velocity, CBF |
| `fr3_arm_controller` | ✅ | ✅ | Position (non usato) |
| `fr3_effort_controller` | ❌ | ⚠️ da creare + verificare bug | Torque |
| `rt_velocity_executor_controller` | ✅ | ⚠️ da verificare in Gazebo | Velocity (reale) |
| `rt_torque_controller` | ✅ | ⚠️ gravity issue | Torque (reale) |

### 11.2 Launch Files

| Launch | Esiste? | Stato | Funzione |
|--------|---------|-------|----------|
| `franka_simulation.launch.py` | ✅ | Attivo (semplice) | Solo position |
| `move_group.launch.py` | ✅ | Attivo (completo CBF) | CBF pipeline |
| `sim_velocity.launch.py` | ❌ | **DA CREARE** | Velocity pipeline |
| `sim_acceleration.launch.py` | ❌ | **DA CREARE** | Acceleration pipeline |
| `sim_torque.launch.py` | ❌ | **DA CREARE** | Torque pipeline |

### 11.3 Nodi

| Nodo | Package | Esiste? | Pipeline |
|------|---------|---------|----------|
| `online_avoidance_controller` | franka_simulation | ✅ | CBF |
| `velocity_control_blender` | franka_simulation | ✅ | CBF |
| `franka_motion_server` | franka_simulation | ✅ | CBF |
| `obstacle_synchronizer` | franka_simulation | ✅ | CBF |
| `sim_acceleration_bridge` | franka_simulation | ❌ | **DA CREARE** |
| `cbf_safety_filter` | franka_experiments | ✅ | Accel CBF (reale) |
| `cbf_OSCBF_filter` | franka_experiments | ✅ | Torque OSCBF (reale) |
| `ee_pentagon_velocity_commander` | franka_experiments | ✅ | Velocity trajectory |
| `pentagon_torque_commander` | franka_experiments | ✅ | Torque trajectory |
| `pentagon_qddot_commander` | franka_experiments | ✅ | Accel trajectory |
| `real_time_distance` | franka_experiments | ✅ | Distance sensor |
| `safe_avoidance_test` | franka_simulation | ⚠️ solo .pyc | Demo |

---

## Appendice A — Topic Map Completo

| Topic | Tipo | Publisher | Subscriber | Pipeline |
|-------|------|-----------|------------|----------|
| `/joint_states` | JointState | joint_state_broadcaster | avoidance, blender, CBF filters | tutte |
| `/robot_description` | String | RSP | Gazebo spawn, MoveIt | tutte |
| `/fr3_velocity_controller/commands` | Float64MultiArray(7) | velocity_control_blender, esterno | fr3_velocity_controller | Velocity, CBF |
| `/fr3_effort_controller/commands` | Float64MultiArray(7) | esterno | fr3_effort_controller | Torque (da creare) |
| `/velocity_blender/trajectory` | JointTrajectory | franka_motion_server | velocity_control_blender | CBF |
| `/avoidance/min_distance` | Float32 | online_avoidance_controller | velocity_control_blender | CBF |
| `/avoidance/closest_constraint` | Float64MultiArray | online_avoidance_controller | velocity_control_blender | CBF |
| `/obstacle_scene` | MarkerArray/custom | obstacle_synchronizer | online_avoidance_controller | CBF |
| `/planning_scene` | PlanningScene | obstacle_synchronizer | MoveIt, RViz | CBF |
| `/collision_object` | CollisionObject | obstacle_synchronizer | MoveIt | CBF |
| `/robot_capsules_markers` | MarkerArray | online_avoidance_controller | RViz | CBF |
| `/NS_1/torque_cmd` | Float64MultiArray(7) | cbf_safety_filter/pentagon_torque | rt_torque_controller | Accel, Torque (reale) |
| `/NS_1/torque_safe` | Float64MultiArray(7) | cbf_OSCBF_filter | rt_torque_controller | Torque OSCBF (reale) |
| `/NS_1/qddot_nom` | Float64MultiArray(7) | pentagon_qddot_commander | cbf_safety_filter | Accel (reale) |
| `/cbf/per_link_distances` | MultiLinkDistance | real_time_distance | cbf_safety_filter, OSCBF_filter | CBF obstacle |
| `/clock` | Clock | Gazebo | tutti (sim_time) | tutte |

---

## Appendice B — Struttura Directory Post-Refactoring

```
franka_simulation/
├── config/
│   ├── controllers.yaml              ← aggiungere fr3_effort_controller
│   ├── fr3_arm_controller.yaml       ← mantenere
│   ├── fr3_velocity_controller.yaml  ← mantenere
│   ├── fr3_effort_controller.yaml    ← NUOVO (torque pipeline)
│   ├── avoidance_params.yaml         ← mantenere
│   ├── velocity_blender_params.yaml  ← mantenere
│   ├── motion_server_params.yaml     ← mantenere
│   ├── obstacle_synchronizer_params.yaml ← mantenere
│   ├── kinematics.yaml               ← mantenere
│   ├── ompl_planning.yaml            ← mantenere
│   ├── fr3_complete.yaml             ← mantenere
│   ├── safe_avoidance_test_params.yaml ← RIMUOVERE (sorgente mancante)
│   └── fr3_gripper.yaml              ← opzionale rimozione
│
├── launch/
│   ├── franka_simulation.launch.py   ← rinominare: sim_position.launch.py
│   ├── move_group.launch.py          ← mantenere (CBF pipeline), fix bug
│   ├── sim_velocity.launch.py        ← NUOVO
│   ├── sim_acceleration.launch.py    ← NUOVO
│   └── sim_torque.launch.py          ← NUOVO
│
├── scripts/
│   ├── franka_motion_server.py       ← mantenere
│   ├── franka_motion_client.py       ← mantenere
│   ├── obstacle_synchronizer.py      ← mantenere
│   ├── online_avoidance_controller.py ← mantenere
│   ├── velocity_control_blender.py   ← mantenere
│   ├── image_publisher.py            ← mantenere
│   ├── human_pose_node.py            ← mantenere
│   └── sim_acceleration_bridge.py    ← NUOVO
│
├── scripts/utils/                    ← mantenere tutti
│
├── src/
│   ├── real_time_distance.py         ← ELIMINARE (duplicato)
│   └── utils.py                      ← ELIMINARE (duplicato)
│
├── test/
│   ├── avoidance_test.py             ← mantenere (test valido)
│   ├── collision_test_demo.py        ← mantenere
│   ├── franka_motion_demo.py         ← mantenere
│   ├── launch/                       ← NUOVO (test infrastructure)
│   │   ├── test_velocity_pipeline.launch.py
│   │   ├── test_acceleration_pipeline.launch.py
│   │   ├── test_torque_pipeline.launch.py
│   │   └── test_cbf_pipeline.launch.py
│   ├── scripts/                      ← NUOVO
│   │   ├── test_velocity_publisher.py
│   │   ├── test_acceleration_publisher.py
│   │   ├── test_torque_publisher.py
│   │   └── check_pipeline.sh
│   ├── config/                       ← NUOVO
│   │   └── test_publishers.yaml
│   └── README.md                     ← NUOVO
```

---

## Appendice C — Testing

### Lanciare ogni pipeline con un singolo comando

```bash
# Velocity pipeline (Gazebo + auto publisher)
ros2 launch franka_simulation test_velocity_pipeline.launch.py

# Acceleration pipeline (Gazebo + integrator + auto publisher)
ros2 launch franka_simulation test_acceleration_pipeline.launch.py

# Torque pipeline (Gazebo + effort controller + auto publisher)
ros2 launch franka_simulation test_torque_pipeline.launch.py

# CBF pipeline (Gazebo + MoveIt + avoidance controller)
ros2 launch franka_simulation test_cbf_pipeline.launch.py
```

### Validare una pipeline in esecuzione

```bash
# Controlla controller, topic, rate — passa il nome della pipeline come argomento
./franka_simulation/test/scripts/check_pipeline.sh velocity
./franka_simulation/test/scripts/check_pipeline.sh acceleration
./franka_simulation/test/scripts/check_pipeline.sh torque
./franka_simulation/test/scripts/check_pipeline.sh cbf
```

### Topic map per ogni pipeline

| Pipeline | Topic di input (da nodo esterno) | Topic di output (al controller) | Controller |
|----------|----------------------------------|----------------------------------|------------|
| velocity | `/fr3_velocity_controller/commands` | — | `fr3_velocity_controller` |
| acceleration | `/sim_acceleration_bridge/accel_cmd` | `/fr3_velocity_controller/commands` | `fr3_velocity_controller` |
| torque | `/fr3_effort_controller/commands` | — | `fr3_effort_controller` |
| CBF | `MoveIt2 action` | `/fr3_velocity_controller/commands` | `fr3_velocity_controller` |

### Cosa aspettarsi in Gazebo

| Pipeline | Comportamento visibile |
|----------|------------------------|
| velocity | joint1 oscilla ±0.15 rad/s a 0.1 Hz |
| acceleration | joint1 accelera fino al clamp, poi decelerazione |
| torque | robot fermo (zeros = equilibrio fisica), poi oscillazione con ±2 Nm |
| CBF | robot si muove verso waypoint, capsule visibili in RViz, avoidance attivo |

### Cosa aspettarsi in RViz

| Pipeline | Visualizzazione |
|----------|-----------------|
| tutte | Robot state aggiornato in tempo reale da `/joint_states` |
| CBF | Capsule markers (MarkerArray su `/robot_capsules_markers`) |
| CBF | Obstacle markers se `spawn_obstacles:=true` |

### Comandi ROS2 manuali

```bash
# Velocity: pubblica un comando immediato
ros2 topic pub --once /fr3_velocity_controller/commands \
  std_msgs/Float64MultiArray "{data: [0.1, 0, 0, 0, 0, 0, 0]}"

# Acceleration: invia accelerazione al bridge
ros2 topic pub --once /sim_acceleration_bridge/accel_cmd \
  std_msgs/Float64MultiArray "{data: [0.3, 0, 0, 0, 0, 0, 0]}"

# Torque: invia torchio zero (equilibrio)
ros2 topic pub --once /fr3_effort_controller/commands \
  std_msgs/Float64MultiArray "{data: [0, 0, 0, 0, 0, 0, 0]}"

# Verifica hardware interfaces
ros2 control list_hardware_interfaces

# Verifica controller attivi
ros2 control list_controllers

# Monitora rate topic
ros2 topic hz /fr3_velocity_controller/commands
ros2 topic hz /joint_states
```
