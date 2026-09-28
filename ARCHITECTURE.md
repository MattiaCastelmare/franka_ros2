# ARCHITECTURE.md — architettura del sistema di controllo, così com'è nel codice

**Ambito.** Ricostruito leggendo il codice sorgente del repository (branch `humble-mattia`,
snapshot 2026-09-25), non `README.md` né i commenti — dove questi ultimi divergono dal
codice è segnalato esplicitamente in **§13**. `build/` e `install/` non sono state usate
come fonte (sono copie generate). Ogni affermazione riporta `file:riga`. Le zone dove non
è stato possibile verificare qualcosa dal codice sono in **§14**, non inventate.

Il repository non implementa **una** pipeline di controllo ma **cinque**, in gran parte
indipendenti, che condividono lo stesso livello hardware (`franka_hardware` +
`franka_rt_controllers`):

| Sigla | Launch file / entry point | Stato | Cosa fa |
|---|---|---|---|
| **A** | `franka_experiments/launch/torque_control_stack.launch.py` | **quella attivamente sviluppata** (ISO layer, IMM tracker, multi-CP, tutte le feature CBF recenti la riguardano) | spazio-accelerazione: generatore di moto → CBF-QP → dinamica inversa → controllore di coppia RT |
| **B** | `franka_experiments/launch/velocity_cbf_control_stack.launch.py` | pipeline cinematica parallela, a due fasi | comando di velocità EE → CBF-QP di velocità → controllore di velocità RT |
| **C** | `franka_experiments/launch/minimal.launch.py` (`use_torque_controller:=cbf`) | "raramente usata" (`franka_experiments/config/launch_defaults.yaml:89`) | dinamica inversa Pinocchio **dentro** il controllore C++ RT; nessun commander/CBF viene avviato automaticamente |
| **D** | `franka_simulation` (pacchetto separato) | Gazebo Ignition, quattro pipeline selezionabili + una pipeline di avoidance | non condivide nodi con A/B/C, ma A **dipende in runtime** da utility di D (vedi §11, §13) |
| **E** | `franka_sim` (pacchetto separato, **niente ROS 2**) | training offline | addestra la policy Safe-RL (SAC) che, esportata in ONNX, viene eseguita da `rl_policy_commander` dentro **A** |

---

## 1. Mappa dei package

| Package | Tipo | Scopo | Note |
|---|---|---|---|
| `franka_experiments` | ROS 2 Python (ament_python) | pacchetto di ricerca principale: commander di moto, filtri CBF-QP, stima distanza da profondità, layer ISO, calibrazione, logging | dipende in runtime da `franka_simulation` (`franka_experiments/package.xml:30`, `<exec_depend>franka_simulation</exec_depend>`) |
| `franka_rt_controllers` | ROS 2 C++ (`ros2_control` plugin) | 3 controllori RT: `RtTorqueController`, `RtVelocityExecutorController`, `CBFTorqueController` | vedi §5 per la discrepanza nomi file/classe/istanza |
| `franka_hardware` | ROS 2 C++ | `SystemInterface` verso libfranka (`FrankaHardwareInterface`), azione `error_recovery`, servizi `Set*` | nessun clamp software oltre al controllo NaN/Inf (§5) |
| `franka_simulation` | ROS 2 Python + msg/action | Gazebo Ignition + MoveIt2, 4 pipeline (posizione/velocità/accelerazione/coppia) + pipeline CBF/avoidance con Pinocchio | vedi §11 |
| `franka_sim` | Python puro, **zero import ROS** (verificato: `grep -rn "rclpy" franka_sim/` → 0 risultati) | ambiente Gymnasium MuJoCo, CBF filter duplicato, training SAC (Stable-Baselines3), export ONNX | vedi §11 |
| `franka_bringup` | ROS 2 launch/config | bringup base: `robot_state_publisher`, `ros2_control_node`, `joint_state_broadcaster`, gripper | `franka_bringup/launch/franka.launch.py:87-173` |
| `franka_description` | URDF/xacro + mesh | modello FR3, incluse le mesh di collisione usate sia dal mascheramento percezione sia dal self-collision CBF | `franka_arm.srdf.xacro` condiviso da tutta la famiglia Franka (fr3/fer/fp3), non specifico FR3 |
| `franka_fr3_moveit_config` | ROS 2 MoveIt2 | `move_group.launch.py`, planning **solo OMPL** (nessun Pilz) | vedi §12 |
| `franka_gazebo` | ROS 2 C++ | integrazione Ignition **generica**, non FR3-specifica; **non è usata da `franka_simulation`** | vedi §13 (divergenza di percezione) |
| `franka_gripper` | ROS 2 C++ | `franka_gripper_node`: azioni `homing`/`move`/`grasp`/`gripper_action`, servizio `stop` | |
| `franka_robot_state_broadcaster` | ROS 2 C++ (`ros2_control` broadcaster) | pubblica lo stato completo del robot | 9 topic, vedi §5 |
| `franka_semantic_components` | ROS 2 C++ (libreria, non nodo) | wrapper tipizzati sopra le state/command interface grezze | usato da `franka_robot_state_broadcaster` e dai controllori |
| `franka_msgs` | ROS 2 interfacce | messaggi/servizi/azioni custom | vedi §12 |
| `integration_launch_testing` | test | test di bringup base (gripper, example controller) | **non tocca nessuno degli stack CBF** |

---

## 2. Livelli architetturali

```
 PERCEZIONE          real_time_distance (D455)  →  /cbf/per_link_distances
       │
 PIANIFICAZIONE/TASK pentagon_qddot_commander | rl_policy_commander  →  /NS_1/qddot_nom
       │
 FILTRO DI SICUREZZA cbf_safety_filter (CBF-QP, OSQP)  →  /NS_1/qddot_safe
       │
       │              qddot_to_torque (M(q)q̈+C(q,q̇), Pinocchio)  →  /NS_1/torque_cmd
       ▼
 HARDWARE            rt_torque_controller (RT, 1 kHz)  →  FrankaHardwareInterface  →  libfranka/firmware
```

Questo è lo schema della pipeline **A**; le pipeline **B** e **C** sono varianti descritte
in §9-§10.

---

## 3. Tabella BLOCCHI

Colonna **RT**: "Y" solo per codice che gira nel thread `SCHED_FIFO` di `ros2_control_node`
a 1 kHz (o nel thread dedicato di `cbf_safety_filter._qp_tick`, elevato a priorità
`SCHED_FIFO` 50 — non lo stesso canale).

### Pipeline A — stack coppia/accelerazione (`torque_control_stack.launch.py`)

| Blocco | File | Input | Output | Frequenza | RT |
|---|---|---|---|---|---|
| Camera D455 | esterno (`realsense2_camera`, `rs_launch.py`) | — | `/camera/camera/depth/image_rect_raw`, `/camera/camera/depth/camera_info` | driver (profilo pinnato, es. 848×480×90 → `launch_defaults.yaml`) | N |
| `real_time_distance` | `franka_experiments/franka_experiments/nodes/real_time_distance.py` | depth+info (sopra), TF, `fr3_complete.yaml` | `/cbf/per_link_distances` (`MultiLinkDistance`), `/human_robot/multi_distance`, `/human_robot/distance`, overlay opzionale | event-driven sul frame di profondità (nominale = fps camera) | N |
| `pentagon_qddot_commander` | `.../nodes/pentagon_qddot_commander.py:83-` | `/NS_1/joint_states`, `/NS_1/cbf_status`, `/NS_1/iso_safety` | `/NS_1/qddot_nom` (`Float64MultiArray[7]`), `/NS_1/q_des_state`, `/NS_1/ee_desired`, `/NS_1/ee_actual` | timer 100 Hz (`rate_hz`, L131, `create_timer` L622) | N |
| `rl_policy_commander` (alternativo, `motion_source:=rl`) | `.../nodes/rl_policy_commander.py:108-` | `/NS_1/joint_states`, `/cbf/per_link_distances`, `/NS_1/cbf_status` | `/NS_1/qddot_nom` (stesso topic, mutuamente esclusivo), `/NS_1/rl_status` | timer (`rate_hz`, 0.0 = eredita dal sim) | N |
| `cbf_safety_filter` | `.../nodes/cbf_safety_filter.py:129-` | `/NS_1/franka/joint_states`, `/NS_1/qddot_nom`, `/cbf/per_link_distances` (BEST_EFFORT), `/NS_1/iso_safety` | `/NS_1/qddot_safe`, `/NS_1/cbf_status` | **due timer**: build vincoli `_update_constraints` @ `cbf_update_rate_hz` (~50 Hz, L398); QP `_qp_tick` @ `qp_rate_hz` (~100 Hz, L400) | **Y** per `_qp_tick` (auto-elevazione `SCHED_FIFO` 50, L813-851) |
| `qddot_to_torque` | `.../nodes/qddot_to_torque.py:60-` | `/NS_1/franka/joint_states`, `/NS_1/qddot_safe` | `/NS_1/torque_cmd` (`Float64MultiArray[7]`), `/NS_1/torque_saturation` (diagnostico) | event-driven (un publish per messaggio `qddot_safe` ricevuto) | N |
| `RtTorqueController` | `franka_rt_controllers/src/rt_torque_controller.cpp` | `torque_cmd` (default, L291), `/NS_1/qddot_safe` (default `accel_topic`, L297), stato posizione/velocità | 7 interfacce comando `effort` | 1000 Hz reale / 100 Hz fake (YAML generata, vedi §5) | **Y** |
| `FrankaHardwareInterface` | `franka_hardware/src/franka_hardware_interface.cpp` | comando effort da `RtTorqueController` | `franka::RobotState` ↔ firmware via `Robot`/libfranka `ActiveControl` | stessa frequenza del loop | **Y** |
| `iso_safety_monitor` (opzionale, off di default) | `.../nodes/iso_safety_monitor.py:115-` | `/NS_1/franka/joint_states`, `/cbf/per_link_distances`, `/NS_1/cbf_status` | `/NS_1/iso_safety` (`Float64MultiArray[5]`), servizio `/NS_1/safety_reset` (`std_srvs/Trigger`) | timer @ `1/qp_rate_hz` (L169) | N |
| `iso_evidence_logger` (opzionale) | `.../nodes/iso_evidence_logger.py:137-` | vari topic percezione/CBF | **nessuno** (logger passivo, scrive solo su disco) | timer 100 Hz (`sample_rate_hz`, L148) | N |
| `experiment_logger` | `.../nodes/experiment_logger.py:108-` | fino a 12 topic (joint_states, distanze, comandi, `robot_state`, ecc.) | CSV su disco | timer 100 Hz (L132,L297) | N |
| `move_group` | `franka_fr3_moveit_config/launch/move_group.launch.py` | URDF/SRDF, joint_states, TF | servizi/azioni MoveIt (OMPL) | avviato di default ma **non usato dal path di controllo live** (§13) | N |
| `image_republisher` | eseguibile `image_publisher` del pacchetto `franka_simulation` (`franka_simulation/scripts/image_publisher.py:13`, nodo `image_republisher`) | color stream | topic ripubblicato per visualizzazione | reattivo | N |
| `trajectory_overlay_node` (opzionale) | `.../nodes/trajectory_overlay_node.py` | immagine colore, `/NS_1/ee_desired`, `/NS_1/ee_actual` | overlay video (rosso=comandato, blu=misurato) | reattivo | N |

### Pipeline B — stack di velocità (`velocity_cbf_control_stack.launch.py`)

| Blocco | File | Input | Output | Frequenza | RT |
|---|---|---|---|---|---|
| `ee_pentagon_velocity_commander` | `.../nodes/ee_pentagon_velocity_commander.py:70-` | joint_states | `tracking_topic` = `/NS_1/tracking_qdot` | timer 200 Hz (L85,L184) | N |
| `cbf_velocity_filter` | `.../nodes/cbf_velocity_filter.py:61-` | joint_states, distanze (chiave configurabile, default risolto verso `/cbf/per_link_distances` — vedi nota in §13), `/NS_1/tracking_qdot` | `/NS_1/qdot_cmd` (`Float64MultiArray[7]`) | timer 100 Hz (`control_rate_hz`, L138) | N |
| `RtVelocityExecutorController` (istanza `rt_velocity_executor_controller`) | `franka_rt_controllers/src/rt_velocity_blender_controller.cpp` | `/NS_1/qdot_cmd` (default `command_topic`, L195) | 7 interfacce comando `velocity` | 1000/100 Hz | **Y** |

Fase 1 (`bypass_cbf:=true`, default): la camera e `real_time_distance` **non** partono;
`cbf_velocity_filter` passa `tracking_qdot → qdot_cmd` invariato
(`velocity_cbf_control_stack.launch.py:18-22`).

### Pipeline C — controllore CBF-in-C++ (isolamento, `minimal.launch.py`)

| Blocco | File | Input | Output | Frequenza | RT |
|---|---|---|---|---|---|
| `CBFTorqueController` | `franka_rt_controllers/src/cbf_torque_controller.cpp` | **path 1** (preferito): `sensor_msgs/JointState` su `/NS_1/q_des_state` (FOH, interpolazione locale, L266-276); **path 2** (legacy): `Float64MultiArray` su `/NS_1/qddot_safe`; **path 3**: hold PD locale su timeout | 7 interfacce comando `effort` | 1000/100 Hz | **Y** — esegue RNEA Pinocchio (`pinocchio::rnea`, `computeGeneralizedGravity`) **dentro** il loop RT, poi sottrae la gravità |

`minimal.launch.py` da solo avvia **solo** bringup + questo controllore (+ camera/distanza/logger
opzionali) — **nessun commander e nessun nodo CBF Python** viene avviato automaticamente; per
farlo girare in anello chiuso occorre lanciare `pentagon_qddot_commander` + `cbf_safety_filter`
a mano sopra `minimal.launch.py use_torque_controller:=cbf` (nessun launch file nel repo assembla
questo percorso end-to-end).

Un quarto percorso, ancora diverso, esiste solo per test: `pentagon_torque_commander.py` con
`cbf_mode:False` scrive direttamente su `/NS_1/torque_cmd`, bypassando **ogni** filtro CBF — è
cablato unicamente in `franka_experiments/test/launch/test_torque_fake.launch.py:82-101`.

### Codice morto (non raggiungibile da nessun launch/test/script)

Confermato con `grep` sull'intero repository (esclusi `build/`, `install/`):

| File/nodo | Evidenza |
|---|---|
| `nodes/cbf_OSCBF_filter.py` (eseguibile `cbf_oscbf_filter`) | zero occorrenze fuori da `setup.py` e dal commento header di `config/oscbf_params.yaml:6-8`, che documenta una pipeline mai assemblata (`pentagon_torque_commander → oscbf_filter → rt_torque_controller`) |
| `utils/cbf_qp.py` | header proprio: "no importer... superseded-by: cbf_safety_filter inline OSQP + utils/cbf_qp_assembly.py" (`cbf_qp.py:2`) |
| `utils/cbf_kinematics.py` | shim di compatibilità, ri-esporta `CBFKinematics` da `utils/kinematics.py` (`:7`) |
| `utils/cbf_constraints.py` | header proprio: "no importer; velocity-era ZCBF rows (b = -gamma·h), non l'HOCBF usato dallo stack" (`:2`) |
| `utils/cbf_hard_limits.py::workspace_face_rows` | zero importatori live; il proprio test (`test/test_cbf_hard_constraints.py`) e `test_rl_policy.py:509` notano che "ha perso il suo ultimo importatore live nel commit `4d4d450`" |
| `nodes/ee_circle_velocity_commander.py`, `ee_random_waypoints_velocity_commander.py` | registrati come eseguibili (`setup.py:42-43`) ma nessun launch file li avvia |

---

## 4. Tabella ARCHI

| Sorgente | Destinazione | Segnale | Tipo msg | Dimensione/unità | Note |
|---|---|---|---|---|---|
| camera D455 (esterno) | `real_time_distance` | `/camera/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 16-bit, mm→m (`_DEPTH_TO_M=0.001`) | stream **grezzo**, non allineato al colore, nonostante `align_depth` sia ON di default (quello serve solo alle bag registrate, §7) |
| camera D455 | `real_time_distance` | `/camera/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | K/D | latched al primo messaggio, gli altri ignorati |
| `real_time_distance` | `cbf_safety_filter` | `/cbf/per_link_distances` | `franka_msgs/MultiLinkDistance` | N righe (1 per control point + extra se `multi_obstacle_k>1`) | QoS BEST_EFFORT, depth 1 |
| `pentagon_qddot_commander` \| `rl_policy_commander` | `cbf_safety_filter` | `/NS_1/qddot_nom` | `std_msgs/Float64MultiArray` | 7 (rad/s²) | mutuamente esclusivi, mai entrambi attivi |
| `cbf_safety_filter` | `qddot_to_torque` | `/NS_1/qddot_safe` | `Float64MultiArray` | 7 (rad/s²) | |
| `cbf_safety_filter` | `pentagon_qddot_commander`, `rl_policy_commander`, `experiment_logger` | `/NS_1/cbf_status` | `Float64MultiArray` | 11 elementi (semantica non tutta verificata, §14) | usato dal commander per il *governor* di fase, non un comando |
| `qddot_to_torque` | `RtTorqueController` (C++, RT) | `/NS_1/torque_cmd` | `Float64MultiArray` | 7 (N·m) | stesso nome topic condiviso — mai simultaneamente — da `pentagon_torque_commander` (test) e letto (mai) da `cbf_OSCBF_filter` (morto) |
| `iso_safety_monitor` | `cbf_safety_filter`, `pentagon_qddot_commander` | `/NS_1/iso_safety` | `Float64MultiArray` | 5 | non è un comando, latcha uno stato |
| `RtTorqueController` | `FrankaHardwareInterface` | interfaccia comando `effort` | `hardware_interface::LoanedCommandInterface` | 7 (N·m) | non un topic ROS, è l'interfaccia `ros2_control` |
| `FrankaHardwareInterface` ↔ firmware | — | `franka::RobotState` / comandi ActiveControl | struct libfranka nativa | — | via `Robot::readOnce`/`writeOnce`, `robot.cpp:159-292` |
| `FrankaRobotStateBroadcaster` | qualunque sottoscrittore | `~/current_pose`, `~/last_desired_pose`, `~/desired_end_effector_twist`, `~/measured_joint_states`, `~/external_wrench_in_stiffness_frame`, `~/external_wrench_in_base_frame`, `~/external_joint_torques`, `~/desired_joint_states`, `~/robot_state` | `PoseStamped`/`TwistStamped`/`JointState`/`WrenchStamped`/`franka_msgs/FrankaRobotState` | — | publish su thread **separato** dal thread RT (`franka_robot_state_broadcaster.cpp:206-233`, jitter non RT-safe) |
| qualunque client | `FrankaHardwareInterface`'s action server | `error_recovery` | `franka_msgs/action/ErrorRecovery` | — | `franka_action_server.cpp:19-35` |
| `ee_pentagon_velocity_commander` (pipeline B) | `cbf_velocity_filter` | `/NS_1/tracking_qdot` | `Float64MultiArray` | 7 (rad/s) | |
| `cbf_velocity_filter` | `RtVelocityExecutorController` | `/NS_1/qdot_cmd` | `Float64MultiArray` | 7 (rad/s) | |
| motion generator | `CBFTorqueController` (pipeline C) | `/NS_1/q_des_state` | `sensor_msgs/JointState` | posizione/velocità/(effort=qddot) | interpolato FOH nel controllore |

---

## 5. Il ciclo di controllo real-time

### 5.1 Dove nasce, e a quale frequenza

`ros2_control_node` (controller_manager, pacchetto esterno non vendorizzato in questo repo)
è lanciato da `franka_bringup/launch/franka.launch.py:121-132`. La frequenza **non** viene
dal callback di libfranka ma da una chiave `controller_manager.ros__parameters.update_rate`
nella YAML dei controller passata a quel nodo.

Il file statico di default, `franka_bringup/config/controllers.yaml:31-35`, imposta
`update_rate: 1000`, `thread_priority: 98` — ma **nessuno dei due stack "completi" lo usa**:
entrambi passano `controllers_yaml:='__auto__'`
(`franka_experiments/franka_experiments/utils/launch_support.py:462-463`), che fa generare
**una YAML temporanea nuova a ogni singolo lancio** da
`pick_controllers_yaml()`/`generate_rt_controllers_yaml()` (`launch_support.py:407-426`,
scritta con `tempfile.mkstemp`, `:372`). In quella YAML generata:
`update_rate: {1000 if is_real else 100}` — pipeline A: `launch_support.py:278`; pipeline
B: `:164`; pipeline C: `:52`. `thread_priority: 85` solo se `is_real` (`:281,:167,:55`).

Il thread SCHED_FIFO stesso non è impostato da questo repository: viene creato da
`controller_manager`/`realtime_tools` (esterni, solo header inclusi, es.
`rt_torque_controller.hpp:11`). Ciò che il repo fa in proprio è **pinnare** quel thread una
volta creato: `franka_experiments/scripts/pin_rt_thread.sh` cerca (via `ps -L -o tid=,cls=`)
l'unico thread di classe `FF` del processo `ros2_control_node` e gli applica
`taskset -cp <CPU> <tid>` (CPU 3 di default), **senza toccare il resto del processo**
(commento nello script) e **richiedendo che `isolcpus` sia già impostato a livello kernel**
(non lo imposta lui stesso).

`tools/rt-tuning/franka-rt-tuning.sh` (script di sistema separato, **non lanciato da nessun
launch file**) sposta l'IRQ della NIC del robot, imposta il governor CPU su `performance`,
ferma `irqbalance` e disattiva il throttle `sched_rt_runtime_us`. Documenta esplicitamente
(righe 10-18) che il thread di `cbf_safety_filter._qp_tick` (elevato a `SCHED_FIFO` 50) può,
se non isolato bene, affamare la quota RT del core isolato e far ritardare fino a 50 ms il
thread FCI (priorità 85) — un rischio noto e **solo parzialmente mitigato**, non risolto.

### 5.2 `FrankaHardwareInterface` / `Robot`

`franka_hardware/src/franka_hardware_interface.cpp` esporta come interfacce di comando
`effort`, `velocity`, `position`, `cartesian_velocity`, `cartesian_pose_command`,
`elbow_command` (costruite dinamicamente dal blocco `ros2_control` dell'URDF,
`.cpp:130-163`), e come interfacce di stato posizione/velocità/sforzo per giunto più
`robot_state`, `robot_model`, `cartesian_pose_state`, `elbow_state`, `robot_time`
(`.cpp:93-128`).

`read()` (`.cpp:205-221`) chiama `robot_->readOnce()`; `write()` (`.cpp:229-262`) fa **solo**
un controllo NaN/Inf su tutti i buffer di comando (`.cpp:223-235`) e poi smista verso
**una sola** interfaccia attiva. Non c'è alcun clamp di ampiezza, limite di giunto o
workspace a questo livello — l'unica rete di sicurezza software qui è quel controllo
NaN/Inf più il **rate limiter di libfranka stesso sul torque** (sempre attivo di default,
`robot.hpp:318`, applicato in `robot.cpp:163-166`); i limiter su velocità/posizione/
cartesiano esistono ma sono disattivi di default (`robot.hpp:319-328`).

Non viene usato il classico loop bloccante `franka::Robot::control(callback)` (zero
occorrenze), ma la API non bloccante **ActiveControl**:
`startTorqueControl()`/`startJointVelocityControl()`/ecc. (`robot.cpp:298-356`), con
retry-una-volta su `franka::ControlException` via `automaticErrorRecovery()`.

`communication_constraints_violation` è uno dei ~30 campi booleani di `franka::Errors`
(libfranka) — **nessun codice C++ di questo repo vi si aggancia per agire**; viene solo
tradotto in `franka_msgs::msg::Errors` (`franka_semantic_components/src/translation_utils.cpp:113-115`)
e pubblicato dentro `FrankaRobotState`. La sua "gestione" reale è tutta preventiva:
isolamento dei core + pinning del thread (§5.1).

### 5.3 `RtTorqueController` (pipeline A)

Interfaccia comando: `effort` (`rt_torque_controller.cpp:19-27`). Sottoscrive
`command_topic` (default `"torque_cmd"`, `:291`, **confermato via grep diretto**) per il
feedforward τ_ff e `accel_topic` (default `/NS_1/qddot_safe`, `:297`, **confermato**) per
l'accelerazione filtrata dal CBF.

Ad ogni ciclo (`update()`, `:54-202`): se τ_ff è stale, **hold di posizione** (`τ =
Kp·(q_hold−q) − Kd·q̇`, `:82-100`); altrimenti integra `q̇_des`/`q_des` da q̈_safe con un
inviluppo di velocità che **riproduce la curva firmware di libfranka** (non quella più
permissiva di `franka_description`, bug trovato nei log del 14-15 settembre, commento
`:204-233`), con anti-windup su errore di posizione/velocità; legge finale
`τ = ffScale(i)·τ_ff + Kp[i]·p + Kd[i]·e` (`:197-198`).

**Gravità non aggiunta qui** — commento esplicito `:120-122`: "la gravità g(q) è
compensata dal firmware Franka — NON aggiungerla qui" (**confermato via grep diretto**).

### 5.4 Nomi disallineati: `RtVelocityExecutorController` / `rt_velocity_blender_controller` / "blender"

Un solo controllore, tre nomi diversi a tre livelli:
- **classe C++**: `franka_rt_controllers::RtVelocityExecutorController`
- **file sorgente/header**: `rt_velocity_blender_controller.cpp`/`.hpp` (nome legacy — la
  sua stessa descrizione nel manifest dice "No blending logic", vedi sotto)
- **nome istanza runtime**: `rt_velocity_executor_controller` (chiave YAML generata,
  `launch_support.py:181-182`)

Il manifest plugin `franka_rt_controllers/rt_velocity_blender_controller.xml` (letto
per intero) contiene **due** classi, non una:

```xml
<class name="franka_rt_controllers/RtVelocityExecutorController" .../>
<class name="franka_rt_controllers/RtTorqueController" .../>
```

Non esiste una seconda libreria/pacchetto "blender" — è puro residuo di rinomina, non due
controllori diversi.

**Discrepanza aggiuntiva trovata qui**: la descrizione XML di `RtTorqueController`, in
questo stesso file, recita *"applies an optional low-pass filter, clips to per-joint
limits"* — cioè **ripete la stessa affermazione del README** (§13) — ma il codice C++ non
dichiara mai (`auto_declare`) né legge `lpf_alpha`/`tau_max_scale`. Il commento
**nel codice stesso** lo ammette: `rt_torque_controller.cpp:295` — *"(che oggi passa
lpf_alpha/tau_max_scale/urdf_path senza che il C++ li dichiari → ignorati)"* — un gap
auto-documentato, non solo un'inferenza esterna.

Il file `franka_rt_controllers/franka_rt_controllers.xml` (nome "generico"), invece,
registra **solo** `CBFTorqueController`, da una libreria separata (`cbf_torque_controller`,
target CMake proprio, `CMakeLists.txt:66-80`) — l'organizzazione dei due manifest non
segue l'intuizione del nome file.

### 5.5 `franka_robot_state_broadcaster`

Pubblica 9 topic (tabella §4) leggendo l'interfaccia di stato `robot_state`. Le chiamate
`publish()` girano su un **thread separato** (`publishRunner()`,
`franka_robot_state_broadcaster.cpp:213-233`), svegliato da una condition variable dal
thread RT — commento esplicito: "This block is not real-time safe due to jitter introduced
by the ROS 2 publisher" (`:206`). Il campionamento è quindi a 1000/100 Hz ma la
**pubblicazione effettiva può essere disaccoppiata e in ritardo**.

---

## 6. Il blocco CBF-QP (`cbf_safety_filter.py`, pipeline A)

### 6.1 Variabile di decisione e costo

`NX = NV + N_SLACK = 7 + 6 = 13` (`utils/cbf_state_rows.py:1300-1301`). Il vettore di
decisione è `[q̈(7), s_obs, s_sc, s_qlim, s_sing, s_cap, s_spd]` — **uno slack per
famiglia di vincolo**, non uno slack condiviso (design della pipeline live; il file morto
`cbf_qp.py` ne aveva uno solo condiviso). Costo:
`P = diag(1,...,1, ρ_obs, ρ_sc, ρ_qlim, ρ_sing, ρ_cap, ρ_spd)`
(`cbf_safety_filter.py:224-232`), termine lineare `q[:7] = −q̈_nom` (`:1202`) →
costo = `½‖q̈−q̈_nom‖² + Σ_g ½ρ_g·s_g²`. Ogni riga CBF è **soft** (rilassabile via slack); il
solo vincolo **hard** è il box giunto accelerazione/velocità/posizione (`hard_accel_box`,
`utils/cbf_hard_limits.py:129-224`).

### 6.2 Solver

`import osqp` (`cbf_safety_filter.py:45`, **confermato via grep diretto**), istanza nativa
`osqp.OSQP()` in `_solve()` (`:1294-1358`). `setup()` solo se cambia il numero di righe
(quantizzato a blocchi via `pad_rows_to_block`, `utils/cbf_qp_assembly.py:215-267`),
altrimenti `.update()`. `warm_start=True`, `max_iter=osqp_max_iter` (default **20000**,
`fr3_control.yaml:2123`), `verbose=False`.

### 6.3 Formulazione HOCBF

Confermato HOCBF (non ZCBF di primo ordine): `build_row_rhs`
(`utils/cbf_qp_assembly.py:120-198`) costruisce
`h_qp = k1·(A·q̇ − v_obs) + k0·h̄ + J̇q̇ [+ b_ff]`, cioè impone
`a^T·q̈ + s ≥ k1·ḣ + k0·h̄ + ċ` ⇔ `ḧ + k1·ḣ + k0·h ≥ 0` con **funzioni classe-K lineari**,
guadagni `k0_cbf = 25.0` e `k1_cbf = 10.5` (`fr3_control.yaml:277-278`). Con
`enable_zone_ladder`, `k0`/`k1` possono essere sostituiti riga-per-riga in funzione della
distanza misurata, solo per le righe ostacolo (`cbf_state_rows.py:2582-2606`).
Il termine `ċ = J̇q̇` (drift di grado relativo 2, da Pinocchio) conferma che non è una CBF
del primo ordine.

### 6.4 Righe di vincolo effettivamente assemblate

| Tipo riga | File:funzione | Dipende da | Attiva di default? | Descrizione |
|---|---|---|---|---|
| Ostacolo (per control point) | `cbf_state_rows.py` ciclo principale, `:1817-2280` | `d` (surface gap), Jacobiano Pinocchio, velocità/traccia ostacolo | sempre (guidata dalla percezione, nessun flag) | riga HOCBF `a=n̂ᵀJp`, `h=d−d_safe` |
| Limiti giunto | `joint_limit_rows`, `:55-123`, chiamata `:2323-2352` | `q`, limiti statici, opz. `q̇` | `joint_limit_rows_enabled: true` (`fr3_control.yaml:591`) | `h=q_max−margin−q` / `q−margin−q_min` |
| Self-collision (coppie capsule) | `SelfCollisionRowBuilder`, `:947-1167`, chiamata `:2361-2380` | frame capsula Pinocchio, esclusioni SRDF | `self_collision_rows_enabled: true` (`fr3_control.yaml:844`) | `h=‖p_a−p_b‖−r_a−r_b−margin` |
| Singolarità (σ_min) | `cbf_singularity.py` (intero file) | SVD Jacobiano, gradiente per differenze finite | `singularity_rows_enabled: true` (`fr3_control.yaml:652`) | `h=σ_min(J̃)−σ_floor`, max 1 riga |
| Cap di ritirata | `retreat_cap_speed`/`retreat_cap_rhs`, `cbf_state_rows.py:148-260` | `v_obs`, `h̄` | `retreat_cap_enabled: true` (`fr3_control.yaml:710`) | limita quanto in fretta il braccio può arretrare |
| Cap velocità task-space | `link_speed_row`/`ssm_speed_cap`, `:307-364`, `:2440-2500` | distanza, opz. parametri SSM ISO | `link_speed_rows_enabled: true` (`fr3_control.yaml:800`) | limita la velocità cartesiana di un control point |
| Riga TCP velocità ridotta ISO | `:2502-2531` | `iso_tcp_reduced_speed` | solo se `iso_enabled` **e** `iso_mode=='reduced'` (`iso_enabled: false` di default) | cap extra 250 mm/s su `FR3_TCP_LINK` |
| Box cartesiano/workspace | `cbf_hard_limits.py::workspace_face_rows` | — | **morto** (§3) | non fa parte del set live |
| Box hard accelerazione/velocità/posizione | `hard_accel_box`, `cbf_hard_limits.py:129-224` | `q,q̇`, limiti statici | sempre (unico vincolo **hard**, non-CBF) | intersezione dei tre box |

### 6.5 Livello di infeasibilità: scala a tre gradini

`cbf_qp_assembly.py:386-482`, invocata da `cbf_safety_filter._solve` (`:1329-1352`):
- **Livello 0**: QP completo (`SOLVED` o, se `accept_inaccurate_qp`, `SOLVED_INACCURATE`),
  clippato nel box hard.
- **Livello 1** (`box_only_solve`): nuova istanza OSQP, stesso costo, **solo** box —
  tutte le righe CBF eliminate; `eps_abs=eps_rel=1e-6`.
- **Livello 2** (`braking_command`): forma chiusa `clip(−k_brake·q̇, lb, ub)`, nessun solver.

Ogni livello viene contato/loggato come "safety-chain fault".

### 6.6 Uso di Pinocchio

**Solo cinematica, nessuna dinamica** in questo blocco (niente massa/Coriolis — quello è
downstream, in `qddot_to_torque.py`, che carica un **proprio** modello Pinocchio e calcola
`τ = M(q)·q̈_nom + C(q,q̇)·q̇`, **senza gravità** — docstring `qddot_to_torque.py:1-9`,
**confermato via lettura diretta**). Nel filtro CBF: caricamento modello
`pin.buildModelFromUrdf(...)` (`cbf_safety_filter.py:176`, modello separato senza mano) +
un secondo modello con mano/dita per il self-collision
(`build_urdf_with_sc()`, `cbf_state_rows.py:1517-1518`); FK e frame placement
(`utils/kinematics.py:459,463`); Jacobiani via `getFrameJacobian(...,LOCAL_WORLD_ALIGNED)`
(`:460,481`); derivata temporale del Jacobiano per il termine di drift
(`computeJointJacobiansTimeVariation`+`getFrameJacobianTimeVariation`, `:462,501-503`);
posizionamento capsule self-collision (`cbf_state_rows.py:1031-1036`); SVD per la
singolarità con un `pin.Data` proprio (`cbf_singularity.py:128-136`).

### 6.7 Multi control-point

`cbf_safety_filter.py` **non sceglie** i control point — è puro consumatore:
`_on_distances` (`:693`) sottoscrive `MultiLinkDistance` su `/cbf/per_link_distances`
(`:319`) e `ConstraintBuilder.build` itera `for ob in obs.items` (`cbf_state_rows.py:1817`)
**una riga per ogni voce ricevuta**. Il "multi-CP" è quindi interamente ereditato dalla
percezione (§7), che emette già una riga per control point più eventuali righe extra
(`multi_obstacle_k`).

### 6.8 Flag di feature (default effettivo)

| Flag | Effetto sul QP | Default `fr3_control.yaml` | Default `launch_defaults.yaml` (via argomento di lancio) |
|---|---|---|---|
| `enable_lateral_evasion` | bias obiettivo laterale su righe vicine (`cbf_state_rows.py:2311`) | `false` (`:1667`) | `true` (`:213`) |
| `enable_outrun_evasion` | bias se il punto non può superare l'ostacolo lungo n̂ (`:2319`) | `false` (`:1731`) | `true` (`:218`) |
| `enable_livelock_escape` | direzione di fuga dal nullspace delle righe attive (`:2323`) | `false` (`:1824`) | `true` (`:223`) |
| `enable_latency_compensation` | anticipa `h` in base al tempo cieco misurato | `true` (`:1940`, **"NOT validated on hardware since being turned on"** nel commento) | `true` (`:262`) |
| `enable_uncertainty_margin` | stringe `h` con `k_σ·σ(v)·t_lat` dalla covarianza IMM | `false` (`:1598`) | `true` (`:207`) |
| `enable_zone_ladder` | `k0`/`k1` per-riga in funzione del gap misurato | `false` (`:333`) | `true` (`:232`) |
| `enable_vobs_in_hdot` | usa `n̂ᵀv_obs` tracciato invece dello scalare clampato | `false` (`:1512`) | `true` (`:273`) |
| `enable_velocity_standoff` | `d_safe_eff = d_safe + t·v_app` | `false` (`:1539`) | `true` (`:314`) |
| `enable_sensor_range_uncertainty` | stringe `h` col modello di rumore del sensore | `false` (`:1989`) | **non esposto come argomento di lancio** — resta `false` |
| `iso_enabled` / `iso_ssm_speed_rows` | layer ISO master / sostituisce il cap con la SSM ISO/TS-15066 | `false`/`false` | `false`/`false` (`launch_defaults.yaml:285,287`) |

Nota architetturale: quasi tutti i flag "avanzati" sono **accesi di default in
`launch_defaults.yaml`** anche quando `fr3_control.yaml` da solo li avrebbe `false` — è il
file di lancio, non la config, a determinare il comportamento reale di un run normale.

---

## 7. La pipeline di percezione

### 7.1 Camera: una sola alimenta il canale di sicurezza

Solo la D455 "scene camera" alimenta la stima di distanza live (avviata da `rs_launch.py`,
`torque_control_stack.launch.py:528-536`). La D405 da polso (eye-in-hand) è **solo
registrazione**: `launch_defaults.yaml` la descrive come "Recording only: no node consumes
it"; zero consumatori del suo file di calibrazione (`camera_EE_extrinsic.yaml`) fuori dalla
calibrazione stessa (confermato via grep sull'intero repo).

`real_time_distance.py` ha **esattamente due** `create_subscription`
(`:429-430`): `depth_topic` (default `/camera/camera/depth/image_rect_raw`,
`sensor_msgs/Image`) e `depth_camera_info` (default `/camera/camera/depth/camera_info`).
**Nessuna sottoscrizione a colore/RGB.** Nonostante `align_depth.enable` sia `true` di
default (per far sì che le bag registrate portino sempre lo stream allineato), lo stream
**allineato al colore non è quello consumato dal nodo live** — serve solo a script offline
(`scripts/compare_range_noise.py`, `latency_budget.py`, ecc.) e al replay da bag
(`torque_control_stack.launch.py::_depth_bag_player`, `:272-298`, che rimappa lo stream
registrato **sul nome grezzo**).

### 7.2 Catena di calcolo (depth → distanza pubblicata)

1. **Deduplica frame** (`real_time_distance.py:592-634`): la D455 ripubblica ~2/3 dei
   frame due volte con lo stesso stamp; scartati per timestamp identico.
2. **Iniezione ostacolo simulato** (opzionale, off di default): `utils/obstacle_sim.py`.
3. **Lookup TF** (`utils/tf_manager.py::lookup_all`, `:87-125`): fallback a 3 livelli
   (stamp esatto → ultimo → cache invecchiata).
4. **Control point**: 11 punti su 5 segmenti (`utils/distance_utils.py::define_control_points`,
   `:255-307`), ultimo punto della mano offset di 0.10 m lungo l'asse z di `fr3_link8`.
5. **Maschera robot + z-buffer**: `utils/mask_builder.py::MaskBuilder.rebuild`
   (`:123-265`) — **ricostruita ad ogni frame** (nessuna cache), da mesh campionate con
   `trimesh` (3000 punti/link, `fr3_complete.yaml:90`, import a `real_time_distance.py:55`)
   trasformate dalla TF live per-link e proiettate. Lo stesso z-buffer alimenta un gate di
   profondità (`utils/distance_engine.py:310-392`) che "riapre" i pixel chiaramente
   davanti alla superficie del modello — questo è ciò che permette di vedere una mano che
   passa **davanti** al braccio, non solo la sua sagoma 2D.
6. **Motore di distanza** (`utils/distance_engine.py::DistanceEngine.compute`,
   `:257-537`): retro-proiezione in **spazio camera** (non base, per pixel), poi per ogni
   control point `surface = max(‖p−cp‖ − raggio − margine, 0)` (`:475-531`) — la distanza
   pubblicata è quindi un **gap di superficie**, non la distanza centro-centro; confermato
   anche dalla docstring di `franka_msgs/msg/LinkDistance.msg:23-28`.
7. **Filtro passa-basso / anti-spike**: `distance_engine.py::_lpf_pass` (`:588-694`) —
   EMA solo in recupero, istantaneo in avvicinamento, con limite di velocità plausibile
   (`lpf_v_max_approach`).
8. **Clustering multi-ostacolo** (solo se `multi_obstacle_k>1`): componenti connesse 2D
   con gate di profondità (`utils/obstacle_clusters.py::label_points`, `:206-239`).
9. **Tracking**: trasformazione centroide camera→base **prima** del filtro di Kalman
   (`utils/obstacle_track_pipeline.py:139-191`), poi un **Interacting Multiple Model
   (IMM)** — due modelli a covarianza costante, uno "generico" e uno "balistico"
   (media di gravità, `[0,0,−9.81]`) — con mixing/riponderazione bayesiana
   (`utils/obstacle_tracker.py::IMMTrack`, `:325-635`). **Confermato attivo**:
   `fr3_complete.yaml`'s `tracking.imm_enabled: true`, con commento
   **"TEMPORARY for hardware validation (Sep 2026)"** — coerente con `obstacle_tracker.py`
   modificato e `test_obstacle_imm.py` non ancora tracciato in `git status`.
10. **Guardia self-detection**: `utils/self_detection.py::SelfDetectionMonitor` sospende
    **solo l'annotazione di velocità**, mai la distanza, quando il punto "ostacolo" si
    muove in modo cinematicamente coerente col robot stesso.
11. **Pubblicazione**: `/cbf/per_link_distances` (`MultiLinkDistance`, BEST_EFFORT — quella
    che il CBF-QP legge), `/human_robot/multi_distance` (`MultiDistance`, legacy),
    `/human_robot/distance` (`HumanRobotDistance`, legacy), overlay opzionale.

### 7.3 Frame ed estrinseci

| File | Trasformazione | Consumato da |
|---|---|---|
| `camera_extrinsics.yaml` | `base → camera_color_optical_frame` | **l'unico** letto da `real_time_distance.py` (`utils/config.py::load_extrinsics`, `real_time_distance.py:116`) |
| `camera_link_extrinsics.yaml` | `base → camera_link` | solo lo static TF publisher (`torque_control_stack.launch.py:556-574`), per completezza dell'albero TF/MoveIt — **non letto** da `real_time_distance.py` |
| `camera_EE_extrinsic.yaml` | `fr3_link8 → d405_color_optical_frame` | scritto da `handeye_eye_in_hand_node.py`; **nessun consumatore a runtime** |
| `ee_tag_extrinsics.yaml` | `fr3_link8 → tag36h11:0` | solo `handeye_calibration_node.py` (calibrazione camera fissa) |

Due metodi di calibrazione, **nessuno dei due usa `cv2.calibrateHandEye`** — entrambi
solutori custom con `scipy.optimize.least_squares`: `handeye_calibration_node.py` (camera
fissa + tag su EE, risolve `X·T_CT=T_BE·Y` congiuntamente) e `handeye_eye_in_hand_node.py`
+ `utils/handeye_solver.py` (camera su flangia + tag fisso, forma chiusa tipo Park&Martin
poi rifinita non-linearmente).

---

## 8. Il layer di sicurezza ISO — cosa dichiara `franka_experiments/SAFETY.md`

Il pacchetto stesso lo dichiara esplicitamente (`SAFETY.md:5-7`):

> "This package implements a set of **ISO-alignment measures** on a research cell. It is
> **not a compliant system**, it is **not certified**, and nothing in it is a **safety
> function** in the sense of ISO 10218 / ISO 13849-1."

Motivo strutturale (`SAFETY.md:20-31`): la catena CBF è "single-channel Python over
best-effort DDS", senza ridondanza né tasso di guasto definito; per un robot Classe II
(FR3, 17.8 kg) servirebbe PL d/SIL 2, irraggiungibile da un nodo Python su kernel
general-purpose. Con `iso_c_intrusion` conforme (`C=0.85 m`, nessun dispositivo di
protezione certificato dietro la pipeline depth), la distanza di separazione minima
richiesta **eccede la portata stessa del braccio** (`d_floor=0.92 m > reach 0.855 m`,
`SAFETY.md:88-94`) — per questo `iso_enabled` resta `false` di default e
`cbf_safety_filter` solleva `ValueError` se qualcuno lo attiva senza aver spostato uno dei
tre termini (nessun bypass previsto, `:99-101`). Il termine "protective stop" è
deliberatamente riservato alla funzione certificata e **non appare** nei topic/log di
questo pacchetto — quello che implementa `iso_safety_monitor` è definito **"a
non-safety-rated stop"** (`SAFETY.md:36-39`).

---

## 9. Pipeline B in dettaglio — perché una nota a parte

Il docstring di `velocity_cbf_control_stack.launch.py:7` dichiara che `real_time_distance`
pubblica su `/human_robot/multi_distance` per questo stack. La percezione (§7.2, punto 11)
mostra che `real_time_distance` pubblica **sempre entrambi** i topic
(`/human_robot/multi_distance` e `/cbf/per_link_distances`) indipendentemente dallo stack;
`cbf_velocity_filter.py` legge da una chiave di configurazione (`:183-189`) il cui valore
risolto effettivo **non è stato accertato con certezza incrociata** dai due lati (percezione
vs. filtro) in questa analisi — vedi §14.

---

## 10. `franka_simulation` e `franka_sim`

`franka_simulation` (Gazebo Ignition + MoveIt2): quattro launch selezionabili
(`sim_position.launch.py`, `sim_velocity.launch.py`, `sim_acceleration.launch.py`,
`sim_torque.launch.py`) più `move_group.launch.py` per la pipeline CBF/avoidance completa,
che avvia (tra gli altri) `online_avoidance_controller.py` (Pinocchio confermato:
`import pinocchio as pin`, `franka_simulation/scripts/online_avoidance_controller.py:11,46`),
`velocity_control_blender.py` (CBF-QP di blending su velocità,
`franka_simulation/scripts/utils/velocity_blender_core.py:252-304,512-702`),
`obstacle_synchronizer.py`, e `human_pose_node.py` — **confermato** uso di MediaPipe
(`import mediapipe as mp`, `franka_simulation/scripts/human_pose_node.py:34`). Tutto il
codice sorgente Python vive sotto `franka_simulation/scripts/` (non nella directory
`ament_python` `franka_simulation/franka_simulation/`, che contiene solo un `__init__.py`
vuoto); `franka_simulation/src/` è vuoto.

`franka_sim` (MuJoCo, **zero import ROS** — `grep -rn "rclpy" franka_sim/` non trova
nulla): `envs/franka_cbf_env.py` (`import mujoco`, ambiente Gymnasium `FrankaCBF-v0`) più
un CBF filter duplicato (`envs/cbf_filter.py`). `train.py` addestra con
**SAC di Stable-Baselines3** (`from stable_baselines3 import SAC`, `train.py:30,199` —
confermato), `export_onnx.py` esporta l'attore in ONNX. `rl_policy_commander.py` (pipeline
A) esegue quel grafo ONNX sul robot reale.

---

## 11. Package di supporto (survey leggero)

| Package | Cosa fornisce | Riferimento |
|---|---|---|
| `franka_bringup` | `franka.launch.py`: `robot_state_publisher`, `ros2_control_node`, `joint_state_publisher` (fonde `franka/joint_states` + gripper), spawner `joint_state_broadcaster` (sempre) e `franka_robot_state_broadcaster` (solo se non fake hardware), include `franka_gripper/launch/gripper.launch.py` se `load_gripper` | `franka_bringup/launch/franka.launch.py:87-173` |
| `franka_fr3_moveit_config` | `move_group.launch.py`: costruisce `robot_description`/`_semantic` da `franka_description`, **solo planner OMPL** (nessuna dipendenza Pilz nel `package.xml`) | SRDF stesso è un thin-wrapper che include `franka_description/robots/common/franka_arm.srdf.xacro` (condiviso da tutta la famiglia Franka, non specifico FR3) |
| `franka_gazebo` | integrazione Ignition **generica** (`franka_ign_ros2_control`, plugin `ros2_control` per Ignition) + esempi di bringup (velocità/posizione/impedenza) | **non è un dipendente né una dipendenza di `franka_simulation`** — nessun riferimento incrociato trovato in nessuna direzione (§13) |
| `franka_gripper` | `franka_gripper_node`: azioni `homing`/`move`/`grasp`/`gripper_action` (compatibile MoveIt), servizio `stop`; più `scripts/fake_gripper_state_publisher.py` per hardware fake | |
| `franka_robot_state_broadcaster` | vedi §5.5 | |
| `franka_semantic_components` | libreria (non nodo): `franka_robot_model`, `franka_robot_state`, `franka_cartesian_pose_interface`, `franka_cartesian_velocity_interface` — wrapper tipizzati sopra le interfacce grezze | |
| `franka_msgs` | messaggi: `CollisionIndicators`, `Elbow`, `Errors` (~35 booleani, uno per condizione libfranka), `Float64StampedArray`, `FrankaRobotState`, `GraspEpsilon`, `HumanRobotDistance` (legacy), `LinkDistance` (perceptione CBF corrente), `MultiDistance`/`MultiLinkDistance`; servizi `Set*` (stiffness/collision/load/frame); azioni `ErrorRecovery`, `Grasp`/`Homing`/`Move` | |
| `integration_launch_testing` | due test `launch_testing`: gripper (`Move` action, convergenza posizione) e bringup dell'example controller — **zero riferimenti** a `cbf`/`avoidance`/`franka_experiments`/`franka_simulation` (grep) | non copre nessuno stack CBF |

---

## 12. Divergenze codice ↔ documentazione/commenti

1. **`rt_torque_controller`: LPF e clipping dichiarati ma inesistenti.** Sia il
   `README.md` (tabella package) sia il manifest plugin stesso
   (`rt_velocity_blender_controller.xml`, blocco `RtTorqueController`) affermano
   "optional low-pass filter, per-joint clipping". Il codice non dichiara né legge mai
   `lpf_alpha`/`tau_max_scale` — **ammesso nel codice stesso**:
   `rt_torque_controller.cpp:295`, "(che oggi passa lpf_alpha/tau_max_scale/urdf_path
   senza che il C++ li dichiari → ignorati)". Il comportamento reale è PD in spazio
   giunto + inviluppo di velocità + hold-on-stale (§5.3), non un LPF/clip.

2. **Tre nomi per un controllore.** Classe `RtVelocityExecutorController`, file
   `rt_velocity_blender_controller.*`, istanza `rt_velocity_executor_controller` (§5.4).
   Innocuo funzionalmente, ma fuorviante leggendo l'albero dei file.

3. **`pentagon_qddot_commander` non usa MoveIt, contrariamente al proprio docstring e al
   commento del launch file.** Il commento di `torque_control_stack.launch.py:451-455`
   afferma: "pentagon_qddot_commander genera il pentagono via i servizi MoveIt compute_fk
   / compute_cartesian_path, serviti da move_group." **Verificato con grep diretto**
   (`grep -rniE "moveit|compute_fk|compute_cartesian_path|move_group"` su
   `pentagon_qddot_commander.py`, `utils/trajectory.py`, `utils/kinematics.py` → **zero
   risultati**, exit code 1). Il nodo costruisce la propria cinematica con Pinocchio
   (import `utils.kinematics`). `move_group` viene comunque avviato di default, ma
   **nessuna prova che il path di controllo live lo interroghi**.

4. **Il docstring dello stesso nodo è anch'esso stale**: `pentagon_qddot_commander.py`
   riga 2 dice "Pentagon trajectory for cbf_torque_controller — no CBF filter", ma
   l'unico launch file che lo avvia lo collega invece a `cbf_safety_filter` (un filtro
   CBF, non `cbf_torque_controller`) — l'opposto di quanto scritto.

5. **`cbf_OSCBF_filter.py` e la pipeline OSCBF documentata in `config/oscbf_params.yaml`
   non sono mai assemblate da nessun launch file, test o script** (§3, "codice morto").
   Il README la presenta come "an alternative torque path" — esiste come file, ma non
   come percorso eseguibile end-to-end in questo repository.

6. **`franka_experiments` dipende in runtime da `franka_simulation`**, cosa che nessuno
   dei due README rende esplicita: `package.xml:30` (`exec_depend`),
   `utils/simulation_imports.py:1-60` (bridge che ri-esporta funzioni CBF/Pinocchio di
   `franka_simulation` per `capsule_overlay_node.py`, con `ImportError` esplicito se
   `franka_simulation` non è compilato), e **quattro** launch file di
   `franka_experiments` che avviano l'eseguibile `image_publisher` di `franka_simulation`
   come `image_republisher`. La dipendenza è **unidirezionale** (grep su
   `franka_simulation/` per `franka_experiments` → 0 risultati).

7. **`franka_gazebo` è una terza integrazione Ignition, scollegata da quella di
   `franka_simulation`.** `franka_simulation/package.xml` non dipende da `franka_gazebo`;
   include direttamente `ros_gz_sim`/`ros_gz_bridge`. Un lettore del README potrebbe
   supporre che la pipeline Gazebo di `franka_simulation` passi da `franka_gazebo` —
   non è così.

8. **Nessun clamp software di limiti giunto/workspace nell'hardware interface** —
   non contraddice esplicitamente alcun documento, ma è un'assunzione facile da fare
   leggendo "hardware abstraction" nel README senza controllare il codice: l'unica rete
   qui è NaN/Inf reject + rate-limiter torque di libfranka (§5.2); tutto il resto è
   delegato al controllore RT e ai riflessi firmware.

---

## 13. Incerto o non verificato

- **Topic effettivo letto da `cbf_velocity_filter.py` per le distanze (pipeline B).** La
  percezione conferma che `real_time_distance` pubblica sempre sia
  `/human_robot/multi_distance` (default hardcoded, la chiave non esiste in
  `fr3_complete.yaml`) sia `/cbf/per_link_distances`; il lato filtro conferma solo che
  legge "per_link_distances topic or configured key" (`cbf_velocity_filter.py:183-189`).
  Non è stata fatta una verifica incrociata finale del valore YAML risolto a runtime per
  questo stack specifico — potrebbe essere l'uno o l'altro.
- **Semantica completa degli 11 campi di `/NS_1/cbf_status`.** Il commento nel codice
  (`cbf_safety_filter.py:327-356`) ne descrive esplicitamente solo 5; i campi 5-8
  sarebbero "i numeri del MONITOR" (riferimenti a `fr3_control.yaml:2137,2156,2173`) ma
  non tutti gli 11 sono stati tracciati singolarmente.
- **Frequenza effettiva di `rl_policy_commander`** quando `rate_hz=0.0` ("eredita dal
  rate del sim") — non individuato il valore numerico risolto in `utils/rl_policy.py`.
  `ee_circle_velocity_commander.py`/`ee_random_waypoints_velocity_commander.py` risultano
  senza wiring in launch/test/script, ma non è stata esclusa in modo esaustivo
  l'invocazione diretta (`ros2 run`) da uno script di `scripts/*.py` non grepato per
  `subprocess`/`ExecuteProcess`.
- **`cbf_evasion.py`, `evasion_direction.py`, `livelock.py`**: la logica di
  evasione/livelock è stata vista solo attraverso i punti di chiamata in
  `cbf_state_rows.py`, non letta come file a sé; la matematica esatta non è
  indipendentemente verificata riga per riga.
- **`oscbf_params.yaml`**: non è stato accertato se sia consumato da un qualunque path
  live (probabilmente solo da `cbf_OSCBF_filter.py`, che è morto — §3).
- **Priorità `thread_priority` nella YAML generata vs. priorità SCHED_FIFO reale**: non
  tracciato dentro `controller_manager`/`realtime_tools` (esterni al repo) se
  `thread_priority: 85/98` diventi numericamente la `rtprio` del thread FF.
  `pin_rt_thread.sh` presuppone che quel thread esista già quando esegue il polling.
  - `handeye_solver.py` è stato caratterizzato come "strutturalmente simile a Park &
  Martin" in base alla matematica (SVD di una matrice di correlazione), non perché il
  codice citi quel riferimento — è un'interpretazione, non un'affermazione del repo.
- **`camera_intrinsics.yaml` e `depth_intrinsics.yaml`**: zero consumatori Python/YAML
  trovati via grep in tutto il repo — potrebbero essere dump di riferimento inerti, o
  consumati da uno strumento esterno (notebook, script shell) non incluso nella ricerca.
- **franka_hardware/franka_action_server.cpp, franka_param_service_server.cpp**: letti
  solo a livello di firma (nomi di servizio/azione), non ogni corpo di lambda.
- **`ee_pentagon_velocity_commander.py` non è stato riletto per confermare che
  `tracking_topic` risolva letteralmente alla stringa `/NS_1/tracking_qdot`** — dedotto
  dal default coerente in `fr3_control.yaml:30` e dal docstring del modulo, non da una
  citazione di riga diretta sul valore effettivo pubblicato.

---

*Fine documento. Nessun file del repository è stato modificato nel produrre questa
analisi, a parte la creazione di questo stesso file.*
