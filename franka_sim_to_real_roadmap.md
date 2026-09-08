# Architettura e Roadmap Sim-to-Real: Safe RL + CBF per Franka Emika

Questo documento traccia l'architettura software e la roadmap di sviluppo per implementare una pipeline di **Safe Reinforcement Learning (Safe RL)** con **Control Barrier Functions (CBF)** per il robot Franka Emika. L'obiettivo è addestrare una policy in simulazione (Sim) e trasferirla in modo sicuro sul robot reale (Real) sfruttando la potenza di calcolo di una GPU RTX 4070.

---

## 🏗️ Architettura del Progetto e Struttura dei File

La struttura separa nettamente l'ambiente di addestramento (indipendente da ROS 2 per massimizzare le performance di calcolo) dal modulo di deployment reale inserito nel workspace ROS 2 esistente.

```text
franka_workspace/
├── franka_sim/                      # Modulo di Simulazione (Standalone / Addestramento)
│   ├── envs/
│   │   └── franka_cbf_env.py        # Ambiente Gymnasium custom con MuJoCo e filtro CBF
│   ├── config.yaml                  # Iperparametri RL, configurazioni task e limiti fisici
│   ├── train.py                     # Script di training (Stable-Baselines3 con supporto CUDA)
│   └── export_onnx.py               # Esportazione della policy in formato ONNX
│
└── franka_experiments/              # Workspace ROS 2 Esistente
    └── franka_experiments/
        └── nodes/
            ├── cbf_safety_filter.py # Filtro di sicurezza esistente
            └── rl_policy_commander.py # NUOVO: Nodo di inferenza ONNX e comando policy RL
```

---

## 📄 Dettaglio dei File da Creare

### Fase 1: Il Blocco di Simulazione (`franka_sim/`)

#### 1. `franka_sim/envs/franka_cbf_env.py`
* **Tipo di file:** Script Python (Classe Custom `gymnasium.Env`).
* **Obiettivo:** Definire l'ambiente di simulazione del Franka Emika simulando sia la cinematica/dinamica che l'effetto dello "shielding" probabilistico/deterministico del CBF.
* **Logica Interna:**
  * `__init__`: Definizione dello spazio delle osservazioni (es. Joint Positions, Velocities, End-Effector Pose, Target Distance) e dello spazio delle azioni (es. Joint Velocities).
  * `step(action)`: Riceve l'azione dalla policy di RL, la passa attraverso la formula matematica del filtro CBF (analoga a quella reale), applica l'azione corretta al motore fisico MuJoCo, e calcola la Reward Function (Task reward + penalizzazioni).
  * `reset()`: Riposiziona il robot nello stato iniziale e genera un nuovo target per l'episodio.

#### 2. `franka_sim/config.yaml`
* **Tipo di file:** File di configurazione YAML.
* **Obiettivo:** Centralizzare i parametri per garantire la totale replicabilità degli esperimenti scientifici.
* **Logica Interna:**
  * **Task parameters:** Coordinate dell'area di lavoro, soglie di tolleranza del target.
  * **RL hyperparameters:** Algoritmo scelto (es. SAC o PPO), learning rate, batch size, gamma, buffer size ottimizzati per la VRAM della RTX 4070.
  * **CBF boundaries:** Coefficienti di tolleranza ($lpha$), margini di sicurezza degli ostacoli.

#### 3. `franka_sim/train.py`
* **Tipo di file:** Script Python di Addestramento.
* **Obiettivo:** Avviare la pipeline di ottimizzazione della policy sfruttando l'accelerazione hardware CUDA.
* **Logica Interna:**
  * Inizializzazione dell'ambiente custom (`franka_cbf_env`).
  * Configurazione del Logger (es. TensorBoard) per tracciare curve di convergenza, errori di tracking e violazioni dei vincoli.
  * Utilizzo di algoritmi a controllo continuo di **Stable-Baselines3** (consigliato: Soft Actor-Critic - SAC).
  * Implementazione di `CheckpointCallback` per salvare automaticamente i pesi del modello migliore (`best_model.zip`).

#### 4. `franka_sim/export_onnx.py`
* **Tipo di file:** Script Python di Utility.
* **Obiettivo:** Isolare la rete neurale dell'Attore (Policy) ed esportarla in formato standard ONNX per eliminare le dipendenze da librerie di training pesanti nel mondo reale.
* **Logica Interna:**
  * Caricamento del file `best_model.zip`.
  * Estrazione del modulo PyTorch `actor`.
  * Tracing/Esportazione del grafo computazionale verso un file `.onnx`.

### Fase 2: Integrazione nel Workspace Reale (`franka_experiments/`)

#### 5. `franka_experiments/franka_experiments/nodes/rl_policy_commander.py`
* **Tipo di file:** Nodo ROS 2 Python.
* **Obiettivo:** Rappresentare il cervello esecutivo Sim-to-Real sul robot fisico, garantendo un'esecuzione deterministica ad alta frequenza (>100Hz).
* **Logica Interna:**
  * Sottoscrizione ai topic di stato del robot (lettura di giunti ed end-effector tramite moduli esistenti come `frame_grabber.py`).
  * Inizializzazione del runtime di inferenza (`onnxruntime.InferenceSession`) caricando il file `.onnx`.
  * Nel loop di controllo temporizzato:
    1. Vettorizzazione dello stato attuale nello stesso identico formato delle osservazioni usate in Sim.
    2. Esecuzione dell'inferenza ONNX veloce.
    3. Pubblicazione dell'azione desiderata sul topic di input del filtro CBF (`cbf_safety_filter.py`).

---

## 📈 Tabella di Marcia per lo Sviluppo con Claude Code

Seguire questo ordine logico sequenziale per guidare Claude Code nello sviluppo guidato dei file:

| Step | Focus Principale | File Coinvolti | Promp di Input Consigliato per Claude Code |
| :--- | :--- | :--- | :--- |
| **1** | **Ambiente di Simulazione** | `franka_cbf_env.py`<br>`config.yaml` | *"Crea un ambiente Gymnasium custom per il robot Franka combinando MuJoCo come motore fisico e i vincoli matematici del filtro CBF presenti nel mio file cbf_safety_filter.py."* |
| **2** | **Pipeline di Addestramento** | `train.py`<br>`export_onnx.py` | *"Scrivi uno script di training usando Stable-Baselines3 (SAC). Assicurati che utilizzi l'accelerazione CUDA per la mia RTX 4070, configuri TensorBoard e includa un modulo per esportare la policy finale in ONNX."* |
| **3** | **Nodo ROS 2 di Deployment** | `rl_policy_commander.py` | *"Crea un nodo ROS 2 in Python che carichi la policy ONNX, raccolga la posizioni dei giunti dai topic di stato, esegua l'inferenza e pubblichi i comandi sul topic di input del nostro cbf_safety_filter.py real."* |

---

## 🔬 Consigli per la Stesura del Paper Scientifico

1. **Il Ruolo dello Shielding:** Enfatizza come l'inclusione del filtro CBF all'interno del ciclo di apprendimento (Sim) e di esecuzione (Real) garantisca la proprietà di **Safe Exploration**. Il robot non viola mai i vincoli geometrici né in simulazione né sulla macchina reale.
2. **Mitigazione del Sim-to-Real Gap:** Nel testo del paper, documenta come le discrepanze di attrito e dinamica tra MuJoCo e il Franka reale vengano assorbite e corrette istantaneamente dal filtro CBF reale. La policy appresa si occupa del task macroscopico, mentre il CBF gestisce la sicurezza microscopica a livello hardware.
3. **Analisi delle Frequenze:** L'uso di ONNX Runtime nel nodo ROS 2 permette di mantenere tempi di ciclo estremamente bassi e deterministici. Includi nel paper un grafico del jitter temporale del nodo per dimostrare l'efficacia dell'architettura in tempo reale.
