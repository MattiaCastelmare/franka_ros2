# Simulazione FR3 con Gazebo sul Mac

Questa guida usa la configurazione già preparata per il Mac M2, con Docker Desktop e Xpra, sul branch `fr3-student-sim`. La prova avvia il controller di [inseguimento circolare](progress.md) e visualizza il robot in Gazebo.

## 1. Apri Docker Desktop e la repository

Nel **Terminale del Mac**, esegui questi comandi.

```bash
open -a Docker
cd "$HOME/Desktop/uni/Magistrale/Primo anno/SECONDO SEMESTRE/Robotica 2/repository/franka_ros2"
git branch --show-current
```

Il branch deve risultare `fr3-student-sim`. Aspetta che Docker Desktop abbia completato l’avvio. Puoi controllare che il motore risponda con questo comando.

```bash
docker info --format '{{.ServerVersion}}'
```

## 2. Avvia i container e collega Xpra

Sempre nel **Terminale del Mac**, dalla cartella della repository, esegui lo [script di connessione](../.devcontainer/connect-xpra.sh). Lo script avvia i servizi Docker del progetto e collega il client grafico sul Mac. Quando va a buon fine, entra nel container.

```bash
bash .devcontainer/connect-xpra.sh
docker exec -it franka_ros2 bash
```

Lo script sostituisce `docker start franka_ros2` della guida Ubuntu/WSL per questo avvio. La finestra Gazebo compare quando lanci la simulazione.

## 3. Avvia la simulazione circolare

Ora sei **dentro il container**. Carica l’ambiente ROS 2 e il workspace compilato, poi avvia il controller.

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

ros2 launch fr3_student_sim student.launch.py \
  controller:=circle_tracking/controller.py \
  task_config:=/ros2_ws/src/fr3_student_sim/config/circle_tracking.yaml \
  wall:=false gui:=true duration:=60
```

Il robot segue la traiettoria circolare senza parete. `gui:=true` apre Gazebo tramite Xpra. La scena 3D può restare nera per alcuni secondi mentre carica. `duration:=60` indica 60 secondi **simulati** dopo l’assestamento del robot. Alla fine Gazebo si chiude automaticamente. Nel terminale, `RUN started` indica che il controller ha iniziato la prova e `saved ... .npz` mostra il percorso del risultato.

## 4. Interrompi la prova o chiudi il progetto

- Per interrompere una prova in corso, premi **Ctrl+C** nel terminale che l’ha avviata.
- Per ripetere la prova nella stessa shell, rilancia il comando `ros2 launch` del punto 3.
- Per uscire dalla shell del container, digita `exit`. Tornato nel **Terminale del Mac**, puoi elencare i risultati e fermare i servizi.

```bash
ls -lt fr3_student_sim/runs
docker compose stop
```

`docker compose stop` conserva i container, i volumi di compilazione e la configurazione Xpra. I risultati `.npz` sono nella cartella `fr3_student_sim/runs` della repository sul Mac.

## Se Gazebo non compare

Se il terminale mostra `RUN started` e la finestra manca, apri **un secondo terminale sul Mac**, torna nella cartella del punto 1 e verifica il collegamento grafico.

```bash
bash .devcontainer/connect-xpra.sh
docker compose ps
tail -n 30 "$HOME/Library/Logs/Franka-Xpra/client.log"
```

`docker compose ps` deve mostrare entrambi i servizi attivi, `franka_ros2` e `xpra_proxy`. Nel log del client, `Attached to xpra server` indica che Xpra si è collegato. Se la prova è già terminata, rilanciala dal punto 3. Per esaminare Gazebo, il launch stampa il percorso di `gazebo.log` nel container. Nella stessa cartella trovi `gazebo_gui.log`.

## Se aggiungi, rimuovi o rinomini file del pacchetto

Il pacchetto è già compilato per la simulazione descritta sopra. Dopo aver cambiato l’elenco dei file di `fr3_student_sim`, ricompilalo **dentro il container** prima di lanciare di nuovo la simulazione.

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fr3_student_sim
source /ros2_ws/install/setup.bash
```

Per modifiche a file già installati con `--symlink-install`, basta riavviare la prova.
