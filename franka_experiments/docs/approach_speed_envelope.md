# Inviluppo della velocità di avvicinamento e collo di bottiglia della schivata

2026-10-01, branch `humble-mattia` @ 762b096. Solo lettura/misura: nessun file del percorso di sicurezza
(`cbf_safety_filter`, `cbf_state_rows`, `rt_torque_controller`, `real_time_distance`) è stato modificato.
**Niente è stato provato sul robot.** Etichette: **MISURATO** (bag o replay del codice attuale),
**MODELLATO** (`ball_closed_loop.py` con braccio modellato, o `approach_envelope.py`), **ASSUNTO**.
Rumore tra repliche di percezione ±2 hit / ±2 cm: ogni confronto è la media di 3 repliche.

## 0. Risposta in breve

**Il limite è il tempo di preavviso.** La percezione dà ~0.30 s (mediana 0.29–0.30 s, replay di
`ball_throws_3`); l'oracolo mostra che servono ≥0.45 s. Il preavviso è limitato dal rilascio della palla,
meno ~0.15 s in cui la palla resta fusa col lanciatore. Né la depth, né le frequenze, né il tracker, né
l'autorità del braccio lo spiegano. Inviluppo (MODELLATO, percezione ideale quindi limite *superiore*):
schivata di una palla puntata al centro fino a ~2 m/s; a 3 m/s si colpisce di 1–3 cm; da 4 m/s no.

## 1. Fase 0 — inventario

* **Bag** (`franka_experiments/rosbag/`, ignorata da git, 159 GB: *presente in locale*). Con verità di
  terreno: `ball_throws_2` (9 passate), `ball_throws_3` (6). `ball_throws_4`: braccio mai mosso, nessun lancio
  (docs/ball_throw_closed_loop.md §2). `ball_throws`: solo depth allineata, senza TF/giunti/depth grezza,
  non riproducibile. 
* **Parametri attivi** (file:riga): `obstacle_velocity_max` 2.0 (`fr3_control.yaml:1174`; sul percorso fast-trust
  il tetto è `obstacle_velocity_fast_max` 6.0, `:1355`); `obstacle_velocity_min_frames` 8 (`:1248`) e
  `min_span_s` 0.265 s → 24 frame a 90 Hz (`:1329`, `obstacle_input_rate_hz` 90); `max_thresh` 0.7
  (`fr3_complete.yaml:196`); `cluster_min_points` 5 (`:401`); `roi_pad_px` 180 (`:162`); `pixel_step` 6 (`:146`);
  `cluster_max_radius_m` 1.2 (`:460`); `cluster_depth_jump_m` 0.10 (`:424`); `gate_mahalanobis` 4.0,
  `gate_max_m` 0.5, `sigma_v0` 2.0, `confirm_window_s` 0.056; `enable_latency_compensation` true
  (`fr3_control.yaml:2040`, `launch_defaults.yaml:262`); profilo camera `848x480x90` (`launch_defaults.yaml:41`).
  `lpf_v_max_approach` non è in nessun yaml: vale il default 2.0 del motore (`distance_engine.py:188`).
* **Divergenze documenti ↔ codice.**
  1. `reactivity_evasion_report.md` dice «il launch fissa 848x480x30»; il codice fissa **x90** (commit 5de8965).
  2. Il report dà un tetto d'associazione di 4.25 m/s @30 fps: con `sigma_v0` 2.0 / gate 4.0 attuali non
     è più vero (§2, H4).
  3. `fr3_control.yaml` ha `obstacle_velocity_source: residual`, ma `launch_defaults.yaml:203` lo imposta a
     `tracker`: vale il launch.
  4. `ball_throw_eval.py` non classifica le 4 cause: stampa le colonne (row/tracked/trust/cbf). La
     classificazione è in `approach_probe.py causes` (prima causa che si applica, soglia «serve» ASSUNTA 450 ms).
  5. `cbf_scenarios.py` ha un solo messaggio in volo: con latenza > periodo (48 ms vs 13 ms) il filtro non
     riceve mai nulla (verificato: palla 1.5 m/s, 76 fps, ritorno 0). `approach_envelope.py` lo corregge con una
     coda FIFO applicata a runtime al sorgente; a 30 fps riproduce il risultato originale.
* **Baseline test** (container, `pytest test`): **1170 passati, 1 fallito** già esistente e estraneo:
  `test_node_self_references.py::test_every_node_self_call_resolves` (`handeye_eye_in_hand_node.py:200`,
  `_parameters` non definito). Il report di settembre ne contava 10.

## 2. Fase 1 — ipotesi

| | verdetto | misura |
|---|---|---|
| H1 la depth perde la palla | **RIFIUTATA come limite**, confermata come degrado | vedi sotto |
| H2 palla fusa nel blob del lanciatore | **CONFERMATA** (13/15 passate) | vedi sotto |
| H3 frequenze | **RIFIUTATA** come limite principale (≤ ~35 ms recuperabili) | vedi sotto |
| H4 tetto d'associazione | **RIFIUTATA** (< ~8 m/s non limita) | vedi sotto |
| H5 cap / spike gate / gate 0.7 m | **RIFIUTATA** nelle bag; il gate 0.7 m è un limite latente | vedi sotto |
| H6 conferma traccia | **RIFIUTATA** (costa ~30 ms) | vedi sotto |
| H7 autorità del braccio | **RIFIUTATA** con ≥0.45 s di preavviso | vedi sotto |

**H1 (MISURATO, `approach_probe.py depth`, nodo vero frame per frame).** Nei 0.6 s prima dell'avvicinamento
massimo, a 1.0–1.5 m la depth copre ~40 % dei pixel attesi di una palla da 3.6 cm di raggio (160 px visti
contro 377 attesi; ~380 px a zero-depth nel disco), con 4–5 campioni sulla griglia `pixel_step` 6 contro
`cluster_min_points` 5 (P(≥5) 36–55 %). Tuttavia, appena la palla è nel ROI e separata, ha un cluster proprio nel
58–93 % dei frame e una traccia dopo 1–3 frame: la palla è marginale, non invisibile.
`bt3`: `approach_env/bt3_depth.csv`; `bt2`: `bt2_default.csv`.

**H2 (MISURATO, stesso comando, ROI 700 e `cluster_max_radius_m` 3.0 solo come prova, nel probe).**
Prima della separazione la palla è dentro un blob di 44–1900 punti e raggio 0.2–1.7 m (mano, avambraccio o intero
lanciatore + parete). Durata della fusione prima del cluster proprio: `bt3` 0.38–0.90 s in 5/6 passate; `bt2`
0.23–1.00 s in 8/9. Con la configurazione di default (raggio massimo 1.2 m) i blob più grandi vengono
scartati: la palla non ha alcun cluster. Il cluster proprio compare a 0.26–0.42 s (mediana 0.33 s con ROI 180,
0.39 s con ROI 700 su `bt3`; 0.44 / 0.37 s su `bt2`): **allargare il ROI non lo anticipa**, perché prima la
palla è fusa. Rilascio→cluster proprio (rilascio = primo frame colore con velocità > metà di quella di volo,
risoluzione 33 ms, stima in ritardo quindi la perdita è sottostimata): `bt3` 0.07–0.22 s (mediana ~0.12),
`bt2` 0.16–0.26 s (mediana ~0.18, con due passate anomale escluse: la 4 e la 9). Valore usato: **~0.15 s**.
Con la sola eccezione della passata 2 di `bt3` (rilascio 1.03 s prima, cluster 0.81 s prima) il preavviso
di cluster è 0.3–0.45 s.

**H3 (MISURATO, `approach_probe.py rates`, `latency_budget.py bag`; bag live `ball_throws_3`, 88.7 s).**
Camera: 89.8 frame distinti/s (periodo 11.12 ms, p99 16.5), ma a raffiche: 66 % dei frame arriva < 4 ms dopo il
precedente, 33 % dei gap > 25 ms (3 frame ogni ~33 ms); cattura→ricezione 25.6 ms (p99 42). Il 40 % dei messaggi
è duplicato (stesso stamp). Nodo: pubblica **75.8 Hz** (74.4 in `bt4`, 86.5 in `bt2`), periodo dei frame
elaborati p99 33 ms, max 55 ms; 1245 frame su 7970 (16 %) non diventano messaggio (`bt4` 17 %, `bt2` 3.7 %).
Cattura→messaggio ricevuto: **48.5 ms** mediana, 66 ms p99 (il vecchio budget ipotizzava 12–14 ms a 30 fps).
`/tf` è a 15 Hz (periodo 66.7 ms). Hop di calcolo (`latency_budget.py cbf`, container a riposo): rebuild
0.87 ms, tick QP 0.33 ms. Impatto: portare 48 ms a 13 ms vale ~0.035 s di preavviso, cioè +1–2 cm di
clearance nella scala dell'oracolo (§3), contro ~0.15 s della fusione.

**H4 (MODELLATO, `approach_probe.py assoc`: `TrackManager` come da configurazione, centroidi sintetici,
rumore 1 cm).** A 90 fps un oggetto isolato è seguito fino a 8 m/s (conferma al frame 3); a 10 m/s al frame 5; a 15 m/s mai.
A 30 fps fino a 8 m/s, a 15 fps fino a 6 m/s (a 8 m/s non più). Il tetto è circa 4·`sigma_v0` = 8 m/s al
primo aggancio, non v·dt: la cifra «4.25 m/s» del report è superata. Con rumore del centroide 3 cm (ASSUNTO)
nascono 4–8 id per oggetto ma resta seguito fino a 10 m/s (68 % dei frame).

**H5.** (i) `obstacle_velocity_max` 2.0 non agisce sulla palla (3.4–3.9 m/s): passa dal fast-trust con tetto 6.0
(`fast` 6/6 passate, `approach_env/replay_bt3_r*`). (ii) Spike gate `lpf_v_max_approach` 2.0 m/s: ritarda una
caduta di distanza al più di un frame (11–13 ms) (`distance_engine.py:680`); il test del documento a 6 m/s
non dà guadagno. (iii) Gate di pubblicazione 0.7 m: **replay con `max_thresh` 2.0, 3 repliche contro 3
baseline su `bt3`: lead della prima riga sulla palla mediana 287/294/294 ms contro 293/291/299 ms,
fast-trust e CBF invariati** (`approach_env/replay_bt3_thr20_r*`). Nelle bag non limita perché c'è sempre
qualcosa a < 0.7 m da un control point (lanciatore/chi recupera) e allora tutti i CP sono pubblicati. Il
modello a oggetto singolo invece lo vede come vincolo forte (+18 cm a 3 m/s; §3): è un limite **latente**, non
misurato su dati reali, da verificare con scena vuota attorno al braccio (§6).

**H6 (MISURATO, `ball_throw_eval.py score` sulle repliche).** Prima riga sulla palla → fast-trust: 0–45 ms
(es. 289→258 ms, 307→307). Il gate a span (24 frame) non viene mai raggiunto (0–1 passate su 6: la traccia
vive 9–29 frame), quindi tutto passa dal fast-trust (frames_seen ≥ 3). Errore di velocità della traccia vs
verità: 0.66 m/s nei primi 5 frame, 0.26 m/s dopo; rapporto sulla velocità vera 0.95; direzione 4°.

**H7 (MODELLATO, `ball_closed_loop.py --threat 0 --ball-only --oracle 8`, 3 repliche per bag).** Con la
spinta laterale perfetta e preavviso 0.45 s il braccio schiva (§3): l'autorità basta; con 0.30 s no. Peak
comando 6 rad/s² (box), |q̇|/inviluppo ≤ 0.72.

**Distribuzione delle 4 cause** (`ball_throw_eval.py score` + `approach_probe.py causes`, soglia «serve» 450 ms
ASSUNTA, tratta dal documento sull'oracolo). Per passata, 1 non nelle righe / 2 non tracciata / 3 velocità non fidata
/ 4 troppo tardi / ok:

| bag | passate | 1 | 2 | 3 | 4 | ok |
|---|---|---|---|---|---|---|
| `ball_throws_3` live (config di allora) | 6 | 0 | 0 | 0 | 6 | 0 |
| `ball_throws_2` live (config di allora: stride 10, ROI 90) | 9 | 0 | 8 | 0 | 1 | 0 |
| `ball_throws_3` replay codice attuale, 3 repliche | 6 | 0 | 0 | 0 | 5 / 5 / 5 | 1 / 1 / 1 |
| `ball_throws_2` replay codice attuale, 3 repliche | 9 | 0 | 0 | 0 | 8 / 7 / 8 | 1 / 2 / 1 |

Con il codice attuale tutto il peso è la causa 4 (la palla è nelle righe, tracciata e fidata, ma il CBF agisce
con meno di 450 ms); le cause 1–3 sono state risolte dai commit 053ce18 / 95a5c00 e dal ROI 180. Le «ok» sono
le passate con più preavviso (bt3-2: CBF a ~500 ms).

## 3. Fase 2 — inviluppo

`approach_envelope.py`: filtro/QP veri, braccio a doppio integratore a riposo, palla di 3.6 cm puntata sul
flangia (centro), percezione emulata con parametri **misurati** (fps 76, cattura→CBF 48 ms, ritardo
rilascio→separazione 0.15 s) e velocità del tracker ideale (verità + 2 cm/s). Gap minimo di superficie
in cm (`H` = colpo, gap < 0). **È un limite superiore**: la percezione reale è peggiore.

| scenario \ v [m/s] | 0.5 | 1 | 1.5 | 2 | 3 | 4 | 5 | 6 |
|---|---|---|---|---|---|---|---|---|
| (a) già tracciato da lontano | +23 | +22 | +23 | +16 | −1 H | −5 H | −7 H | −7 H |
| (b) visto la prima volta al gate 0.7 m | +23 | +21 | +19 | +12 | −3 H | −7 H | −8 H | −8 H |
| (c) lancio puntato, rilascio a 1.5 m | +23 | +22 | +22 | +15 | −2 H | −5 H | −8 H | −9 H |
| (c) rilascio a 2.0 m | +23 | +22 | +24 | +15 | −2 H | −6 H | −7 H | −6 H |
| (c) rilascio a 3.0 m | +23 | +23 | +22 | +16 | −1 H | −6 H | −6 H | −8 H |
| (c) rilascio a 4.0 m | +84* | +23 | +23 | +16 | −1 H | −5 H | −7 H | −7 H |

\* la palla non è ancora arrivata a fine simulazione. La distanza di rilascio non cambia l'esito: il tempo di
preavviso si esaurisce al gate/separazione, non al rilascio.

**Vincolo attivo** (rilassando un vincolo alla volta; guadagno di gap in cm rispetto alla cella; colpo che
diventa mancato = «flip»), per gli scenari (a), (c) 1.5 m e 2 m:

| v | vincolo | note |
|---|---|---|
| ≤ 2 | nessuno | schivata |
| 3 | gate di pubblicazione 0.7→1.5 m (+18 cm, flip); latenza 48→13 ms (+2…+3 cm, flip in 3 su 4 scenari); autorità braccio (+0…+1) | margine di 1–3 cm |
| 4 | gate di pubblicazione (+5…+6, flip) in (a) e (c) 2 m; nessuno da solo in (b) e (c) 1.5 m; tutti insieme +10…+12 cm | |
| 5 | nessuno da solo (latenza −6…−7; gate −3…−5); tutti insieme +3 cm | combinazione |
| 6 | nessuno; tutti insieme −2…−3 cm | **tempo di volo totale** |

Quindi, in modello: ~2 m/s (dead-centre) con la catena attuale; 3 m/s sul limite; oltre 4 m/s solo
rilassando insieme gate, latenza e separazione; > 5 m/s fisicamente non schivabile con questo braccio da
questa distanza. Il gate 0.7 m domina *in modello* ma non nelle bag (H5): non attribuire quel numero al sistema reale.

**Validazione con percezione reale** (closed loop, repliche del codice attuale, `ball_closed_loop.py`, media
di 3 repliche; colpi = clearance < 0): tiri come registrati, 3.4–4.5 m/s: `bt2` 1.0/9, `bt3` 1.0/6; **puntati al
centro** (`--threat 0`): `bt2` 4.7/9, `bt3` 5/6; solo palla (`--ball-only`): `bt2` 3.0/9 (clr +0.7 cm),
`bt3` 2.3/6 (+0.4 cm). Coerente con la tabella (da 3 m/s in su colpisce). Per passata (`bt3` r1, solo palla)
la clearance simulata cresce col preavviso: cluster a 0.67 s → +8.2 cm; 0.22–0.41 s → −5.5 … +3.4 cm.

## 4. Fase 3 — candidati, ordinati per guadagno atteso

Scala oracolo (`--oracle 8 --oracle-lead L`, solo palla, aimed; colpi `bt2`/9, `bt3`/6, clearance media
in cm; media di 3 repliche, MODELLATO):

| preavviso L | 0 (attuale, nessun oracolo) | 0.30 | 0.35 | 0.40 | 0.45 | 0.60 |
|---|---|---|---|---|---|---|
| bt2 colpi | 3.0 | 2.0 | 2.3 | 2.3 | **1.0** | 2.3 |
| bt3 colpi | 2.3 | 3.0 | 2.0 | 1.3 | **1.0** | 3.0 |
| bt2 clearance | +0.7 | +1.8 | +2.0 | +2.7 | +4.4 | +6.6 |
| bt3 clearance | +0.4 | +1.1 | +3.3 | +5.1 | +6.7 | +5.7 |

Ogni +0.05 s di preavviso vale ~+1…+1.5 cm. Il guadagno è ~0 prima di 0.4 s e si ferma (non peggiora) oltre 0.45.

| # | candidato | guadagno atteso (come) | costo | rischio percorso di sicurezza | hardware |
|---|---|---|---|---|---|
| 1 | **Separare la palla dal blob del lanciatore** (es. segmentazione per differenza di profondità/moto del cluster dominante) | +0.15 s (H2): 0.30→0.45; colpi `bt2` 3.0→1.0, `bt3` 2.3→1.0, clearance +0.7→+4.4, +0.4→+6.7 cm (oracolo 0.45 s) | medio: logica in `real_time_distance`/clustering, verificabile offline su 15 passate | **alto**: tocca la percezione che alimenta il CBF; falsi positivi su mani/persone; da fare dietro flag | validazione finale sì |
| 2 | **Lanciare da più lontano** (rilascio ≥ 3.3 m a 3.7 m/s) | preavviso = volo − ~0.15–0.2 s: la passata `bt3`-2 (rilascio 1.03 s prima) ha cluster a 0.81 s e +8.2 cm; equivale a (1) senza codice | nullo | nullo | sì (protocollo §6) |
| 3 | Stream meno a raffiche / meno latenza (48→~13 ms) | ~+0.035 s ≈ +1…+2 cm (oracolo) | medio-alto (driver/DDS, `align_depth` off, `FASTRTPS`) | basso | sì |
| 4 | Predizione dell'intento del lanciatore | oltre 0.45 s l'oracolo non riduce i colpi (`bt2` 2.3/9, `bt3` 3.0/6 a 0.60 s) e dà +2 cm di clearance | alto | alto | sì |
| 5 | Camera in altra posizione | **NON DETERMINABILE** con questi strumenti (nessuna bag di un'altra posa) | medio | basso | sì |

Non servono: ROI più largo, `max_thresh` più alto (nelle bag), tracker/gate (H4–H6), box/slew più larghi
(documento esistente), righe di predizione balistica (già misurate peggiori).

## 5. Limiti di quello che ho misurato

* L'inviluppo è a oggetto singolo, braccio a riposo e fermo, percezione con velocità ideale: **sovrastima**.
* Il braccio nel closed loop è modellato (esagera le escursioni ~1.6–2×, `ball_throw_closed_loop.md` §6).
* Il probe elabora *ogni* frame distinto (il nodo vivo ne scarta ~16 %): risponde alla domanda
  sull'informazione, non sui tempi. Per i tempi valgono le repliche in tempo reale.
* Il rilascio è stimato dalla verità a 30 Hz (±33 ms), in ritardo; le passate `bt2`-4/9 sono anomale.
* H2 usa ROI 700 e raggio 3.0 solo dentro il probe; la configurazione spedita non è stata cambiata.
* Una sola scena, un solo lanciatore (+ chi recupera), pallina rosa: nessuna generalizzazione.

## 6. Protocollo per la prossima sessione sul robot

1. `colcon build --packages-select franka_experiments` (l'install è una copia), stack di coppia, D455 a 848x480x90.
2. Registrare con `start_rosbag` (include `/NS_1/franka/joint_states`). Primo controllo:
   `python3 scripts/smoothness_report.py <bag>` (dice se il braccio si è mosso).
3. Verità di terreno: `ball_throw_eval.py truth` (pallina rosa, `--static-frac` 0.3) e velocità della palla
   dalla verità (`passes()`), **non** dal tracker.
4. Condizioni, **≥ 10 passate ciascuna**, ordine randomizzato, stessa pallina:
   (A) lancio da ~2 m (come ora); (B) da 3 m; (C) da 4 m (limite `max_depth_m` 4.0 della camera); velocità
   mirata 3–4 m/s e una serie lenta 1.5–2 m/s per confermare l'inviluppo; (D) *scena vuota intorno al braccio*
   (nessuno a < 0.7 m dal robot, lancio da dispositivo o da più lontano) per verificare il gate 0.7 m (H5),
   confrontando `max_thresh` 0.7 contro 1.5 in repliche della stessa bag.
5. Per ogni passata riportare: velocità vera, distanza di rilascio, `row_lead`/`fast_lead`/`cbf_lead`
   (`ball_throw_eval.py score`), il cluster proprio (`approach_probe.py depth`), e il gap minimo dal truth.
6. Per decidere se (1) vale: confrontare i `cbf_lead` di (A) e (C); atteso ~+0.4–0.6 s in (C).

## 7. Riproduzione

Strumenti nuovi (solo misura): `scripts/approach_probe.py` (`rates`, `depth`, `assoc`, `causes`),
`scripts/approach_envelope.py`. Script di contorno e dati grezzi in `rosbag/approach_env/` (git-ignorata:
`rep.sh`, `cl.sh`, `clall.sh`, `clall2.sh`, `rel.py`, `envelope.json`, CSV). Comandi principali, nel container:
`approach_probe.py rates rosbag/ball_throws_3` · `approach_probe.py depth rosbag/ball_throws_3
rosbag/ball_throws_3_truth.npz [--perception-overrides y.yaml]` · `approach_probe.py assoc --fps 90` ·
`approach_envelope.py --json out.json` · `ball_closed_loop.py --bag rosbag/ball_throws_3 --dist
rosbag/approach_env/replay_bt3_r1 --truth rosbag/ball_throws_3_truth.npz --threat 0 --ball-only [--oracle 8
--oracle-lead 0.45]` · `ball_throw_eval.py score rosbag/approach_env/replay_bt3_r1 --truth …`.
