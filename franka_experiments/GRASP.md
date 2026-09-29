# Primo modulo H2R: evidenza di oggetto vicino alla mano ACTIVE

`ros2 run franka_experiments grasp` avvia un nodo diagnostico separato.
Non viene avviato da `robot_handover.launch.py` e non pubblica comandi.
Il launch di registrazione include `/handover/hand_object` se il nodo è attivo.
Il compare visualizer mostra lo stato sotto le coordinate del palmo e un
marcatore verde sul centroide confermato. Dati scaduti o di un'altra mano
non producono marcatori. L'overlay non attende questo topic per mostrare le immagini.

Input: RGB e aligned depth della camera esterna, CameraInfo,
HandTrackingRaw (solo 0, 5, 9, 17) e HandState. Raw e stato devono avere lo
stesso timestamp e la stessa mano fisica. Usa la calibrazione del tracker;
non esegue MediaPipe né modifica selezione, Kalman, W75 o geometry.

La ROI 3D è centrata sul palmo (raggio 14 cm). Il colore viene campionato
sul palmo osservato, senza soglie fisse sul colore della pelle. Una maschera
conservativa della mano, un margine RGB-D di 15 mm e i salti di profondità
escludono parte della mano e gli artefatti al suo bordo. I cluster residui
devono essere compatti, vicini alla mano e non tagliati dalla ROI.
Più cluster plausibili rendono l'osservazione ambigua.

La conferma richiede almeno tre osservazioni e 0,3 s di continuità rispetto
al palmo. L'uscita dalla presenza ha isteresi di 0,15 s. Cambio mano, dati
non osservabili, inversione temporale e gap oltre 0,2 s azzerano la storia.
Predizioni, misure stimate e input scaduti non creano conferme. Un watchdog
invalida l'output se gli input si arrestano (timeout predefinito 0,25 s).

`HandObjectState.valid=false` significa non osservabile, non mano vuota.
`object_present` è evidenza temporale di un cluster associato alla mano,
non prova di grasp, intenzione di handover o autorizzazione al movimento.
Durante l'isteresi `object_age` indica l'età del centroide mantenuto.
Il centroide rappresenta la superficie visibile; non è una posa di presa.
La confidence è euristica, non una probabilità calibrata.

Con quattro punti non si segmentano esattamente dita e pollice: oggetti
piccoli, dello stesso colore della mano, aderenti al palmo, trasparenti o
coperti dalla mano possono non essere rilevati. Tavolo, robot, altra mano
e abbigliamento vicini restano possibili fonti di falsi positivi.

Scelta architetturale ispirata a:
- [Yang et al., ICRA 2021](https://arxiv.org/abs/2011.08961): ROI attorno al
  palmo, segmentazione della mano e separazione RGB-D prima delle prese.
- [Rosenberger et al., RA-L 2020](https://arxiv.org/abs/2006.01797): maschere
  della mano e margini di esclusione separati dalla generazione delle prese.

Questi lavori usano segmentatori appresi: la maschera geometrica/colore qui
è una baseline leggera, non una loro riproduzione né una soluzione con le
stesse prestazioni. Per ora niente YOLO, AnyGrasp o nuovi pesi da scaricare.

Verifica offline iniziale: tre bag del 29/09/2026, 7.246 HandState analizzati,
zero conferme oggetto nella versione finale; controllo visivo dei falsi
positivi delle versioni intermedie. Non è una misura di recall: mancano
sequenze annotate con oggetti reali presentati alla mano. Prima di usare
l'output per decisioni servono prove con mano vuota, oggetti diversi,
mano vicino al tavolo/robot, cambio mano e occlusione.
