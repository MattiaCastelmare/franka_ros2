# RL training handoff (state on 2026-10-09)

Read this first if you are continuing the franka_sim RL work on another machine
(human or Claude). It records where the models stand, how to get them, how to
start new runs from the last checkpoints and what has already been tried and rejected.
General usage of the module is in `README.md`, architecture in
`../franka_sim_to_real_implementation_status.md`.

---

## 1. Setup on a new PC

Trained models are **not in git** (`models/`, `*.zip`, `*.onnx` are ignored; the
full folder is ~20 GB). A curated pack travels separately:

| | |
|---|---|
| File | `rl_starter_pack_2026-10-09.tar.gz` (762 MB, 772 files) |
| Where | Google Drive of the repo owner |
| Contents | best ensembles, their 11 member checkpoints, resume parents + replay buffers, configs, reference eval traces, `models/RL_PACK_SHA256SUMS` |

```bash
git clone git@github.com:MattiaCastelmare/franka_ros2.git && cd franka_ros2
git checkout humble-mattia
# put the tarball in the repo's franka_sim/ folder, then:
cd franka_sim && tar xzf rl_starter_pack_2026-10-09.tar.gz && sha256sum -c models/RL_PACK_SHA256SUMS --quiet && cd ..
# container (training stack: torch 2.13, SB3 2.9, mujoco 3.4, gymnasium 0.29, onnx — pinned in ../Dockerfile)
USER_UID=$(id -u) USER_GID=$(id -g) docker compose up -d --build
```

Everything runs **inside the `franka_ros2` container**; the repo is mounted at
`/ros2_ws/src`. The scripts in `runs/eval_all/` run on the host and `docker exec`
into the container by that name. The compose file caps the container at **20 GiB**.

Sanity checks after setup:
```bash
docker exec franka_ros2 bash -lc 'cd /ros2_ws/src && python3 -m franka_sim.scripts.validate_actuation'
docker exec franka_ros2 bash -lc 'cd /ros2_ws/src && OMP_NUM_THREADS=1 python3 -m pytest franka_sim/tests -q'
```

---

## 2. Current best models

Metric: **held** = target held at the end of a 5 s episode; **coll** = collision.
"Shield" = the obstacle rows of the CBF safety filter (`env.cbf_obstacle_enabled`).
Seed sets: **held-out** = `3000:100 + 4000:300` (400 eps), **fresh** = `5000:400`.
Each seed set is run in a moving-obstacle (dyn) and a static-obstacle scenario.

| Model | File | Fresh, shield OFF (dyn / static held, coll) | Fresh, shield ON (dyn / static held) |
|---|---|---|---|
| **ens_off_big11** (BEST) | `models/sac_c2_ens_off/ens_off_big11.onnx` | 394 / 391, 5 coll total | 395 / 353 |
| ens_off_mix6 | `models/sac_c1_ens_off/ens_off_mix6.onnx` | 392 / 383, 7 coll | 393 / 353 |
| ens_soup9 (previous deploy) | `models/sac_v11_ens9/ens_soup9.onnx` | 386 / 321, 72 coll | 395 / 361 |
| ft_prec 3.75M (best single, shield-trained) | `models/sac_v11_ft_prec/checkpoints/sac_3750000_steps.zip` | — | 342 / 300 |

Full table with 95 % CIs: `cd runs/eval_all && python3 c2/final_table.py`
(the needed traces are in the pack) → `runs/eval_all/c2/final_table.txt`.

**ens_off_big11** = mean action of 11 shield-off fine-tuned SAC policies (24-D obs).
Member list is in `models/sac_c2_ens_off/config.yaml` and `runs/eval_all/c2/list_ens.txt`:
`sac_b3_ft_v10c` 2.75/3.25/4.0M, `sac_b4_clear_mix` 4.0M, `sac_b4_clear` 4.0M,
`sac_b4_clear_filt` 3.5M, `sac_c2_off_ps3` 4.25M, `sac_c2_off_ps2` 4.5M,
`sac_c2_off_s4` 3.25M, `sac_c2_off_s3` 3.5M, `sac_c2_off_s2` 4.0M.

Open problems (where better results can come from):
- Static case with the shield ON: shield-off policies lose ~30 static held when the shield
  is turned back on (they approach below d_safe and the shield then blocks them).
  big11 ON static 353 vs soup9 361 (n.s.).
- ~65 % of static episodes still dip below d_safe = 0.15 m with the shield off.
- Single policies lose mostly by **reached-not-held** (drift at the target, per-network
  approximation bias); ensembling of different seeds fixes most of it.

---

## 3. How the good models were made (recipes)

The chain: `sac_v10` (from scratch, 2M) → `sac_v10c` (continuation to 4M; **2.5M ckpt is the parent of everything**)
→ shield-ON fine-tunes (`v11_ft_prec`, `v12_prec_s2/s3`) and shield-OFF fine-tunes (`b3_ft_v10c`, `c2_off_*`).

All fine-tunes resume a checkpoint **plus its replay buffer** with relabelling:

```bash
# shield-OFF member recipe (b3_ft_v10c / c2_off_s*): parent v10c 2.5M, +1.5M steps
docker exec franka_ros2 bash -lc "cd /ros2_ws/src && export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 && \
  python3 -u -m franka_sim.train --config franka_sim/config_c2_off_s2.yaml --exp-name sac_NEWNAME \
    --resume franka_sim/models/sac_v10c/checkpoints/sac_2500000_steps.zip \
    --resume-buffer franka_sim/models/sac_v10c/checkpoints/replay_buffer_latest.pkl --relabel-buffer \
    --total-timesteps 1500000 --checkpoint-every-episodes 400 --save-replay-buffer --no-episode-onnx" \
  > franka_sim/runs/sac_NEWNAME_train.log 2>&1
```

- Different parent (parent diversity for the ensemble), e.g. `c2_off_ps3`:
  `--resume .../sac_v12_prec_s3/checkpoints/sac_3750000_steps.zip --resume-buffer .../sac_v12_prec_s3/checkpoints/replay_buffer_latest.pkl
  --relabel-buffer --relabel-from franka_sim/models/sac_v12_prec_s3/config.yaml --total-timesteps 1250000`.
- Shield-ON recipe (ft_prec / prec_s2 / prec_s3): `config_v11_ft_prec.yaml` (or `config_v12_prec_s*.yaml`),
  same v10c parent + buffer, `--total-timesteps 1250000` (v11 used 1.5M).
- **Change `rl.seed` in a copied config for each new run.** Configs are generated by
  `runs/eval_all/<batch>/mk_*.py`; batch launchers are `runs/eval_all/c2/launch_c2.sh` (setsid + systemd-inhibit, detached).
- Checkpoints land every 50k steps in `models/sac_NEWNAME/checkpoints/sac_<N>_steps.zip`.
- Do not trust `best_model.zip` (EvalCallback on 10 eps); select checkpoints with the 400-seed eval below.

Parents in the pack: `sac_v10c` 2.5M + buffer, `sac_v12_prec_s2` / `sac_v12_prec_s3` 3.75M + buffers,
`sac_b3_ft_v10c` buffer (end of run), `sac_v11_ft_prec` 3.75M (no buffer: v11 did not save one;
resuming without a buffer wrecked the policy in v12 — avoid).

---

## 4. Evaluate and build an ensemble

```bash
cd franka_sim
# list lines: TAG RUN CKPT CONFIG   (RUN=ens → CKPT = comma list run/checkpoints/ckpt, no .zip)
echo "my_4000k sac_NEWNAME checkpoints/sac_4000000_steps franka_sim/models/sac_NEWNAME/config.yaml" > /tmp/list.txt
runs/eval_all/c1/eval.sh my_eval 3 < /tmp/list.txt                 # held-out seeds, shield OFF and ON → runs/eval_all/traces/my_eval/
CHUNKS="$(for s in $(seq 5000 25 5375); do printf "%s:25 " $s; done)" runs/eval_all/c1/eval.sh my_conf 3 < /tmp/list.txt   # fresh seeds 5000:400
cd runs/eval_all && python3 c1/summary.py traces/my_eval            # held / rnh / never-reached / coll, McNemar vs references
python3 c1/pair.py 'traces/my_eval/my_4000k_off_{sc}_*.json' 'traces/c2_ens/ens_big11_off_{sc}_*.json'   # paired test
```

Export an ensemble to ONNX (what `rl_policy_commander` runs on the robot):

```bash
docker exec franka_ros2 bash -lc 'cd /ros2_ws/src && PYTHONPATH=/ros2_ws/src python3 -m franka_sim.export_ensemble_onnx \
  --models franka_sim/models/A/checkpoints/sac_4000000_steps.zip franka_sim/models/B/checkpoints/... \
  --out franka_sim/models/sac_NEW_ens/ens_NAME.onnx'
```
Members must share obs layout / deploy sections (24-D and 27-D members cannot be mixed). Always run the zero-action baseline (`--model zero`) next to a new policy.

Eval rules learned the hard way:
- Evaluate with `env.cbf_obstacle_on_prob=null` (eval.sh does it), otherwise the training-time shield mix overrides the flag.
- Compare on the **same seeds** with McNemar; confirm any winner on the fresh set (winner's curse is real: the
  best of many checkpoints on one set regresses ~20-40 held on new seeds).
- Training is bit-deterministic **on the same machine**: same config + seed reproduces the run exactly.
  On a different GPU/CPU expect different numbers — re-run a reference (e.g. big11) on the new PC before comparing.

---

## 5. Already tried — do not repeat without a new idea

| Lever | Result |
|---|---|
| obs 24 → 51-D (v_obs, per-CP distances) | fails to learn (v10_obs51, b3_obs51) |
| γ 0.997 | fails (v10_g997) |
| Training from scratch with shield OFF (b3, 9 runs) | 0 success: exploration failure; the shield is what lets SAC discover reaching |
| Clearance penalties, shield on/off mixing, buffer mask, v_obs widening (b4) | none beats ft_v10c; costs reaching, does not cut collisions |
| Lower SAC target entropy −14/−21/−28 (c1) | worse; reached-not-held rises |
| lr 3e-5, batch 2048 (c2) | worse than control |
| static_fraction 0.75, potential obstacle shaping, UTD 2 (v11) | below baseline |
| Weight averaging **across** runs | fails (works only within one run) |
| Resume without replay buffer (`--policy-warmup`) | wrecks the policy for ~0.75M steps |

**Seed variance dominates**: replicas of the same recipe differ by up to ~80 held/400
(ft_v10c = lucky seed 39; its replicas s2/s3 are much worse). Any A/B needs **≥ 2-3 seeds per arm**
and means over several checkpoints (e.g. 3.0-3.75M).

Untried ideas worth a look: more diverse shield-off members (new parents / seeds) for the ensemble;
a fine-tune that keeps the shield ON for the static case while staying shield-off-competent; distilling
big11 into a single network; fixing the J̇q̇ = 0 sim quirk (`_build_obstacles` runs twice on the same state —
would change the MDP of all policies, so retrain/re-eval everything if fixed).

---

## 6. Machine gotchas

- `OMP_NUM_THREADS=1` for every training/eval process (torch's default threads make a batch-1 MLP ~10x slower).
- GPU throughput saturates around ~320 fps total; on the RTX 4070 laptop ≤ 10 parallel runs
  (~1.4 GiB each + 0.23 GB replay buffer for 24-D), ≈ 17 h per 2M steps at 10 runs. Leave ≤ 3 eval jobs while training.
  On the laptop: `powerprofilesctl set performance` (power-saver halves fps).
- OOM inside the 20 GiB container kills runs silently (exit 137) — check `docker stats` before adding jobs.
- MuJoCo rendering in the container uses the Intel iGPU unless you point EGL at NVIDIA:
  `echo '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}' > /tmp/10_nvidia.json` and
  `MUJOCO_GL=egl __EGL_VENDOR_LIBRARY_FILENAMES=/tmp/10_nvidia.json`.
- Python pins: `qpsolvers==4.3.3` + `osqp<1.0`, `protobuf<5` (already in the Dockerfile; floating them breaks the CBF QP).
- `train.log` can lag (stdout buffering) — `python3 -u` (the recipes above use it) or read `evaluations.npz` / tfevents in `runs/`.
