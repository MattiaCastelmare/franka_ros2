# PAPER_IMPLEMENTATION_ACTIONS — priority table

Companion to `PAPER_IMPLEMENTATION_AUDIT.md`. Actions ordered by priority.
Effort: **S** ≤1 day · **M** 2–5 days · **L** 1–2 weeks · **XL** >2 weeks.
Priority: **P0** now · **P1** important · **P2** useful-not-urgent.

| # | Task | Why (scientific motivation) | Files involved | Prio | Effort | Expected impact |
|---|---|---|---|---|---|---|
| 1 | **Log safety, not just tracking**: publish/record `min h(t)`, `h<0` events, slack `∫s²`, min obstacle distance, intervention rate | Today evaluation is tracking RMS only; the *safety* claim is unmeasured → auto-reject | `nodes/experiment_logger.py`, `franka_logs/plot_franka_log.py`, subscribe `/NS_1/cbf_status`, `/cbf/per_link_distances` | **P0** | M | Enables every safety claim |
| 2 | **Freeze a benchmark scenario** (scripted moving obstacle; reuse `datasets/bag_*`) + seed obstacle trajectories | No controlled, repeatable test set exists | `scripts/run_benchmark.py` (new), `datasets/`, `test/launch/` | **P0** | M | Repeatability, statistics |
| 3 | **Add baselines**: unsafe-ref, APF/Flacco repulsion, CBF-only (no avoidance-first) | A CBF-avoidance paper needs comparators; parts already exist (`bypass`, `avoid_enable`) | commander params, `cbf_velocity_filter bypass_cbf`, new APF node | **P0** | M | Isolates the contribution |
| 4 | **Ablate the avoidance-first layer**: toggle tangential-redirect / null-repulsion / governor independently | The novelty lives here; each piece must be shown to matter | `nodes/pentagon_qddot_commander.py` (805–958), `utils/avoidance.py` | **P0** | S–M | Defends novelty |
| 5 | **Multi-seed runs + mean±std / CI** in `report.md` | No statistics today; reviewers require variance | `scripts/aggregate.py` (new), `plot_franka_log.py` | **P0** | S | Statistical credibility |
| 6 | **Decide ISO/TS 15066**: implement `[d,v_rel]` PFL barrier **or** delete the framing | README/refs cite Ferraguti'20 but code has no `v_rel`/PFL/energy → false alignment | `nodes/cbf_safety_filter.py` (new barrier row), `nodes/real_time_distance.py` (v_rel), README | **P1** | M (impl) / S (drop) | Fixes a false claim |
| 7 | **Fix OSCBF obstacle path**: use filtered distance/direction (kill phantom-origin), enable obstacle/ws/vel CBFs in a tested config | Current path re-introduces the raw-`closest_point_human` base-origin bug; OSCBF is joint-limits-only by default | `nodes/cbf_OSCBF_filter.py` (462–526), `config/oscbf_params.yaml` | **P1** | S–M | OSCBF becomes a real reproduction |
| 8 | **Wire self-collision into the live QP** (or move it out of `utils/`) | `self_collision.py` is tested but unused → reviewer confusion + missing OSCBF Eq.51 | `utils/self_collision.py`, `nodes/cbf_safety_filter.py` / `cbf_OSCBF_filter.py` | **P1** | M | Honest whole-body safety, OSCBF 3→4 |
| 9 | **Add singularity CBF** `h=√det(JJᵀ)−ε` | OSCBF Eq.43; commander already computes `w=√det(JJᵀ)` — near-free | `nodes/cbf_safety_filter.py` / `cbf_OSCBF_filter.py` | **P1** | S | Prevents singular lock-up |
| 10 | **Quarantine dead code** (`cbf_qp.py`, `cbf_constraints.py`) — move to `legacy/` or delete | Three parallel CBF implementations mislead readers about "the method" | `utils/cbf_qp.py`, `utils/cbf_constraints.py` | **P1** | S | Repo legibility |
| 11 | **Remove "TEMPORARY" isolation-test branch** from the live commander hot path | `pentagon_qddot_commander.py:801` `if self.isolation_test:` in production node | `nodes/pentagon_qddot_commander.py` | **P1** | S | Cleanliness, no accidental activation |
| 12 | **Pin dependencies + capture git SHA/config per run** | README `pip install` unversioned; runs not reproducible | `setup.py`/`requirements.txt` (new), `experiment_logger.py` | **P1** | S | Reproducibility |
| 13 | **CI**: run unit tests + a headless `use_fake_hardware` integration test asserting `h≥0` | Tests exist but aren't gated; no invariant test | `.github/`, `test/` | **P1** | M | Regression safety |
| 14 | **Dynamic-obstacle velocity inflation** `h−γ‖v_rel‖`, γ=0.25 | OSCBF Eq.52; moving-human safety at speed | `nodes/cbf_safety_filter.py`, `nodes/real_time_distance.py` | **P2** | M | Dynamic-obstacle safety |
| 15 | **Delay-compensated barrier** (RAM'22 Eq.17–18) for the ~30 Hz camera | Latency undermines the guarantee; `predict_state` (dead) is a starting point | `utils/cbf_constraints.py` (revive), `cbf_safety_filter.py` | **P2** | M | Safety-at-speed |
| 16 | **Guarantee-gap study**: sweep `γ`,`k0/k1`,`ρ`,latency → plot `min h` | Honest characterisation disarms the "not formally safe" objection | `scripts/`, `plot_franka_log.py` | **P2** | M | Publishability |
| 17 | **Correct README claims** (distance ≠ Flacco depth-space; OSCBF constraint status) | Rule 4: code is truth; README currently overstates | `README.md` | **P2** | S | Integrity |
| 18 | **Sim↔real gap** report using `use_fake_hardware` | No sim-vs-real evidence today | launch, `experiment_logger.py` | **P2** | M | Reviewer confidence |

**Sequencing:** P0 (1–5) unblocks everything → do first. Then P1 (6–13) to make the
method honest and whole. P2 (14–18) strengthens for submission. Items 7–10 are cheap
and high-legibility — batch them in one cleanup pass.
