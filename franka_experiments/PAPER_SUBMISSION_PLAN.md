# PAPER_SUBMISSION_PLAN — 8–12 week roadmap

Companion to `PAPER_IMPLEMENTATION_AUDIT.md` and `PAPER_IMPLEMENTATION_ACTIONS.md`.
This is the executive memo: what to build, in what order, to turn
`franka_experiments` into a credible submission.

---

## 1. Positioning

**Working title.**
*"Avoidance-First Manipulation: Task-Consistent Reactive Steering with
Control-Barrier-Function Certificates for Human-Robot Coexistence on the Franka
FR3."*

**The one-sentence thesis.** Instead of letting a CBF filter both *avoid* and
*stop* the robot (the standard design in Morton & Pavone 2025, Ferraguti 2020/2022,
Landi 2019), we generate a **feasible, task-consistent avoidance direction in the
commander** and use the CBF only as a **certificate**, modulating speed from
**feasibility evidence** (QP slack, safety-chain fault, manipulability) — never from
raw distance — so the arm steers around obstacles at speed and slows only as a last
resort.

**Why this is the right angle (not the alternatives).**
- *"We reproduce OSCBF"* — not defensible: constraints are reduced and off by
  default; the cost is exact but that alone is a re-implementation, not a
  contribution.
- *"ISO/TS 15066 CBF"* — not defensible without new code: there is no `v_rel`/energy
  term anywhere.
- *"Avoidance-first + certificate + feasibility governor"* — **defensible**: the
  code already implements it (`utils/avoidance.py`, commander 805–958); it targets a
  real, cited weakness of CBF filters (boundary-parking); the certificate reuses the
  exact OSCBF task-consistent cost, so the safety machinery is on solid ground.

**Target venues.** ICRA / IROS (system + experiments) primary; RA-L (with IROS
option) if the feasibility-governor stability argument is tightened.

---

## 2. Milestones

### Weeks 1–2 — Instrumentation & benchmark (P0, ACTIONS 1–2, 5, 12)
- Add safety signals to `experiment_logger.py` + `plot_franka_log.py`: `min h(t)`,
  `h<0` count/duration, slack energy, min obstacle distance, intervention rate,
  boundary-parking time (time within ε of `h=0`).
- Define the **canonical scenario**: fixed FR3, a scripted moving obstacle (extend
  `datasets/bag_02_moving_robot`), 3 obstacle regimes (static / slow / fast), N≥10
  seeded obstacle trajectories.
- `scripts/run_benchmark.py` (launch + record + plot, one command) and
  `scripts/aggregate.py` (mean±std/CI). Pin deps; store git SHA + YAML per run.
- **Exit criterion:** one command produces a `report.md` with safety + tracking
  numbers for one condition.

### Weeks 3–4 — Baselines & method cleanup (P0/P1, ACTIONS 3, 7, 10–11)
- Implement/enable baselines: unsafe-reference, APF/Flacco repulsion (new small
  node), CBF-only (`avoid_enable:=false`).
- Cleanup that a reviewer will see: quarantine `cbf_qp.py`/`cbf_constraints.py`;
  remove the `isolation_test` branch from the live commander; fix the OSCBF obstacle
  path (filtered point) so the certificate variant is bug-free.
- **Exit criterion:** all four control conditions run on the benchmark unattended.

### Weeks 5–6 — Ablations & the governor argument (P0/P1, ACTIONS 4, 9)
- Ablate: −tangential-redirect, −null-repulsion, −feasibility-governor, and
  **distance-governed vs feasibility-governed** speed (the key comparison).
- Add the singularity CBF (near-free) so avoidance never drives into a singularity.
- Write the feasibility-governor description + an informal stability/auto-resume
  argument (rate-limited β, C⁰ virtual time).
- **Exit criterion:** ablation table populated; the governor's benefit is visible in
  throughput-at-equal-safety.

### Weeks 7–8 — Whole-body honesty + ISO/TS 15066 decision (P1, ACTIONS 6, 8)
- **Wire `self_collision.py`** into the live QP (OSCBF Eq.51) → the "whole-body
  safety" claim becomes true.
- **Decide ISO/TS 15066**: either implement the single-parabola `h(d,ḋ)` /
  `[d,v_rel]` PFL barrier (RAM'22 Eq.17 → Ferraguti'20) *or* delete the framing from
  README/paper. Recommendation: implement the `h(d,ḋ)` version — it is scoped and
  gives a certifiable-safety hook.
- **Exit criterion:** every claim in the draft maps to code that runs in the
  benchmark.

### Weeks 9–10 — Real-robot runs, figures, first full draft
- Full benchmark on **real FR3** + RealSense (and matching `use_fake_hardware` sim
  for the sim↔real gap, ACTION 18).
- Generate all figures/tables from `report.md` outputs; write Methods + Experiments;
  record the demonstration video.
- **Exit criterion:** complete internal draft with real numbers.

### Weeks 11–12 — Guarantee-gap study, polish, submission (P2, ACTIONS 13, 16)
- Guarantee-gap sweep (`γ`,`k0/k1`,`ρ`,latency vs `min h`) — the honesty section.
- CI (unit + headless `h≥0` integration test). Related-work search to substantiate
  the novelty claim (currently **[IP]**). Internal review, revise, submit.
- **Buffer:** absorb hardware slippage here (real-robot access is the top schedule
  risk).

---

## 3. Experiments matrix (the core table)

Conditions × obstacle regimes × N≥10 seeds; report mean±std.

| Metric | unsafe-ref | APF/Flacco | CBF-only | **CBF + avoidance-first (ours)** |
|---|---|---|---|---|
| min `h(t)` (safety) | (violates) | ? | ≥0 | **≥0** |
| `h<0` events | many | ? | ~0 | **0** |
| min obstacle distance [m] | — | — | — | — |
| Task-completion time [s] (throughput) | best | — | worst (parks) | **near-unsafe** |
| Boundary-parking time [s] | 0 | — | high | **~0** |
| Hard CBF intervention rate | — | — | high | **low** |
| EE tracking RMS [m] | best | — | — | — |
| Auto-resume after clear | — | — | — | **yes** |

**Ablations (ours, minus one component):** −redirect · −null-repulsion · −governor ·
distance-governed(β from distance) vs feasibility-governed(β from slack/fault/w).

---

## 4. Risks & mitigations

| Risk | Likelihood | Mitigation |
|---|---|---|
| Real-robot time slips | High | Do sim benchmark first; real runs only confirm; buffer in wk 11–12 |
| Reviewer: "novelty vs OSCBF?" | High | The certificate *is* OSCBF; the **architecture + governor** is the claim; ablate distance- vs feasibility-governed |
| Reviewer: "not formally safe" (slack/latency) | Med | Don't overclaim; include the guarantee-gap study (ACTION 16) as an honest section |
| Governor causes oscillation/stall | Med | Asymmetric rate-limited β already implemented; add a stability argument + auto-resume traces |
| APF baseline unfairly weak | Med | Tune APF to its best; report its local-minima failures as *known*, not as a strawman |
| Single-lab / single-robot | Med | Release code + rosbags + seeds; add sim↔real gap; frame as a *system* paper |
| ISO/TS 15066 half-implemented | Low | Binary decision in wk 7–8: implement `h(d,ḋ)` **or** remove the framing entirely |

---

## 5. Definition of done (submission-ready)

- [ ] Safety metrics logged, plotted, and in `report.md`.
- [ ] Benchmark reproducible with one command; deps pinned; git SHA + config stored.
- [ ] ≥3 baselines + ≥4 ablations, N≥10 seeds, mean±std.
- [ ] Every paper claim maps to code that runs in the benchmark (no dead-code
      claims; no ISO/TS 15066 claim unless the barrier exists).
- [ ] Real FR3 + sim results; sim↔real gap reported.
- [ ] Guarantee-gap section (honest CBF-vs-practice).
- [ ] Dead code quarantined; `isolation_test` removed; self-collision wired or moved.
- [ ] CI green (unit + headless `h≥0`).
- [ ] Related-work search done; novelty **[IP]→[EM]**.
