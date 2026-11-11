#!/usr/bin/env python3
"""Generate appendix B (`B_equation_index.md`) from the numbered equations of chapters 04–13. Host.

Prints Markdown on stdout; writes nothing itself. Regenerate with
    python3 -I -B docs/framework_study/tools/stage7_verification/gen_equation_index.py \
        > docs/framework_study/B_equation_index.md

For every equation tag `(C.N)` / `(C.Na)` of chapter C (C = 4 … 13):
- **defined at**: the first line of chapter C that starts with the tag (after indentation, `**`,
  `| ` or `- `), or else the first line that contains it;
- **statement**: the text after the tag on that line (truncated, Markdown pipes escaped);
- **code**: the verified pointers of `MANUAL` (below) first, then backticked tokens that look like code (`file.py`, `Class.method`, `function`,
  `name::name`, `L123`) on every line of the chapter that mentions the tag, in order of first
  appearance, at most five. The chapter text is the evidence; this index only points to it.
- **cited by**: other chapters and chapter 16 lines that mention the tag.

A tag without any code token is listed in the "equations without a code pointer" section: either a
pure-mathematics statement (a derivation step, an assumption) or a gap. `check_docs.py` counts them.
"""
import os
import re
import sys
from collections import OrderedDict

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
DOCS = os.path.abspath(os.path.join(HERE, '..', '..'))
CH = ['04_math_foundations.md', '05_perception_and_distance.md', '06_nominal_motion.md',
      '07_cbf_obstacle_barrier.md', '08_constraints_and_qp.md', '09_torque_realization.md',
      '10_alternative_controllers.md', '11_iso_safety_layer.md', '12_simulation.md',
      '13_observability_and_tools.md']
OTHER = ['01_architecture.md', '02_ros_graph_and_dataflow.md', '03_configuration.md',
         '14_build_launch_test.md', '15_code_status.md', '16_issues_and_discrepancies.md',
         'SIMULATION_VALIDATION_PROTOCOL.md']
CODE = re.compile(r'`([^`\n]{2,80})`')
LOOKS_CODE = re.compile(r'^(?:[A-Za-z_][\w./:]*[\w)]|[A-Za-z_]\w*\(\)?)(?:\s+L\d+(?:[–-]\d+)?)?$')


# Pointers for equations whose chapter gives the code in the section's code map rather than next to
# the equation. Every symbol was checked to exist at the baseline (`git grep` at ba68077, C-047);
# the chapter section named in the equation's row is the evidence that it implements the equation.
MANUAL = {
    '5.7': ['obstacle_tracker.KalmanTrack'], '5.9': ['obstacle_tracker.IMMTrack'],
    '5.10': ['obstacle_tracker.IMMTrack'],
    '5.12': ['ballistic_prediction.predict_trajectory', 'ballistic_prediction.add_predicted_hits'],
    '5.14': ['handeye_solver.initial_guess', 'handeye_solver.refine'],
    '6.1': ['pentagon_qddot_commander._tick', '_step_nominal'],
    '6.4': ['pentagon_qddot_commander._step_nominal'], '6.6': ['pentagon_qddot_commander._step_nominal'],
    '6.7': ['pentagon_qddot_commander._step_nominal'], '6.8': ['pentagon_qddot_commander._step_nominal'],
    '6.9': ['pentagon_qddot_commander._step_nominal'], '6.10': ['pentagon_qddot_commander._step_nominal'],
    '7.1': ['distance_engine.DistanceEngine.compute'],
    '7.3': ['cbf_state_rows.ConstraintBuilder.build'],
    '7.12': ['cbf_state_rows.ConstraintBuilder.build', 'cbf_state_rows.evidence_margin'],
    '8.3': ['cbf_qp_assembly.build_qp_objective'],
    '8.10': ['self_collision.segment_segment_closest', 'cbf_state_rows.SelfCollisionRowBuilder.build'],
    '8.11': ['cbf_singularity.SingularityRowBuilder'],
    '9.2': ['qddot_to_torque._compute_tau'], '9.3': ['qddot_to_torque._compute_tau'],
    '9.5': ['rt_torque_controller.cpp::RtTorqueController::update'],
    '10.1': ['cbf_velocity_filter._control_loop', 'cbf_OSCBF_filter._build_obstacle_cbf'],
    '10.3': ['cbf_velocity_filter._control_loop'], '10.5': ['cbf_velocity_filter._control_loop'],
    '10.11': ['cbf_OSCBF_filter._build_obstacle_cbf'], '10.12': ['cbf_OSCBF_filter._build_obstacle_cbf'],
    '11.1': ['iso_ssm.protective_separation'], '11.2': ['iso_ssm.protective_separation',
                                                       'iso_ssm.stopping_distance'],
    '11.5': ['iso_ssm.protective_separation'],
    '12.3': ['simexp.obstacle_laws.position', 'simexp.obstacle_laws.velocity'],
    '12.4': ['simexp.obstacle_laws.position', 'simexp.obstacle_laws.velocity'],
    '12.5': ['simexp.obstacle_laws.position', 'simexp.obstacle_laws.velocity'],
    '12.6': ['simexp.obstacle_laws._wander'], '12.8': ['simexp.obstacle_laws.pursue'],
    '12.7': ['franka_simulation scenario_obstacles_system.cpp PreUpdate'],
    '12.16': ['franka_simulation gz_robot_description._add_armature'],
}


def read(fn):
    return open(os.path.join(DOCS, fn), encoding='utf-8').read().splitlines()


def esc(t):
    return t.replace('|', '\\|').strip()


def main():
    rows, docs = [], {fn: read(fn) for fn in CH + OTHER if os.path.exists(os.path.join(DOCS, fn))}
    for fn in CH:
        c = int(fn[:2])
        lines = docs[fn]
        tag_re = re.compile(r'\((%d\.\d+[a-z]?)\)' % c)
        tags = OrderedDict()
        for i, ln in enumerate(lines):
            for m in tag_re.finditer(ln):
                tags.setdefault(m.group(1), []).append(i)
        for tag, idx in tags.items():
            start = re.compile(r'^\s*(?:\*\*|\| |- )?\(%s\)' % re.escape(tag))
            defs = [i for i in idx if start.match(lines[i])]
            end = re.compile(r'\(%s\)\s*`?\s*$' % re.escape(tag))
            ends = [i for i in idx if end.search(lines[i])]
            if defs:
                d = defs[0]
                stmt = lines[d].split(f'({tag})', 1)[1]
            elif ends:                       # chapters 05, 10, 12 put the tag at the end of the line
                d = ends[0]
                stmt = lines[d].rsplit(f'({tag})', 1)[0]
                stmt = re.sub(r'\(\d+\.\d+[a-z]?\)\s*$', '', stmt.rstrip())  # a second tag before it
            else:
                d = idx[0]
                stmt = lines[d]
            stmt = re.sub(r'^\W*\*\*\s*', '', stmt).strip(' *|`')
            k = d + 1
            while not stmt and k < len(lines) and k <= d + 3:
                stmt = lines[k].strip(' *|$`')
                k += 1
            if len(stmt) > 110:
                stmt = stmt[:107] + '…'
            code = []
            for tok in MANUAL.get(tag, []):
                code.append(tok)
            for i in idx:
                for j in range(max(i - 1, 0), min(i + 3, len(lines))):
                    for tok in CODE.findall(lines[j]):
                        if (LOOKS_CODE.match(tok) and len(tok) > 2 and tok not in code
                                and not re.fullmatch(r'[\d.eE+-]+', tok)):
                            code.append(tok)
            cited = []
            for ofn, olines in docs.items():
                if ofn == fn:
                    continue
                if any(f'({tag})' in ol for ol in olines):
                    cited.append(ofn[:2] if ofn[0].isdigit() else 'protocol')
            rows.append((fn, tag, d + 1, stmt, code[:5], cited))

    p = print
    p('# Appendix B — Equation index')
    p()
    p('> **Generated** (Stage 7) by `tools/stage7_verification/gen_equation_index.py` from the numbered '
      'equations of chapters 04–13. Do not edit by hand: change the chapter and regenerate. Baseline '
      '`ba68077`. The chapter is the evidence (its labels, its `file::function` citations, its '
      'verification scripts); this index only points to it. "Code" lists the code tokens the chapter '
      'writes next to the equation, which is where to look first, not a complete call graph.')
    p()
    n_code = sum(1 for r in rows if r[4])
    p(f'{len(rows)} numbered equations in 10 chapters; {n_code} carry at least one code pointer in '
      f'their chapter; {len(rows) - n_code} do not (listed at the end).')
    p()
    cur = None
    for fn, tag, line, stmt, code, cited in rows:
        if fn != cur:
            cur = fn
            p(f'## [{fn}]({fn})')
            p()
            p('| Eq. | Line | Statement (from the chapter) | Code (chapter pointers) | Also cited in |')
            p('|---|---:|---|---|---|')
        p(f'| ({tag}) | {line} | {esc(stmt) or "—"} | '
          f'{", ".join("`" + c.replace("|", "/") + "`" for c in code) or "—"} | '
          f'{", ".join(sorted(set(cited))) or "—"} |')
    p()
    p('## Equations without a code pointer next to them')
    p()
    p('Each is either a derivation step, an assumption or a definition used by a neighbouring equation '
      '(the chapter section explains which), or a gap to close in a later documentation pass. None of '
      'them is a finding by itself.')
    p()
    for fn, tag, line, stmt, code, cited in rows:
        if not code:
            p(f'- ({tag}) — [{fn}]({fn}) line {line}: {esc(stmt)[:90]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
