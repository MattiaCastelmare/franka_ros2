#!/usr/bin/env python3
"""SIM. Batch runner of the Gazebo CBF experiments (spec Phases 5 and 6).

    ros2 run franka_simulation_experiments run_gz_scenarios --grid phase5 \\
        [--batch-dir DIR] [--max-cells N]
    ros2 run franka_simulation_experiments run_gz_scenarios \\
        --scenarios static slow --conditions kf imm --seeds 0 1 --perception true emulated
    ros2 run franka_simulation_experiments run_gz_scenarios --report --batch-dir DIR \\
        [--report-path franka_simulation/docs/gz_cbf_experiments_report.md]

Every cell is a FRESH headless Gazebo (``ign gazebo -s --headless-rendering``)
with the whole stack, started and torn down by process group, so no state
crosses from one cell to the next (no reset to verify). Results go to
``<batch-dir>/cells/<cell>/`` (the run itself, cell_result.json) and
``<batch-dir>/cells.csv``, every row tagged SIM with its RTF and achieved QP
rate. A batch resumes: a cell with a cell_result.json is not re-run.

``--rescore`` re-runs the scorers on the cells of a batch without Gazebo (after
a scoring fix); ``--report`` writes the Phase 5 tables (simexp.report) from a finished batch,
with the offline column (cbf_scenarios.py) computed alongside.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from simexp import batch, sim_config  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--grid', choices=['phase5', 'emulated30', 'flags'],
                    help='a predefined grid (simexp.batch)')
    ap.add_argument('--scenarios', nargs='*', default=[])
    ap.add_argument('--conditions', nargs='*', default=['kf'])
    ap.add_argument('--seeds', nargs='*', type=int, default=[0])
    ap.add_argument('--perception', nargs='*', default=['true'],
                    choices=['true', 'emulated', 'false'])
    ap.add_argument('--batch-dir', default='')
    ap.add_argument('--max-cells', type=int, default=0)
    ap.add_argument('--report', action='store_true', help='write the report of --batch-dir')
    ap.add_argument('--rescore', action='store_true',
                    help='re-run the scorers on every cell of --batch-dir (no Gazebo), rebuild the CSV')
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--report-path', default='')
    a = ap.parse_args()

    bd = a.batch_dir or os.path.join(sim_config.sim_runs_root(), 'batches',
                                     a.grid or 'custom')
    if a.rescore:
        print(f'SIM rescore -> {batch.rescore(bd, workers=a.workers, log=lambda m: print(m, flush=True))}')
        return
    if a.report:
        from simexp import report
        path = report.write(bd, a.report_path or None)
        print(f'SIM report -> {path}')
        return
    if a.grid == 'phase5':
        cells = batch.phase5_grid()
    elif a.grid == 'emulated30':
        cells = batch.emulated30_grid()
    elif a.grid == 'flags':
        cells = batch.flags_grid()
    else:
        if not a.scenarios:
            ap.error('--grid or --scenarios')
        cells = [dict(scenario=s, condition=c, seed=k, perception=p)
                 for k in a.seeds for s in a.scenarios for c in a.conditions
                 for p in a.perception]
    if a.max_cells:
        cells = cells[:a.max_cells]
    print(f'SIM batch: {len(cells)} cells -> {bd}', flush=True)
    csv_path = batch.run_grid(bd, cells, log=lambda s: print(s, flush=True))
    print(f'SIM batch done -> {csv_path}')


if __name__ == '__main__':
    main()
