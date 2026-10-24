#!/usr/bin/env python3
"""How much warning does the perception give for each ball pass?  (per pass: first row on the ball, track >= 3 frames)

    python3 scripts/throw_lead.py <live bag> <bag_replay output holding /cbf/per_link_distances> <truth.npz>

"On the ball" = a row whose obstacle point is within 0.15 m of the colour-tracked ball (ball_throw_eval truth) at the
row's capture time. Leads are seconds before the closest approach to a control point.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ball_closed_loop as B  # noqa: E402


def main(bag, dist, truth):
    rec = B.Recording(bag, dist)
    T = np.load(truth)
    tt, ok, P = T['t'], T['ok'], T['p']
    out = []
    for p in B.find_passes(rec, (tt, ok, P)):
        tc = p['tc']
        first_row = first_trk = None
        for tcap, _, m in rec.dist:
            if not (tc - 1.5 < tcap < tc + 0.05):
                continue
            b = B._ball_at(tt, ok, P, tcap)
            if b is None:
                continue
            rows = [l for l in m.links if l.valid and np.linalg.norm(
                [l.closest_point_human.x - b[0], l.closest_point_human.y - b[1],
                 l.closest_point_human.z - b[2]]) < 0.15]
            if rows:
                first_row = tcap if first_row is None else first_row
                if first_trk is None and any(l.frames_seen >= 3 for l in rows):
                    first_trk = tcap
        out.append((tc - first_row if first_row else np.nan, tc - first_trk if first_trk else np.nan))
    o = np.array(out, float)
    print(f'{os.path.basename(dist.rstrip("/")):32s} first row {np.nanmedian(o[:, 0]):.2f}s  '
          f'track>=3 {np.nanmedian(o[:, 1]):.2f}s  per pass {np.round(o[:, 1], 2)}')


if __name__ == '__main__':
    main(*sys.argv[1:4])
