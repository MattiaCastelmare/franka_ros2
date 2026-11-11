#!/usr/bin/env python3
"""Obstacle motion and the scenario catalog (chapter 12 §5, §7). Host, read-only.

Production code: `simexp.obstacle_laws.position/velocity/pursue`, `simexp.scenario_catalog`
(`load`, `all_names`, `resolve`, `_spawn`), `utils.gz_world.WANDER_RATIOS/PARK_AT`. The C++ system
(`franka_simulation/src/scenario_obstacles_system.cpp`) cannot run without Gazebo: its formulas are
checked by text match against the Python mirror.

  1. (12.3)-(12.7): velocity() is the time derivative of position() for every analytic law
     (central finite difference)                                                                   (NUM)
  2. the C++ PreUpdate evaluates the same formulas at t = simTime - t_start, parks before the start
     and publishes the LAW velocity, not a measured one; three copies of WANDER_RATIOS agree       (FACT)
  3. (12.8) pursuit is explicit Euler with the step clipped to the time since the start; for a fixed
     target it moves exactly speed*t along the line                                                (NUM)
  4. (12.9) ballistic throws: p(Tf) = flange_q0 for every catalog speed                           (NUM)
  5. seeds: seed 0 has no start offset and no jitter; seed k > 0 draws the offset in [0, 1/90 s)
     and moves p0 only for scenarios with jitter_m > 0                                             (FACT)
  6. the catalog: 100 names, classified static / moving / combined / none                          (INFO/FACT)
"""
import os
import sys
import collections

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common6 as C6  # noqa: E402
import numpy as np  # noqa: E402
from simexp import obstacle_laws as L, scenario_catalog as S  # noqa: E402
from utils import gz_world  # noqa: E402

R = C6.C.Report('check_obstacle_laws (chapter 12 §5, §7)')

# 1
specs = {
    'linear': dict(law='linear', p0=[0.4, 0.1, 0.5], v0=[-0.3, 0.2, 0.05]),
    'ballistic': dict(law='ballistic', p0=[0.4, 0.1, 0.5], v0=[-1.0, 0.2, 2.0], gravity=[0, 0, -9.81]),
    'oscillate': dict(law='oscillate', p0=[0.4, 0.1, 0.5], axis=[0, 2, 0], amplitude=0.2, period=4.0,
                      phase_deg=-90.0),
    'wander': dict(law='wander', p0=[0.4, 0.1, 0.5], box=[0.1, 0.15, 0.05], period=10.0,
                   phase_deg_xyz=[10.0, 20.0, 30.0]),
}
h = 1e-5
for name, sp in specs.items():
    err = max(np.max(np.abs((L.position(sp, t + h) - L.position(sp, t - h)) / (2 * h) - L.velocity(sp, t)))
              for t in np.linspace(0.0, 12.0, 61))
    R.below(f'1. {name}: max |dp/dt - velocity()| over t in [0, 12] s [m/s]', err, 1e-6)
R.fact('1. static: position = p0 and velocity = 0',
       np.allclose(L.position(dict(law='static', p0=[1, 2, 3]), 5.0), [1, 2, 3])
       and not L.velocity(dict(law='static', p0=[1, 2, 3]), 5.0).any())
sp = specs['wander']
pts = np.array([L.position(sp, t) for t in np.linspace(0, 200, 20001)])
R.fact('1. wander stays inside p0 +/- box (bounded by construction)',
       bool(np.all(np.abs(pts - np.asarray(sp['p0'])) <= np.asarray(sp['box']) + 1e-12)))

# 2
cpp = C6.src('franka_simulation/src/scenario_obstacles_system.cpp', 'cpp')
R.fact('2. C++ laws: p0 + v0 t; p0 + v0 t + 0.5 g t^2; p0 + axis A sin(w t + phase) with w = 2 pi / period; '
       'wander per axis with w0 * ratios[i]',
       all(x in cpp for x in ('p = ob.p0 + ob.v0 * t;', 'p = ob.p0 + ob.v0 * t + 0.5 * ob.gravity * t * t;',
                              'ob.v = ob.v0 + ob.gravity * t;', 'const double w = 2.0 * M_PI / ob.period;',
                              'p = ob.p0 + ob.axis * (ob.amplitude * std::sin(w * t + ob.phase));',
                              'const double wi = w0 * ob.ratios[i];',
                              'p[i] = ob.p0[i] + ob.box[i] * std::sin(a);')))
R.fact('2. C++ time: t_end = simTime (already advanced), running when t_end >= t_start - 1 ns, '
       't = max(0, t_end - t_start); before the start the obstacle is parked and v = 0',
       all(x in cpp for x in ('const double t_end = ToSeconds(info.simTime);',
                              'const bool running = t0.has_value() && t_end >= *t0 - 1e-9;',
                              'const double t = std::max(0.0, t_end - *t0);',
                              'p = ob.use_park ? ob.park : ob.p0;')))
R.fact('2. C++ truth: positions = worldPose (the simulated pose), velocities = ob.v (the LAW velocity, '
       'written into a Pose position field), every truth_period (default 0.001 s); the start time is '
       'latched once from /sim/episode_start',
       'const auto pose = ignition::gazebo::worldPose(ob.entity, ecm);' in cpp
       and 'vv->mutable_position()->set_x(ob.v.X());' in cpp
       and 'truth_period_ = el->Get<double>("truth_period", 0.001).first;' in cpp
       and 'if (!t_start_) {' in cpp)
R.fact('2. WANDER_RATIOS agree in obstacle_laws, gz_world and the C++ default; PARK_AT z = -20 m',
       tuple(L.WANDER_RATIOS) == tuple(gz_world.WANDER_RATIOS) == (1.0, (1 + 5 ** 0.5) / 2, 1 + 2 ** 0.5)
       and gz_world.PARK_AT == (0.0, 0.0, -20.0))

# 3 pursuit
tgt = np.array([0.5, 0.0, 0.6])
p0 = np.array([0.5, 0.6, 0.6])
steps = 300
P = L.pursue(p0, 0.5, np.tile(tgt, (steps, 1)), 1e-3)
R.close('3. pursuit, fixed target, 0.3 s at 0.5 m/s: distance travelled [m]', float(np.linalg.norm(P[-1] - p0)),
        0.15, 1e-12, kind='NUM')
R.fact('3. C++ pursuit: v = speed * unit(target - p) at the start of the step, p += v * min(dt, t) '
       '(explicit Euler; no clamp at the target)',
       'const double h = std::min(dt, t);' in cpp and 'ob.v = n > 1e-9 ? d * (ob.speed / n) : Vector3d::Zero;' in cpp
       and 'ob.p += ob.v * h;' in cpp)

# 4 ballistic throws
cfg = S.load()
fl = np.asarray(cfg['flange_q0'], float)
errs = []
for n in S.throw_names(cfg):
    if not n.startswith('throw_ballistic_v'):
        continue
    o = S.resolve(n, 0, cfg)['obstacles'][0]
    v = float(n[len('throw_ballistic_v'):])
    Tf = float(cfg['ballistic_throws']['release_distance']) / v
    errs.append(float(np.linalg.norm(L.position(o, Tf) - fl)))
R.below(f'4. {len(errs)} ballistic throws: max |p(Tf) - flange_q0| [m]', max(errs), 1e-12)

# 5 seeds
names = S.all_names(cfg)
r0 = [S.resolve(n, 0, cfg) for n in names]
r1 = [S.resolve(n, 1, cfg) for n in names]
R.fact('5. seed 0: start_offset_s = 0 for every scenario', all(r['start_offset_s'] == 0.0 for r in r0))
R.fact('5. seed 1: 0 <= start_offset_s < 1/90 s', all(0.0 <= r['start_offset_s'] < 1 / 90 for r in r1))
moved = [r0[i]['name'] for i in range(len(names))
         if any(not np.allclose(a['p0'], b['p0']) for a, b in zip(r0[i]['obstacles'], r1[i]['obstacles']))]
jit = [r['name'] for r in r0 if r['jitter_m'] > 0]
R.fact(f'5. seed 1 moves p0 only where jitter_m > 0: moved {moved}, jitter scenarios {jit}', moved == jit)

# 6 classification
kinds = collections.Counter()
for r in r0:
    laws = [o['law'] for o in r['obstacles']]
    k = ('none' if not laws else 'static' if all(x == 'static' for x in laws)
         else 'moving' if all(x != 'static' for x in laws) else 'combined')
    kinds[(r['source'], k)] += 1
R.info(f'6. {len(names)} scenarios by (source, kind): {dict(sorted(kinds.items()))}')
R.info('6. scenes: ' + str(dict(collections.Counter(r['scene'] for r in r0))))
R.fact('6. 100 names; the only combined (static + moving) scenarios are pentagon_multi_mixed and p4_laws',
       len(names) == 100 and sorted(r['name'] for r in r0 if len({o['law'] == 'static' for o in r['obstacles']}) == 2)
       == ['p4_laws', 'pentagon_multi_mixed'])
sys.exit(R.finish())
