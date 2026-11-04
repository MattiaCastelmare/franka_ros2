#!/usr/bin/env python3
"""Numerically audit CP derivatives on saved joint states (no robot connection).

Input NPZ: qc and vc, N x 7 joint positions/velocities. This consumes the
extracted IRIM2 kinematics inputs and compares analytical J, Jdot with central
differences of the SAME CP positions used by perception. Exit 1 on failure.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import pinocchio as pin
import yaml

from franka_experiments.utils.distance_utils import define_control_points
from franka_experiments.utils.kinematics import CBFKinematics, build_urdf_no_hand


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('input_npz')
    ap.add_argument('--stride', type=int, default=20)
    ap.add_argument('--output', required=True)
    ap.add_argument('--config', default=str(Path(__file__).resolve().parents[1] / 'config/fr3_complete.yaml'))
    args = ap.parse_args()
    cfg = yaml.safe_load(Path(args.config).read_text())
    model = pin.buildModelFromUrdf(build_urdf_no_hand())
    kin = CBFKinematics(model)
    kin.configure_control_points(cfg['robot'], cfg['distance'])
    data = model.createData()

    def positions(q):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        transforms = {name: (data.oMf[model.getFrameId(name)].rotation,
                              data.oMf[model.getFrameId(name)].translation)
                      for name in cfg['robot']['segment_links']}
        return np.array([p['point'] for p in define_control_points(transforms, cfg['robot'], cfg['distance'])])

    inputs = np.load(args.input_npz)
    errors, derrors, accel_errors = [], [], []
    labels = list(kin._control_points)
    eps = 1e-6
    for q, v in zip(inputs['qc'][::args.stride], inputs['vc'][::args.stride]):
        kin.update(q, v)
        jac, djac = zip(*(kin.control_point_jacobian(label) for label in labels))
        jac, djac = np.array(jac), np.array(djac)
        fd = np.empty_like(jac)
        for k in range(7):
            delta = np.zeros(model.nv); delta[k] = eps
            fd[:, :, k] = (positions(pin.integrate(model, q, delta))
                            - positions(pin.integrate(model, q, -delta))) / (2*eps)
        kin.update(pin.integrate(model, q, v*eps), v)
        jp = np.array([kin.control_point_jacobian(label)[0] for label in labels])
        kin.update(pin.integrate(model, q, -v*eps), v)
        jm = np.array([kin.control_point_jacobian(label)[0] for label in labels])
        df = (jp-jm)/(2*eps)
        errors.append(float(np.max(np.abs(jac-fd))))
        derrors.append(float(np.max(np.abs(djac-df))))
        accel_errors.append(float(np.max(np.linalg.norm((djac-df) @ v, axis=1))))
    result = dict(n_configurations=len(errors), n_control_points=len(labels),
                  jacobian_max_abs_error_m_per_rad=max(errors),
                  jdot_max_abs_error=max(derrors),
                  drift_accel_max_error_m_s2=max(accel_errors),
                  passed=max(errors+derrors+accel_errors) < 1e-6)
    Path(args.output).write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
