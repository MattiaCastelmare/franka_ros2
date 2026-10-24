"""Reading a run back: CBFDIAG fields, the qdd ratio, the pentagon tracking error."""
import numpy as np
import pytest

from simexp import analysis as A

LINE = ('[cbf_safety_filter-11] [INFO] [1] [cbf_safety_filter]: CBFDIAG t=12.540 n_c=1 '
        'n_act=0 d_min=inf link=sing:fr3_link8 hdot=-0.009(raw-0.055) h_qp=+1.882 '
        's[obs/sc/qlim/sing/cap/spd]=0.100/0.000/0.250/0.000/0.000/0.000 '
        'qdd_cmd_rad=+2.000 qdd_real_rad=+1.500 trk_err=0.069 slew=....... | worst=j4:q̇=+0.72(0.27)\n')


def test_cbfdiag_fields_and_ratio(tmp_path):
    p = tmp_path / 'launch.log'
    p.write_text('noise\n' + LINE * 3 + LINE.replace('qdd_cmd_rad=+2.000', 'qdd_cmd_rad=+0.100'))
    rows = A.parse_cbfdiag(str(p))
    assert len(rows) == 4 and rows[0]['n_c'] == '1' and rows[0]['link'] == 'sing:fr3_link8'
    r = A.qdd_ratio(rows, min_cmd=0.5)
    assert r['n_used'] == 3 and r['ratio_median'] == pytest.approx(0.75)
    assert A.slack_max(rows) == {'obs': 0.1, 'sc': 0.0, 'qlim': 0.25, 'sing': 0.0,
                                 'cap': 0.0, 'spd': 0.0}


def test_pentagon_tracking(tmp_path):
    p = tmp_path / 'pentagon_run_x.csv'
    rows = ['time,ee_x,ee_y,ee_z,ee_x_des,ee_y_des,ee_z_des']
    for k in range(100):
        rows.append(f'{k * 0.01},{0.4},{0.0},{0.5},{0.4},{0.003 if k >= 50 else 0.0},{0.5}')
    p.write_text('\n'.join(rows) + '\n')
    r = A.pentagon_tracking(str(p), t_from=0.5)
    assert r['n'] == 50 and r['p50_m'] == pytest.approx(0.003) and r['max_m'] == pytest.approx(0.003)
