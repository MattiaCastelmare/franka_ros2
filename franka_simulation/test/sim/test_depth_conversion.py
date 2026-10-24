"""Gazebo 32FC1 metres -> D455 16UC1 millimetres, and the optional fidelity stages (H2, H11)."""
import numpy as np
import pytest

from utils import depth_conversion as D


def test_metres_to_mm_rounds_and_maps_no_return_to_zero():
    d = np.array([[0.4004, 1.2346, np.inf, -np.inf], [np.nan, 0.0, -0.5, 70.0]], np.float32)
    mm = D.metres_to_mm(d)
    assert mm.dtype == np.uint16
    assert mm.tolist() == [[400, 1235, 0, 0], [0, 0, 0, 0]]


def test_the_engine_reads_back_the_metres():
    """distance_engine multiplies by 0.001 (_DEPTH_TO_M): within half a millimetre."""
    rng = np.random.default_rng(0)
    d = rng.uniform(0.2, 6.0, (480, 848)).astype(np.float32)
    back = D.metres_to_mm(d).astype(np.float32) * np.float32(0.001)
    assert np.abs(back - d).max() <= 0.0005 + 1e-6


def test_all_stages_off_is_the_encoding_alone():
    st = D.FidelityStages()
    img = np.full((4, 4), 1.5, np.float32)
    assert st.push(10.0, img, tag='stamp') is True
    (stamp, tag, out), = st.pop_due(10.0)
    assert (stamp, tag) == (10.0, 'stamp') and (out == 1500).all()


def test_delay_keeps_the_capture_stamp():
    st = D.FidelityStages(delay_s=0.025)
    st.push(1.000, np.ones((2, 2), np.float32), tag=1)
    assert st.pop_due(1.020) == []
    (stamp, tag, _), = st.pop_due(1.025)
    assert stamp == 1.000 and tag == 1


def test_dropout_rate_and_seeded_replay():
    a = D.FidelityStages(dropout_p=0.2, seed=7)
    b = D.FidelityStages(dropout_p=0.2, seed=7)
    ka = [a.push(k, np.ones((2, 2), np.float32)) for k in range(4000)]
    kb = [b.push(k, np.ones((2, 2), np.float32)) for k in range(4000)]
    assert ka == kb
    assert abs(1 - np.mean(ka) - 0.2) < 0.02


def test_range_noise_follows_the_stereo_model():
    f, b, sd = 428.8, 0.095, 0.20
    z = np.full((300, 300), 2.0, np.float32)
    noisy = D.add_range_noise(z, np.random.default_rng(1), f_px=f, baseline_m=b, sigma_d_px=sd)
    assert np.std(noisy - z) == pytest.approx(D.range_sigma(2.0, f, b, sd), rel=0.03)
    holes = z.copy()
    holes[0, 0] = np.nan
    assert np.isnan(D.add_range_noise(holes, np.random.default_rng(1), f_px=f,
                                      baseline_m=b, sigma_d_px=sd)[0, 0])


def test_bad_stage_parameters_are_refused():
    with pytest.raises(ValueError):
        D.FidelityStages(dropout_p=1.0)
    with pytest.raises(ValueError):
        D.FidelityStages(range_noise=True, f_px=0.0)
