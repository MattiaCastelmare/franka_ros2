import numpy as np

from franka_experiments.utils.state_governor import directional_fade

A = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]])   # margin rises with q6 acceleration


def test_zero_fade_is_the_identity():
    nom = np.array([1.0, -2.0, 0.5, 0.0, 0.3, -4.0, 1.0])
    np.testing.assert_array_equal(directional_fade(nom, np.full(7, -1.0), A, 0.0, 3.0), nom)


def test_only_the_harmful_component_is_removed():
    nom = np.array([1.0, -2.0, 0.5, 0.0, 0.3, -4.0, 1.0])
    out = directional_fade(nom, np.zeros(7), A, 1.0, 3.0)
    assert out[5] == 0.0                              # the part driving the margin down is gone
    np.testing.assert_array_equal(np.delete(out, 5), np.delete(nom, 5))   # the rest of the task runs


def test_a_helpful_nominal_passes_through():
    nom = np.array([1.0, -2.0, 0.5, 0.0, 0.3, +4.0, 1.0])
    np.testing.assert_array_equal(directional_fade(nom, np.zeros(7), A, 1.0, 3.0), nom)


def test_velocity_heading_down_the_margin_is_braked_along_the_gradient_only():
    qd = np.zeros(7); qd[5] = -1.0; qd[0] = 0.7
    out = directional_fade(np.zeros(7), qd, A, 1.0, 3.0)
    assert out[5] == 3.0 and out[0] == 0.0           # braking on q6, nothing on the unrelated joint
