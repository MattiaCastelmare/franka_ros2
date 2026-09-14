"""The PFL ceiling and the reduced-speed derate (roadmap Step 9).

``iso_v_pfl`` is the bound that survives a total perception failure: with the
depth camera unplugged and no obstacle row in the QP, it is still the speed at
which a contact stays inside the biomechanical limit of the region it hits.

What is pinned here:

* the shipped ``iso_v_pfl`` is the number ``scripts/iso_pfl_speed.py`` actually
  computes for the shipped configuration. A ceiling that drifted from its own
  derivation is a number, not a ceiling;
* the filter RAISES, not clamps, when ``link_speed_max`` exceeds it (that half
  lives in ``test_iso_dsafe_floor``; here we check the CONFIGURATION is the one
  the raise will fire on, and say why);
* ``iso_tcp_reduced_speed`` is the standard's 250 mm/s and not a tuned number.

None of this is PFL compliance: **[R]** that requires force/pressure measurement
with a PFMD per ISO 10218-2:2025 clause 6.3.3 and Annex N. See SAFETY.md.
"""

import os
import subprocess
import sys

import pytest
import yaml

PKG = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..')
CONFIG = os.path.join(PKG, 'config', 'fr3_control.yaml')
SCRIPT = os.path.join(PKG, 'scripts', 'iso_pfl_speed.py')


@pytest.fixture(scope='module')
def params():
    with open(CONFIG) as fh:
        return yaml.safe_load(fh)['params']


# ── the number and its derivation agree ──────────────────────────────────────

def test_the_shipped_v_pfl_is_what_the_script_computes(params):
    out = subprocess.run([sys.executable, SCRIPT], capture_output=True, text=True,
                         cwd=PKG)
    assert out.returncode == 0, out.stderr
    line = next(l for l in out.stdout.splitlines() if 'iso_v_pfl' in l and 'm/s' in l)
    computed = float(line.split(':')[1].split()[0])
    assert params['iso_v_pfl'] == pytest.approx(computed, abs=0.005), (
        f'iso_v_pfl={params["iso_v_pfl"]} in fr3_control.yaml but '
        f'{computed:.3f} from scripts/iso_pfl_speed.py — one of them moved')


def test_the_script_prints_all_three_required_warnings():
    out = subprocess.run([sys.executable, SCRIPT], capture_output=True, text=True,
                         cwd=PKG).stdout
    assert 'QUASI-STATIC vs TRANSIENT' in out
    assert 'lower' in out and 'v_PFL' in out          # mass warning
    assert 'PFMD' in out and '6.3.3' in out and 'Annex N' in out
    assert 'NOT PFL COMPLIANCE' in out


def test_a_payload_lowers_the_ceiling():
    def v(extra):
        out = subprocess.run([sys.executable, SCRIPT, '--payload', str(extra)],
                             capture_output=True, text=True, cwd=PKG).stdout
        line = next(l for l in out.splitlines() if 'iso_v_pfl' in l and 'm/s' in l)
        return float(line.split(':')[1].split()[0])
    assert v(3.0) < v(0.0)


def test_the_transient_limit_is_about_twice_the_quasi_static_one():
    def v(*extra):
        out = subprocess.run([sys.executable, SCRIPT, *extra],
                             capture_output=True, text=True, cwd=PKG).stdout
        line = next(l for l in out.splitlines() if 'iso_v_pfl' in l and 'm/s' in l)
        return float(line.split(':')[1].split()[0])
    assert v('--transient') == pytest.approx(2.0 * v(), rel=1e-3)


# ── the configuration the ceiling applies to ─────────────────────────────────

def test_the_shipped_link_speed_max_is_above_v_pfl_and_that_is_the_finding(params):
    """Not a bug to be fixed by editing one of the two numbers.

    ``link_speed_max = 1.3`` is a tuned research value; ``iso_v_pfl = 0.68`` is
    the quasi-static hands-and-fingers ceiling. They disagree, so
    ``iso_enabled: true`` refuses to start until the operator lowers the first
    (``link_speed_max:=0.68 retreat_cap_max_speed:=0.60`` on the launch line, or
    in the YAML). That refusal is the mechanism; this test records that it is
    currently armed.
    """
    assert params['iso_enabled'] is False
    assert params['link_speed_max'] > params['iso_v_pfl']


def test_the_ordering_invariant_is_stated_and_currently_holds(params):
    """retreat_cap_max_speed < link_speed_max: the retreat cap is the inner
    bound and the speed row the outer one. Inverting them makes the outer row
    unreachable."""
    assert params['retreat_cap_max_speed'] < params['link_speed_max']


def test_lowering_link_speed_max_to_v_pfl_would_break_the_ordering(params):
    """Which is exactly why the filter's message names BOTH overrides."""
    assert params['retreat_cap_max_speed'] > params['iso_v_pfl'], (
        'if this ever stops holding, the second half of the filter\'s error '
        'message (retreat_cap_max_speed:=...) is no longer needed and should go')


# ── reduced speed ────────────────────────────────────────────────────────────

def test_reduced_speed_is_the_standards_250_mm_per_s(params):
    """[S] ISO 10218-1:2025 (5.5.3) / -2:2025 (5.5.6). Not a tuned number: it
    is the value the standard requires for manual modes, and applying it as a
    cell-wide derate is the [E] part, not the 0.25."""
    assert params['iso_tcp_reduced_speed'] == pytest.approx(0.25)


def test_the_shipped_mode_is_automatic(params):
    assert params['iso_mode'] == 'automatic'
