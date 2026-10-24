"""Speed-and-separation-monitoring math, ISO 10218-2:2025 Annex L.

OWNS
----
The four closed forms the ISO layer is built on, and nothing else:

* :func:`stopping_distance`     — ``S_r + S_s``
* :func:`protective_separation` — ``S_p``
* :func:`ssm_speed_cap`         — the exact inversion of ``S_p`` in ``v_r``
* :func:`pfl_speed`             — ``v_PFL`` from Annex M's biomechanical limits

Pure numpy. No ROS, no classes, no state — so the QP row builder, the
independent monitor and the offline scripts all consume ONE implementation and
cannot drift into three.

NORMATIVE STATUS
----------------
ISO 10218-2:2025 **Annex L** (normative) gives ``S_p`` as the sum of three
integrals plus ``C + Z_d + Z_r`` — that STRUCTURE is **[S]**::

    S_p = S_h + S_r + S_s + C + Z_d + Z_r

The closed forms below assume constant ``v_h``, constant ``v_r`` during ``T_r``,
and constant deceleration ``a_s`` during ``T_s = v_r/a_s`` — that is **[E]**.
The inversion in :func:`ssm_speed_cap` is exact with respect to THESE CLOSED
FORMS, not with respect to the standard's integrals — also **[E]**.

``S_s`` computed as ``v_r²/(2·a_s)`` is conservative only if ``a_s`` is the
**minimum** measured deceleration over the whole Annex H test grid
(``scripts/iso_constants_measure.py stop``), and must be checked against the
stopping distances that script reports. An ``a_s`` taken as a mean, or from the
commanded rather than the realized deceleration, makes every number here
optimistic by exactly that ratio.

WHAT IT IS NOT
--------------
This module computes bounds. It is not a safety function, it does not know
whether the numbers fed to it were measured, and it cannot make the
single-channel Python chain that calls it reach PL d / SIL 2. See
``franka_experiments/SAFETY.md``.
"""

from __future__ import annotations

import numpy as np

__all__ = ['stopping_distance', 'protective_separation', 'ssm_speed_cap',
           'pfl_speed']


def stopping_distance(v_r, *, t_r, a_s) -> float:
    """[m] ``S_r + S_s`` — how far the robot travels once a trip is decided.

    ::

        S_r = v_r · t_r            during the reaction time, still at speed
        S_s = v_r² / (2·a_s)       while decelerating at a constant a_s

    ``v_r`` is clamped at ``>= 0``: a receding part never shrinks the distance
    the robot needs, and a negative speed here would return a shorter one.

    Args:
        v_r: robot speed toward the obstacle [m/s].
        t_r: reaction time ``T_r`` [s].
        a_s: realized Cartesian deceleration ``a_s`` [m/s²], strictly positive.

    Returns:
        ``S_r + S_s`` in metres.  **[E]** model, **[S]** terms.
    """
    v = max(float(v_r), 0.0)
    a = max(float(a_s), 1e-9)
    return float(v * float(t_r) + v * v / (2.0 * a))


def protective_separation(v_r, v_h, *, t_r, a_s, c, z_d, z_r) -> float:
    """[m] the protective separation distance ``S_p``.

    ::

        T_s = v_r / a_s
        S_h = v_h · (t_r + T_s)     the human keeps approaching throughout
        S_r = v_r · t_r             the robot keeps going during the reaction
        S_s = v_r² / (2·a_s)        and then decelerates
        S_p = S_h + S_r + S_s + C + Z_d + Z_r

    ``C`` is the ISO 13855:2024 intrusion distance — a function of the
    protective device's DETECTION CAPABILITY, not a tuning knob. ``Z_d`` and
    ``Z_r`` are the sensing and robot position uncertainties.

    Both speeds are clamped at ``>= 0``: a receding obstacle or a receding robot
    never shrinks ``S_p``.

    Args:
        v_r: robot speed toward the obstacle [m/s].
        v_h: human approach speed [m/s].
        t_r, a_s: reaction time [s] and realized deceleration [m/s²].
        c, z_d, z_r: ``C``, ``Z_d``, ``Z_r`` [m].

    Returns:
        ``S_p`` in metres. **[S]** structure, **[E]** closed form.
    """
    v = max(float(v_r), 0.0)
    h = max(float(v_h), 0.0)
    a = max(float(a_s), 1e-9)
    t = float(t_r)
    s_h = h * (t + v / a)
    s_r = v * t
    s_s = v * v / (2.0 * a)
    return float(s_h + s_r + s_s + float(c) + float(z_d) + float(z_r))


def ssm_speed_cap(d, v_h, *, t_r, a_s, c, z_d, z_r, v_max) -> float:
    """[m/s] the largest robot speed TOWARD the obstacle that keeps ``d >= S_p``.

    Exact inversion of :func:`protective_separation` in ``v_r``. Substituting
    ``S_p(v_r) = d`` and collecting gives a quadratic in ``v_r``::

        v_r²/(2a) + v_r·(t_r + v_h/a) + (v_h·t_r + c + z_d + z_r − d) = 0

    whose positive root is, with ``b = a·t_r + v_h``::

        disc = b² + 2·a·(d − c − z_d − z_r − v_h·t_r)
        v    = sqrt(disc) − b          (0.0 when disc <= 0)

    ``disc <= 0`` is the STOP REGION: no positive approach speed satisfies the
    separation requirement, so the cap is exactly ``0.0``. ``d <= c + z_d + z_r``
    is tested for FIRST and short-circuits, whatever ``v_h`` and ``t_r`` are —
    at exactly that boundary the quadratic returns a float epsilon rather than
    zero, and a cap of 1e-17 reads as "creep allowed" to every consumer.

    Args:
        d: measured separation distance [m].
        v_h: human approach speed [m/s].
        t_r, a_s: reaction time [s] and realized deceleration [m/s²].
        c, z_d, z_r: ``C``, ``Z_d``, ``Z_r`` [m].
        v_max: flat ceiling the result is clipped to [m/s].

    Returns:
        ``min(v, v_max)``, floored at ``0.0``. **[E]**.
    """
    # The irreducible part of S_p. Inside it no positive speed can satisfy the
    # requirement, and the quadratic below already says so — but only to within
    # a float epsilon (sqrt(b*b) - b is 1e-17, not 0, at exactly the boundary).
    # Tested as EXACTLY 0.0 because a cap of 1e-17 reads as "creep allowed" to
    # every consumer, and the stop region must not be a creep region.
    floor = float(c) + float(z_d) + float(z_r)
    if not np.isfinite(d) or float(d) <= floor:
        return 0.0
    a = max(float(a_s), 1e-9)
    h = max(float(v_h), 0.0)
    t = float(t_r)
    b = a * t + h
    disc = b * b + 2.0 * a * (float(d) - float(c) - float(z_d) - float(z_r) - h * t)
    if not np.isfinite(disc) or disc <= 0.0:
        return 0.0
    v = float(np.sqrt(disc)) - b
    if not np.isfinite(v) or v <= 0.0:
        return 0.0
    return float(min(v, float(v_max)))


def pfl_speed(f_max, k_body, m_robot, m_human) -> float:
    """[m/s] the power-and-force-limited speed ``v_PFL``.

    ::

        mu    = 1 / (1/m_human + 1/m_robot)      reduced mass [kg]
        v_PFL = f_max / sqrt(mu · k_body)

    **[S]** formula and simplification: ISO 10218-2:2025 **Annex M**
    (informative; formerly ISO/TS 15066 Annex A). ``m_robot = M/2 + payload``
    with ``M`` the manipulator mass — the ``M/2`` is the annex's own effective-
    mass simplification, not a guess made here.

    WORKED EXAMPLE (the module's own regression pin) — the Annex M
    **hands-and-fingers** row, quasi-static, on an FR3 with no payload and no
    gripper::

        F_max = 140 N        k = 75 N/mm = 75000 N/m       m_H = 0.6 kg
        M     = 17.8 kg  ->  m_R = M/2 = 8.9 kg
        mu    = 1/(1/0.6 + 1/8.9)          = 0.5620 kg
        v_PFL = 140 / sqrt(0.5620 · 75000) = 0.682 m/s

    Three things this number is NOT, all of them load-bearing:

    * it is the QUASI-STATIC limit. The transient limit for a region is roughly
      twice as high; the quasi-static one is used because a clamping event
      cannot be excluded in a cell with no dedicated clamping-hazard analysis
      **[E]**.
    * it is per BODY REGION. Another region's ``F_max``/``k``/``m_H`` gives
      another speed; the binding one is the smallest over the regions a contact
      can actually reach.
    * it is a CALCULATION. Claiming PFL requires force/pressure MEASUREMENT with
      a PFMD per ISO 10218-2:2025 clause 6.3.3 and Annex N **[R]**. This
      supports a preliminary risk assessment and nothing beyond it.

    Args:
        f_max: permissible force for the body region [N].
        k_body: effective spring constant of the region [N/m] — note N/m, not
            the N/mm the annex tabulates.
        m_robot: effective robot mass ``M/2 + payload`` [kg].
        m_human: effective mass of the body region [kg].

    Returns:
        ``v_PFL`` in m/s.
    """
    m_r = max(float(m_robot), 1e-9)
    m_h = max(float(m_human), 1e-9)
    mu = 1.0 / (1.0 / m_h + 1.0 / m_r)
    return float(float(f_max) / np.sqrt(mu * max(float(k_body), 1e-9)))
