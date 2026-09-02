"""Shared utilities for RealTimeDistance ROS 2 nodes.

Centralises boilerplate used by real_time_distance.py:

  PerfTimer          — lightweight named-stage wall-clock profiler
  get_safety_zone()  — distance → zone label classification
  build_cp_messages()— populate MultiDistance + MultiLinkDistance from CP results
  no_obs_warn()      — throttled 'no obstacle' debug log
"""

# TODO[LEGACY]: compatibility shim: contents split into utils/perception_msgs.py and utils/logging_utils.py | confidence: high | superseded-by: utils/perception_msgs.py + utils/logging_utils.py | flagged: 2026-09-01
from __future__ import annotations

import math
import time

import numpy as np

from franka_msgs.msg import (
    HumanRobotDistance, LinkDistance, MultiDistance, MultiLinkDistance)
from geometry_msgs.msg import Point, Vector3

# MOVED in Phase 2: the message builders now live in
# utils/perception_msgs.py and PerfTimer in utils/logging_utils.py.
# Re-exported here so every existing import keeps working unchanged.
from franka_experiments.utils.logging_utils import PerfTimer  # noqa: F401
from franka_experiments.utils.perception_msgs import (  # noqa: F401
    build_cp_messages,
    find_pt_confidence,
    get_safety_zone,
    no_obs_warn,
)


# ── Performance profiler ──────────────────────────────────────────────────────

# ── Zone classification ───────────────────────────────────────────────────────

# ── CP message builders ───────────────────────────────────────────────────────

# ── Throttled warning ─────────────────────────────────────────────────────────

