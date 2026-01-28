"""Runtime state container for velocity_control_blender.

This module centralizes *dynamic* fields to keep the ROS node high-level.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from .velocity_blender_core import EmergencyRecoveryState
from .velocity_blender_ros_helpers import PolylineProgress, VelocityBlenderDiagnostics


@dataclass
class BlenderRuntimeState:
	"""All runtime state previously stored as Node attributes."""

	n_dof: int

	# Joint state + avoidance
	q: Optional[np.ndarray] = None
	qdot_avoid: Optional[np.ndarray] = None
	qdot_avoid_filt: Optional[np.ndarray] = None
	qdot_prev: Optional[np.ndarray] = None

	# Hazard signals
	hazard: str = "none"
	paused: bool = False

	closest_d: float = 999.0
	closest_j_row: Optional[np.ndarray] = None
	closest_hazard: str = "none"

	# Multi-constraint list
	constraints_rows: Optional[np.ndarray] = None
	constraints_d: Optional[np.ndarray] = None
	constraints_prev_rows: Optional[np.ndarray] = None
	constraints_last_wall: float = 0.0

	# Filtered closest constraint row
	closest_j_row_filt: Optional[np.ndarray] = None
	closest_j_row_filt_init: bool = False

	# Emergency / stop gate state
	er_state: EmergencyRecoveryState = field(default_factory=EmergencyRecoveryState)
	stop_active: bool = False
	stop_log_wall: float = 0.0
	stop_enter_wall: Optional[float] = None
	stop_phase: str = "HOLD"
	stop_warn_wall: float = 0.0
	stop_d_dot_last: float = 0.0

	# Trajectory + progress
	trajectory_points: List[np.ndarray] = field(default_factory=list)
	current_index: int = 0
	active: bool = False
	progress: PolylineProgress = field(default_factory=lambda: PolylineProgress(progress_index=0, progress_s=0.0))
	traj_s: Optional[np.ndarray] = None

	# Stall detection
	stall_prog_s_last: Optional[float] = None
	stall_prog_wall_last: Optional[float] = None
	stall_prog_flag: bool = False

	# Diagnostics / counters
	diag: VelocityBlenderDiagnostics = field(
		default_factory=lambda: VelocityBlenderDiagnostics(
			last_diag_wall=0.0,
			last_robust_wall=0.0,
			infeasible_count=0,
			emergency_enter_count=0,
			stop_gate_count=0,
			prev_emergency=False,
			prev_stop_gate=False,
		)
	)

	# Throttled logs
	reactive_log_wall: float = 0.0
	multi_log_wall: float = 0.0

	def __post_init__(self) -> None:
		n = int(self.n_dof)
		if self.q is None:
			self.q = np.zeros(n, dtype=float)
		if self.qdot_avoid is None:
			self.qdot_avoid = np.zeros(n, dtype=float)
		if self.qdot_avoid_filt is None:
			self.qdot_avoid_filt = np.zeros(n, dtype=float)
		if self.qdot_prev is None:
			self.qdot_prev = np.zeros(n, dtype=float)
		if self.closest_j_row is None:
			self.closest_j_row = np.zeros(n, dtype=float)
		if self.closest_j_row_filt is None:
			self.closest_j_row_filt = np.zeros(n, dtype=float)
		if self.constraints_rows is None:
			self.constraints_rows = np.zeros((0, n), dtype=float)
		if self.constraints_d is None:
			self.constraints_d = np.zeros((0,), dtype=float)
		if self.constraints_prev_rows is None:
			self.constraints_prev_rows = np.zeros((0, n), dtype=float)
