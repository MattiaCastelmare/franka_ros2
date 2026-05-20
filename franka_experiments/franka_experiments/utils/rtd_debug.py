"""RTD debug/profiling helpers for RealTimeDistance.

All classes are usable standalone; the node decides which to instantiate
based on the ``debug_extreme`` parameter.  When the parameter is False none
of these objects are created, so overhead is exactly zero.
"""
from __future__ import annotations

import collections
import faulthandler
import io
import sys
import threading
import time
import traceback
import tracemalloc
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np


# ── Log-tag formatter ────────────────────────────────────────────────────────

def rtd_tag(
    step: str,
    elapsed_ms: float,
    frame_id: int = -1,
    thread: Optional[str] = None,
    **extras,
) -> str:
    """Build a [RTD][THREAD=...][FRAME=...][STEP=...][Xms] tag string."""
    t = thread or threading.current_thread().name
    base = f'[RTD][THREAD={t}][FRAME={frame_id}][STEP={step}][{elapsed_ms:.1f}ms]'
    if extras:
        kv = '  '.join(f'{k}={v}' for k, v in extras.items())
        return f'{base}  {kv}'
    return base


# ── Per-stage statistics ─────────────────────────────────────────────────────

class StageStats:
    """Ring-buffer stats (avg / max / p95) for one named pipeline stage."""

    def __init__(self, window: int = 200):
        self._buf: collections.deque = collections.deque(maxlen=window)
        self._running_max = 0.0

    def record(self, ms: float) -> None:
        self._buf.append(ms)
        if ms > self._running_max:
            self._running_max = ms

    @property
    def avg(self) -> float:
        return (sum(self._buf) / len(self._buf)) if self._buf else 0.0

    @property
    def max(self) -> float:
        return self._running_max

    @property
    def p95(self) -> float:
        if not self._buf:
            return 0.0
        return float(np.percentile(list(self._buf), 95))

    @property
    def last(self) -> float:
        return self._buf[-1] if self._buf else 0.0


# ── Multi-stage profiler ─────────────────────────────────────────────────────

class RtdProfiler:
    """Aggregates per-stage timings, FPS, slow-frame detection, and summaries."""

    SLOW_MS = 100.0

    def __init__(self, window: int = 200):
        self._stages: Dict[str, StageStats] = {}
        self._total    = StageStats(window)
        self._fps_buf: collections.deque = collections.deque(maxlen=window)
        self._dropped  = 0
        self._slow     = 0
        self._n_frames = 0
        self._lock     = threading.Lock()
        self._last_summary_t = time.monotonic()

    # Context-manager factory for a named stage
    def stage(self, name: str) -> '_StageCM':
        with self._lock:
            if name not in self._stages:
                self._stages[name] = StageStats()
        return _StageCM(self._stages[name], name)

    def record_frame(self, total_ms: float) -> None:
        with self._lock:
            self._total.record(total_ms)
            self._n_frames += 1
            self._fps_buf.append(time.perf_counter())
            if total_ms > self.SLOW_MS:
                self._slow += 1

    def record_drop(self) -> None:
        with self._lock:
            self._dropped += 1

    @property
    def fps(self) -> float:
        with self._lock:
            if len(self._fps_buf) < 2:
                return 0.0
            dt = self._fps_buf[-1] - self._fps_buf[0]
            return (len(self._fps_buf) - 1) / dt if dt > 0 else 0.0

    def should_print_summary(self, interval_s: float = 1.0) -> bool:
        now = time.monotonic()
        if now - self._last_summary_t >= interval_s:
            self._last_summary_t = now
            return True
        return False

    def summary(self, extra_lines: Optional[List[str]] = None) -> str:
        lines = ['=== RTD PERF ===']
        lines.append(f'FPS: {self.fps:.1f}')
        with self._lock:
            n, d, s = self._n_frames, self._dropped, self._slow
            lines.append(
                f'Frames: {n}  Dropped: {d} ({100*d/max(1,n):.1f}%)  '
                f'Slow>{self.SLOW_MS:.0f}ms: {s}')
            for nm, st in self._stages.items():
                lines.append(
                    f'  {nm:<24s} avg={st.avg:7.1f}ms  '
                    f'max={st.max:8.1f}ms  p95={st.p95:7.1f}ms')
            lines.append(
                f'  {"total":<24s} avg={self._total.avg:7.1f}ms  '
                f'max={self._total.max:8.1f}ms  p95={self._total.p95:7.1f}ms')
        if extra_lines:
            lines.extend(extra_lines)
        lines.append('=' * 20)
        return '\n'.join(lines)


class _StageCM:
    def __init__(self, stats: StageStats, name: str):
        self._st = stats
        self.name = name
        self.elapsed_ms = 0.0
        self._t0 = 0.0

    def __enter__(self) -> '_StageCM':
        self._t0 = time.perf_counter()
        return self

    def __exit__(self, *_) -> None:
        self.elapsed_ms = (time.perf_counter() - self._t0) * 1000.0
        self._st.record(self.elapsed_ms)


# ── Per-frame timeline ───────────────────────────────────────────────────────

class FrameTrace:
    """Lightweight timeline for a single depth frame: marks stages + elapsed."""

    def __init__(self, frame_id: int):
        self.frame_id = frame_id
        self._t0 = time.perf_counter()
        self._events: List[Tuple[str, float]] = []

    def mark(self, stage: str) -> float:
        """Record a stage event and return elapsed ms since frame start."""
        ms = (time.perf_counter() - self._t0) * 1000.0
        self._events.append((stage, ms))
        return ms

    @property
    def total_ms(self) -> float:
        return (time.perf_counter() - self._t0) * 1000.0

    def summary(self) -> str:
        parts = [f'F={self.frame_id}']
        for stage, ms in self._events:
            parts.append(f'{stage}@{ms:.1f}ms')
        parts.append(f'total={self.total_ms:.1f}ms')
        return '  '.join(parts)


# ── Thread heartbeat ─────────────────────────────────────────────────────────

class ThreadHeartbeat:
    """Threads call beat() periodically; watchers call age() to detect stalls."""

    def __init__(self):
        self._beats: Dict[str, float] = {}
        self._lock = threading.Lock()

    def beat(self, name: Optional[str] = None) -> None:
        key = name or threading.current_thread().name
        with self._lock:
            self._beats[key] = time.monotonic()

    def age(self, name: str) -> float:
        """Seconds since last beat for this thread (inf if never beat)."""
        with self._lock:
            t = self._beats.get(name)
        return (time.monotonic() - t) if t is not None else float('inf')

    def all_ages(self) -> Dict[str, float]:
        now = time.monotonic()
        with self._lock:
            return {k: now - v for k, v in self._beats.items()}


# ── Freeze detector ──────────────────────────────────────────────────────────

class FreezeDetector:
    """Daemon watchdog: calls dump_fn when any thread exceeds its heartbeat threshold."""

    def __init__(
        self,
        heartbeat: ThreadHeartbeat,
        thresholds_s: Dict[str, float],
        dump_fn: Callable[[str], None],
        poll_s: float = 0.5,
    ):
        self._hb      = heartbeat
        self._thresh  = thresholds_s
        self._dump    = dump_fn
        self._fired: Dict[str, bool] = {k: False for k in thresholds_s}
        t = threading.Thread(target=self._run, name='rtd_freeze_wdog', daemon=True)
        t.start()

    def _run(self) -> None:
        while True:
            time.sleep(0.5)
            for name, thr in self._thresh.items():
                age   = self._hb.age(name)
                fired = self._fired.get(name, False)
                if age > thr and not fired:
                    self._fired[name] = True
                    self._dump(
                        f'FREEZE DETECTED: thread={name} '
                        f'age={age:.2f}s > threshold={thr:.1f}s'
                    )
                elif age <= thr and fired:
                    self._fired[name] = False  # recovered


# ── Thread / stack dump ──────────────────────────────────────────────────────

def dump_all_threads(context: str = '') -> str:
    sep = '=' * 60
    lines = [f'\n{sep}', f'THREAD DUMP — {context}', sep]
    frames = sys._current_frames()
    for t in threading.enumerate():
        frame = frames.get(t.ident)
        lines.append(
            f'\n[Thread: {t.name}  id={t.ident}  '
            f'alive={t.is_alive()}  daemon={t.daemon}]'
        )
        if frame:
            lines.extend(traceback.format_stack(frame))
        else:
            lines.append('  (no frame captured)')
    lines.append(sep)
    return '\n'.join(lines)


def dump_faulthandler(context: str = '') -> str:
    buf = io.StringIO()
    faulthandler.dump_traceback(file=buf, all_threads=True)
    return f'[faulthandler] {context}\n{buf.getvalue()}'


def install_faulthandler() -> None:
    """Install faulthandler to stderr (safe to call multiple times)."""
    faulthandler.enable()


# ── Memory tracker ───────────────────────────────────────────────────────────

class MemoryTracker:
    """Thin wrapper around tracemalloc."""

    def __init__(self, enabled: bool = True, top_n: int = 5):
        self._enabled = enabled
        self._top_n   = top_n
        if enabled:
            tracemalloc.start()

    def snapshot_str(self) -> str:
        if not self._enabled:
            return ''
        cur, peak = tracemalloc.get_traced_memory()
        return f'mem current={cur/1024:.1f}KB  peak={peak/1024:.1f}KB'

    def top_allocations(self) -> str:
        if not self._enabled:
            return ''
        snap  = tracemalloc.take_snapshot()
        stats = snap.statistics('lineno')[: self._top_n]
        lines = ['[tracemalloc top allocs]']
        lines.extend(str(s) for s in stats)
        return '\n'.join(lines)

    @staticmethod
    def array_mb(arr: Optional[np.ndarray]) -> float:
        return (arr.nbytes / 1_048_576) if arr is not None else 0.0


# ── Queue saturation monitor ─────────────────────────────────────────────────

class QueueMonitor:
    """Wraps drop-oldest queue put/get with saturation tracking."""

    def __init__(self, q, name: str):
        self._q     = q
        self.name   = name
        self._drops = 0
        self._puts  = 0
        self._get_wait: collections.deque = collections.deque(maxlen=100)

    def put_drop_oldest(self, item) -> bool:
        """Drop-replace put; returns True if an item was dropped."""
        import queue
        self._puts += 1
        try:
            self._q.put_nowait(item)
            return False
        except queue.Full:
            try:
                self._q.get_nowait()
            except queue.Empty:
                pass
            self._q.put_nowait(item)
            self._drops += 1
            return True

    def get(self, timeout: Optional[float] = None):
        t0   = time.perf_counter()
        item = self._q.get(timeout=timeout)
        self._get_wait.append((time.perf_counter() - t0) * 1000.0)
        return item

    @property
    def size(self) -> int:
        return self._q.qsize()

    @property
    def drops(self) -> int:
        return self._drops

    @property
    def puts(self) -> int:
        return self._puts

    @property
    def avg_get_wait_ms(self) -> float:
        return (sum(self._get_wait) / len(self._get_wait)) if self._get_wait else 0.0

    def stats_str(self) -> str:
        drop_pct = 100 * self._drops / max(1, self._puts)
        return (
            f'{self.name}: size={self.size} '
            f'puts={self._puts} drops={self._drops}({drop_pct:.1f}%) '
            f'avg_get_wait={self.avg_get_wait_ms:.1f}ms'
        )
