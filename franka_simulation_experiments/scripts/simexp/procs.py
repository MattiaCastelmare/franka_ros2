"""Start a ``ros2 launch`` in its own process group, and make sure it ends.

Gazebo is a ruby wrapper around a server, a bridge per topic and a controller
manager inside the server. A single SIGINT to the launch process leaves
orphans often enough to matter, and an orphaned server keeps publishing
``/clock`` into the next run. So every launch here gets its own session
(``setsid``) and is torn down as a group: SIGINT, then SIGTERM, then SIGKILL.
Afterwards, any process still carrying this run's ``IGN_PARTITION`` in its
environment is killed too.
"""
from __future__ import annotations

import os
import signal
import subprocess
import time


def _pids_with_env(key: str, value: str) -> list[int]:
    out = []
    needle = f'{key}={value}'.encode()
    for d in os.listdir('/proc'):
        if not d.isdigit() or int(d) == os.getpid():
            continue
        try:
            with open(f'/proc/{d}/environ', 'rb') as f:
                if needle in f.read().split(b'\0'):
                    out.append(int(d))
        except OSError:
            continue
    return out


class Launch:
    """``with Launch(['pkg', 'file.launch.py', 'a:=b'], log, env) as L: ...``"""

    def __init__(self, args, log_path: str, env: dict | None = None,
                 partition: str | None = None):
        self.partition = partition or f'fr3cbf_{os.getpid()}_{int(time.time() * 1e3) % 100000}'
        self.env = dict(os.environ, **(env or {}))
        self.env['IGN_PARTITION'] = self.partition
        self.args = ['ros2', 'launch', *args]
        self.log_path = log_path
        self.proc = None

    def start(self) -> 'Launch':
        os.makedirs(os.path.dirname(self.log_path) or '.', exist_ok=True)
        self._log = open(self.log_path, 'w')
        self.proc = subprocess.Popen(self.args, stdout=self._log, stderr=subprocess.STDOUT,
                                     env=self.env, start_new_session=True)
        return self

    def alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def stop(self, grace_s: float = 20.0) -> None:
        if self.proc is None:
            return
        pgid = self.proc.pid
        for sig, wait in ((signal.SIGINT, grace_s), (signal.SIGTERM, 5.0),
                          (signal.SIGKILL, 5.0)):
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                break
            t_end = time.time() + wait
            while time.time() < t_end:
                if self.proc.poll() is not None and not _group_alive(pgid):
                    break
                time.sleep(0.2)
            else:
                continue
            break
        for pid in _pids_with_env('IGN_PARTITION', self.partition):
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass
        self._log.close()

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()
        return False


def _group_alive(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def leftovers(partition: str) -> list[int]:
    """PIDs still running with this partition (for tests asserting a clean stop)."""
    return _pids_with_env('IGN_PARTITION', partition)
