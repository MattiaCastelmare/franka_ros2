"""Run a process with its output in a file, and always stop it cleanly.

    python3 -m fr3_student_sim.quiet_process <log file> <command...>

Used by the launch for Gazebo: the Franka Gazebo plugin aborts in its
destructor at every shutdown (an upstream bug, after the run is over and the
log saved), which would end each run with ~60 lines of stack trace and
"process has died", and hands a core dump of Gazebo to the crash reporter,
which delays the shutdown by more than 15 s. Here the command runs in its own
process group with its output in the log file; on Ctrl+C or shutdown (SIGINT /
SIGTERM) the group is killed at once, so nothing outlives the launch. The exit
code is always 0.
"""
import os
import signal
import subprocess
import sys


def main():
    log_path, cmd = sys.argv[1], sys.argv[2:]
    with open(log_path, 'w') as log:
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)

        def stop(*_):
            # SIGKILL at once: the run is already over and saved, and a graceful
            # stop only reaches the plugin's abort and a slow core dump. SIGKILL
            # cannot be ignored, so there is nothing to wait for (and waiting
            # here, inside the interrupted proc.wait(), would block on its lock).
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            os._exit(0)

        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        proc.wait()
    os._exit(0)


if __name__ == '__main__':
    main()
