"""Where the run logs go: ``runs/`` in the package SOURCE folder, wherever you launch from."""
import os

from ament_index_python.packages import get_package_share_directory


def runs_dir() -> str:
    """``<source>/fr3_student_sim/runs`` when built with --symlink-install (the
    installed files are links to the source), else ``~/fr3_student_sim_runs``."""
    share = get_package_share_directory('fr3_student_sim')
    src = os.path.dirname(os.path.realpath(os.path.join(share, 'package.xml')))
    if os.path.realpath(share) in (src, os.path.dirname(src)):     # a plain copy, no links
        return os.path.expanduser('~/fr3_student_sim_runs')
    return os.path.join(src, 'runs')
