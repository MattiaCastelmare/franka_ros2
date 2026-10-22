"""pytest for franka_simulation's pure-logic modules (scripts/utils/).

The scripts are installed as plain programs with ``utils/`` beside them, so the
tests put ``scripts/`` on sys.path the same way ``ros2 run`` resolves it.
"""
import os
import sys

SCRIPTS = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'scripts')
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)
