"""Put scripts/ (simexp, the nodes) and franka_simulation's scripts/ (utils) on sys.path,
the way the installed programs resolve them."""
import os
import sys

HERE = os.path.dirname(os.path.realpath(__file__))
for p in (os.path.join(HERE, '..', 'scripts'),
          os.path.join(HERE, '..', '..', 'franka_simulation', 'scripts')):
    p = os.path.normpath(p)
    if p not in sys.path:
        sys.path.insert(0, p)
