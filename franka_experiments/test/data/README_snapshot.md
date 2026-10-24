# `torque_control_stack_94cd7b0_snapshot.json`

Golden expansion of `launch/torque_control_stack.launch.py::_launch_all`, made with
`test/_launch_snapshot.py` from the file at commit `94cd7b0`, the last commit before
`external_plant` / `motion_source:=none` existed. Variants: defaults,
`use_fake_hardware:=true` and `motion_source:=rl`. It is read by
`test_torque_stack_external_plant_off_identical.py`.

The snapshot depends on the environment it was made in: the installed packages and
the shipped YAMLs (`launch_defaults.yaml`, `fr3_complete.yaml`). Temp files are stored
as content hashes. So it also fails when one of those YAMLs changes. That is
intended: the launch then really does start something different.

Regenerate it after a **deliberate** change, inside the container, from
`franka_experiments/test`:

```bash
python3 - <<'PY'
import json, sys; sys.path.insert(0, '.')
from _launch_snapshot import snapshot
src = '../launch/torque_control_stack.launch.py'   # or a `git show <sha>:...` copy
variants = {'defaults': {}, 'fake_hardware': {'use_fake_hardware': 'true'},
            'rl': {'motion_source': 'rl'}}
json.dump({'source': 'franka_experiments/launch/torque_control_stack.launch.py @ <sha>',
           'variants': variants,
           'snapshots': {k: snapshot(src, **v) for k, v in variants.items()}},
          open('data/torque_control_stack_94cd7b0_snapshot.json', 'w'),
          indent=1, sort_keys=True)
PY
```

Review the JSON diff before committing it.
