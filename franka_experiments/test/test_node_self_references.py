"""Every ``self._foo(...)`` a node calls must actually be defined.

Written after a real outage: a patch added
``create_subscription(..., self._cbf_status_cb, ...)`` and
``self._governor_sigma(t)`` to pentagon_qddot_commander but the edit that was
supposed to insert the two method bodies silently did nothing (``str.replace``
does not raise when its pattern is absent). ``__init__`` then died on
``AttributeError``, the commander never published ``qddot_nom``, the CBF filter
braked forever, and the arm did not move at all.

Nothing caught it: the file still parsed, and a name-resolution sweep does not
see attribute access. This test does — statically, with no ROS, no robot and no
imports, so it runs anywhere.

Scope is deliberately narrow: only ``self._underscore`` CALLS, which are ours by
convention. Public ``self.foo()`` is left alone because it may be inherited from
rclpy's Node.
"""

import ast
import os

NODES_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                         'franka_experiments', 'nodes')
UTILS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                         'franka_experiments', 'utils')


def _class_defs(cls: ast.ClassDef):
    """Names bound directly in the class body: methods AND class constants.

    Class-level constants matter: ``_STOP_RAMP = ...`` in the class body is read
    as ``self._STOP_RAMP`` and is perfectly legitimate.
    """
    out = {n.name for n in cls.body
           if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))}
    for n in cls.body:
        if isinstance(n, ast.Assign):
            for t in n.targets:
                if isinstance(t, ast.Name):
                    out.add(t.id)
        elif isinstance(n, ast.AnnAssign) and isinstance(n.target, ast.Name):
            out.add(n.target.id)
    return out


def _assigned_attrs(cls: ast.ClassDef):
    """Every ``self.x`` ever WRITTEN anywhere in the class.

    Must recurse into tuple/list targets: ``self._model, self._data = f()`` is an
    Assign whose single target is a Tuple, and treating only bare Attribute
    targets reports both halves as undefined. Also covers ``for self.x in ...``
    and ``with ... as self.x``, which bind the same way.
    """
    out = set()

    def _collect(t):
        if isinstance(t, ast.Attribute):
            if isinstance(t.value, ast.Name) and t.value.id == 'self':
                out.add(t.attr)
        elif isinstance(t, (ast.Tuple, ast.List)):
            for e in t.elts:
                _collect(e)
        elif isinstance(t, ast.Starred):
            _collect(t.value)

    for node in ast.walk(cls):
        if isinstance(node, ast.Assign):
            for t in node.targets:
                _collect(t)
        elif isinstance(node, (ast.AnnAssign, ast.AugAssign)):
            _collect(node.target)
        elif isinstance(node, (ast.For, ast.AsyncFor)):
            _collect(node.target)
        elif isinstance(node, ast.withitem) and node.optional_vars is not None:
            _collect(node.optional_vars)
    return out


def _referenced_self_attrs(cls: ast.ClassDef):
    """Every ``self._foo`` READ → {name: [line, ...]}.

    Reads, not just calls: the outage this test exists for was a callback passed
    BY REFERENCE — ``create_subscription(..., self._cbf_status_cb, 10)`` — which
    is an attribute load, not a Call node. Checking only call sites would have
    missed it exactly as the original review did.
    """
    refs = {}
    for node in ast.walk(cls):
        if (isinstance(node, ast.Attribute)
                and isinstance(node.ctx, ast.Load)
                and isinstance(node.value, ast.Name) and node.value.id == 'self'
                and node.attr.startswith('_')
                and not node.attr.startswith('__')):
            refs.setdefault(node.attr, []).append(node.lineno)
    return refs


def _sources(directory):
    for fn in sorted(os.listdir(directory)):
        if fn.endswith('.py') and not fn.startswith('__'):
            yield os.path.join(directory, fn)


def _check(path):
    """Return a list of human-readable problems for one source file."""
    with open(path, encoding='utf-8') as fh:
        tree = ast.parse(fh.read(), path)
    problems = []
    for cls in [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]:
        known = _class_defs(cls) | _assigned_attrs(cls)
        # Anything inherited: a base class we cannot see statically means we
        # cannot be sure, so only flag classes whose bases are Node/object-like.
        base_names = {b.id if isinstance(b, ast.Name)
                      else getattr(b, 'attr', '') for b in cls.bases}
        if not base_names <= {'Node', 'object', ''}:
            continue
        for name, lines in _referenced_self_attrs(cls).items():
            if name not in known:
                problems.append(
                    f'{os.path.basename(path)}:{lines[0]} '
                    f'{cls.name}.{name} is used but never defined or assigned'
                    + (f' (also at lines {lines[1:]})' if len(lines) > 1 else ''))
    return problems


def test_every_node_self_call_resolves():
    problems = []
    for path in _sources(NODES_DIR):
        problems += _check(path)
    assert not problems, (
        'undefined self-methods (the node would die on AttributeError at '
        'construction or on the first tick):\n  ' + '\n  '.join(problems))


def test_every_util_self_call_resolves():
    problems = []
    for path in _sources(UTILS_DIR):
        problems += _check(path)
    assert not problems, ('undefined self-methods:\n  ' + '\n  '.join(problems))


def test_the_checker_actually_catches_the_outage():
    """Guard the guard: reproduce the exact shape of the bug it exists for."""
    import tempfile
    src = '''
class Thing(Node):
    def __init__(self):
        self.create_subscription(X, 't', self._never_defined, 10)

    def _tick(self):
        v = self._also_missing(1.0)
        return self._present() + v

    def _present(self):
        return 0
'''
    with tempfile.NamedTemporaryFile('w', suffix='.py', delete=False) as fh:
        fh.write(src)
        tmp = fh.name
    try:
        problems = _check(tmp)
        names = ' '.join(problems)
        assert '_never_defined' in names, (
            'a callback passed BY REFERENCE must be caught — that is the exact '
            f'shape of the outage: {problems}')
        assert '_also_missing' in names, problems
        assert '_present' not in names, 'defined methods must not be flagged'
    finally:
        os.unlink(tmp)


def test_the_checker_accepts_callables_stored_on_self():
    """A method held in an attribute is legitimate and must not be flagged."""
    import tempfile
    src = '''
class Thing(Node):
    def __init__(self):
        self._hook = lambda x: x

    def _tick(self):
        return self._hook(1)
'''
    with tempfile.NamedTemporaryFile('w', suffix='.py', delete=False) as fh:
        fh.write(src)
        tmp = fh.name
    try:
        assert _check(tmp) == []
    finally:
        os.unlink(tmp)


def test_the_checker_accepts_tuple_unpacking_and_class_constants():
    """Both produced false positives on the first version of this check."""
    import tempfile
    src = '''
class Thing(Node):
    _STOP = 3

    def __init__(self):
        self._model, self._data = build()
        for self._item in range(3):
            pass

    def _tick(self):
        return self._model + self._data + self._STOP + self._item
'''
    with tempfile.NamedTemporaryFile('w', suffix='.py', delete=False) as fh:
        fh.write(src)
        tmp = fh.name
    try:
        assert _check(tmp) == [], _check(tmp)
    finally:
        os.unlink(tmp)


if __name__ == '__main__':
    import pytest
    raise SystemExit(pytest.main([__file__, '-v', '-s']))
