"""R1 + R2: the CBF filter's executor is configurable (``executor_mode``).

R1 made it configurable with the default unchanged (``multi``); R2 made
``single`` the shipped default (docs/framework_study/refactoring/
R2_SINGLE_EXECUTOR_DEFAULT.md). What is pinned here, and why:

* the shipped default is ``single`` (R2), and ``main()`` with the shipped
  configuration really spins a ``SingleThreadedExecutor``; the installed copy
  of ``fr3_control.yaml`` (the one the node reads) says the same as the source;
* ``multi`` stays selectable explicitly (a ROS parameter override, as
  ``cbf_param_overrides:=executor_mode=multi`` does), and builds exactly the
  historical executor: ``MultiThreadedExecutor`` with 3 workers;
* ``single`` builds a ``SingleThreadedExecutor``;
* invalid values fail LOUDLY, both where launches set them (a ROS parameter
  override through ``declare_from_spec``) and in ``make_executor`` itself, and
  ``single`` together with a SCHED_FIFO priority is refused;
* ``main()`` still starts, spins and shuts down the node in the same order, and
  cleans up when the executor cannot be built;
* ``executor_mode`` cannot reach the control path: no code outside
  ``main``/``make_executor`` reads it.

That the CONTROL OUTPUT is unchanged is shown on recorded data, not here: the
S6B bag replayed through the baseline install and through the R1 install, in
both modes, gives bit-identical per-tick commands
(docs/framework_study/refactoring/R1_EXECUTOR_CONFIGURATION.md).
"""
import ast
import os
import types

import pytest
import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

import franka_experiments.nodes.cbf_safety_filter as M
from franka_experiments.utils.config import CBF_PARAM_SPEC
from franka_experiments.utils.params import declare_from_spec

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
YAML = os.path.join(PKG, 'config', 'fr3_control.yaml')
SPEC = {'executor_mode': CBF_PARAM_SPEC['executor_mode']}


@pytest.fixture
def ros():
    """An initialised default context for the duration of one test."""
    rclpy.init()
    try:
        yield
    finally:
        if rclpy.ok():
            rclpy.shutdown()


# ── configuration ──────────────────────────────────────────────────────────

def test_shipped_default_is_single():
    """R2: the source configuration ships ``single``."""
    with open(YAML) as f:
        params = yaml.safe_load(f)['params']
    assert params['executor_mode'] == 'single'
    # 'single' refuses a SCHED_FIFO priority, so the shipped default can only
    # start if the shipped priority is 0 (make_executor).
    assert params['qp_thread_fifo_prio'] == 0


def test_spec_and_helper_agree_on_the_modes():
    kind, kw = CBF_PARAM_SPEC['executor_mode']
    assert kind == 'str'
    assert tuple(kw['choices']) == M.EXECUTOR_MODES == ('multi', 'single')


def test_ros_override_selects_single(ros):
    node = Node('r1_override_single', parameter_overrides=[
        Parameter('executor_mode', value='single')])
    try:
        P = declare_from_spec(node, {'executor_mode': 'multi'}, SPEC)
        assert P.executor_mode == 'single'
    finally:
        node.destroy_node()


def test_ros_override_selects_multi_over_a_single_yaml(ros):
    """R2: the previous executor stays selectable without editing the YAML."""
    node = Node('r2_override_multi', parameter_overrides=[
        Parameter('executor_mode', value='multi')])
    try:
        P = declare_from_spec(node, {'executor_mode': 'single'}, SPEC)
        assert P.executor_mode == 'multi'
    finally:
        node.destroy_node()


def test_yaml_value_is_used_without_override(ros):
    node = Node('r1_no_override')
    try:
        assert declare_from_spec(node, {'executor_mode': 'multi'}, SPEC).executor_mode == 'multi'
    finally:
        node.destroy_node()


@pytest.mark.parametrize('bad', ['bogus', 'Multi', 'SINGLE', '', 'multi '])
def test_invalid_override_refuses_to_start(ros, bad):
    node = Node('r1_override_bad', parameter_overrides=[
        Parameter('executor_mode', value=bad)])
    try:
        with pytest.raises(ValueError, match='executor_mode'):
            declare_from_spec(node, {'executor_mode': 'multi'}, SPEC)
    finally:
        node.destroy_node()


def test_missing_key_refuses_to_start(ros):
    node = Node('r1_missing')
    try:
        with pytest.raises(KeyError, match='executor_mode'):
            declare_from_spec(node, {}, SPEC)
    finally:
        node.destroy_node()


# ── make_executor ─────────────────────────────────────────────────────────

def test_multi_is_the_historical_three_thread_executor(ros):
    ex = M.make_executor('multi')
    try:
        assert type(ex) is MultiThreadedExecutor
        assert ex._executor._max_workers == 3
    finally:
        ex.shutdown()


def test_multi_keeps_accepting_a_fifo_priority(ros):
    # Historical behaviour: the FIFO knob is independent of the executor.
    ex = M.make_executor('multi', qp_thread_fifo_prio=50)
    try:
        assert type(ex) is MultiThreadedExecutor
    finally:
        ex.shutdown()


def test_single_builds_a_single_threaded_executor(ros):
    ex = M.make_executor('single')
    try:
        assert type(ex) is SingleThreadedExecutor
    finally:
        ex.shutdown()


@pytest.mark.parametrize('bad', ['bogus', '', None, 3])
def test_make_executor_rejects_unknown_modes(ros, bad):
    with pytest.raises(ValueError, match='executor_mode'):
        M.make_executor(bad)


def test_single_with_fifo_priority_is_refused(ros):
    with pytest.raises(ValueError, match='qp_thread_fifo_prio'):
        M.make_executor('single', qp_thread_fifo_prio=1)


# ── main(): start-up and shutdown ─────────────────────────────────────────

class _StubFilter(Node):
    """Stands in for CBFSafetyFilter: main() only needs P and a node."""
    mode = 'multi'
    fifo = 0
    destroyed = False

    def __init__(self):
        super().__init__('r1_stub_filter')
        self.P = types.SimpleNamespace(executor_mode=type(self).mode,
                                       qp_thread_fifo_prio=type(self).fifo)

    def destroy_node(self):
        type(self).destroyed = True
        return super().destroy_node()


def _run_main(monkeypatch, mode, fifo=0):
    _StubFilter.mode, _StubFilter.fifo, _StubFilter.destroyed = mode, fifo, False
    monkeypatch.setattr(M, 'CBFSafetyFilter', _StubFilter)
    spun = {}

    def fake_spin(self):
        spun['type'] = type(self)
        spun['nodes'] = list(self.get_nodes())
        spun['workers'] = getattr(getattr(self, '_executor', None), '_max_workers', None)

    monkeypatch.setattr(MultiThreadedExecutor, 'spin', fake_spin)
    monkeypatch.setattr(SingleThreadedExecutor, 'spin', fake_spin)
    assert not rclpy.ok()
    M.main(args=[])
    return spun


def test_main_multi_spins_the_three_thread_executor(monkeypatch):
    spun = _run_main(monkeypatch, 'multi')
    assert spun['type'] is MultiThreadedExecutor and spun['workers'] == 3
    assert len(spun['nodes']) == 1 and isinstance(spun['nodes'][0], _StubFilter)
    assert _StubFilter.destroyed and not rclpy.ok()


def test_main_single_spins_a_single_threaded_executor(monkeypatch):
    spun = _run_main(monkeypatch, 'single')
    assert spun['type'] is SingleThreadedExecutor
    assert len(spun['nodes']) == 1
    assert _StubFilter.destroyed and not rclpy.ok()


def _run_real_main(monkeypatch, args):
    """``main()`` with the REAL filter (installed configuration), spin stubbed.

    The node is constructed exactly as ``ros2 run`` constructs it, so
    ``executor_mode`` comes from the installed ``fr3_control.yaml`` plus any
    ``--ros-args`` override; only ``spin`` is replaced, so no callback runs.
    """
    spun = {}

    def fake_spin(self):
        spun['type'] = type(self)
        spun['workers'] = getattr(getattr(self, '_executor', None), '_max_workers', None)
        nodes = list(self.get_nodes())
        spun['n_nodes'] = len(nodes)
        spun['node_type'] = type(nodes[0]).__name__
        spun['mode'] = nodes[0].P.executor_mode
        spun['config_path'] = nodes[0].P.config_path

    monkeypatch.setattr(MultiThreadedExecutor, 'spin', fake_spin)
    monkeypatch.setattr(SingleThreadedExecutor, 'spin', fake_spin)
    assert not rclpy.ok()
    M.main(args=args)
    assert not rclpy.ok()
    return spun


def test_main_with_the_shipped_configuration_spins_a_single_threaded_executor(monkeypatch):
    """R2 end to end: no override -> SingleThreadedExecutor, from the INSTALLED YAML.

    Also catches a stale install: the node reads the copy colcon installed
    (``P.config_path``), not the source file, so the two must agree.
    """
    spun = _run_real_main(monkeypatch, [])
    assert spun['type'] is SingleThreadedExecutor
    assert (spun['n_nodes'], spun['node_type'], spun['mode']) == (1, 'CBFSafetyFilter', 'single')
    with open(spun['config_path']) as f:
        installed = yaml.safe_load(f)['params']['executor_mode']
    with open(YAML) as f:
        source = yaml.safe_load(f)['params']['executor_mode']
    assert installed == source == 'single', (spun['config_path'], installed, source)


def test_main_explicit_multi_override_spins_the_three_thread_executor(monkeypatch):
    """R2 rollback path: ``-p executor_mode:=multi`` still gives the historical MT(3)."""
    spun = _run_real_main(monkeypatch, ['--ros-args', '-p', 'executor_mode:=multi'])
    assert spun['type'] is MultiThreadedExecutor and spun['workers'] == 3
    assert (spun['n_nodes'], spun['node_type'], spun['mode']) == (1, 'CBFSafetyFilter', 'multi')


def test_main_refusal_cleans_up(monkeypatch):
    with pytest.raises(ValueError, match='qp_thread_fifo_prio'):
        _run_main(monkeypatch, 'single', fifo=10)
    assert _StubFilter.destroyed and not rclpy.ok()


# ── the mode cannot reach the control path ─────────────────────────────────

def test_executor_mode_is_read_only_by_main():
    src_root = os.path.join(PKG, 'franka_experiments')
    readers = []
    for dirpath, _, files in os.walk(src_root):
        for f in files:
            if not f.endswith('.py'):
                continue
            path = os.path.join(dirpath, f)
            tree = ast.parse(open(path).read(), path)
            for fn in ast.walk(tree):
                if not isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    continue
                for n in ast.walk(fn):
                    hit = (isinstance(n, ast.Attribute) and n.attr == 'executor_mode') or \
                          (isinstance(n, ast.Constant) and n.value == 'executor_mode')
                    if hit:
                        readers.append((os.path.relpath(path, src_root), fn.name))
    assert set(readers) == {('nodes/cbf_safety_filter.py', 'main')}, readers
