"""Reduced-filter dry run: owner contracts, non-commanding composition and stack. No ROS graph or hardware."""
import ast
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from catheter_control.orchestration.configuration import ACTIVE_CONTROLLER_PARAMETERS, locate_stack, resolve_stack
from catheter_control.orchestration.reduced_filter_owner import ReducedFilterOwner, encoder_sample
from catheter_control.planning.mppi import CatheterMppi, MppiConfig
from catheter_control.orchestration.reduced_filter_settings import reduced_filter_contract
from catheter_control.safety.hardware_contract import load_hardware_contract
from control_interface_py.encoder_coordinates import counts_to_logical

PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE/'catheter_control'/'reduced_filter_dry_run.py'
CONFIG = PACKAGE/'config'
LIMITS = PACKAGE.parent/'control_interface'/'config'/'catheter_limits.yaml'


class Aligner:

    def __init__(self):
        self.history, self.pending, self.committed, self.history_lost = [], [], None, False

    def add_encoder(self, stamp, shaft, tendon):
        self.history.append((stamp, shaft, tendon))

    def add_image(self, stamp, points):
        self.pending.append(stamp)
        return True


class Stream:
    """Scripted ReducedFilterStream: process() returns queued (status, committed stamp[, law count])."""

    def __init__(self, results, maximum_laws=None):
        self.aligner, self.results, self.estimator = Aligner(), list(results), None
        self.options = {} if maximum_laws is None else {'maximum_laws': maximum_laws}

    def process(self):
        if not self.results:
            return None
        status, committed, *laws = self.results.pop(0)
        if status in ('initialized', 'tracking'):
            self.aligner.committed = (committed, None, None)
            self.estimator = SimpleNamespace(laws=SimpleNamespace(means=np.zeros((laws[0] if laws else 1, 2))),
                                             options=self.options)
        if self.aligner.pending:
            self.aligner.pending.pop(0)
        return SimpleNamespace(status=status, reason=None if status == 'tracking' else status,
                               consecutive_rejections=int(status == 'holding'))


class Backend:

    def __init__(self, refuse=False):
        self.prepared, self.roots, self.refuse, self.kernels = [], [], refuse, set()
        self.stream = None

    def root(self, stream, plan_ns):
        if self.refuse:
            raise ValueError('stale')
        self.roots.append(plan_ns)
        return SimpleNamespace(plan_ns=plan_ns, law_count=len(stream.estimator.laws.means))

    def example_root(self, stream, plan_ns, law_count):
        self.roots.append(plan_ns)
        return SimpleNamespace(plan_ns=plan_ns, law_count=law_count, example=True)

    def prepare(self, root, counts, step_s):
        self.prepared.append((root.plan_ns, root.law_count, list(counts), step_s))
        self.kernels |= {(c, root.law_count) for c in counts}

    def prepared_for(self, root, counts):
        return all((c, root.law_count) in self.kernels for c in counts)


def owner(results, backend=None, maximum_laws=None):
    return ReducedFilterOwner(Stream(results, maximum_laws), backend or Backend(), candidate_counts=(512, 513), step_s=.2,
                              clock=iter(np.arange(0, 100, .001)).__next__)


COUNTS = [26850., 0., -68642., 0., 0., 0.]


def test_kernels_prepare_after_initialization_and_readiness_needs_catch_up():
    o = owner([('initialized', 10), ('tracking', 20)])
    o.add_encoder(15, COUNTS)
    assert o.health().estimator_health == 'INITIALIZING' and not o.health().ready
    o.add_image(10, None)
    o.add_image(20, None)
    o.process()
    assert o.backend.prepared == [(10, 1, [512, 513], .2)] and not o.health().ready
    o.process()
    assert o.health() == o.health().__class__('TRACKING', True, None, 0)


def test_each_new_law_count_is_prepared_in_the_owner_before_planning():
    o = owner([('initialized', 10, 1), ('tracking', 20, 2), ('tracking', 30, 2), ('tracking', 40, 1)])
    o.add_encoder(45, COUNTS)
    for _ in range(4):
        o.process()
    assert [(row[0], row[1]) for row in o.backend.prepared] == [(10, 1), (20, 2)]
    assert o.health().ready and o.plan_root().reason is None
    o.backend.kernels.clear()
    assert o.plan_root().reason == 'kernel_not_prepared'


def test_every_law_count_is_prepared_at_the_first_step():
    backend = Backend()
    o = owner([('initialized', 10, 1), ('tracking', 20, 2), ('tracking', 30, 1)], backend, maximum_laws=2)
    o.add_encoder(35, COUNTS)
    o.process()
    assert [(row[0], row[1]) for row in backend.prepared] == [(10, 1), (10, 2)]   # current count from the real root
    assert [count for count, _ in o.preparations] == [1, 2]
    o.process()
    o.process()
    assert len(backend.prepared) == 2 and o.health().ready   # no preparation inside a later two-law step


def test_holding_is_not_ready_and_fault_latches():
    o = owner([('initialized', 10), ('holding', 10), ('fault', 10), ('tracking', 30)])
    o.add_encoder(15, COUNTS)
    o.process()
    o.process()
    assert o.health().estimator_health == 'HOLDING' and not o.health().ready
    o.process()
    o.process()
    assert o.health().estimator_health == 'FAULT' and not o.health().ready


def test_plan_root_uses_newest_encoder_and_measured_joint_position():
    o = owner([('initialized', 10)])
    o.add_encoder(15, COUNTS)
    o.process()
    o.add_encoder(25, COUNTS)
    rooted = o.plan_root()
    assert rooted.reason is None and rooted.root.plan_ns == 25
    np.testing.assert_array_equal(rooted.joint_position, counts_to_logical(np.array(COUNTS)[None])[0])


def test_root_failure_is_a_refused_plan_not_a_fault():
    backend = Backend()
    o = owner([('initialized', 10)], backend)
    o.add_encoder(15, COUNTS)
    o.process()
    backend.refuse = True
    rooted = o.plan_root()
    assert rooted.root is None and rooted.reason.startswith('root_refused') and o.health().estimator_health != 'FAULT'
    assert owner([]).plan_root().reason == 'not_ready'


def test_encoder_sample_is_the_shared_conversion_owner():
    from runtime_supervision.reduced_filter_shadow import encoder_sample as observer_sample
    assert observer_sample is encoder_sample


def test_dry_run_contract_is_controller_only_and_refuses_rotation():
    contract = load_hardware_contract(LIMITS, 'imricor_test')
    limited = reduced_filter_contract(contract, [10., 0., 4.5, 4., 25., 25.], 10.8)
    assert limited.position_upper[2] == 10.8 and limited.velocity_max[1] == 0.0
    assert contract.position_upper[2] == 15.0   # hardware profile untouched
    with pytest.raises(ValueError, match='rotation'):
        reduced_filter_contract(contract, [10., 7., 4.5, 4., 25., 25.], 10.8)
    with pytest.raises(ValueError, match='tendon'):
        reduced_filter_contract(contract, [10., 0., 4.5, 4., 25., 25.], 15.5)


def test_plain_sampling_has_two_fixed_candidate_counts():
    contract = load_hardware_contract(LIMITS, 'imricor_test')
    planner = CatheterMppi(SimpleNamespace(), contract, MppiConfig(samples=512, grouped_mode_sampling=False))
    assert planner.candidate_counts() == (512, 513)
    with pytest.raises(ValueError):
        CatheterMppi(SimpleNamespace(), contract, MppiConfig(samples=512)).candidate_counts()


def calls(name):
    tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
    return [node for node in ast.walk(tree) if isinstance(node, ast.Call)
            and getattr(node.func, 'attr', getattr(node.func, 'id', None)) == name]


def test_dry_run_node_cannot_command():
    publishers = [(p.args[0].id, p.args[1].value) for p in calls('create_publisher')]
    assert publishers == [('DiagnosticArray', '/catheter_mppi/status'), ('String', '/catheter_mppi/dry_run_plan')]
    for name in ('create_client', 'ActionClient', 'create_service'):
        assert not calls(name)
    text = SOURCE.read_text(encoding='utf-8')
    for word in ('SET_ZERO', 'ControlStream', '/teleop/', '/manager/', 'set_armed'):
        assert word not in text


def test_dry_run_stack_resolves_with_decided_settings():
    stack = resolve_stack(locate_stack('hardware_reduced_filter_dry_run_no_rotation', CONFIG), config_root=CONFIG,
                          allowed_parameters=set(ACTIVE_CONTROLLER_PARAMETERS), expected_node_name='catheter_mppi')
    p = stack.parameters
    assert p['estimator_backend'] == 'reduced_filter_dry_run'
    assert p['mppi_engaged_gain_maximum_first_step_shift'] == 0.0 and not p['mppi_engaged_gain_scenarios']
    assert not any(p[k] for k in ('backlash_compensation_enabled', 'takeup_transaction_enabled',
                                  'engaged_gain_enabled', 'reversal_scheduler_enabled', 'mppi_grouped_mode_sampling'))
    assert (p['plan_rate_hz'], p['planning_deadline_s'], p['reduced_filter_tendon_upper_mm']) == (5.0, .05, 10.8)
    assert p['reduced_filter_startup_pending_images'] == 16
    assert (p['task_gc_enabled'], p['task_gc_maximum_s'], p['task_gc_growth_mb']) == (True, 300.0, 256.0)
    assert p['controller_velocity_max'][1] == 0.0


def test_every_existing_stack_keeps_command_timeout_above_plan_period():
    for path in sorted((CONFIG/'stacks').glob('*.yaml')):
        stack = resolve_stack(path, config_root=CONFIG, allowed_parameters=set(ACTIVE_CONTROLLER_PARAMETERS))
        if stack.parameters.get('estimator_backend', 'v171') == 'v171':
            assert .15 > 1.0/stack.parameters.get('plan_rate_hz', 15.0), path.name


def test_history_consuming_nodes_keep_every_encoder_sample():
    from catheter_control.orchestration.estimator_owner import encoder_history_qos
    qos = encoder_history_qos()
    assert qos.depth >= 512   # about 2.8 s at the device's ~180 Hz
    shadow = PACKAGE.parent/'runtime_supervision'/'runtime_supervision'/'reduced_filter_shadow.py'
    for path in (SOURCE, shadow):
        subscriptions = [ast.unparse(node) for node in ast.walk(ast.parse(path.read_text(encoding='utf-8')))
                         if isinstance(node, ast.Call) and getattr(node.func, 'attr', None) == 'create_subscription'
                         and 'DeviceStream' in ast.unparse(node)]
        assert subscriptions and all('encoder_history_qos()' in call for call in subscriptions), path.name


def test_repeated_sigint_during_shutdown_still_writes_the_report():
    import os
    import signal
    import time
    from catheter_control.reduced_filter_dry_run import finish
    events = []

    class Executor:
        def shutdown(self):
            os.kill(os.getpid(), signal.SIGINT)   # the session stop's second SIGINT
            time.sleep(.05)
            events.append('shutdown')

    node = SimpleNamespace(write_report=lambda: events.append('report'), destroy_node=lambda: events.append('destroy'))
    previous = signal.getsignal(signal.SIGINT)
    try:
        finish(Executor(), node)
    finally:
        signal.signal(signal.SIGINT, previous)
    assert events == ['shutdown', 'report', 'destroy']


def test_plans_follow_committed_image_stamps_at_the_plan_period():
    from catheter_control.orchestration.reduced_filter_settings import plan_due

    def plan_images(commit_ms, period_ms=200.0, count=40, skip=()):
        """Committed image stamps every commit_ms; skipped plans keep the due time, as in _owner_tick."""
        last, planned = None, []
        for k in range(count):
            image_ns = int(round(k*commit_ms*1e6))
            if plan_due(last, image_ns, period_ms/1e3) and k not in skip:
                last = image_ns
                planned.append(k)
        return planned
    assert plan_images(100.0)[:4] == [0, 2, 4, 6]                    # every second commit at 10 Hz
    assert {b-a for a, b in zip(plan_images(95.0), plan_images(95.0)[1:])} == {2}   # 190 ms, never faster
    assert {b-a for a, b in zip(plan_images(100/3), plan_images(100/3)[1:])} == {6}
    assert plan_images(100.0, skip={2})[:4] == [0, 3, 5, 7]           # a skipped plan is retried next commit


def test_skipped_plans_do_not_advance_the_due_image():
    tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
    owner_tick = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == '_owner_tick')
    guarded = [n for n in ast.walk(owner_tick) if isinstance(n, ast.If) and 'self._plan_tick()' in ast.unparse(n.test)]
    assert len(guarded) == 1 and 'plan_due(self.last_plan_image_ns' in ast.unparse(guarded[0].test)
    assigned = [ast.unparse(n) for n in ast.walk(owner_tick) if isinstance(n, ast.Assign)
                and any('last_plan_image_ns' in ast.unparse(t) for t in n.targets)]
    inside = [ast.unparse(n) for n in guarded[0].body]
    assert assigned and all(a in inside for a in assigned)
    plan_tick = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == '_plan_tick')
    returns = sorted(ast.unparse(n.value) for n in ast.walk(plan_tick) if isinstance(n, ast.Return))
    assert returns == ['False', 'False', 'True'] and ast.unparse(plan_tick.body[-1]) == 'return True'


def test_planning_runs_in_the_owner_callback_not_an_independent_timer():
    timers = [ast.unparse(call.args[1]) for call in calls('create_timer')]
    assert timers == ['self._owner_tick', 'self._status_tick']
    tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
    owner_tick = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == '_owner_tick')
    assert any(isinstance(n, ast.Call) and ast.unparse(n.func) == 'self._plan_tick' for n in ast.walk(owner_tick))
