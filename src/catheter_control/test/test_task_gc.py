"""Inert scheduling/lifecycle tests: no ROS services or hardware output."""
from types import SimpleNamespace
import pytest
from catheter_control.orchestration.task_gc import TaskGcPolicy, gc_work


class Collector:
    def __init__(self, enabled=True):
        self.enabled = enabled
        self.thresholds = (700, 10, 10)
        self.calls = 0
    def isenabled(self): return self.enabled
    def get_threshold(self): return self.thresholds
    def set_threshold(self, *values): self.thresholds = values
    def disable(self): self.enabled = False
    def enable(self): self.enabled = True
    def collect(self, generation):
        assert generation == 2
        self.calls += 1


def fixture(enabled=True):
    state = SimpleNamespace(now=0., rss=1000)
    collector = Collector(enabled)
    policy = TaskGcPolicy(enabled=True, maximum_s=10., growth_mb=1.,
                          clock=lambda: state.now, rss=lambda: state.rss,
                          collector=collector)
    return policy, collector, state


@pytest.mark.parametrize('original_enabled', [False, True])
@pytest.mark.parametrize('exit_kind', ['complete', 'cancel', 'fault', 'shutdown'])
def test_restores_original_policy_at_quiescent_boundary(original_enabled, exit_kind):
    policy, collector, _ = fixture(original_enabled)
    assert policy.begin() and not collector.enabled
    with policy.work() as admitted:
        assert admitted
        policy.stop()  # All lifecycle exits request maintenance after stopping output.
        assert not policy.reserve_collection()
        assert not policy.begin()
    assert policy.reserve_collection()
    assert not policy.begin()
    with policy.work() as admitted:
        assert not admitted
    collector.thresholds = (1, 2, 3)
    policy.collect_reserved()
    assert collector.enabled == original_enabled
    assert collector.thresholds == (700, 10, 10)
    assert collector.calls == 1
    assert not policy.reserve_collection()
    assert policy.begin()
    policy.stop()
    assert policy.reserve_collection()
    policy.collect_reserved()


@pytest.mark.parametrize('guard', ['duration', 'memory', 'rss_failure'])
def test_guard_fault_reasons(guard):
    policy, _, state = fixture()
    policy.begin()
    if guard == 'duration': state.now = 10.
    elif guard == 'memory': state.rss += 1024**2
    else:
        def broken(): raise OSError('unreadable')
        policy.rss = broken
    assert policy.guard_reason() == {
        'duration': 'task_gc_duration_limit',
        'memory': 'task_gc_rss_growth_limit',
        'rss_failure': 'task_gc_rss_unavailable'}[guard]


def test_exception_in_work_releases_admission():
    policy, _, _ = fixture()
    with pytest.raises(RuntimeError), policy.work():
        raise RuntimeError('callback failed')
    assert policy.workers == 0


def test_collection_exception_still_restores():
    policy, collector, _ = fixture()
    def broken(_): raise RuntimeError('collection failed')
    collector.collect = broken
    policy.begin(); policy.stop()
    assert policy.reserve_collection()
    with pytest.raises(RuntimeError): policy.collect_reserved()
    assert collector.enabled and not policy.pending and not policy.collecting


def test_disabled_policy_does_not_change_gc():
    collector = Collector()
    policy = TaskGcPolicy(collector=collector)
    assert policy.begin()
    policy.stop()
    assert not policy.reserve_collection()
    assert collector.enabled and collector.calls == 0


def test_repeated_arm_cannot_reset_duration_or_memory_budget():
    policy, _, state = fixture()
    policy.begin()
    state.now = 9.
    state.rss += 512*1024
    assert policy.begin()
    state.now = 10.
    assert policy.guard_reason() == 'task_gc_duration_limit'
    assert policy.baseline == 1000


def test_failed_initial_rss_read_leaves_gc_untouched():
    policy, collector, _ = fixture()
    def broken(): raise OSError('unreadable')
    policy.rss = broken
    with pytest.raises(OSError): policy.begin()
    assert collector.enabled and policy.original is None and not policy.active


def test_decorator_skips_only_reserved_maintenance():
    policy, _, _ = fixture()
    class Owner:
        _task_gc = policy
        calls = 0
        @gc_work
        def tick(self): self.calls += 1
    owner = Owner()
    owner.tick()
    policy.begin(); owner.tick(); policy.stop()
    assert policy.reserve_collection()
    owner.tick()
    assert owner.calls == 2
    policy.collect_reserved(); owner.tick()
    assert owner.calls == 3


@pytest.mark.parametrize('value', [0., -1., float('inf'), float('nan')])
def test_bounds_validation(value):
    with pytest.raises(ValueError): TaskGcPolicy(maximum_s=value)
    with pytest.raises(ValueError): TaskGcPolicy(growth_mb=value)


@pytest.mark.parametrize('family', ['baseline', 'experimental'])
def test_shared_owner_stops_output_before_collecting(family):
    from threading import RLock
    from catheter_control.node import CatheterControlNode
    policy, collector, state = fixture()
    owner = SimpleNamespace(_lock=RLock(), _task_gc=policy,
                            armed=True, mode_claimed=True, family=family)
    events = []
    def fault(reason):
        events.append(('output_stopped', reason))
        owner.armed = owner.mode_claimed = False
        policy.stop()
    owner._fault_locked = fault
    def collect(_):
        assert not owner.armed and not owner.mode_claimed
        assert policy.workers == 0
        events.append(('collect', None))
    collector.collect = collect
    policy.begin()
    state.now = 10.
    with policy.work():
        CatheterControlNode._gc_maintenance_tick(owner)
        assert events == [('output_stopped', 'task_gc_duration_limit')]
    CatheterControlNode._gc_maintenance_tick(owner)
    assert events[-1] == ('collect', None) and collector.enabled


def test_owner_disarm_defers_restoration_until_maintenance():
    from threading import RLock
    from catheter_control.node import CatheterControlNode
    from catheter_control.safety.lifecycle import ControllerState
    policy, collector, _ = fixture()
    owner = SimpleNamespace(_lock=RLock(), _task_gc=policy,
                            _shutting_down=False, armed=True, mode_claimed=True)
    owner._release_locked = lambda: setattr(owner, 'mode_claimed', False)
    policy.begin()
    response = SimpleNamespace(success=False, message='')
    CatheterControlNode._arm_cb(owner, SimpleNamespace(data=False), response)
    assert response.success and not owner.armed and not owner.mode_claimed
    assert owner.state == ControllerState.DISARMED
    assert policy.pending and not collector.enabled
    CatheterControlNode._gc_maintenance_tick(owner)
    assert collector.enabled
