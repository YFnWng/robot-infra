"""Callback instrumentation shared by controller orchestration code."""
from functools import wraps


def instrument_timer(name):
    """Measure timer release lateness and complete callback duration."""
    def decorate(callback):
        @wraps(callback)
        def measured(self, *args, **kwargs):
            started = self._steady()
            probe = self._timer_probes.get(name)
            if probe is not None:
                self._timing.record_seconds(
                    f"{name}_timer_lateness", probe.observe(started))
            try:
                return callback(self, *args, **kwargs)
            finally:
                self._timing.record_seconds(
                    f"{name}_callback_duration", self._steady()-started)
        return measured
    return decorate
