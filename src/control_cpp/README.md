# control_cpp

`control_cpp` is the production-language migration surface for the controller.
Its first executable, `control_shadow`, is deliberately non-commanding: it
subscribes to device, marker, manager, target, and worker-result topics and
publishes only versioned shadow requests, timing, and diagnostics.

The shell uses isolated single-threaded executors for input ingestion,
request/result validation, the 100 Hz heartbeat trace, and diagnostics. Worker
results must match the shell epoch, retained request sequence, input
watermarks, and current target revision, and must be finite and fresh.

The shell has no publisher for `/teleop/control`, `/manager/control`, or any
device command service. Existing Python command authority is unchanged.
Bringup starts the shell only with `start_cpp_shadow:=true`; the adapter around
the Python reference output additionally requires `start_shadow_worker:=true`.
Neither option enables controller command output.
