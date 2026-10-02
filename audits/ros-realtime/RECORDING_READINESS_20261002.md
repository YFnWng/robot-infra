# Coordinated recording readiness correction

## Scope and evidence

ROS workspace: `robot-infra`; owners: `experiments/session_recording.py`,
`experiments/recording_session.py`, and `bringup/research_session.launch.py`.
No model, perception algorithm, controller, manager or firmware changes.

F-001 (observed): session
`attribution_twoaxis_points_grouped_n512_mount01_repeat01_20261002T204546875791Z`
became ready and then failed. Its external `bag.log` reports SQLite error 5,
`database is locked`; the recorder returned -6 and `metadata.yaml` was absent.
Camera/controller were stopped by the supervisor; camera finalized normally.

F-002 (hypothesis): the previous once-per-second live read-only SQLite query
could contend with the recorder's writes. The log proves lock contention, not
which reader or filesystem behavior caused it. The new implementation removes
this unnecessary possible source without claiming sole-cause confirmation.

## Execution and gate changes

The supervisor spins subscription callbacks using `rclpy.spin_once` with a
100 ms timeout. Graph probes run in that same process once per second, not in
controller callbacks. Receipt freshness uses monotonic time; camera diagnostics
must report the exact owned video directory. No active database is opened.

| Gate | Evidence | Effect |
| --- | --- | --- |
| Owned processes alive | Child process poll on each loop | Child exit fails session |
| Recorder initialized/subscribed | Storage file existence plus `/rosbag2_recorder` subscriptions in namespace `/` | Required topics missing/file absent prevents readiness |
| Camera recording active/fresh | Existing diagnostics, exact directory, 2 s receipt age | Loss after readiness stops owned processes |
| Telemetry fresh | Existing POS/ENC, markers, controller receipt ages under 2 s | Task readiness withheld; no safety threshold changes |
| Finalized recording valid | SQLite integrity/topics/messages and video evidence after child shutdown | Failure cannot be reported complete |

Recorder graph checks add no high-rate subscribers. Required topic names and
recorder node identity are unchanged; preexisting recorder conflict rejection
is preserved. Graph evidence does not guarantee unpaused durable recording.
The recorder starts unpaused and must not be paused externally during a trial.

Shutdown order remains controller -> bag -> camera. Bounded SIGINT/TERM/KILL
behavior and escalation/failure recording are unchanged. The latest failing
readiness snapshot now persists before shutdown. No failed raw data was repaired
or deleted; retry must allocate a new session directory.

## Verification and remaining qualification

Unit tests prohibit SQLite connections during live probing, check missing
subscriptions/storage and node/namespace filtering, and retain finalization
order/integrity tests. The opt-in real rosbag test publishes synthetic telemetry
on localhost-only domain 117, runs repeated readiness probes for six seconds,
then checks graceful exit, finalized metadata and recorded required messages.
No cameras, actuator links, motor power or command publishers participate.

Observed test result: opt-in real rosbag smoke passed (1 test, 8.95 s total).
That elapsed test duration is not a controller latency measurement.
Relevant `experiments` and `bringup` package tests: 71 passed, 1 opt-in test
skipped; the skipped smoke test was run separately as described above.
Installed recorder CLI and launch argument inspection also passed without
starting the recorder or hardware processes.

Representative dual-camera/GPU/video hardware-load qualification remains
pending. Synthetic testing is not a full-stack timing or durable-storage
performance qualification. File locking behavior on the external volume and
other possible database readers remain unmeasured.
