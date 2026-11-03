"""Process environment for the reduced-filter processes (dry run, commanding backend, shadow observer).

numpy's OpenBLAS and torch's OpenMP otherwise start one pool per process sized to every logical CPU.
With the camera and controller processes on the same CPUs, the oversubscribed pools stalled filter
steps for hundreds of milliseconds; single-threaded pools removed the stalls at no cost to the median
step (audits/ros-realtime/REALTIME_DEPLOYMENT_GAP_AUDIT.md, 2026-11-03). Other processes are unchanged.
"""

REDUCED_FILTER_THREAD_ENVIRONMENT = {"OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1"}
