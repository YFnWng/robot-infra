"""Non-actuating warmed FK microbenchmark, not ROS timing qualification."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
import platform
import time

import numpy as np
import torch

from cr_meta_lnn.deployment import load_runtime_bundle
from catheter_control.safety.validation import percentile_metrics


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--model-manifest", type=Path, required=True)
    parser.add_argument("--repeats", type=int, default=100)
    parser.add_argument("--warmup", type=int, default=20)
    args = parser.parse_args(argv)
    if args.repeats < 10 or args.warmup < 1:
        parser.error("require repeats >= 10 and warmup >= 1")
    torch.set_num_threads(2)
    bundle = load_runtime_bundle(args.model_manifest, device="cpu",
                                 options={"adaptation_enabled": False})
    original_geometry = bundle.runtime.distal_model.kinematics
    devices = ["cpu"] + (["cuda"] if torch.cuda.is_available() else [])
    rows = []
    with torch.inference_mode():
        for device in devices:
            for dtype in (torch.float32, torch.float64):
                for batch in (1, 21, 512, 1536):
                    geometry = copy.deepcopy(original_geometry).to(device, dtype)
                    strain = (torch.randn(batch, 24, generator=torch.Generator().manual_seed(17))*50).to(device, dtype)
                    pose = torch.eye(4, device=device, dtype=dtype).expand(batch, 4, 4)
                    functions = {"full_fk_tip_slice": lambda: geometry(pose, strain)[:, -1],
                                 "endpoint_fk": lambda: geometry.endpoint(pose, strain)}
                    expected, actual = [f() for f in functions.values()]
                    torch.testing.assert_close(actual, expected)
                    error = float((actual-expected).abs().max())*1000
                    for _ in range(args.warmup):
                        for function in functions.values():
                            function()
                    if device == "cuda":
                        torch.cuda.synchronize()
                    samples = {name: [] for name in functions}
                    for repeat in range(args.repeats):
                        order = list(functions.items())
                        if repeat % 2:
                            order.reverse()
                        for name, function in order:
                            if device == "cuda":
                                torch.cuda.synchronize()
                            started = time.perf_counter_ns()
                            function()
                            if device == "cuda":
                                torch.cuda.synchronize()
                            samples[name].append((time.perf_counter_ns()-started)/1e6)
                    rows.append({"device": device, "dtype": str(dtype), "batch": batch,
                                 "maximum_coordinate_difference_mm": error,
                                 "timing_ms": {name: percentile_metrics(values) for name, values in samples.items()},
                                 "median_speedup": float(np.median(samples["full_fk_tip_slice"])/np.median(samples["endpoint_fk"]))})
    report = {"scope": __doc__, "torch": torch.__version__, "platform": platform.platform(),
              "model_manifest": str(args.model_manifest.absolute()),
              "manifest_sha256": hashlib.sha256(args.model_manifest.read_bytes()).hexdigest(),
              "section_lengths_m": original_geometry.section_lengths.tolist(),
              "rigid_tip_m": float(original_geometry.rigid_tip_length),
              "threads": torch.get_num_threads(), "repeats": args.repeats, "warmup": args.warmup,
              "cuda_device": torch.cuda.get_device_name() if "cuda" in devices else None,
              "rows": rows}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(report, stream, indent=2)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
