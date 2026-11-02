"""Create an explicit diagnostic snapshot; never launch or command hardware."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('qualified_root', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    source = args.qualified_root.resolve()
    original = source / 'command_trial_manifest.json'
    raw = original.read_bytes()
    manifest = json.loads(raw)
    if manifest['filter_config']['input_correction_policy'] != 'preserve_equilibrium_gap':
        raise ValueError('diagnostic requires the existing coherent-input trial')
    if not manifest['qualification']['command_capable']:
        raise ValueError('requires an explicitly selected control-trial manifest')
    module = (Path(__file__).resolve().parents[4] / 'cr_meta_lnn' / 'deployment'
              / 'experimental' / 'joint_engagement_filter.py')
    if 'float | None' not in module.read_text():
        raise ValueError('source does not support the diagnostic null override')
    destination = args.output.resolve()
    destination.mkdir(parents=True, exist_ok=False)
    shutil.copytree(source / 'packages', destination / 'packages')
    shutil.copyfile(module, destination / 'packages' / 'cr_meta_lnn' / 'deployment'
                    / 'experimental' / 'joint_engagement_filter.py')
    manifest['filter_config']['held_input_registration_maximum_shift'] = None
    manifest['bundle_name'] += '_unclamped_input_diagnostic'
    manifest['diagnostic_override'] = {
        'purpose': 'Temporary XZ-path clamp ablation; not a qualified promotion',
        'parent_manifest': str(original),
        'parent_manifest_sha256': hashlib.sha256(raw).hexdigest(),
        'filter_source_sha256': hashlib.sha256(module.read_bytes()).hexdigest(),
        'held_input_registration_maximum_shift': None,
    }
    output = destination / 'command_trial_manifest.json'
    output.write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n')
    print(output)


if __name__ == '__main__':
    main()
