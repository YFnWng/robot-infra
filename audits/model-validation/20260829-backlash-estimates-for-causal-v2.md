# Backlash estimates used by causal experiment v2

## Decision

The broad identification session `20260829_163456` already identifies useful
take-up scales. Causal experiment v2 therefore uses threshold-clearing, not
maximum-range, half-excursions:

| Logical coordinate | Historical fitted range | v2 requested | Minimum resolved |
|---|---:|---:|---:|
| insertion | 3.12--4.42 mm | 6.0 mm | 5.0 mm |
| rotation | 29.60--60.95 deg | 75.0 deg | 65.0 deg |
| effective bend | 3.11 mm reversal deadband; 4.45 mm release side | 5.5 mm | 4.75 mm |

The minimum-resolved values are deliberately above the conservative fitted
bounds. If the measured run start and configured margins cannot support them,
trajectory construction fails before control is enabled instead of silently
collecting another within-dead-zone run.

## Evidence

Insertion comes from
`cr_meta_lnn/evaluation/real_insertion_backlash_v45.json`. The insertion-only
play fits are 3.1165, 3.5019, and 3.9349 mm for learned-gain slow, medium, and
fast subsets. The corresponding nominal-gain values are 3.5019, 3.9349, and
4.4215 mm. The 4.4215 mm value is used as the conservative bound.

Rotation comes from
`cr_meta_lnn/evaluation/real_rotation_chain_speed_v47.json`. Per-speed
play-plus-lag widths are 33.38, 29.60, and 60.95 degrees for slow, medium, and
fast motion. The pooled speed-conditioned width is 30.93 degrees. The causal
minimum is placed above the 60.95 degree fast-motion fit.

Effective bending comes from
`cr_meta_lnn/evaluation/real_effective_bending_asymmetric_v38.json`. Its
asymmetric persistent fit has 0.287 mm pull and 4.455 mm release widths; its
relaxation fit gives a 3.108 mm reversal deadband. Because the old compensated
bend episodes mix physical shafts 0 and 2, this is an effective-coordinate
threshold, not an independently identified shaft-2 backlash width. Causal
block B remains necessary to isolate that column.

The first causal hardware run, `20260913_165450_causal_proximal`, executed
42,328 command traces and 8,497 accepted UKF updates with no safety event and
no adaptation update. Its requested `[2 mm, 20 deg, 1.5 mm]` half-excursions
were observed to remain predominantly in lost motion. It is retained as the
live-UKF negative control.

## Resolved trajectory at the normal start

At approximately `[20 mm, 0 deg, 0 mm]`, the v2 plan lasts about 878.7 seconds
and remains within:

- insertion-only: 14--26 mm;
- rotation-only: -75--75 degrees;
- isolated bend-shaft block: 14.5--25.5 mm insertion and 2--13 mm bend;
- compensated bend block: approximately 20 mm insertion and 2--13 mm bend.

These are inside the causal usable envelope `[5,35] mm` for insertion,
`[-250,250] deg` for rotation, and `[1,14] mm` for bend. Speeds remain at the
previously qualified slow/fast settings; only accumulated excursion changed.

## Interpretation boundary

These values are initialization estimates from an offline, model-conditioned
trajectory. They select the experiment range; they are not yet production
take-up parameters. Causal v2 must refine onset timing from source-timestamped
raw encoders and past-only UKF updates. No future-smoothed posterior may be
used in the live adaptation path.
