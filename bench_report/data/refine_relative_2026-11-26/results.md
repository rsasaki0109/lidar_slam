# Refinement with relative factors on GrandTour CON-1 / CON-2: results (plan.txt, pre-registered 2026-11-26)

Question: is there a relative-factor setting (RA, `refine_relative_translation_sigma` /
`_rotation_sigma_rad`) where the window plane BA improves on the pose graph on CON-1 / CON-2,
the two runs where the 2026-11-17 screen (`../refine_stability_2026-11-17`) found it worse?
Inputs rebuilt from the raw missions (`tools/fetch_con.sh`, `tools/gt_to_bag.py`,
`tools/prep_con.sh`), backend inputs recorded by `python -m lidarslam map --backend-input` and
replayed per variant (`tools/run171.sh`), scores `tools/score171.py`, raw numbers `scores.json`.
The rebuilt inputs reproduce the 11-17 numbers to 0.1 mm (pose graph 0.3844 / 0.2907 m, gate off
0.4497 / 0.3518 m, gate on 0.3848 / 0.2907 m, RA 0.05 m 0.3931 / 0.3003 m).

## ATE (m) of trajectory_refined against the development references, ratio to the pose graph, 1 um sensitivity (m)

| Variant | CON-1 ATE | / PG | sens | CON-2 ATE | / PG | sens |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| PG pose graph | 0.3844 | 1.000 | - | 0.2907 | 1.000 | - |
| DEF gate on (default) | 0.3848 | 1.001 | 0.0004 | 0.2907 | 1.000 | 0.0000 |
| OFF gate off | 0.4497 | 1.170 | - | 0.3518 | 1.210 | - |
| RA1 0.01 m / 0.00175 rad | 0.3822 | 0.994 | 0.0030 | 0.2949 | 1.014 | 0.0013 |
| RA2 0.02 m / 0.0035 rad | 0.3841 | 0.999 | 0.0032 | 0.2974 | 1.023 | 0.0014 |
| RA5 0.05 m / 0.0087 rad | 0.3931 | 1.023 | 0.0173 | 0.3003 | 1.033 | 0.0079 |
| RA10 0.10 m / 0.0175 rad | 0.4209 | 1.095 | 0.0371 | 0.3031 | 1.042 | 0.0210 |
| RA2G RA2 with the gate on | 0.3841 | 0.999 | 0.0032 | 0.2974 | 1.023 | 0.0014 |

## Decision

A variant helps iff on both runs ATE <= 0.98 x pose graph and sensitivity <= 0.10 m.
**None helps.** The closest, RA1, is 0.994 on CON-1 and 1.014 on CON-2. The refinement does not
improve on the pose graph on these missions with relative factors; the gate (the current
default, which keeps the pose graph there) remains the right choice. Defaults are unchanged.

## Reading

- Tighter relative factors pull the refinement monotonically toward the pose graph (RA10 > RA5 >
  RA2 ~ RA1) but not below it on CON-2: the window BA has no gain to give on these runs.
- With relative factors on, the gate discards no window (`gated_windows=0`), so RA2G equals RA2
  byte for byte: the factors keep every window's change under the 0.10 m / 1 deg gate.
- All sensitivities are at most 37 mm (the gate-off run moves 0.77 / 1.62 m, 11-17 screen).
