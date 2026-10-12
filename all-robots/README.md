# VAMP RRTConnect `range` sweep: Panda, UR5, Fetch, Baxter (all MotionBenchMaker sets)

Follow-up to https://github.com/KavrakiLab/vamp/issues/128 ("did you sweep all robots?"). Same method as the Panda
packet in the repo root, extended to every robot with bundled MBM problems in VAMP v0.6.4 (resources are identical
on current `main`): Panda, UR5, Fetch (7 sets x 100 problems) and Baxter (3 sets x 600 problems).

Setup: `vamp-planner==0.6.4` built from the PyPI sdist (GCC 14.2, Release, Python 3.12), RRTC via
`vamp.configure_robot_and_planner_with_kwargs(robot, "rrtc", range=x)`; "default" is no kwargs
(`ROBOT_RRT_RANGES`: panda 1.0, ur5 1.5, fetch 1.0, baxter 0.5; dynamic_domain on everywhere). One process pinned
to one core (taskset) in Daytona 4 vCPU sandboxes, AMD EPYC Zen 4 (9254 / 9354 / 9354P / 9454P; one CPU per set and
seed block, recorded in `summary.json`). Each (problem, seed) uses the same Halton stream for every config
(`halton(); skip(seed*10007)`), configs interleaved, each timed 3x (median of VAMP's planning + simplification ns).
A trial counts only if VAMP's `validate`/`validate_motion` accept every waypoint and edge of the simplified path
(bounds test on interior waypoints only: 142/679 Fetch and 9/1470 Baxter problems have start/goal joints a hair
outside the joint limits, e.g. wrist_roll = -pi; see `bounds.json`). Medians are over (problem, seed) pairs where
both configs succeed; `median_total_us_fail_inf` in `summary.json` counts failures as +inf instead.

Grid (seeds 0-19): range 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0. Candidate rule, fixed before reading
results (`summarize.py`): lowest geometric mean over sets of median-total ratio vs default, among configs whose
total valid count is within 0.5% of trials of the default's. Confirmation: candidate vs default on fresh seeds 20-39
(Panda: 1.25 and 1.5; UR5: both neighbours; Fetch: the rule pick 0.75 and the fastest config 1.5).

## Result

Geometric mean over sets of median total time vs the default (seeds 0-19 / 20-39), and valid paths on seeds 20-39:

| robot | default | best | Δ time s0-19 / s20-39 | valid s20-39 default / best |
|---|---|---|---|---|
| Panda | 1.0 | 1.25 | -2.0% / -1.8% (wins all 7 sets in both blocks) | 13787 / 13752 of 13980 |
| UR5 | 1.5 | 1.5 (default) | 1.25: +1.4% / +0.7%; 2.0: +1.6% / +0.5% | 13514 / 13501 (1.25), 13559 (2.0) of 13780 |
| Fetch | 1.0 | per scene: 1.5 | 1.5: -12.6% / -13.2%, but fewer solved on bookshelves | 12618 / 12169 of 13580 |
| Baxter | 0.5 | 0.5 (default) | 0.25: +29.1% / +26.9% time, more solved | 27607 / 28761 of 29400 |

Per set, median total time default -> best (µs), seeds 0-19 | seeds 20-39, valid default / best (s20-39):

| robot, range | set | s0-19 | s20-39 | valid |
|---|---|---|---|---|
| Panda 1.0 -> 1.25 | bookshelf_small | 159 -> 159 (-0.4%) | 148 -> 148 (-0.2%) | 1980 / 1956 |
| | bookshelf_tall | 158 -> 155 (-2.1%) | 149 -> 146 (-1.7%) | 1984 / 1977 |
| | bookshelf_thin | 179 -> 175 (-2.4%) | 170 -> 165 (-2.8%) | 1966 / 1973 |
| | box | 266 -> 256 (-3.9%) | 253 -> 242 (-4.4%) | 1980 / 1978 |
| | cage | 731 -> 714 (-2.3%) | 721 -> 705 (-2.3%) | 1966 / 1960 |
| | table_pick | 151 -> 151 (-0.4%) | 143 -> 142 (-0.6%) | 1973 / 1975 |
| | table_under_pick | 265 -> 258 (-2.3%) | 228 -> 227 (-0.5%) | 1938 / 1933 |
| Fetch 1.0 -> 1.5 | bookshelf_small | 1889 -> 1637 (-13.3%) | 1765 -> 1540 (-12.8%) | 1818 / 1600 |
| | bookshelf_tall | 2231 -> 1934 (-13.3%) | 2105 -> 1844 (-12.4%) | 1788 / 1674 |
| | bookshelf_thin | 1823 -> 2652 (+45.5%) | 1780 -> 2626 (+47.5%) | 1571 / 1355 |
| | box | 667 -> 496 (-25.8%) | 680 -> 500 (-26.4%) | 1858 / 1904 |
| | cage | 6754 -> 4418 (-34.6%) | 6219 -> 3768 (-39.4%) | 1881 / 1892 |
| | table_pick | 404 -> 353 (-12.7%) | 376 -> 325 (-13.4%) | 1903 / 1922 |
| | table_under_pick | 395 -> 333 (-15.8%) | 367 -> 312 (-14.9%) | 1799 / 1822 |
| Baxter 0.5 -> 0.25 | both_arms_easy | 7497 -> 8916 (+18.9%) | 8377 -> 9881 (+17.9%) | 9248 / 9670 of 9900 |
| | both_arms_medium | 7015 -> 10252 (+46.1%) | 7147 -> 10839 (+51.6%) | 9345 / 9646 of 9840 |
| | both_arms_hard | 7143 -> 8854 (+23.9%) | 10463 -> 11952 (+14.2%) | 9014 / 9445 of 9660 |

Valid denominators: 20 seeds x valid problems (Panda/UR5/Fetch ~2000 per set). Full grids, planning-only medians,
paired win counts and the failure-as-inf medians: `summary.json`; every trial: `data/*.json.gz`.

Reading it:
- **Panda**: 1.25 and 1.5 are both ~2% faster than 1.0 over all 7 sets (1.5 is better on box/table_under_pick but
  +1-4% on cage); 1.25 wins every set in both seed blocks, valid counts within 0.3%. The 3-set Panda result in the repo root
  (Intel Xeon 8488C) points the same way.
- **UR5**: 1.5 is already best overall. 1.25 is ~2% faster on bookshelf_small and 2.0 is 1-4% faster on box
  and table_under_pick, but each loses elsewhere.
- **Fetch**: no single range dominates. 1.5-2.0 is 13-42% faster on box, cage and the table sets with as many or
  more valid paths, and still faster per failure-as-inf median on bookshelf_small/tall, but finds 6-12% fewer valid
  paths there and is much worse on bookshelf_thin (+46%, 14% fewer valid). Under the success-preserving rule the default 1.0 stays. Separately,
  4.8% of solved Fetch paths at the default fail VAMP's own validator (mostly `validate_motion`) (2.4-8% across ranges,
  falling as range grows; Panda 1.4%, UR5 1.9%, Baxter 1.1%).
- **Baxter**: 0.5 is fastest. Ranges >= 0.75 collapse success (0.75: 56%; 1.0: 19%; 3.0: 1% of trials,
  mostly unsolved within 1M iterations). 0.25 gives a valid path in 97.8% vs 93.9% of trials at ~+27% median time.

Reproduce: put `problems.tar.bz2` for each robot from VAMP `resources/<robot>/` in `res/<robot>/` and
`resources/problem_tar_to_pkl_json.py` in `res/`, run `sh setup.sh` (builds VAMP 0.6.4 in `/home/daytona/venv`;
edit the path), then e.g. `taskset -c 1 python sweep.py --robot fetch --tag explore` and
`python sweep.py --robot fetch --tag confirm --seed-offset 20 --ranges 0.75,1.5`; gzip `out/*.json` into `data/`
and run `python summarize.py`. Limits: VAMP's own validator only; one CPU family (AMD Zen 4).

Touko Ursin, touko.ursin@gmail.com
