# VAMP RRTConnect on Panda: range 1.5 vs the shipped 1.0 (MotionBenchMaker box / table_pick / table_under_pick)

VAMP v0.6.4 (pip `vamp-planner` 0.6.4), `vamp.configure_robot_and_planner_with_kwargs("panda", "rrtc")`, whose
Panda default is `ROBOT_RRT_RANGES["panda"] = 1.0` (dynamic_domain on). Single core (taskset), Intel Xeon
Platinum 8488C. Each (problem, seed) uses the same Halton stream for every config (`halton(); skip(seed*10007)`);
configs are interleaved per problem/seed, each timed 3x (median of VAMP's own planning + simplification ns).
A trial counts only if VAMP's `validate` / `validate_motion` accept every waypoint and edge of the simplified
path. Medians are over the (problem, seed) pairs where both configs succeed.

Median total time (plan + simplify), µs — stock range 1.0 vs range 1.5:

| set | seeds 0–19 (exploration, 7 configs) | seeds 20–39 (fresh confirmation) | valid trials stock / 1.5 (s20–39) |
|---|---|---|---|
| box | 277.3 → 263.7 (−4.9%) | 284.9 → 267.9 (−6.0%) | 1980 / 1980 of 2000 |
| table_pick | 157.8 → 155.2 (−1.7%) | 161.6 → 158.7 (−1.8%) | 1973 / 1969 of 1980 |
| table_under_pick | 253.7 → 245.5 (−3.2%) | 250.8 → 245.1 (−2.3%) | 1938 / 1947 of 2000 |

Planning-only medians: box −15.6% / −14.8%, table_under_pick −8.9% / −6.7%, but table_pick +5.4% / +5.3%
(the gain there comes from cheaper simplification). range 1.25 is in between; dynamic_domain=false alone is
within ±1%. Full per-config tables: `summary.json`; every trial: `*.json.gz`.

Reproduce: `pip install vamp-planner==0.6.4`, put MotionBenchMaker `resources/panda/problems.pkl` in `res/`, then
`python seedgrid.py --seeds 20 --seed-offset 20 --reps 3 --configs stock,r150 --out out.json`
(the script imports vamp from `./vamppy`; point `sys.path` at your install instead).

Limits: one CPU model, three Panda sets, VAMP's own validator (no independent mesh checker in this packet).

All four robots (Panda, UR5, Fetch, Baxter), all MBM sets, range 0.25-3.0: [`all-robots/`](all-robots/README.md).

Touko Ursin, touko.ursin@gmail.com
