#!/usr/bin/env python3
"""Fixed-seed, success-matched RRTConnect `range` sweep on VAMP v0.6.4 MotionBenchMaker sets, any robot.

Same method as ../seedgrid-20261011/seedgrid.py (Panda): seed s = Halton stream skipped by s*SKIP samples
(identical per seed across configs); configs are interleaved per (problem, seed) and each is timed REPS times
(median of VAMP's own planning + simplification ns kept, path from rep 0). A trial counts as success only if
VAMP's own validate/validate_motion accept every waypoint and edge of the simplified path. Medians are over the
(problem, seed) pairs where both the config and stock succeed. 'stock' = configure_robot_and_planner_with_kwargs
with no kwargs, i.e. ROBOT_RRT_RANGES[robot] and dynamic_domain on.
One JSON per set is written as soon as the set finishes."""
import argparse, json, pickle, statistics, sys, time, platform, os
from pathlib import Path
H = Path(__file__).parent
import vamp
from vamp.constants import ROBOT_RRT_RANGES
SKIP = 10_007  # rng.skip is O(n); planning time is measured inside VAMP and excludes it

ap = argparse.ArgumentParser()
ap.add_argument('--robot', required=True); ap.add_argument('--seeds', type=int, default=20); ap.add_argument('--reps', type=int, default=3)
ap.add_argument('--seed-offset', type=int, default=0); ap.add_argument('--sets', default='')
ap.add_argument('--ranges', default='0.25,0.5,0.75,1.0,1.25,1.5,2.0,2.5,3.0'); ap.add_argument('--limit', type=int, default=0)
ap.add_argument('--tag', default='explore'); ap.add_argument('--out-dir', default=str(H / 'out'))  # summarize.py reads data/*.json.gz (gzip these)
a = ap.parse_args()

default = float(ROBOT_RRT_RANGES[a.robot])
CONFIGS = {'stock': {}}
for x in (float(v) for v in a.ranges.split(',') if v):
    if x != default:  # range == default is the stock config
        CONFIGS[f'r{int(round(x * 100)):03d}'] = {'range': x}
data = pickle.load(open(H / 'res' / a.robot / 'problems.pkl', 'rb'))
sets = a.sets.split(',') if a.sets else sorted(data['problems'])
setups = {k: vamp.configure_robot_and_planner_with_kwargs(a.robot, 'rrtc', **kw) for k, kw in CONFIGS.items()}
cpu = next((l.split(':', 1)[1].strip() for l in open('/proc/cpuinfo') if l.startswith('model name')), platform.processor())
meta = dict(robot=a.robot, default_range=default, vamp=getattr(vamp, '__version__', '0.6.4'), python=platform.python_version(),
            host=platform.node(), cpu=cpu, affinity=sorted(os.sched_getaffinity(0)), seed_offset=a.seed_offset, seeds=a.seeds,
            reps=a.reps, skip=SKIP, configs=CONFIGS)
print(json.dumps(meta), flush=True)
out_dir = Path(a.out_dir); out_dir.mkdir(parents=True, exist_ok=True)
t0 = time.time()
for pname in sets:
    probs = [p for p in data['problems'][pname] if p['valid']]
    if a.limit: probs = probs[:a.limit]
    rows = []; ts = time.time()
    for p in probs:
        env = vamp.problem_dict_to_vamp(p)
        for s in range(a.seed_offset, a.seed_offset + a.seeds):
            for k, (mod, fn, settings, simp) in setups.items():
                times = []; ok = False; plan_ns = []
                for rep in range(a.reps):
                    rng = mod.halton(); rng.skip(s * SKIP)
                    r = fn(p['start'], p['goals'], env, settings, rng)
                    if not r.solved: break
                    sp = mod.simplify(r.path, env, simp, rng)
                    plan_ns.append(int(r.nanoseconds)); times.append(int(r.nanoseconds) + int(sp.nanoseconds))
                    if rep == 0:
                        path = [list(map(float, c)) for c in sp.path.numpy()]
                        # collision checks without the bounds test (some Fetch MBM start/goal joints sit ~1e-6 outside
                        # the joint limits, e.g. wrist_roll = -pi), plus the bounds test on every interior waypoint.
                        # check_bounds.py: out-of-bounds start/goal: Panda 0, UR5 0, Baxter 9/1470 goals, Fetch 12 starts and 130
                        # goals of 679 problems; elsewhere this equals check_bounds=True on every waypoint and edge.
                        ok = (all(mod.validate(q, env, False) for q in path) and all(mod.validate(q, env, True) for q in path[1:-1])
                              and all(mod.validate_motion(x, y, env, False) for x, y in zip(path, path[1:])))
                rows.append(dict(set=pname, index=p['index'], seed=s, config=k, solved=bool(times), valid=ok,
                                 total_ns=statistics.median(times) if times else None,
                                 plan_ns=statistics.median(plan_ns) if plan_ns else None))
    key = lambda r: (r['index'], r['seed'])
    base = {key(r): r for r in rows if r['config'] == 'stock'}
    summ = {}
    for k in CONFIGS:
        C = {key(r): r for r in rows if r['config'] == k}
        common = [x for x in C if C[x]['valid'] and base[x]['valid']]
        med = lambda D, ks, f='total_ns': statistics.median([D[x][f] / 1000 for x in ks]) if ks else None
        summ[k] = dict(trials=len(C), valid=sum(r['valid'] for r in C.values()), common=len(common),
                       median_total_us_all_valid=med(C, [x for x in C if C[x]['valid']]),
                       median_total_us_common=med(C, common), stock_median_total_us_common=med(base, common),
                       median_plan_us_common=med(C, common, 'plan_ns'), stock_median_plan_us_common=med(base, common, 'plan_ns'),
                       wins_paired=sum(C[x]['total_ns'] < base[x]['total_ns'] for x in common))
    out = dict(meta, set=pname, problems=len(probs), summary=summ, rows=rows, seconds=time.time() - ts)
    (out_dir / f'{a.robot}-{a.tag}-s{a.seed_offset}-{a.seed_offset + a.seeds - 1}-{pname}.json').write_text(json.dumps(out))
    line = ' '.join(f"{k}:{v['median_total_us_common'] / v['stock_median_total_us_common'] - 1:+.1%}({v['valid']})"
                    for k, v in summ.items() if v['common'])
    print(f"{a.robot} {pname} n={len(probs)} {time.time() - ts:.0f}s | {line}", flush=True)
print(f"{a.robot} all done {time.time() - t0:.0f}s", flush=True)
