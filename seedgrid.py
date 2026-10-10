#!/usr/bin/env python3
"""Fixed-seed, success-matched RRTC config grid on VAMP v0.6.4 Panda MotionBenchMaker sets.
Seed s = Halton stream skipped by s*SKIP samples (identical per seed across configs). Configs are interleaved
per (problem, seed) and each is timed REPS times (median planning+simplify ns kept, path from rep 0).
A trial counts as success only if VAMP's own validate/validate_motion accept every waypoint and edge."""
import argparse, json, pickle, statistics, sys, time, platform
from pathlib import Path
H = Path(__file__).parent
sys.path.insert(0, str(H / 'vamppy'))  # local vamp-planner 0.6.4 build; drop this line to use the pip install
import vamp
SKIP = 10_007  # rng.skip is O(n); planning time is measured inside VAMP and excludes it
CONFIGS = {'stock': {}, 'dd_off': {'dynamic_domain': False},
           'r075': {'range': 0.75}, 'r075_dd_off': {'range': 0.75, 'dynamic_domain': False},
           'r125': {'range': 1.25}, 'r125_dd_off': {'range': 1.25, 'dynamic_domain': False},
           'r150': {'range': 1.5}}
ap = argparse.ArgumentParser(); ap.add_argument('--seeds', type=int, default=10); ap.add_argument('--reps', type=int, default=3)
ap.add_argument('--problems', default='box,table_pick,table_under_pick'); ap.add_argument('--limit', type=int, default=0)
ap.add_argument('--seed-offset', type=int, default=0); ap.add_argument('--configs', default=''); ap.add_argument('--out', default=str(H / 'out' / 'seedgrid.json')); a = ap.parse_args()
if a.configs: CONFIGS = {k: CONFIGS[k] for k in a.configs.split(',')}
data = pickle.load(open(H / 'res' / 'problems.pkl', 'rb'))
setups = {k: vamp.configure_robot_and_planner_with_kwargs('panda', 'rrtc', **kw) for k, kw in CONFIGS.items()}
rows = []; t0 = time.time()
for pname in a.problems.split(','):
    probs = [p for p in data['problems'][pname] if p['valid']]
    if a.limit: probs = probs[:a.limit]
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
                        ok = all(mod.validate(q, env, True) for q in path) and all(mod.validate_motion(x, y, env, True) for x, y in zip(path, path[1:]))
                rows.append(dict(set=pname, index=p['index'], seed=s, config=k, solved=bool(times), valid=ok,
                                 total_ns=statistics.median(times) if times else None,
                                 plan_ns=statistics.median(plan_ns) if plan_ns else None))
    print(pname, "done", round(time.time() - t0, 1), flush=True)
summ = {}
for pname in a.problems.split(','):
    R = [r for r in rows if r['set'] == pname]; summ[pname] = {}
    key = lambda r: (r['index'], r['seed'])
    base = {key(r): r for r in R if r['config'] == 'stock'}
    for k in CONFIGS:
        C = {key(r): r for r in R if r['config'] == k}
        succ = sum(r['valid'] for r in C.values())
        common = [x for x in C if C[x]['valid'] and base[x]['valid']]
        med = lambda D, ks: statistics.median([D[x]['total_ns'] / 1000 for x in ks]) if ks else None
        summ[pname][k] = dict(trials=len(C), valid=succ, common=len(common), median_total_us_all_valid=med(C, [x for x in C if C[x]['valid']]),
                              median_total_us_common=med(C, common), stock_median_total_us_common=med(base, common),
                              wins_paired=sum(C[x]['total_ns'] < base[x]['total_ns'] for x in common))
out = dict(seed_offset=a.seed_offset, vamp='0.6.4', host=platform.node(), cpu=platform.processor(), seeds=a.seeds, reps=a.reps, configs=CONFIGS, summary=summ, rows=rows,
           seconds=time.time() - t0)
Path(a.out).parent.mkdir(parents=True, exist_ok=True); Path(a.out).write_text(json.dumps(out))
print(json.dumps(summ, indent=1))
