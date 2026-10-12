#!/usr/bin/env python3
"""Aggregate data/*.json.gz into summary.json and pick each robot's confirmation candidate.

Selection rule (fixed before the exploration results were read): per robot, for each non-stock config take the
per-set ratio median_total_us_common / stock_median_total_us_common over seeds 0-19; score = geometric mean of
those ratios over all MBM sets of that robot. Configs whose summed valid-trial count is more than 0.5% of trials
below stock's are ineligible. The lowest-scoring eligible config is confirmed against stock on seeds 20-39
(if every score is >= 1, that is the closest competitor and the confirmation tests whether the default holds)."""
import json, glob, gzip, math, sys
from collections import defaultdict
from pathlib import Path
H = Path(__file__).parent
runs = defaultdict(dict)  # (robot, tag) -> set -> payload (rows dropped)
for f in sorted(glob.glob(str(H / 'data' / '*.json.gz'))):
    d = json.load(gzip.open(f)); rows = d.pop('rows')
    for k, sm in d['summary'].items():  # failure-aware median: invalid/unsolved trials count as +inf
        t = sorted(r['total_ns'] / 1000 if r['valid'] else math.inf for r in rows if r['config'] == k)
        sm['median_total_us_fail_inf'] = t[len(t) // 2] if len(t) % 2 else (t[len(t) // 2 - 1] + t[len(t) // 2]) / 2
        sm['solved'] = sum(r['solved'] for r in rows if r['config'] == k)
    runs[(d['robot'], d['tag'] if 'tag' in d else Path(f).name.split('-')[1])][d['set']] = d
out = {}
for (robot, tag), sets in sorted(runs.items()):
    any_d = next(iter(sets.values()))
    cfgs = list(any_d['configs'])
    scores = {}
    for k in cfgs[1:]:
        ratios = [s['summary'][k]['median_total_us_common'] / s['summary'][k]['stock_median_total_us_common'] for s in sets.values()]
        valid = sum(s['summary'][k]['valid'] for s in sets.values()); vstock = sum(s['summary']['stock']['valid'] for s in sets.values())
        trials = sum(s['summary'][k]['trials'] for s in sets.values())
        scores[k] = dict(geomean_ratio=math.exp(sum(map(math.log, ratios)) / len(ratios)), valid=valid, stock_valid=vstock,
                         trials=trials, eligible=valid >= vstock - 0.005 * trials, range=any_d['configs'][k]['range'])
    elig = {k: v for k, v in scores.items() if v['eligible']}
    best = min(elig, key=lambda k: elig[k]['geomean_ratio']) if elig and tag == 'explore' else None
    out[f'{robot}/{tag}'] = dict(
        robot=robot, tag=tag, default_range=any_d['default_range'], seeds=[any_d['seed_offset'], any_d['seed_offset'] + any_d['seeds'] - 1],
        reps=any_d['reps'], cpu=sorted({s['cpu'] for s in sets.values()}), cpu_by_set={n: s['cpu'] for n, s in sets.items()},
        seconds=sum(s['seconds'] for s in sets.values()), scores=scores, candidate=best,
        sets={n: dict(problems=s['problems'], summary=s['summary']) for n, s in sorted(sets.items())})
(H / 'summary.json').write_text(json.dumps(out, indent=1))
for key, r in out.items():
    print(f"\n## {key}  default={r['default_range']}  seeds {r['seeds']}  cpu={r['cpu']}  candidate={r['candidate']}")
    cfgs = list(next(iter(r['sets'].values()))['summary'])
    print('set | ' + ' | '.join(cfgs))
    for n, s in r['sets'].items():
        sm = s['summary']
        cells = [f"{sm['stock']['median_total_us_all_valid']:.0f}us v{sm['stock']['valid']}/{sm['stock']['trials']}"]
        cells += [f"{sm[k]['median_total_us_common'] / sm[k]['stock_median_total_us_common'] - 1:+.1%} v{sm[k]['valid']}" for k in cfgs[1:]]
        print(n + ' | ' + ' | '.join(cells))
    print('geomean | - | ' + ' | '.join(f"{r['scores'][k]['geomean_ratio'] - 1:+.1%}{'' if r['scores'][k]['eligible'] else '*'}" for k in cfgs[1:]))
