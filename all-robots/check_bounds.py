"""Count MBM problems whose start / goals lie outside VAMP's joint bounds (validate(q, env, True) vs False)."""
import pickle, sys, vamp, json
res = {}
for robot in sys.argv[1:]:
    mod = getattr(vamp, robot); data = pickle.load(open(f'res/{robot}/problems.pkl', 'rb'))
    n = s_out = g_out = 0
    for ps in data['problems'].values():
        for p in ps:
            if not p['valid']: continue
            env = vamp.problem_dict_to_vamp(p); n += 1
            s_out += mod.validate(p['start'], env, False) and not mod.validate(p['start'], env, True)
            g_out += any(mod.validate(g, env, False) and not mod.validate(g, env, True) for g in p['goals'])
    res[robot] = dict(valid_problems=n, start_out_of_bounds=s_out, goal_out_of_bounds=g_out)
print(json.dumps(res))
