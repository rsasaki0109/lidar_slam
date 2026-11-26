"""Score the screen (plan.txt). usage: score171.py [SCORES.json]"""
import importlib.util, json, re, sys
from pathlib import Path
import numpy as np
import zarr
spec = importlib.util.spec_from_file_location('ate', '/home/user/lidar_slam_ros2/bench_report/scripts/p1_grandtour_ate.py')
ate = importlib.util.module_from_spec(spec); spec.loader.exec_module(ate)
B = Path('/home/user/bench/refine171')
def ref(s):
    z = zarr.open(f'{B}/data/{s}/ref/prism_position', mode='r')
    return z['timestamp'][:].astype(float), z['point'][:]
def ate_rms(tum, rt, rp):
    est = np.loadtxt(tum)
    m = (rt >= est[0, 0]) & (rt <= est[-1, 0])
    ep = np.stack([np.interp(rt[m], est[:, 0], est[:, k]) for k in (1, 2, 3)], 1)
    R, t = ate.align_se3(ep, rp[m])
    return float(np.sqrt((np.linalg.norm(ep @ R.T + t - rp[m], axis=1) ** 2).mean()))
def maxdiff(a, b):
    A = {round(r[0], 6): r[1:4] for r in np.loadtxt(a)}; C = {round(r[0], 6): r[1:4] for r in np.loadtxt(b)}
    return float(max(np.linalg.norm(A[k] - C[k]) for k in set(A) & set(C)))
res = {}
for s in ('con-1', 'con-2'):
    rt, rp = ref(s)
    pg = ate_rms(B / f'runs/{s}/DEF/0/out/trajectory_optimized.tum', rt, rp)
    res[s] = {'PG': (pg, None, '')}
    for v in ('DEF', 'OFF', 'RA1', 'RA2', 'RA5', 'RA10', 'RA2G'):
        d = B / f'runs/{s}/{v}'
        a = ate_rms(d / '0/out/trajectory_refined.tum', rt, rp)
        sens = maxdiff(d / '0/out/trajectory_refined.tum', d / '1um/out/trajectory_refined.tum') if (d / '1um/out/trajectory_refined.tum').exists() else None
        rep = (d / '0/out/map_refinement_report.yaml').read_text()
        info = ' '.join(f'{k}={v2}' for k, v2 in re.findall(r'(\w+): (\S+)', rep) if k in ('status', 'improved_windows', 'gated_windows', 'windows'))
        res[s][v] = (a, sens, info)
print('| variant | CON-1 ATE | /PG | sens | CON-2 ATE | /PG | sens |')
print('|---|---|---|---|---|---|---|')
helps = []
for v in ('PG', 'DEF', 'OFF', 'RA1', 'RA2', 'RA5', 'RA10', 'RA2G'):
    row = [v]; ok = v != 'PG'; ratios = []
    for s in ('con-1', 'con-2'):
        a, sens, _ = res[s][v]; r = a / res[s]['PG'][0]; ratios.append(r)
        row += [f'{a:.4f}', f'{r:.3f}', '-' if sens is None else f'{sens:.4f}']
        ok = ok and r <= 0.98 and sens is not None and sens <= 0.10
    if ok: helps.append((np.mean(ratios), v))
    print('| ' + ' | '.join(row) + ' |')
for s in ('con-1', 'con-2'):
    for v in ('DEF', 'OFF', 'RA1', 'RA2', 'RA5', 'RA10', 'RA2G'):
        print(s, v, res[s][v][2])
print('helps:', sorted(helps) if helps else 'none')
if len(sys.argv) > 1:
    Path(sys.argv[1]).write_text(json.dumps({s: {v: {'ate_refined' if v != 'PG' else 'ate_optimized': a,
        'ratio_to_pg': a / res[s]['PG'][0], 'sensitivity_m': sens, 'report': info}
        for v, (a, sens, info) in res[s].items()} for s in res}, indent=1) + '\n')
