"""Derive reproducible scenario choices from the six November 19 match pairs."""
import argparse
import csv
import json
import math
from pathlib import Path
import numpy as np


def distribution(values, weights=None):
    if not len(values):
        return None
    values = np.asarray(values)
    if weights is None:
        weights = np.ones(len(values))
    order = np.argsort(values)
    values, weights = values[order], np.asarray(weights)[order]
    return dict(zip(['p05', 'p10', 'p25', 'median', 'p75', 'p90', 'p95'],
        np.interp(np.array([.05, .1, .25, .5, .75, .9, .95]),
                  (np.cumsum(weights)-weights/2)/sum(weights), values).tolist()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--logs', type=Path, default=Path('C:/Users/nicko/Downloads/logs2026/LeagueMeet_11-19-2026'))
    parser.add_argument('--output', type=Path, default=Path('Data/league-20261119/flywheel-model/operating-scenarios.json'))
    args = parser.parse_args()
    folders = [p for p in sorted(args.logs.iterdir()) if p.is_dir() and '184451' not in p.name]
    groups, runs, transitions = {}, [], []
    def add(name, v, dt):
        a, b = groups.setdefault(name, ([], [])); a.extend(v); b.extend(dt)
    for ix, folder in enumerate(folders):
        match, mode = ix//2+1, 'auto' if 'CloseShootingAlone' in folder.name else 'tele'
        with (folder/'shot_detail.csv').open(newline='') as stream:
            rows = [r for r in csv.DictReader(stream) if r['event']=='DETAIL'
                    and math.isfinite(float(r['battery_filtered_v']))]
        t = np.array([float(r['match_t_ms'])/1000 for r in rows])
        v = np.array([float(r['battery_filtered_v']) for r in rows])
        raw = np.array([float(r['battery_raw_v']) for r in rows])
        goal = np.array([float(r['controller_target_rpm']) for r in rows])
        rpm = np.array([float(r['measured_rpm']) for r in rows])
        feed = np.array([r['feeding']=='true' for r in rows])
        enabled = np.array([r['shooter_enabled']=='true' for r in rows])
        dt = np.minimum(.15, np.diff(t, append=t[-1]+np.median(np.diff(t))))
        bands = [(0, 1), (0, 3), (10, 30)] if mode=='auto' else [(0, 1), (0, 3), (3, 30), (30, 60), (60, 90), (90, 120), (60, 120)]
        run = {'match': match, 'mode': mode, 'run': folder.name, 'voltage': {}}
        for lo, hi in bands:
            sel = (t>=lo)&(t<hi)&enabled
            key = f'{mode}_{lo}_{hi}'
            add(key, v[sel], dt[sel])
            run['voltage'][key] = distribution(v[sel], dt[sel])
        # Initial raw values distinguish unloaded rebound from loaded spin-up.
        run['startup_raw_v'] = distribution(raw[t<1], dt[t<1])
        # Nonoverlapping one-second windows, no feeding/recovery or motor start.
        # Match 2 localization was reset accidentally; exclude its target changes.
        if mode=='tele' and match!=2:
            for start in range(5, 119):
                left = np.searchsorted(t, start)
                right = np.searchsorted(t, start+1)
                pad = np.searchsorted(t, start-1)
                if right>=len(t) or right-left<15 or not np.all(enabled[pad:right+1]):
                    continue
                if np.any(feed[pad:right+1]) or np.min(rpm[left:right+1])<2000:
                    continue
                delta = goal[right]-goal[left]
                transitions.append({'match':match,'start_s':start,'initial_target':goal[left],
                    'final_target':goal[right],'delta_rpm':delta,'initial_measured_rpm':rpm[left],
                    'filtered_v':float(np.median(v[left:right+1])),
                    'path_target_range':float(np.ptp(goal[left:right+1]))})
        runs.append(run)
    out = {'method': 'Time-weighted filtered voltage; intervals clipped at 150 ms. Six match pairs; debug run excluded. Target transitions: nonoverlapping 1 s windows after 5 s, motor spinning >2000 RPM throughout, no feed in preceding 1 s or window; match 2 teleop targets excluded because of localization reset. Target delta is end minus start, not single-loop jitter. These are overlapping driving episodes, not independent robot trials.',
           'voltage': {k:distribution(*values) for k,values in groups.items()}, 'runs':runs,
           'transition_windows': transitions, 'transition_summary':{}}
    for label, selected in [('all',transitions), ('late',[x for x in transitions if x['start_s']>=60])]:
        moving = [x for x in selected if abs(x['delta_rpm'])>=50]
        out['transition_summary'][label] = {'eligible_windows':len(selected),'moving_windows_ge50':len(moving),
            'absolute_1s_delta_rpm':distribution([abs(x['delta_rpm']) for x in moving]),
            'up_delta_rpm':distribution([x['delta_rpm'] for x in moving if x['delta_rpm']>0]),
            'down_delta_magnitude_rpm':distribution([-x['delta_rpm'] for x in moving if x['delta_rpm']<0]),
            'initial_target_rpm':distribution([x['initial_target'] for x in moving]),
            'voltage':distribution([x['filtered_v'] for x in moving])}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(out,indent=2))
    print(json.dumps({k:out[k] for k in ['voltage','transition_summary']},indent=2))
    print('Startup raw voltage medians',[(r['match'],r['mode'],r['startup_raw_v']['median']) for r in runs])


if __name__=='__main__':
    main()
