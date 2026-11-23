"""Fit coast drag and powered startup using the Nov 22 tests; validate by recording.

Run with the ShotTuning/LogData directory. Pass --write to update the model.
Logged-command replay isolates plant error from controller and battery prediction.
"""
import argparse
import csv
from collections import defaultdict
from dataclasses import asdict, replace
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from fit_flywheel_dynamics_2026 import pack, replay
from flywheel_dynamics_2026 import Controller, Plant, RAD_PER_RPM, simulate


COAST_TRAIN = '094217'
POWER_TRAIN = {COAST_TRAIN, '104123', '105931'}


def recordings(root):
    starts, volleys, coasts = [], [], []
    for path in sorted(root.glob('*/shot_detail.csv')):
        run = path.parent.name.split('_')[1]
        attempts = defaultdict(list)
        with path.open(newline='') as stream:
            for row in csv.DictReader(stream):
                if row['event'] == 'DETAIL' and row['test_condition'] in {'COLD', 'ALREADY_SPINNING'}:
                    attempts[row['test_attempt_id']].append(row)
        for rows in attempts.values():
            pre = [r for r in rows if r['test_phase'] == 'WAITING_TO_FEED']
            if len(pre) >= 5 and pre[0]['test_condition'] == 'COLD' and float(pre[0]['measured_rpm']) < 200:
                starts.append(pack(pre, {'run': run, 'settings': pre[0]}, 'startup'))
            feed = [r for r in rows if r['test_phase'] == 'FEEDING']
            if pre and feed:
                cue = float(feed[0]['sequence_start_t_ms']) / 1000
                samples = [r for r in pre + feed if -.25 <= float(r['match_t_ms']) / 1000 - cue <= .62]
                if len(samples) >= 5:
                    volleys.append(pack(samples, {'run': run}, 'volley', cue))
            # First disable sample can contain a cleared RPM. Start at the first
            # spinning reading, then follow the contiguous zero-command trace.
            coast = [r for r in rows if r['test_phase'] == 'COAST_CAPTURE']
            while coast and float(coast[0]['measured_rpm']) < 500:
                coast.pop(0)
            if len(coast) >= 5 and all(float(r['applied_power']) == 0 for r in coast):
                coasts.append(pack(coast, {'run': run}, 'coast'))
    return starts, volleys, coasts


def coast_curve(t, initial, constant, viscous):
    return initial * np.exp(-viscous*t) + constant * np.expm1(-viscous*t) / viscous


def calibrate(baseline, starts, coasts):
    coast, = [s for s in coasts if s['run'] == COAST_TRAIN]
    t, rpm = coast['t'], coast['rpm']
    use = (t > .25) & (rpm >= 500)
    # Fit initial RPM as a nuisance parameter since encoder RPM is quantized.
    result = least_squares(lambda x: coast_curve(t[use], x[2], x[0], x[1]) - rpm[use],
                           [100, .08, rpm[0]], bounds=([0, .0001, rpm[0]-100], [350, .3, rpm[0]+100]))
    constant, viscous, _ = result.x
    train = [s for s in starts if s['run'] in POWER_TRAIN]

    def plant_at(x):
        inertia = x[0] * 1e-4
        return replace(baseline, inertia_kg_m2=inertia, ke_v_s_rad=x[1], sensor_delay_s=x[2],
                       drag_nm=constant*RAD_PER_RPM*inertia, viscous_drag_nm_s_rad=viscous*inertia)

    def residual(x):
        plant = plant_at(x)
        return np.concatenate([(replay(plant, s)-s['rpm'])/30 for s in train])

    minimum = baseline.calibrated_weight_count * baseline.weight_inertia_kg_m2 * 1e4
    fit = least_squares(residual, [baseline.inertia_kg_m2*1e4, baseline.ke_v_s_rad, baseline.sensor_delay_s],
                        bounds=([minimum, .014, 0], [3.6, .027, .10]), loss='soft_l1')
    return plant_at(fit.x)


def held_out(plant, starts, volleys, coasts, load):
    def median_error(segments, loaded):
        errors = []
        for s in segments:
            if s['run'] in POWER_TRAIN:
                continue
            error = replay(plant, s, load if loaded else None) - s['rpm']
            if loaded:
                t = s['relative_feed_t']
                error = error[(t >= 0) & (t <= .60)]
            errors.append(np.sqrt(np.mean(error**2)))
        return {'trials': len(errors), 'median_trial_rmse_rpm': round(float(np.median(errors)), 1)}

    crossings = []
    for s in coasts:
        if s['run'] in POWER_TRAIN:
            continue
        rpm = s['rpm'][0]
        viscous = plant.viscous_drag_nm_s_rad / plant.inertia_kg_m2
        constant = plant.drag_nm / plant.inertia_kg_m2 / RAD_PER_RPM
        predicted = (np.log((constant + viscous*rpm) / (constant + viscous*500))/viscous
                     if viscous else (rpm-500)/constant)
        observed = s['t'][np.flatnonzero(s['rpm'] <= 500)[0]]
        crossings.append([round(float(x), 2) for x in (rpm, observed, predicted)])
    timing_errors = []
    timing_trials = 0
    for s in starts:
        row = s['settings']
        if s['run'] in POWER_TRAIN or row['startup_profile_enabled'] != 'true' or row['opmode'] != 'ShooterRecommended2026':
            continue
        enabled = s['target'] > 0
        timing_trials += 1
        t = s['t'][enabled]
        t = t - t[0]
        voltage = s['v'][enabled]
        # Start at the logged enable, use observed bus voltage and typical loop
        # period, but let the controller compute every motor command itself.
        ctrl = Controller(target_rpm=s['target'][enabled][0], period_s=float(np.median(np.diff(t))),
                          kp=float(row['kP']), ks=float(row['kS']), kv=float(row['kV']),
                          startup_profile=True, startup_up_per_s=float(row['startup_slew_up_per_s']),
                          startup_down_per_s=float(row['startup_slew_down_per_s']),
                          ff_gain_min=float(row['voltage_comp_min_gain']), ff_gain_max=float(row['voltage_comp_max_gain']),
                          feed_s=t[-1]+.01)
        prediction = simulate(plant, ctrl, load, voltage=lambda sec: np.interp(sec, t, voltage),
                              duration_s=t[-1])['summary']['measured_ready_75rpm_150ms_s']
        since = None
        for sec, rpm in zip(t, s['rpm'][enabled]):
            if abs(rpm-ctrl.target_rpm) > 75:
                since = None
            elif since is None:
                since = sec
            elif sec-since >= .15:
                if prediction is not None:
                    timing_errors.append(prediction-sec)
                break
    return {'startup': median_error(starts, False), 'volley': median_error(volleys, True),
            'candidate_timing_trials': timing_trials, 'paired_candidate_ready_times': len(timing_errors),
            'median_abs_ready_error_s': round(float(np.median(np.abs(timing_errors))), 3),
            'median_predicted_minus_measured_ready_s': round(float(np.median(timing_errors)), 3),
            'coast_initial_rpm_observed_predicted_s': crossings}


def fit_feed_load(plant, reference, volleys):
    # Preserve the four-lobe shape. Three shared parameters describe this
    # feed setup without inferring four individual release times from one encoder.
    train = [s for s in volleys if s['run'] in POWER_TRAIN]

    def load_at(x):
        shift, stretch, strength = x
        return {**reference, 'centers_s': [shift + stretch*t for t in reference['centers_s']],
                'widths_s': [stretch*w for w in reference['widths_s']],
                'amplitudes_nm': [strength*a/stretch for a in reference['amplitudes_nm']]}

    def residual(x):
        load = load_at(x)
        errors = []
        for s in train:
            t = s['relative_feed_t']
            err = replay(plant, s, load) - s['rpm']
            errors.extend(err[(t >= 0) & (t <= .60)]/30)
        return errors

    result = least_squares(residual, [0, 1, 1], bounds=([-.07, .65, .5], [.10, 1.4, 1.5]),
                           loss='soft_l1')
    return load_at(result.x)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', type=Path)
    parser.add_argument('--model', type=Path, default=Path(__file__).with_name('flywheel_dynamics_2026.json'))
    parser.add_argument('--write', action='store_true')
    args = parser.parse_args()
    data = json.loads(args.model.read_text())
    old = Plant(**data['plant'])
    old_load = data['load']
    starts, volleys, coasts = recordings(args.logs)
    new = calibrate(old, starts, coasts)
    new_load = fit_feed_load(new, old_load, volleys)
    validation = {name: held_out(p, starts, volleys, coasts, load)
                  for name, p, load in [('league', old, old_load), ('startup', new, old_load),
                                        ('experimental_load', new, new_load)]}
    print(json.dumps(validation, indent=2))
    if args.write:
        data['startup_plant'] = asdict(new)
        data['plant_refinement'] = {'date': '2026-11-22', 'coast_train_run': COAST_TRAIN,
                                    'powered_train_runs': sorted(POWER_TRAIN), 'held_out_replay': validation}
        args.model.write_text(json.dumps(data, indent=2) + '\n', encoding='utf-8', newline='\n')


if __name__ == '__main__':
    main()
