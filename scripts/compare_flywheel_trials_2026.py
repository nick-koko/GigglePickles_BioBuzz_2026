"""Export measured and predicted flywheel traces for meet openers and stationary tests."""
import argparse
import csv
import math
import re
from collections import defaultdict
from pathlib import Path

from fit_flywheel_dynamics_2026 import pack, replay
from flywheel_dynamics_2026 import load_model


def number(row, field):
    try:
        return float(row[field])
    except (KeyError, ValueError, TypeError):
        return math.nan


def sample_time(row):
    return (number(row, 'match_t_ms') - number(row, 'shooter_sample_age_ms')) / 1000


def groups(path):
    with path.open(newline='', encoding='utf-8-sig') as stream:
        rows = [r for r in csv.DictReader(stream) if r['event'] == 'DETAIL']
    attempts = defaultdict(list)
    for r in rows:
        if r.get('test_suite') and number(r, 'test_attempt_id') > 0:
            if r['test_phase'] not in {'PAUSED', 'COMPLETE', 'BETWEEN_UNLOADED_TRIALS', 'WAITING_FOR_LOAD'}:
                attempts[r['test_attempt_id']].append(r)
    if attempts:
        outcomes = {}
        match_path = path.with_name('match.csv')
        if match_path.exists():
            with match_path.open(newline='', encoding='utf-8-sig') as stream:
                for event in csv.DictReader(stream):
                    if event['event'] in {'TEST_TRIAL_COMPLETE', 'TEST_TRIAL_ABORTED'}:
                        ids = re.search(r'trial=(\d+) attempt=(\d+)', event['note'])
                        if ids:
                            outcomes[ids.groups()] = event['event'].removeprefix('TEST_TRIAL_')
        for attempt, samples in attempts.items():
            yield attempt, samples, outcomes.get((samples[0]['test_trial_id'], attempt), 'UNCONFIRMED')
    elif rows and rows[0]['opmode'] == 'CloseShootingAlone' and not rows[0].get('test_suite'):
        first = next((r for r in rows if r['sequence_id'] == '1' and r['feeding'] == 'true'), None)
        if first:
            end_ms = number(first, 'sequence_start_t_ms') + 1000 * number(first, 'feed_duration_s')
            yield 'opening', [r for r in rows if number(r, 'match_t_ms') <= end_ms and r['sequence_id'] in {'0','1'}], 'HISTORICAL'


def analyze(rows, attempt, outcome, plant, load):
    first = rows[0]
    # Warm trials can begin with the previous sequence ID still present.
    feeding = next((r for r in rows if r['feeding'] == 'true'), None)
    feed = number(feeding, 'sequence_start_t_ms') / 1000 if feeding else None
    profile = first.get('test_profile') or 'MEET_OPENER'
    condition = first.get('test_condition') or 'COLD'
    identity = dict(session=first['session_id'], trial=first.get('test_trial_id') or 1,
                    attempt=attempt, outcome=outcome, profile=profile, condition=condition)
    trace = []
    blocks = []
    for r in rows:
        t = sample_time(r)
        if not all(math.isfinite(v) for v in (t, number(r,'measured_rpm'), number(r,'battery_raw_v'), number(r,'applied_power'))):
            continue
        if blocks and t <= sample_time(blocks[-1][-1]):
            continue
        if not blocks or t - sample_time(blocks[-1][-1]) > .20:
            blocks.append([])
        blocks[-1].append(r)
    for block in blocks:
        predictions = [None] * len(block)
        if len(block) >= 4:
            segment = pack(block, {}, 'experiment', feed)
            predictions = replay(plant, segment, load if feeding else None)
        for r, rpm in zip(block, predictions):
            t = sample_time(r)
            trace.append(dict(**identity, phase=r.get('test_phase') or r['stage'],
                              sample_s=t, from_feed_s=t-feed if feeding else None,
                              command_rpm=number(r,'controller_target_rpm'), measured_rpm=number(r,'measured_rpm'),
                              plant_replay_rpm=rpm, applied_power=number(r,'applied_power'),
                              battery_v=number(r,'battery_raw_v')))
    before = [r for r in trace if feeding and r['from_feed_s'] <= 0]
    last = before[-1] if before else None
    spin = next((r for r in before if r['command_rpm'] > 0), None)
    settled = None
    if condition == 'COLD' and spin and last and feed-last['sample_s'] <= .20:
        # Require the final uninterrupted in-band run to extend to the feed cue.
        later = feed
        for row in reversed(before):
            if (row['command_rpm'] != last['command_rpm'] or later-row['sample_s'] > .20 or
                    abs(row['measured_rpm'] - last['command_rpm']) > 50):
                break
            later = row['sample_s']
        if last['sample_s'] - later >= .25:
            settled = later - spin['sample_s']
    summary = dict(**identity, samples=len(trace), feed_command_s=feed,
                   command_before_feed=last['command_rpm'] if last else None,
                   last_pre_feed_sample_ms=-1000*last['from_feed_s'] if last else None,
                   measured_before_feed=last['measured_rpm'] if last else None,
                   pre_feed_peak=max((r['measured_rpm'] for r in before), default=None),
                   cold_settled_within_50_s=settled)
    return summary, trace


def write_csv(path, rows):
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folders', nargs='+', type=Path, help='Run folders or parent folders containing runs')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    plant, load, _ = load_model(Path(__file__).with_name('flywheel_dynamics_2026.json'))
    traces, summaries = [], []
    for path in sorted({p.resolve() for folder in args.folders for p in folder.rglob('shot_detail.csv')}):
        for attempt, rows, outcome in groups(path):
            summary, trace = analyze(rows, attempt, outcome, plant, load)
            summaries.append(summary)
            traces.extend(trace)
    if not traces:
        parser.exit(1, 'No usable shooter samples found in experiment or CloseShootingAlone opener logs.\n')
    args.output.mkdir(parents=True, exist_ok=True)
    write_csv(args.output/'trial_summary.csv', summaries)
    write_csv(args.output/'rpm_traces.csv', traces)
    print(f'Exported {len(summaries)} attempts/openers and {len(traces)} timestamped samples.')


if __name__ == '__main__':
    main()
