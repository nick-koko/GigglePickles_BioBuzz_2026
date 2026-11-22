import csv
from pathlib import Path
import sys
import tempfile
import unittest

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))
from compare_flywheel_trials_2026 import groups, analyze
from flywheel_dynamics_2026 import load_model


class CompareTrialsTest(unittest.TestCase):
    def test_separates_retry_unloaded_and_feed_using_sample_clock(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'shot_detail.csv'
            rows = []
            for attempt, phase, times in [(1, 'COASTING_TO_STOP', [100]),
                                          (2, 'WAITING_TO_FEED', [200, 300, 400, 500]),
                                          (2, 'FEEDING', [600, 700]),
                                          (3, 'UNLOADED_HOLD', [1000, 1100, 1200, 1300])]:
                for t in times:
                    rows.append(dict(event='DETAIL', session_id='practice', test_suite='test',
                                     test_attempt_id=attempt, test_trial_id=1 if attempt<3 else 2,
                                     test_phase=phase, test_profile='CURRENT', test_condition='COLD',
                                     match_t_ms=t, shooter_sample_age_ms=20, controller_target_rpm=3196,
                                     sequence_start_t_ms=590, measured_rpm=3200, battery_raw_v=12,
                                     applied_power=.55, feeding=str(phase=='FEEDING').lower()))
            with path.open('w',newline='') as f:
                writer=csv.DictWriter(f,fieldnames=rows[0])
                writer.writeheader()
                writer.writerows(rows)
            path.with_name('match.csv').write_text('event,note\nTEST_TRIAL_ABORTED,trial=1 attempt=1 reason=X\n'
                                                   'TEST_TRIAL_COMPLETE,trial=1 attempt=2 reason=\n')
            plant,load,_=load_model(SCRIPTS/'flywheel_dynamics_2026.json')
            attempts=[analyze(r,a,o,plant,load) for a,r,o in groups(path)]
            self.assertEqual([s['outcome'] for s,_ in attempts],['ABORTED','COMPLETE','UNCONFIRMED'])
            summary,trace=attempts[1]
            self.assertAlmostEqual(summary['last_pre_feed_sample_ms'],10)
            self.assertEqual(summary['measured_before_feed'],3200)
            self.assertEqual(summary['cold_settled_within_50_s'],0)
            self.assertIsNotNone(trace[-1]['plant_replay_rpm'])
            self.assertIsNone(attempts[2][0]['feed_command_s'])
