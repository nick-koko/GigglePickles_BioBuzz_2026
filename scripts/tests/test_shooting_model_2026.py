import csv
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from shooting_model_2026 import controller_step, flight, inertia_change, summarize, annulus_inertia, hardware_comparison, guide_geometry


class ShootingModelTest(unittest.TestCase):
    def test_boost_floor_can_override_overspeed_correction(self):
        result = controller_step(2950, 3050, 14, .5, .02, boost_floor=.75)
        self.assertAlmostEqual(result['controller_power_before_boost'], .3953)
        self.assertAlmostEqual(result['no_boost_output_same_loop'], .47)
        self.assertAlmostEqual(result['applied_power'], .75)
        self.assertAlmostEqual(result['approximate_motor_voltage'], 10.5)

    def test_boost_is_floor_not_addition(self):
        result = controller_step(3150, 2000, 12.5, .5, .02, boost_floor=.75)
        self.assertEqual(result['controller_power_before_boost'], 1)
        self.assertEqual(result['output_before_slew'], 1)

    def test_voltage_only_scales_feedforward_and_is_capped(self):
        result = controller_step(2950, 2850, 11, .5, .02)
        self.assertEqual(result['compensation_gain'], 1.05)
        self.assertAlmostEqual(result['controller_power_before_boost'], .09+.4853*1.05)
        self.assertEqual(controller_step(2950, 2950, 14, .5, .02)['compensation_gain'], 1)

    def test_slew_limits_downward_and_zero_target_stops(self):
        result = controller_step(2950, 2950, 12.5, .75, .02)
        self.assertAlmostEqual(result['applied_power'], .72)
        self.assertEqual(controller_step(0, 2950, 12.5, .75, .02, boost_floor=.75)['applied_power'],0)

    def test_ideal_flight_hits_equal_height_at_known_range(self):
        speed, diameter, coupling = 8, .1, .6
        rpm = speed*60/(coupling*math.pi*diameter)
        result = flight(rpm, diameter, coupling, 45, 1, speed**2/9.80665, 1)
        self.assertAlmostEqual(result['vertical_error_at_goal_plane_m'],0)
        self.assertAlmostEqual(result['ideal_required_rpm'],rpm)
        self.assertTrue(result['descending_at_goal_plane'])
        self.assertTrue(result['reaches_plane_before_ground'])

    def test_flight_marks_unreachable_height_and_ground_crossing(self):
        result = flight(1000, .1, .5, 10, .5, 10, 10)
        self.assertIsNone(result['ideal_required_rpm'])
        self.assertFalse(result['reaches_plane_before_ground'])

    def test_inertia_retains_more_speed_for_same_energy_draw(self):
        self.assertAlmostEqual(inertia_change(3150,2800,1)['ideal_after_rpm'],2800)
        self.assertGreater(inertia_change(3150,2800,1.5)['ideal_after_rpm'],2800)

    def test_invalid_model_inputs_rejected(self):
        with self.assertRaises(ValueError): controller_step(3000,3000,12,.5,0)
        with self.assertRaises(ValueError): flight(3000,.1,.5,90,1,2,1)
        with self.assertRaises(ValueError): inertia_change(3000,3100,2)
        with self.assertRaises(ValueError): inertia_change(3000,0,.5)

    def test_known_ring_dimensions(self):
        each=annulus_inertia(.166*.45359237,.033,.0555)
        self.assertAlmostEqual(each,3.9241154763916884e-5)
        result=hardware_comparison(Path(__file__).resolve().parents[1]/'shooting_hardware_2026.json')
        self.assertAlmostEqual(result['current_weights_inertia_kg_m2'],3*each)
        self.assertAlmostEqual(result['proposed_weights_inertia_kg_m2'],4*each)
        self.assertAlmostEqual(result['comparisons'][1]['rim_speed_m_s'],11.875220230569417)
        self.assertAlmostEqual(result['comparisons'][1]['current_weights_energy_J'],6.404885667335482)
        self.assertIsNone(result['profile']['exit_elevation_deg'])
        self.assertIsNone(result['profile']['exit_height_m'])
        with self.assertRaises(ValueError): annulus_inertia(.1,.06,.05)

    def test_guide_clearance_and_conditional_tangent(self):
        actual=guide_geometry(.072,.106,.072,.280,-.0276)
        nominal=guide_geometry(.072,.106,.07112,.280,-.0276)
        self.assertAlmostEqual(actual['radial_clearance_m'],.070)
        self.assertAlmostEqual(actual['geometric_interference_m'],.002)
        self.assertAlmostEqual(nominal['geometric_interference_m'],.00112)
        self.assertAlmostEqual(actual['approximate_ball_center_height_at_guide_edge_m'],.2617735849056604)
        self.assertTrue(74 < actual['approximate_guide_tangent_elevation_deg'] < 76)
        self.assertAlmostEqual(guide_geometry(.072,.106,.072,.280,0)
                               ['approximate_guide_tangent_elevation_deg'],90)
        with self.assertRaises(ValueError): guide_geometry(.072,.106,.072,.280,-.107)

    def test_summary_excludes_metadata_and_keeps_sessions_separate(self):
        with tempfile.TemporaryDirectory() as directory:
            for session in ('a','b'):
                folder = Path(directory)/session
                folder.mkdir()
                header = ['schema_version','session_id','event','note','match_t_ms',
                          'sequence_active','sequence_id','sequence_start_t_ms',
                          'measured_rpm','controller_power_before_boost','controller_target_rpm']
                with (folder/'shot_detail.csv').open('w', newline='') as stream:
                    writer=csv.writer(stream)
                    writer.writerow(header)
                    writer.writerows([
                        [1,session,'CONFIG','test config',0,'false',0,'',9999,'',''],
                        [1,session,'DETAIL','',100,'false',0,'',3000,.5,3000],
                        [1,session,'DETAIL','',200,'true',1,190,2900,.6,3000],
                        [1,session,'DETAIL','',220,'true',1,190,2800,.7,3000],
                        [1,session,'DETAIL','',1500,'false',1,190,3100,.5,3000]])
            result=summarize([directory])
            self.assertEqual(len(result['runs']),2)
            for run in result['runs']:
                self.assertEqual(run['detail_samples'],4)
                self.assertEqual(run['configurations'][0]['note'],'test config')
                shot=run['volleys'][0]
                self.assertEqual(shot['start_observation_lag_ms'],10)
                self.assertEqual(shot['pre_200ms_rpm']['median'],3000)
                self.assertEqual(shot['feed_measured_rpm']['max'],2900)
                self.assertEqual(shot['feed_controller_target_rpm']['median'],3000)
                self.assertIsNone(shot['feed_battery_v'])
            with self.assertRaisesRegex(ValueError,'Duplicate detailed'):
                import shutil
                shutil.copyfile(Path(directory)/'a'/'shot_detail.csv',Path(directory)/'b'/'copy.csv')
                summarize([directory,str(Path(directory)/'b'/'copy.csv')])


if __name__ == '__main__':
    unittest.main()
