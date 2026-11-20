import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import unittest
from dataclasses import replace

SCRIPTS=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(SCRIPTS))
from flywheel_dynamics_2026 import Plant, Controller, load_model, simulate, control_step, RAD_PER_RPM

class FlywheelDynamicsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plant,cls.load,cls.model=load_model(SCRIPTS/'flywheel_dynamics_2026.json')

    def test_confirmed_annulus_mass_sets_a_physical_inertia_floor(self):
        one=.5*(.166*.45359237)*((.033/2)**2+(.0555/2)**2)
        self.assertAlmostEqual(self.plant.weight_inertia_kg_m2,one,places=12)
        self.assertGreater(self.plant.inertia_kg_m2,4*one)
        added=self.plant.with_weights(5)
        self.assertAlmostEqual(added.inertia_kg_m2-self.plant.inertia_kg_m2,one,places=12)
        self.assertEqual(added.drag_nm,self.plant.drag_nm)

    def test_gaussian_torque_loss_matches_independent_angular_momentum_solution(self):
        plant=replace(self.plant,drag_nm=0,sensor_delay_s=0)
        load={'reference_rpm':3200,'centers_s':[.2,.4,.6,.8],
              'widths_s':[.02]*4,'amplitudes_nm':[.02]*4}
        ctrl=Controller(target_rpm=3200,kp=0,ks=0,kv=0,feed_s=0,
                        boost_start_floor=0,boost_end_floor=0)
        r=simulate(plant,ctrl,load,duration_s=1.2,initial_rpm=3200,h=.0005)
        impulse=4*.02*.02*math.sqrt(2*math.pi)
        expected=3200*math.exp(-impulse/(plant.inertia_kg_m2*3200*RAD_PER_RPM))
        self.assertAlmostEqual(r['rows'][-1]['physical_rpm'],expected,delta=1.0)
        self.assertGreater(r['summary']['load_work_j'],0)

    def test_voltage_scaled_boost_preserves_voltage_until_supply_limit(self):
        c=Controller(kp=0,ks=0,kv=0,boost_delay_s=0,boost_command_lag_periods=0,
                     boost_duration_s=1,boost_voltage_scaled=True)
        for v in [13.8,12.5,11.9]:
            u,*_=control_step(c,3196,v,0,.033,.2)
            self.assertAlmostEqual(u*v,.75*12.5,places=9)
        u,*_=control_step(c,3196,8,0,.033,.2)
        self.assertEqual(u,1)

    def test_faster_downward_rate_does_not_change_upward_ramp(self):
        a=Controller(boost_start_floor=0,boost_end_floor=0)
        b=replace(a,down_slew_per_s=6)
        self.assertEqual(control_step(a,0,12.5,0,.033,-1)[0],
                         control_step(b,0,12.5,0,.033,-1)[0])
        self.assertLess(control_step(b,4000,12.5,.9,.033,-1)[0],
                        control_step(a,4000,12.5,.9,.033,-1)[0])

    def test_late_recovery_does_not_enter_volley_metrics(self):
        a=simulate(self.plant,Controller(),self.load,duration_s=2.9)
        b=simulate(self.plant,Controller(),self.load,duration_s=4)
        for key in ['contact_window_rms_error_rpm','load_weighted_rms_error_rpm','load_weighted_mean_rpm','contact_proxy_range_rpm']:
            self.assertAlmostEqual(a['summary'][key],b['summary'][key],places=9)

    def test_small_step_integration_converges(self):
        c=Controller(down_slew_per_s=3,ff_gain_min=.9,boost_voltage_scaled=True,
                     boost_delay_s=.04,boost_start_floor=.8,boost_end_floor=.9,overspeed_taper_rpm=100)
        a=simulate(self.plant,c,self.load,voltage=11.9,h=.001)['summary']
        b=simulate(self.plant,c,self.load,voltage=11.9,h=.0005)['summary']
        self.assertAlmostEqual(a['load_weighted_rms_error_rpm'],b['load_weighted_rms_error_rpm'],delta=3)
        self.assertAlmostEqual(a['min_contact_rpm'],b['min_contact_rpm'],delta=4)

    def test_no_nonfinite_values_at_supported_parameter_extremes(self):
        for v in [11,14]:
            for count in [3,6]:
                c=Controller(target_rpm=3400,down_slew_per_s=6,ff_gain_min=.85,
                             boost_voltage_scaled=True,boost_start_floor=.95,boost_end_floor=.95)
                r=simulate(self.plant.with_weights(count),c,self.load,voltage=v)
                self.assertTrue(all(math.isfinite(x['physical_rpm']) and x['physical_rpm']>=0
                    and 0<=x['power']<=1 for x in r['rows']))

    def test_startup_only_profile_leaves_running_transitions_identical(self):
        c=Controller(target_rpm=3350)
        initial=dict(initial_rpm=3150,initial_power=.56,initial_target_rpm=3150,
                     target_change_s=.5,target_ramp_s=1)
        a=simulate(self.plant,c,self.load,voltage=11.8,**initial)
        b=simulate(self.plant,replace(c,startup_profile=True),self.load,voltage=11.8,**initial)
        self.assertEqual(a['rows'],b['rows'])
        self.assertIsNone(b['summary']['startup_handoff_s'])

    def test_startup_handoff_is_bounded_and_never_rearms_during_volley(self):
        c=Controller(startup_profile=True,feed_s=1.0)
        r=simulate(self.plant,c,self.load,voltage=11.4)
        self.assertLessEqual(r['summary']['startup_handoff_s'],1.033)
        self.assertFalse(any(x['startup_active'] for x in r['rows'] if x['t_s']>1.04))
        # Timeout is independent of attaining a hard-to-reach target.
        r=simulate(self.plant,replace(c,target_rpm=5900,feed_s=3),self.load,voltage=11.4)
        self.assertLessEqual(r['summary']['startup_handoff_s'],2.033)

    def test_downward_target_cannot_decelerate_faster_than_coast_without_load(self):
        c=Controller(target_rpm=3150,down_slew_per_s=20,feed_s=10)
        r=simulate(self.plant,c,self.load,duration_s=2,initial_rpm=3600,
                   initial_power=.6,initial_target_rpm=3600,target_change_s=.5)
        bound=-self.plant.drag_nm/self.plant.inertia_kg_m2/RAD_PER_RPM
        slopes=[(b['physical_rpm']-a['physical_rpm'])/(b['t_s']-a['t_s'])
                for a,b in zip(r['rows'],r['rows'][1:])]
        self.assertGreaterEqual(min(slopes),bound-1e-6)

    def test_running_ramp_uses_command_history_and_settling_excludes_ramp(self):
        r=simulate(self.plant,Controller(target_rpm=3350),self.load,initial_rpm=3150,
            initial_power=.55,initial_target_rpm=3150,target_change_s=.5,target_ramp_s=1)
        self.assertAlmostEqual(r['rows'][0]['target_rpm'],3150)
        at=lambda t:min(r['rows'],key=lambda x:abs(x['t_s']-t))
        self.assertAlmostEqual(at(1)['target_rpm'],3250)
        self.assertAlmostEqual(at(1.5)['target_rpm'],3350)
        settle=r['summary']['settled_within_50_s']
        self.assertTrue(settle is None or settle>=1.5)

    @unittest.skipUnless(shutil.which('node'),'Node is used only for the optional browser-model parity check')
    def test_python_and_javascript_simulations_agree(self):
        script="""const m=require(process.argv[1]);const data=require(process.argv[2]);
const c=JSON.parse(process.argv[3]);process.stdout.write(JSON.stringify(m.simulate(data.plant,c,data.load,11.9).summary));"""
        c=Controller(down_slew_per_s=3,ff_gain_min=.9,boost_start_floor=.8,boost_end_floor=.9,
                     boost_delay_s=.04,boost_voltage_scaled=True,overspeed_taper_rpm=100)
        from dataclasses import asdict
        out=subprocess.check_output(['node','-e',script,str(SCRIPTS/'flywheel_dynamics_2026.js'),
             str(SCRIPTS/'flywheel_dynamics_2026.json'),json.dumps(asdict(c))],text=True)
        js=json.loads(out);py=simulate(self.plant,c,self.load,voltage=11.9)['summary']
        for key in ['contact_window_rms_error_rpm','load_weighted_rms_error_rpm','load_weighted_mean_rpm','contact_proxy_range_rpm',
                    'min_contact_rpm','rpm_at_feed']:
            self.assertAlmostEqual(py[key],js[key],delta=.00001)

    @unittest.skipUnless(shutil.which('node'),'Node needed for browser-model parity')
    def test_startup_and_running_steps_match_browser_model(self):
        from dataclasses import asdict
        code="""const m=require(process.argv[1]),data=require(process.argv[2]);
process.stdout.write(JSON.stringify(m.simulate(data.plant,JSON.parse(process.argv[3]),data.load,11.8,3.5,.001,JSON.parse(process.argv[4]))));"""
        for initial in [{},dict(initial_rpm=3600,initial_power=.6,initial_target_rpm=3600,target_change_s=.5,target_ramp_s=0),
                        dict(initial_rpm=3150,initial_power=.55,initial_target_rpm=3150,target_change_s=.5,target_ramp_s=1)]:
            c=Controller(target_rpm=3350,startup_profile=True,ff_gain_min=.9,boost_voltage_scaled=True)
            js=json.loads(subprocess.check_output(['node','-e',code,str(SCRIPTS/'flywheel_dynamics_2026.js'),
                str(SCRIPTS/'flywheel_dynamics_2026.json'),json.dumps(asdict(c)),json.dumps(initial)],text=True))
            py=simulate(self.plant,c,self.load,voltage=11.8,**initial)
            for a,b in zip(py['rows'],js['rows']):
                for key in ['physical_rpm','measured_rpm','power','target_rpm','estimated_drive_current_a']:
                    self.assertAlmostEqual(a[key],b[key],delta=1e-7)
                self.assertEqual(a['startup_active'],b['startup_active'])
            self.assertEqual(py['summary']['startup_handoff_s'],js['summary']['startup_handoff_s'])

if __name__=='__main__':
    unittest.main()
