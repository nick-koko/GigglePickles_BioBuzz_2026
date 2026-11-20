#!/usr/bin/env python3
"""Generate model comparisons and the scientific review from a calibrated model."""
import argparse
import csv
from dataclasses import asdict, replace
import html
import json
import math
from pathlib import Path
import re
import subprocess
from statistics import median

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from flywheel_dynamics_2026 import Controller, load_model, simulate

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',type=Path,default=Path(__file__).with_name('flywheel_dynamics_2026.json'))
    parser.add_argument('--output',type=Path,default=Path('Data/league-20261119/flywheel-model'))
    parser.add_argument('--fragment',type=Path)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    plant,load,model=load_model(args.model)
    scenario_defs=json.loads(Path(__file__).with_name('flywheel_scenarios_2026.json').read_text())
    c=Controller()
    cases={
        'Current controller':c,
        'Downward 3 only':replace(c,down_slew_per_s=3),
        'Downward 3 + voltage scaling':replace(c,down_slew_per_s=3,ff_gain_min=.9,boost_voltage_scaled=True),
        'Illustrative earlier ramp + taper':replace(c,down_slew_per_s=3,ff_gain_min=.9,
            boost_voltage_scaled=True,boost_start_floor=.8,boost_end_floor=.9,boost_delay_s=.04,
            overspeed_taper_rpm=100)}
    comparisons=[];results={}
    training=[p['profile'] for p in model['load_profiles'] if p['training']]
    for voltage in [13.8,12.5,11.9,11.3]:
        for name,ctrl in cases.items():
            r=simulate(plant,ctrl,load,voltage=voltage)
            results[name,voltage]=r
            variations=[simulate(plant,ctrl,p,voltage=voltage)['summary'] for p in training]
            comparisons.append({'case':name,'voltage':voltage,**r['summary'],
                'training_load_rms_min':min(s['contact_window_rms_error_rpm'] for s in variations),
                'training_load_rms_max':max(s['contact_window_rms_error_rpm'] for s in variations),
                'training_load_mean_min':min(s['load_weighted_mean_rpm'] for s in variations),
                'training_load_mean_max':max(s['load_weighted_mean_rpm'] for s in variations)})
    (args.output/'scenario-comparisons.json').write_text(json.dumps(comparisons,indent=2))
    with (args.output/'scenario-comparisons.csv').open('w',newline='') as stream:
        keys=[k for k in comparisons[0] if k!='contact_proxy_rpms']+['contact_proxy_rpms']
        w=csv.DictWriter(stream,fieldnames=keys);w.writeheader();w.writerows(comparisons)
    (args.output/'controller-presets.json').write_text(json.dumps({k:asdict(v) for k,v in cases.items()},indent=2))

    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(1,2,figsize=(12,5),sharey=True,layout='constrained')
    names=['Current controller','Downward 3 + voltage scaling','Illustrative earlier ramp + taper']
    colors=['#2874a6','#bd8414','#25845e']
    for ax,voltage in zip(axes,[13.8,11.9]):
        for name,color in zip(names,colors):
            data=results[name,voltage]['rows']
            selected=[r for r in data if 1.85<=r['t_s']<=2.85]
            ax.plot([r['t_s']-2 for r in selected],[r['physical_rpm'] for r in selected],
                    label=name,color=color)
        ax.axhline(c.target_rpm,color='#777777',ls='--',label='Tracking reference')
        ax.axvspan(results[names[0],voltage]['summary']['contact_start_s']-2,
                   results[names[0],voltage]['summary']['contact_end_s']-2,color='#dddddd',alpha=.2)
        ax.set_title(f'{voltage:.1f} V prescribed loaded supply')
        ax.set_xlabel('Seconds relative to feed start');ax.grid(alpha=.2)
    axes[0].set_ylabel('Estimated physical wheel RPM')
    axes[0].legend(fontsize=8,loc='lower left')
    fig.suptitle('Model comparison: voltage consistency and the volley load are separate tuning tasks',fontsize=13)
    fig.supxlabel('Simulations, not robot trials. Common target 3196 RPM is a tracking reference, not an identified ideal scoring RPM.\n'
                  'Shading: empirical load window. Later unloaded RPM is excluded from volley metrics.',fontsize=10)
    fig.savefig(args.output/'voltage-and-boost-comparison.png',dpi=160);plt.close(fig)

    validation=json.loads((args.output/'closed-loop-validation.json').read_text())
    starts=[s for s in validation if s['group']=='startup' and not s['training']]
    shots=[s for s in validation if s['group']=='load']
    fig,axes=plt.subplots(2,3,figsize=(12,7),layout='constrained')
    for ax,s in zip(axes.flat,starts+shots[:3]):
        ax.plot(s['t'],s['rpm'],label='Observed readout',color='#2874a6')
        ax.plot(s['t'],s['prediction'],'--',label='Closed-loop model readout',color='#c73d47')
        ax.set_title(f"Match {s['match']} {s.get('label','spin-up')} · RMSE {s['rmse_rpm']:.0f} RPM")
        ax.set_xlabel('Seconds after command / feed');ax.set_ylabel('Readout RPM');ax.grid(alpha=.2)
    axes[0,0].legend(fontsize=8)
    fig.suptitle('Held-out validation: recomputed controller outputs with measured supply voltage',fontsize=13)
    fig.supxlabel('Matches 2, 4, 6 were excluded from parameter fitting. Loaded validation has four examples; three shown.\n'
                  'Physical wheel RPM during contact remains inferred; this validates the lagged readout.',fontsize=10)
    fig.savefig(args.output/'closed-loop-model-validation.png',dpi=160);plt.close(fig)

    fig,axes=plt.subplots(2,1,figsize=(10,6),layout='constrained')
    for name,color in zip(names,colors):
        data=results[name,13.8]['rows']
        axes[0].plot([r['t_s'] for r in data],[r['physical_rpm'] for r in data],label=name,color=color)
        axes[1].plot([r['t_s'] for r in data],[r['power'] for r in data],color=color)
    axes[0].axhline(c.target_rpm,color='#777777',ls='--');axes[0].set_ylabel('Estimated physical RPM');axes[0].legend(fontsize=8)
    axes[1].set_ylabel('Power command');axes[1].set_xlabel('Seconds after shooter command')
    for ax in axes:ax.axvline(2,color='#888888',ls=':');ax.grid(alpha=.2)
    fig.suptitle('Simulated fresh-supply startup and volley at the same target',fontsize=13)
    fig.supxlabel('13.8 V prescribed supply; feed starts at 2 s. Current controller here also commands 3196 RPM.\n'
                  'These curves do not reuse the recorded 2950-RPM startup trajectory.',fontsize=10)
    fig.savefig(args.output/'startup-model-comparison.png',dpi=160);plt.close(fig)

    startup_rmse=[s['rmse_rpm'] for s in starts];shot_rmse=[s['rmse_rpm'] for s in shots]
    table=''
    for r in comparisons:
        table+='<tr>'+''.join(f'<td>{html.escape(str(v))}</td>' for v in [r['case'],r['voltage'],
          round(r['rpm_at_feed']),round(r['contact_window_rms_error_rpm']),round(r['load_weighted_mean_rpm']),
          round(r['contact_proxy_range_rpm']),f"{r['training_load_rms_min']:.0f}–{r['training_load_rms_max']:.0f}"] )+'</tr>'
    source_motor=model['motor']['url'];source_wheel=model['hardware']['wheel_url']
    weight_energy=.5*model['hardware']['known_ring_inertia_kg_m2']*(c.target_rpm*2*math.pi/60)**2
    operating=build_operating_review(args.output,plant,load,scenario_defs)
    body=f'''<h1>Calibrated flywheel response model</h1>
<p>Built November 20, 2026 from the November 19 meet logs. This models startup and
four-ball-like loading to choose tests; no runtime robot code was changed.</p>
<p><b>The empirical shooting RPM and the ideal flat-RPM scoring point are different.</b>
The existing regression was tuned around a volley with sag and spread. Better regulation
can require a lower command to keep the volley centered. This model uses 3196 RPM as an
adjustable tracking reference, not a discovered scoring optimum. It predicts wheel
behavior, not ball exit speed, launch angle, or made shots.</p>
{operating}
<h2>Confirmed hardware and physical structure</h2>
<p>User-confirmed <a href="{source_motor}">goBILDA 5000-0002-4008 motor</a>, 1:1 direct
drive, <a href="{source_wheel}">72 mm Rhino wheel</a>, and four .166 lb annular weights,
33 mm ID and 55.5 mm OD. The catalog gives 5800 tested free RPM at 12 V, .25 A no-load,
9.2 A stall, and 1.47 kg cm stall torque. Wheel catalog mass is 85 g; its radial mass
distribution is not supplied. The four weights contribute J={4*plant.weight_inertia_kg_m2:.8f}
kg m² and approximately {weight_energy:.2f} J at 3196 RPM.</p>
<pre>drive torque = Kt/R × max(0, applied_power × bus_voltage − Ke × angular_speed)
J × angular_acceleration = drive torque − drag torque − empirical ball-load torque
logged_RPM_model = first-order lag plus transport delay applied to physical speed</pre>
<p>The positive-drive/coast law is an effective fit for this FLOAT, nonnegative-command
controller, not a detailed H-bridge electrical model. Fitted effective total inertia is
{plant.inertia_kg_m2:.8f} kg m², Ke={plant.ke_v_s_rad:.5f} V s/rad, and drag torque
{plant.drag_nm:.5f} Nm. The inferred measurement/response delay is about
{1000*plant.sensor_delay_s:.0f} ms plus a fixed 12 ms smoothing time constant. Effective
inertia, motor constants and timing are coupled; none is an independent measurement
of rotor inertia or encoder latency. Adding weights changes inertia while retaining
the fitted drag torque and the same assumed load/geometry.</p>
<h2>Fitting and held-out validation</h2>
<p>Motor identification uses six spin-ups and 16 conservative post-shot unloaded tails.
Loading is fitted from 16 stable full-stack candidates: count four, four beams blocked,
both collection motors stopped, start within 100 RPM of target, and target variation
under 30 RPM. Matches 1, 3 and 5 are training; matches 2, 4 and 6 are held out.
For loaded data this is 12 training and four held-out volleys.</p>
<p>With actual commands replayed, held-out motor/startup readout errors are about
32–38 RPM RMS, and common-load held-out errors about 36–54 RPM. More importantly,
recomputing controller outputs instead of replaying them gives held-out startup RMS
error {min(startup_rmse):.0f}–{max(startup_rmse):.0f} RPM and loaded RMS error
{min(shot_rmse):.0f}–{max(shot_rmse):.0f} RPM. The closed-loop check uses measured supply
voltage and the actual starting RPM for the loaded segment. These are within-dataset
validation errors, not error bounds for new settings or the inferred physical RPM.</p>
<img src="closed-loop-model-validation.png" alt="Observed and held-out closed-loop model RPM">
<p>Loading uses four smooth empirical torque lobes with a speed-dependent magnitude.
They approximate the observed aggregate load and do not prove four individual contact
or release times. Contact timing and pulse width are not well resolved by roughly
30 ms logging; one fitted width reaches its bound. Individual training-volley fits
are retained for sensitivity comparisons. The model must not infer launch-speed
precision from these convenient lobes.</p>
<h2>Volts matter for boost too</h2>
<p>At fixed .75 power, the simple supply-times-duty estimate is 10.35 V at 13.8 V supply
versus 8.925 V at 11.9 V supply. A 9.375 V nominal boost floor (equivalent to .75 at
12.5 V) instead requests .679 at 13.8 V or .788 at 11.9 V. This comparison does not
claim the motor's instantaneous terminal waveform is a constant DC voltage. The
existing feedforward compensation never scales the boost floor.</p>
<p>The model's voltage-scaled floor is clipped at full power but does not inherit
the feedforward maximum-gain cap. Feedback can always request more than the floor.
The example RPM taper progressively removes added floor power from target to target
+100 RPM; it does not suppress feedback output. Boost still bypasses slew while active.</p>
<h2>Comparison at a fixed tracking reference</h2>
<p>All cases below use target 3196, upward slew 1.5/s, four weights, a 2 s pre-feed
interval, and the same empirical load. The earlier-ramp case is an <b>illustrative,
untested profile</b>: downward 3/s, minimum FF gain .90, voltage-scaled floor rising
.80 to .90 at nominal 12.5 V, delay .04 s, duration .50 s, and 100 RPM overspeed taper.
It is not a recommended robot tune selected from real scoring tests.</p>
<img src="voltage-and-boost-comparison.png" alt="Voltage and boost model comparison">
<p>RMS error below includes the full modeled loading window, including spaces between
lobes. The load-weighted mean and lobe-to-lobe spread describe wheel RPM during the
assumed loads; they are not actual ball exit RPM. Late unloaded overspeed is excluded.
The final range column varies the 12 fitted training load profiles. That is a sensitivity
range, not a confidence interval or an out-of-sample scoring guarantee.</p>
<div class="scroll"><table><tr><th>Case</th><th>Bus V</th><th>RPM at feed</th><th>Shot-window RMS error</th>
<th>Load-weighted mean RPM</th><th>Lobe RPM spread</th><th>RMS range across fitted loads</th></tr>{table}</table></div>
<img src="startup-model-comparison.png" alt="Simulated startup at a fixed reference">
<p>The examples show why correcting startup alone does not choose the best volley.
Removing the fresh-supply overspeed reveals more load sag. Voltage normalization can
make behavior more consistent while still leaving the mean too low for the chosen
tracking reference. Timing and the shape/level of boost then need their own tuning.
The ideal scoring reference must be measured after regulation improves.</p>
<h2>What remains assumed</h2>
<ul><li>For what-if comparisons, bus voltage is prescribed. Current changes do not
recalculate battery/internal wiring sag; the validation checks instead supply the
measured voltage trace.</li><li>No correctly staged full-stack volley began below 11.9 V in this meet. Low-voltage
loaded forecasts therefore need explicit robot testing, even though the motor law
accounts for voltage and some unloaded tails reach lower voltage.</li>
<li>Feeding/contact timing is held fixed when RPM, power or inertia changes. In reality
roller interaction and compression could change timing and extracted energy. The
load-strength, timing and width controls explore this limitation but cannot remove it.</li>
<li>The inferred physical wheel speed is not independently validated by a shaft encoder
or synchronized high-speed video. Ball compression, slip, spin and target outcomes
remain outside this model.</li><li>Do not select an exact gain or millisecond timing from a small modeled improvement.
Use robust differences to select controlled tests; preserve a common target while
comparing controllers, then retune the scoring reference.</li></ul>
<h2>Use and reproducibility</h2>
<p>Simulator: <code>scripts/flywheel_dynamics_2026.py</code>; calibrated parameters:
<code>scripts/flywheel_dynamics_2026.json</code>; identification:
<code>scripts/fit_flywheel_dynamics_2026.py</code>. Python/JavaScript numerical parity,
energy-loss consistency, voltage scaling, integration convergence, startup handoff,
running target transitions and exclusion of late recovery are covered by offline tests.</p>
<pre>python scripts/flywheel_dynamics_2026.py --target 3196 --voltage 11.9 --down 3 --ff-min .9 --boost .8 --boost-end .9 --boost-voltage-scaled --taper 100
python -m unittest discover -s scripts/tests -p test_flywheel_dynamics_2026.py -v</pre>
<p><a href="scenario-comparisons.csv">Scenario comparison CSV</a>;
<a href="scenario-comparisons.json">full scenario metrics</a>;
<a href="fit-validation.json">fit validation</a>;
<a href="closed-loop-validation.json">closed-loop validation arrays</a>.
<a href="../review.html">Full meet analysis</a>. Robot code remains unchanged.</p>'''
    head='''<!doctype html><html><head><meta charset="utf-8"><title>Flywheel response model</title>
<style>body{font:16px/1.5 system-ui,sans-serif;max-width:1100px;margin:40px auto;padding:0 24px;color:#182230}h1{font-size:30px}h2{margin-top:35px}table{border-collapse:collapse;width:100%;font-size:14px}th,td{padding:8px;border-bottom:1px solid #d0d5dd;text-align:left}th{background:#f2f4f7}img{max-width:100%}.scroll{overflow-x:auto}li{margin-bottom:8px}pre{white-space:pre-wrap;background:#f2f4f7;padding:14px}</style></head><body>'''
    (args.output/'model-review.html').write_text(head+body+'</body></html>',encoding='utf-8')
    main_report=args.output.parent/'review.html'
    if main_report.exists():
        existing=main_report.read_text(encoding='utf-8')
        summary_section='''<section id="flywheel-dynamics-model"><h2>Calibrated flywheel response model</h2>
<p>The November 20 follow-up combines the confirmed goBILDA 5000-0002-4008 motor,
72 mm Rhino wheel and four added weights with a fitted motor/coast response and
empirical volley loading. Held-out closed-loop readout error is approximately
43–51 RPM for startup and 33–66 RPM for loaded volleys. These errors do not bound
new-controller forecasts or independently validate physical release RPM.</p>
<p>Voltage-scaling boost can improve voltage consistency, but removing startup
overspeed alone can reveal sag previously compensated by the empirically chosen
RPM. The desired flat scoring RPM remains unknown. The model compares controller
options without treating the existing regression as the ideal no-sag speed.</p>
<p><a href="flywheel-model/model-review.html">Model, validation plots and scenario comparisons</a>.
The interactive comparison is included in the conversation. No robot code changed.</p></section>'''
        if '<section id="flywheel-dynamics-model">' in existing:
            begin=existing.index('<section id="flywheel-dynamics-model">');end=existing.index('</section>',begin)+len('</section>')
            existing=existing[:begin]+summary_section+existing[end:]
        else:
            existing=existing.replace('</body>',summary_section+'\n</body>')
        main_report.write_text(existing,encoding='utf-8')
    if args.fragment:
        library=Path(__file__).with_name('flywheel_dynamics_2026.js').resolve()
        node_code='''const fs=require('fs');const api=require(process.argv[1]);const model=require(process.argv[2]);
const out=process.argv[3];const states=[{}, {aVoltage:10.5,bVoltage:14,weights:3,boost:0,boostEnd:0,down:8,ffMax:1.25,view:'volley'},
{aInitial:3600,aTarget:3150,bInitial:0,bTarget:4000,bVoltage:11.4,aRamp:0,weights:6,boost:1,boostEnd:1,startUp:15,startDown:15}];
states.forEach((s,i)=>{for(const [j,q] of ['rpm','power'].entries())fs.writeFileSync(out+'/chart-state-'+(2*i+j)+'.json',JSON.stringify(api.chartConfig(api.compare(model,s),q)));});'''
        subprocess.run(['node','-e',node_code,str(library),str(args.model.resolve()),str(args.output.resolve())],check=True)
        initial=(args.output/'chart-state-0.json').read_text()
        fragment=args.fragment.read_text(encoding='utf-8')
        payload=json.dumps(model).replace('<','\\u003c')
        fragment=fragment.replace('__MODEL_JSON__',payload).replace('__INITIAL_CHART__',initial)
        fragment=fragment.replace('__SIMULATION_LIBRARY__',library.read_text(encoding='utf-8'))
        scenarios_json=json.dumps(scenario_defs)
        fragment=fragment.replace('__SCENARIOS_JSON__',scenarios_json)
        fragment=fragment.replace('<script>/* Offline companion','<script id="fl-simulation-library">/* Offline companion')
        # Subsequent builds replace the owned script blocks, never the surrounding literal markup.
        fragment=re.sub(r'(<script id="fl-model-data" type="application/json">).*?(</script>)',
                        lambda m:m[1]+payload+m[2],fragment,flags=re.S)
        fragment=re.sub(r'(<script data-viz-chart type="application/json">).*?(</script>)',
                        lambda m:m[1]+initial+m[2],fragment,flags=re.S)
        fragment=re.sub(r'(<script id="fl-simulation-library">).*?(</script>)',
                        lambda m:m[1]+library.read_text(encoding='utf-8')+m[2],fragment,flags=re.S)
        fragment=re.sub(r'(<script id="fl-scenarios" type="application/json">).*?(</script>)',
                        lambda m:m[1]+scenarios_json+m[2],fragment,flags=re.S)
        args.fragment.write_text(fragment,encoding='utf-8')
    print('Wrote',args.output/'model-review.html')
    for r in comparisons:
        if r['voltage'] in [13.8,11.9]:print(r['case'],r['voltage'],round(r['contact_window_rms_error_rpm']),round(r['load_weighted_mean_rpm']))

def build_operating_review(out,plant,load,scenarios):
    """Shared scenario definitions also drive the interactive comparison."""
    cases={'Current':Controller(),
           'Fast startup only':Controller(startup_profile=True),
           'Fast startup + voltage scaling':Controller(startup_profile=True,ff_gain_min=.9,boost_voltage_scaled=True)}
    rows=[]; curves={}
    for key,s in scenarios.items():
        for reverse in ([False,True] if s['initial']>0 and s['initial']!=s['target'] else [False]):
            initial,target=(s['target'],s['initial']) if reverse else (s['initial'],s['target'])
            for name,c in cases.items():
                power=(plant.ke_v_s_rad*initial*2*math.pi/60+plant.drag_nm*plant.resistance_ohm/plant.kt_nm_a)/s['voltage'] if initial else 0
                r=simulate(plant,replace(c,target_rpm=target),load,voltage=s['voltage'],initial_rpm=initial,
                    initial_power=min(1,power),initial_target_rpm=initial if initial else None,
                    target_change_s=.5 if initial else 0,target_ramp_s=s['ramp'] if initial else 0)
                rows.append({'scenario':key,'reverse':reverse,'controller':name,'initial_rpm':initial,
                             'target_rpm':target,'voltage':s['voltage'],**r['summary']})
                curves[key,reverse,name]=r
    (out/'operating-profile-comparisons.json').write_text(json.dumps(rows,indent=2))
    fig,axs=plt.subplots(2,2,figsize=(12,7),sharex=True,layout='constrained')
    for column,key in enumerate(['auto-start','tele-late']):
        for name,color in zip(cases,['#2874a6','#bd8414','#25845e']):
            rs=curves[key,False,name]['rows']
            axs[0,column].plot([r['t_s'] for r in rs],[r['physical_rpm'] for r in rs],label=name,color=color)
            axs[1,column].plot([r['t_s'] for r in rs],[100*r['power'] for r in rs],color=color)
        axs[0,column].plot([r['t_s'] for r in rs],[r['target_rpm'] for r in rs],'--',color='#777777',label='Command')
        axs[0,column].set_title(f"{scenarios[key]['label']} · {scenarios[key]['voltage']} V")
        axs[0,column].set_ylabel('Estimated physical RPM');axs[1,column].set_ylabel('Power command (%)')
        axs[1,column].set_xlabel('Seconds after enable / scenario start');axs[1,column].set_ylim(0,105)
        for ax in axs[:,column]:ax.axvline(2,color='#777777',ls=':');ax.grid(alpha=.2)
    axs[0,0].legend(fontsize=8)
    fig.suptitle('Shared settings: startup from rest versus an already-spinning target transition')
    fig.supxlabel('Simulation only. Running target ramps over 1 s from t=0.5 s. Same empirical volley begins at t=2 s.\n'
                  'Supply is prescribed; changed current does not recalculate voltage sag. Startup-only changes leave the running case identical.',fontsize=10)
    fig.savefig(out/'paired-operating-profiles.png',dpi=160);plt.close(fig)
    table=''.join('<tr>'+''.join(f'<td>{html.escape(str(v))}</td>' for v in
        [s['label'],s['voltage'],f"{s['initial']} → {s['target']}",s['ramp'] if s['initial'] else 'Enable'])+'</tr>' for s in scenarios.values())
    return f'''<h2>Paired startup and running scenarios</h2>
<p>These scenarios use the same controller adjustments, not independently optimized settings.
The two RPM plots stay visible; optional power plots use the same horizontal scale and
0–100% power. Candidate request is max(feedback plus feedforward, boost floor) before slew;
the gap to actual output identifies slew limiting. Flat 100% output indicates no remaining
positive power headroom. Low or zero output while RPM declines slowly indicates coasting.</p>
<table><tr><th>Scenario preset</th><th>Prescribed V</th><th>Initial → final RPM</th><th>Command ramp, s</th></tr>{table}</table>
<p>Presets are representative rounded test conditions, not recorded traces. The fresh
13.8 V autonomous condition is a high-voltage check; filtered voltage during the first
second had median 13.69 V, while raw voltage already sagged under motor load. Start of
teleop uses 12.5 V as an approximation to the first-three-second filtered median of
12.54 V; it does not reproduce rebound followed by startup sag. Late autonomous uses
the 10–30 s median, 12.61 V, and holds 3150 because those constant-RPM autos did not
normally make position-driven setpoint changes. All startup scenarios use the same
3196 tracking reference, not the old 2950-RPM transient workaround.</p>
<p>Across all six teleops, time-weighted filtered voltage during seconds 60–120 had
median 11.79 V, 10th percentile 11.46 V and 5th percentile 11.36 V. The last 30 s
median was 11.76 V. Use 11.8 V as a normal late check, 11.4 V as a lower-range check,
and optionally 11.0 V as an excursion test rather than typical end-of-match voltage.</p>
<p>For target changes, exclude match 2 teleop (localization reset), motor startup,
feeding and the preceding second of recovery. Of 346 eligible nonoverlapping one-second
windows, 278 changed target by at least 50 RPM. Their median absolute net change was
219 RPM and 90th percentile 487 RPM; in the second half these were 202 and 438 RPM
(141 moving windows). Thus +200 RPM over 1 s is a useful typical late check and
±450 RPM over 1 s a larger check. These windows may belong to the same maneuver and
are not independent trials. Reverse the transition to test decreases; set ramp duration
to zero for a deliberately harder instantaneous step. The 3150 starting command is
rounded from the late-window initial-target median of 3148 RPM.</p>
<img src="paired-operating-profiles.png" alt="Paired startup and running RPM with power commands beneath">
<p>The example startup profile uses upward slew 10 power units/s (nominal 0→100% in
100 ms) and downward slew 6/s. At an assumed 33 ms control period this is roughly
four updates; actual robot loop timing varies. This limits the command ramp, not
measured current. The startup latch is armed only when enabling from below 200 RPM.
It releases after 150 ms within ±50 measured RPM with |RPM slope| below 200 RPM/s,
or after 2 s, or on feed start, whichever comes first. A later sag cannot rearm it.
Handoff retains the actual power as the running slew reference; no power reset is
introduced. This is a bounded controller-mode handoff, not a gate that delays feeding.</p>
<p>Already-spinning scenarios begin with shared initial physical/readout RPM and a
plant-balanced initial power, hold the prior command for 0.5 s, then change the target.
Small initial settling is possible because the empirical P+FF equilibrium is not
exactly zero error. “Settled” means physical RPM remains within ±50 of the final
reference through feed start, with at least 250 ms remaining; no settling time is
reported during the target ramp itself. It is not a new shot permission check.</p>
<p>These startup rates extrapolate beyond the historical 1.5/s ramp. The effective motor
model estimates roughly 10 A peak motor drive current for the fast 13.8 V startup,
versus roughly 7.6 A for the existing ramp. Those are effective winding-current
estimates, not battery/fuse current or fuse-safety certification. Supply sag, thermal
history, fuse I²t and other motors are not modeled. Begin hardware trials with a
longer ramp and measure voltage/current before shortening it.</p>
<p>Reducing startup overspeed changes the initial condition for the volley. The scaled
startup example settles near the reference sooner but can increase shot-window sag
unless boost is retuned. A faster startup-only profile leaves the already-spinning
case exactly unchanged; voltage-scaled boost affects both cases. Persistent shot sag
at sub-100% commands suggests controller/boost headroom, while sag at 100% is a
voltage, inertia, loading or motor limit within this model. More slew cannot overcome
that limit, and faster downward slew cannot actively brake a FLOAT/coasting wheel.</p>
<p><a href="operating-scenarios.json">Operating-condition statistics and source windows</a>;
<a href="operating-profile-comparisons.json">All preset comparisons, including reversed transitions</a>.</p>'''


if __name__=='__main__':
    main()
