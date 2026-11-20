"""Replay Match 4 auto/teleop through production intake code and its pre-fix baseline.

Run run_shooter_configuration_checks.ps1 first to compile TeamCode and write its
desktop classpath. Requires a pre-recovery --baseline-ref, JDK, Python/matplotlib.
No hardware access or production source edits. Recorded future sensors are held
fixed, so this is a command/logic replay, not a prediction of changed ball motion.
"""
import argparse
import csv
import hashlib
import html
import json
import os
from pathlib import Path
import subprocess

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

REPO=Path(__file__).resolve().parents[1]
SOURCE='TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/IntakeWithSensorsSubsystem2026.java'
RUNS={'auto':'20261119_183725_413_CloseShootingAlone_bcfb4bd5',
      'teleop':'20261119_183802_809_Pickles2026Teleop_12a5634c'}


def read(path):
    with path.open(newline='',encoding='utf-8-sig') as stream:
        return list(csv.DictReader(stream))


def prefix(mask):
    n=0
    while n<4 and mask&(1<<n):n+=1
    return n


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log-root',type=Path,required=True)
    parser.add_argument('--baseline-ref',required=True)
    parser.add_argument('--output',type=Path,default=REPO/'Data/league-20261119/match4-count-replay')
    parser.add_argument('--confirmation-seconds',type=float,default=.1)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    cpfile=REPO/'build/shooter-config-checks/classpath.txt'
    if not cpfile.exists():parser.error('Run scripts/run_shooter_configuration_checks.ps1 first')
    baseline_ref=subprocess.check_output(['git','rev-parse',args.baseline_ref],cwd=REPO,text=True).strip()
    baseline=subprocess.check_output(['git','show',f'{baseline_ref}:{SOURCE}'],cwd=REPO,text=True)
    if 'ENABLE_SUSTAINED_BEAM_RECOVERY' in baseline:
        parser.error('The selected baseline already contains sustained beam recovery; select the earlier checkpoint')
    baseline=baseline.replace('IntakeWithSensorsSubsystem2026','ReplayLegacyIntake2026')
    replacements={
        'private void applyOutputs() {':'private void applyOutputs() { applyOutputs(System.nanoTime()); }\n    private void applyOutputs(long replayNowNanos) {',
        'updateBallTracking();':'updateBallTracking(replayNowNanos);',
        'private void updateBallTracking() {':'private void updateBallTracking(long replayNowNanos) {',
        'beamSampleNanos = System.nanoTime();':'beamSampleNanos = replayNowNanos;'}
    for old,new in replacements.items():
        assert baseline.count(old)==1,old
        baseline=baseline.replace(old,new,1)
    source=args.output/'ReplayLegacyIntake2026.java';source.write_text(baseline,encoding='utf-8')
    classes=args.output/'classes';classes.mkdir(exist_ok=True)
    cp=cpfile.read_text().strip()
    subprocess.run(['javac','-cp',cp,'-d',str(classes),str(source),
                    str(REPO/'scripts/tests/IntakeLogReplay2026.java')],check=True)
    results=[];data={};validation=[];sources=[]
    for mode,run in RUNS.items():
        rawpath=args.log_root/run/'shot_detail.csv'
        raw=[r for r in read(rawpath) if r['event']=='DETAIL']
        sources.append({'mode':mode,'run':run,'sha256':hashlib.sha256(rawpath.read_bytes()).hexdigest()})
        inputs=args.output/f'{mode}-samples.tsv'
        with inputs.open('w',newline='') as stream:
            w=csv.writer(stream,delimiter='\t');w.writerow(['t_s','sample_ns','mask','feed','jam','mode','tracking','manual','count'])
            for r in raw:
                w.writerow([float(r['match_t_ms'])/1000,
                    1_000_000_000+round((float(r['match_t_ms'])-float(r['beam_sample_age_ms']))*1e6),
                    r['beam_mask'],r['feeding'],r['jam_reverse'],r['intake_mode'],r['tracking_enabled'],r['manual_motor_test'],r['ball_count']])
        output=args.output/f'{mode}-replay.csv'
        subprocess.run(['java','-cp',str(classes)+os.pathsep+cp,'IntakeLogReplay2026',str(inputs),str(output),str(args.confirmation_seconds)],check=True)
        replay=read(output);assert len(raw)==len(replay)
        command_mismatches=[]
        for r,s in zip(raw,replay):
            if (r['intake_applied_control_mode']!=s['legacy_control_mode'] or
                (r['intake_applied_control_mode']=='VELOCITY' and
                (float(r['intake_velocity_command_rpm'])!=float(s['legacy_intake_rpm']) or
                 float(r['transfer_velocity_command_rpm'])!=float(s['legacy_transfer_rpm'])))):
                command_mismatches.append(float(s['t_s']))
        assert not command_mismatches,command_mismatches
        validation.append({'mode':mode,'samples':len(raw),'count_mismatches':0,'command_mode_or_velocity_mismatches':0})
        start=0
        for i,r in enumerate(raw):
            if r['feeding']!='true' or (i and raw[i-1]['feeding']=='true'):continue
            seg=[(a,b) for a,b in zip(raw[start:i],replay[start:i]) if a['feeding']=='false']
            old=int(r['sequence_start_ball_count']);new=int(replay[i]['new_before_commands_count'])
            assert old==int(replay[i]['legacy_before_commands_count'])
            new_full=next((float(b['t_s']) for a,b in seg if int(b['new_count'])==4),None)
            first_different=next((float(b['t_s']) for a,b in seg if
                b['new_intake_rpm']!=b['legacy_intake_rpm'] or b['new_transfer_rpm']!=b['legacy_transfer_rpm']),None)
            longest=held=0
            for k,(a,b) in enumerate(seg):
                dt=(float(seg[k+1][1]['t_s'])-float(b['t_s'])) if k+1<len(seg) else 0
                if int(a['beam_mask'])==15:held+=dt;longest=max(longest,held)
                else:held=0
            max_prefix=max((prefix(int(a['beam_mask'])) for a,b in seg),default=0)
            result={'mode':mode,'volley':int(r['sequence_id']),'feed_s':float(r['match_t_ms'])/1000,
                    'recorded_start_count':old,'replay_start_count':new,
                    'max_observed_prefix':max_prefix,'max_observed_blocked':max((int(a['beam_mask']).bit_count() for a,b in seg),default=0),
                    'longest_four_beam_interval_s':longest,'first_new_full_s':new_full,'first_command_divergence_s':first_different,
                    'status':'unchanged_full' if old==new==4 else
                             'partial_unconfirmed' if new<4 else
                             'conservative_full_without_four_beam_pattern' if max_prefix<4 else 'recovered_full'}
            results.append(result);data[mode,result['volley']]=seg
            start=i
    metadata={'baseline_commit':baseline_ref,'current_source_sha256':hashlib.sha256((REPO/SOURCE).read_bytes()).hexdigest(),
              'confirmation_seconds':args.confirmation_seconds,'sources':sources,'validation':validation,'volleys':results,
              'limits':['Actual physical ball count is not known. Blocked-beam patterns and latched counts are observations/inferences, not verified launches.',
                        'New A override is off throughout; historical logs do not contain that input. Unit tests cover hold/release/X behavior.',
                        'All later beam samples are held fixed after a command diverges. New motor stops could change those future samples in a real robot.',
                        'Input timing uses match time minus cached beam sample age. Intake mode transitions precede the sample in this autonomous; feed commands follow the sample. Both old counts and old command modes/velocity outputs reproduce exactly.']}
    (args.output/'summary.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')
    fields=list(results[0])
    with (args.output/'volley-summary.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fields);writer.writeheader();writer.writerows(results)

    plt.rcParams.update({'font.size':9,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(2,3,figsize=(14,8),layout='constrained')
    selected=[('auto',4),('teleop',8),('teleop',15),('teleop',11),('teleop',13),('teleop',4)]
    for ax,key in zip(axes.flat,selected):
        pairs=data[key];active=[j for j,(a,b) in enumerate(pairs) if int(a['beam_mask'])]
        if active:pairs=pairs[max(0,active[0]-2):]
        t=[float(b['t_s']) for a,b in pairs]
        ax.step(t,[int(a['ball_count']) for a,b in pairs],where='post',label='Recorded count',color='#2874a6')
        ax.step(t,[int(b['new_count']) for a,b in pairs],where='post',label='Updated code',color='#c07816')
        ax.step(t,[prefix(int(a['beam_mask'])) for a,b in pairs],where='post',label='Raw top-down prefix',ls=':',color='#777777',alpha=.8)
        ax.set_title(f'{key[0]} volley {key[1]}');ax.set_ylim(-.1,4.3);ax.set_yticks(range(5));ax.grid(alpha=.2)
        ax.set_xlabel('Seconds after Play');ax.set_ylabel('Positions / count')
    axes[0,0].legend(fontsize=8)
    fig.suptitle('Match 4: unresolved three-count cases and conservative earlier full decisions')
    fig.supxlabel('Recorded sensors are prescribed. Actual ball count and changed ball motion after earlier motor stops are unknown.',fontsize=10)
    fig.savefig(args.output/'replay-cases.png',dpi=160);plt.close(fig)
    def number(v):return '—' if v is None else f'{v:.3f}'
    table=''.join('<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in
         [r['mode'],r['volley'],number(r['feed_s']),r['recorded_start_count'],r['replay_start_count'],number(r['first_new_full_s']),
          f"{1000*r['longest_four_beam_interval_s']:.0f}",r['status']])+'</tr>' for r in results)
    counts={key:sum(r['status']==key for r in results) for key in
            ['unchanged_full','recovered_full','partial_unconfirmed','conservative_full_without_four_beam_pattern']}
    partial='; '.join(f"{r['mode']} volley {r['volley']}: {r['recorded_start_count']} → {r['replay_start_count']}, "
                     f"logged full-beam interval {1000*r['longest_four_beam_interval_s']:.0f} ms"
                     for r in results if r['status']=='partial_unconfirmed') or 'None in this replay'
    report=f'''<!doctype html><html><head><meta charset="utf-8"><title>Match 4 intake logic replay</title>
<style>body{{font:16px/1.5 system-ui;max-width:1200px;margin:32px auto;padding:0 20px}}table{{border-collapse:collapse;width:100%;font-size:14px}}td,th{{padding:7px;border-bottom:1px solid #bbb;text-align:left}}img{{max-width:100%}}.scroll{{overflow-x:auto}}</style></head><body>
<h1>Match 4 intake/counting replay</h1><p>Production code and the pre-fix checkpoint were run against all recorded beam samples:
928 autonomous and 3,448 teleop loops, covering four auto and 16 teleop volleys. The legacy replay reproduces
every logged count and every sampled motor control mode/velocity command: zero mismatches.</p>
<p><b>This is not a verified physical ball count or a new closed-loop robot trial.</b> New motor commands can change
future ball movement. Later recorded sensors stay fixed in this replay. Manual A override is off throughout.</p>
<h2>Findings</h2><ul><li>{counts['unchanged_full']} volleys already reached four in the old code and still reach four.</li>
<li>{counts['recovered_full']} more reach four after the changes and had a four-beam pattern somewhere in their pickup.</li>
<li>{counts['conservative_full_without_four_beam_pattern']} more reach four without any simultaneous four-beam pattern. This may be the
intended conservative earlier stop, but a shorter physical load cannot be ruled out.</li>
<li>{counts['partial_unconfirmed']} remain below four: {html.escape(partial)}. Intake remains enabled.
If these were actual four-ball loads, this setting would not fully address those intake-stop cases.</li></ul>
<p>Replay confirmation setting: {args.confirmation_seconds*1000:.0f} ms. The default 100 ms run leaves
auto volley 4, teleop volley 8 and teleop volley 15 at three; their full-beam intervals are shorter than that setting.</p>
<p>The previously discussed teleop volley 4 has additional earlier history: the new code recovers count two at
27.653 s while the old code stays at one. A middle-low edge then advances to three at 32.330 s and an entry edge
advances to four at 32.498 s. The isolated earlier test seeded the recorded count one near 30 s and therefore
predicted a later recovery. A whole-run replay does not reseed from the old count after each pickup.</p>
<p>One ball versus a sustained pair of upper beams needs a physical check. If one ball can hold shooter and
middle-high blocked for 100 ms during seating, the new recovery can count two and stop transfer early. Video or
a controlled one-ball loading test is needed to distinguish that from a missed second ball.</p>
<img src="replay-cases.png" alt="Recorded counts, updated production counts and raw beam prefixes for six important pickups">
<h2>Every shooting sequence</h2><div class="scroll"><table><tr><th>Mode</th><th>Volley</th><th>Feed time, s</th>
<th>Old count</th><th>New count</th><th>First new full / intake-stop time, s</th><th>Longest four-beam interval, ms</th><th>Assessment</th></tr>{table}</table></div>
<p>Feed timestamps are log time since Play. Count columns are just before feed clears occupancy. Logged blocked intervals are
sampled observations, not exact physical durations. All raw/predicted rows are retained for inspection.</p>
<p>Baseline commit: <code>{baseline_ref}</code>. Startup/shooter parameters, intake/transfer speeds and counting settings were
not edited by this analysis. Confirmation used: {args.confirmation_seconds:.3f} s.</p>
<p><a href="summary.json">Machine-readable results and source hashes</a> · <a href="volley-summary.csv">All volley results</a> ·
<a href="auto-replay.csv">Autonomous replay</a> · <a href="teleop-replay.csv">Teleop replay</a>.</p></body></html>'''
    (args.output/'review.html').write_text(report,encoding='utf-8')
    for r in results:print(r['mode'],r['volley'],f"{r['feed_s']:.3f}",r['recorded_start_count'],'->',r['replay_start_count'],r['status'])
    print('Report:',args.output/'review.html')


if __name__=='__main__':main()
