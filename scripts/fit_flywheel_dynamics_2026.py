#!/usr/bin/env python3
"""Fit an offline flywheel model to the 2026-11-19 competition log cohort.

NumPy/SciPy/Matplotlib are analysis dependencies, never robot dependencies.
Training: matches 1,3,5; held-out validation: matches 2,4,6. Load training uses
only stable full-pattern/count-four feeds with both collection motors stopped.
"""
from __future__ import annotations
import argparse
import csv
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
from statistics import median
import sys

import numpy as np
from scipy.optimize import least_squares

from flywheel_dynamics_2026 import Plant, Controller, RAD_PER_RPM, simulate

MOTOR_URL = 'https://www.gobilda.com/5000-series-12vdc-motor-with-8mm-rex-pinion-shaft/'
WHEEL_URL = 'https://www.gobilda.com/3601-series-rhino-wheel-14mm-bore-72mm-diameter/'
KT = 1.47 * .0980665 / 9.2
R = 12 / 9.2
J_RING = .5 * (.166 * .45359237) * ((.033/2)**2 + (.0555/2)**2)

def number(r, k):
    return float(r[k])

def pack(rows, meta, label, feed=None):
    ct = np.array([(number(r, 'match_t_ms') - number(r, 'shooter_sample_age_ms')) / 1000 for r in rows])
    assert np.all(np.diff(ct) > 0)
    return {**meta, 'label': label, 't': ct - ct[0], 'control_t': ct,
            'relative_feed_t': None if feed is None else ct-feed,
            'rpm': np.array([number(r, 'measured_rpm') for r in rows]),
            'u': np.array([number(r, 'applied_power') for r in rows]),
            'v': np.array([number(r, 'battery_raw_v') for r in rows]),
            'target': np.array([number(r, 'controller_target_rpm') for r in rows]),
            'feed_t': feed}

def prepare(logs):
    plant_segments, load_segments, sources, startup_full = [], [], [], []
    folders = [p for p in sorted(logs.iterdir()) if p.is_dir() and '184451' not in p.name]
    for ix, folder in enumerate(folders):
        path = folder / 'shot_detail.csv'
        raw = path.read_bytes()
        sources.append({'run': folder.name, 'sha256': hashlib.sha256(raw).hexdigest()})
        with path.open(newline='') as stream:
            rows = [r for r in csv.DictReader(stream) if r['event']=='DETAIL'
                    and math.isfinite(number(r, 'shooter_sample_age_ms'))
                    and math.isfinite(number(r, 'battery_raw_v'))]
        meta = {'run': folder.name, 'match': ix//2+1,
                'mode': 'AUTO' if 'CloseShootingAlone' in folder.name else 'TELEOP',
                'training': (ix//2+1)%2 == 1}
        if meta['mode'] == 'AUTO':
            first = next(r for r in rows if r['feeding']=='true')
            t0 = number(first, 'sequence_start_t_ms') / 1000
            a = [r for r in rows if number(r, 'match_t_ms')/1000 < t0]
            plant_segments.append(pack(a, meta, 'startup', t0))
            startup_full.append((meta, rows, t0))
        seen = set()
        for first in rows:
            sid = int(first['sequence_id'])
            if first['feeding']!='true' or sid in seen:
                continue
            seen.add(sid)
            if (meta['mode']=='AUTO' and sid==1) or (meta['mode']=='TELEOP' and meta['match']==2):
                continue
            if int(first['sequence_start_beam_mask'])!=15 or int(first['sequence_start_ball_count'])!=4:
                continue
            t0 = number(first,'sequence_start_t_ms')/1000
            pre = [r for r in rows if t0-.15 <= number(r,'match_t_ms')/1000 <= t0-.01
                   and r['feeding']=='false']
            if not pre or median(number(r,'intake_velocity_command_rpm') for r in pre)!=0 \
                    or median(number(r,'transfer_velocity_command_rpm') for r in pre)!=0:
                continue
            feed = [r for r in rows if int(r['sequence_id'])==sid and r['feeding']=='true']
            target_range = max(number(r,'controller_target_rpm') for r in feed)-min(number(r,'controller_target_rpm') for r in feed)
            error = number(first,'measured_rpm')-number(first,'controller_target_rpm')
            if abs(error)>100 or target_range>=30:
                continue
            segment = [r for r in rows if t0-.2 <= number(r,'match_t_ms')/1000 <= t0+1.08]
            load_segments.append(pack(segment,meta,'volley'+str(sid),t0))
            last = max(number(r,'match_t_ms')/1000-t0 for r in feed if int(r['beam_mask'])!=0)
            tail = [r for r in feed if last+.22 <= number(r,'match_t_ms')/1000-t0 <=1.15]
            if len(tail)>=6:
                plant_segments.append(pack(tail,meta,'tail'+str(sid),t0))
    return plant_segments, load_segments, sources, startup_full

def replay(plant, segment, load=None, h=.003):
    """Replay measured actuator commands to identify/validate the plant, not a new controller."""
    t,u,v = segment['t'],segment['u'],segment['v']
    y0 = segment['rpm'][0] * RAD_PER_RPM
    initial_slope = np.polyfit(t[:min(4,len(t))],segment['rpm'][:min(4,len(t))]*RAD_PER_RPM,1)[0]
    if segment['label']=='startup':
        initial_slope=0
    omega=max(0,y0+(plant.sensor_tau_s+plant.sensor_delay_s)*initial_slope)
    measured=max(0,y0+plant.sensor_delay_s*initial_slope)
    fine_times=[0.0]
    fine_measured=[measured]
    gain = plant.kt_nm_a / plant.resistance_ohm
    for i in range(1,len(t)):
        span=t[i]-t[i-1]; steps=max(1,int(math.ceil(span/h))); step=span/steps
        relax=-math.expm1(-step/plant.sensor_tau_s)
        for j in range(steps):
            torque = 0
            if load is not None:
                elapsed=segment['relative_feed_t'][i-1]+j*step
                torque=sum(a*math.exp(-.5*((elapsed-c)/w)**2) for c,w,a in
                    zip(load['centers_s'],load['widths_s'],load['amplitudes_nm']))
                torque*=omega/(load.get('reference_rpm',3196)*RAD_PER_RPM)
            drive=gain*max(0,u[i-1]*v[i-1]-plant.ke_v_s_rad*omega)
            omega=max(0,omega+step*(drive-plant.drag_nm-torque)/plant.inertia_kg_m2)
            measured+=relax*(omega-measured)
            fine_times.append(t[i-1]+(j+1)*step)
            fine_measured.append(measured)
    query=t-plant.sensor_delay_s
    result=np.interp(query,fine_times,fine_measured)
    result[query<0]=fine_measured[0]+query[query<0]*initial_slope
    return result/RAD_PER_RPM

def fit_plant(segments):
    train=[s for s in segments if s['training']]
    def unpack(p):
        j=p[0]*1e-4
        return Plant(j,p[1],KT,R,j*p[2],.012,sensor_delay_s=p[3],
                     calibrated_weight_count=4,weight_inertia_kg_m2=J_RING)
    def residual(p):
        plant=unpack(p)
        return np.r_[np.concatenate([(replay(plant,s)-s['rpm'])/30 for s in train]),
                     (p[0]-2.2)/.7,(p[1]-.0192)/.003,(p[3]-.03)/.04]
    result=least_squares(residual,[2.2,.0192,25,.04],
        bounds=([4*J_RING*1e4,.014,1,0],[3.6,.027,70,.10]),loss='soft_l1',max_nfev=150)
    return unpack(result.x), {'success':bool(result.success),'evaluations':result.nfev,'cost':result.cost}

def decode_load(p):
    return {'reference_rpm':3196,'centers_s':list(p[:4]),'widths_s':list(p[4:8]),
            'amplitudes_nm':list(p[8:12])}

def fit_load(plant,segments):
    train=[s for s in segments if s['training']]
    p0=[.15,.28,.405,.575]+[.025,.025,.027,.035]+[.09,.09,.09,.10]
    lo=[.09,.21,.33,.48]+[.012]*4+[.005]*4
    hi=[.22,.36,.49,.68]+[.06]*4+[.25]*4
    def residual(p):
        load=decode_load(p)
        prior=np.r_[(np.array(p[:4])-np.array(p0[:4]))/.1,
                    (np.array(p[4:8])-.03)/.05]
        return np.r_[np.concatenate([(replay(plant,s,load)-s['rpm'])/35 for s in train]),prior]
    result=least_squares(residual,p0,bounds=(lo,hi),loss='soft_l1',max_nfev=140)
    common=decode_load(result.x)
    profiles=[]
    for s in segments:
        def individual(p):
            candidate={**common,'centers_s':[c+p[4] for c in common['centers_s']],
                       'amplitudes_nm':list(p[:4])}
            return np.r_[(replay(plant,s,candidate)-s['rpm'])/30,
                         (np.array(p[:4])-common['amplitudes_nm'])/.12,p[4]/.06]
        r=least_squares(individual,common['amplitudes_nm']+[0.0],
            bounds=([.001]*4+[-.06],[.3]*4+[.06]),loss='soft_l1',max_nfev=60)
        profile={**common,'centers_s':[c+r.x[4] for c in common['centers_s']],
                 'amplitudes_nm':list(r.x[:4])}
        profiles.append({'run':s['run'],'label':s['label'],'training':s['training'],
                         'profile':profile,'rmse':float(np.sqrt(np.mean((replay(plant,s,profile)-s['rpm'])**2)))})
    return common,profiles,{'success':bool(result.success),'evaluations':result.nfev,'cost':result.cost}

def validation(plant,segments,load=None):
    return [{k:s[k] for k in ['run','label','match','training']} | {
        'rmse_rpm':float(np.sqrt(np.mean((replay(plant,s,load)-s['rpm'])**2))),
        'bias_rpm':float(np.mean(replay(plant,s,load)-s['rpm'])), 'samples':len(s['rpm'])}
        for s in segments]

def closed_loop_validation(plant,load,segments,loaded):
    """Recompute controller outputs; do not replay recorded power for this check."""
    rows=[]
    zero={**load,'amplitudes_nm':[0.0]*4}
    for s in segments:
        if s['label']!='startup':
            continue
        i=np.flatnonzero(s['target']>0)[0]
        t=s['t'][i:]-s['t'][i]
        vfun=lambda t0,s=s,t=t,i=i:float(np.interp(t0,t,s['v'][i:]))
        c=Controller(target_rpm=float(s['target'][i]),feed_s=10,
            period_s=float(np.median(np.diff(t))),boost_start_floor=0,boost_end_floor=0)
        # State is just before the first controller update: do not add the
        # recorded first update's power and then ramp that same update twice.
        result=simulate(plant,c,zero,voltage=vfun,duration_s=float(t[-1]),initial_power=0)
        pred=np.interp(t,[r['t_s'] for r in result['rows']],
                       [r['measured_rpm'] for r in result['rows']])
        rows.append({'group':'startup','match':s['match'],'training':s['training'],
            'rmse_rpm':float(np.sqrt(np.mean((pred-s['rpm'][i:])**2))),
            't':t.tolist(),'rpm':s['rpm'][i:].tolist(),'prediction':pred.tolist()})
    for s in loaded:
        if s['training']:
            continue
        t=s['relative_feed_t']
        vfun=lambda t0,s=s,t=t:float(np.interp(t0,t,s['v']))
        c=Controller(target_rpm=float(np.median(s['target'])),feed_s=0,
                     period_s=float(np.median(np.diff(t))))
        i=np.flatnonzero(t>=0)[0]
        result=simulate(plant,c,load,voltage=vfun,duration_s=1.05,
                        initial_rpm=float(s['rpm'][i]),initial_power=float(s['u'][i]))
        keep=t>=0
        pred=np.interp(t[keep],[r['t_s'] for r in result['rows']],
                       [r['measured_rpm'] for r in result['rows']])
        rows.append({'group':'load','match':s['match'],'label':s['label'],'training':False,
            'rmse_rpm':float(np.sqrt(np.mean((pred-s['rpm'][keep])**2))),
            't':t[keep].tolist(),'rpm':s['rpm'][keep].tolist(),'prediction':pred.tolist()})
    return rows

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--logs',type=Path,default=Path(r'C:\Users\nicko\Downloads\logs2026\LeagueMeet_11-19-2026'))
    parser.add_argument('--output',type=Path,default=Path('Data/league-20261119/flywheel-model'))
    parser.add_argument('--model',type=Path,default=Path(__file__).with_name('flywheel_dynamics_2026.json'))
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    segments,loaded,sources,startups=prepare(args.logs)
    print('Segments:',len(segments),'unloaded;',len(loaded),'loaded',flush=True)
    plant,plant_fit=fit_plant(segments)
    print('Plant:',asdict(plant),flush=True)
    load,profiles,load_fit=fit_load(plant,loaded)
    print('Load:',load,flush=True)
    pv,lv=validation(plant,segments),validation(plant,loaded,load)
    cv=closed_loop_validation(plant,load,segments,loaded)
    model={'schema_version':1,'date':'2026-11-20','motor':{'sku':'5000-0002-4008',
        'url':MOTOR_URL,'nominal_v':12,'tested_free_rpm':5800,'stall_current_a':9.2,
        'no_load_current_a':.25,'stall_torque_nm':1.47*.0980665},
        'hardware':{'wheel_url':WHEEL_URL,'wheel_diameter_m':.072,'wheel_catalog_mass_kg':.085,
        'weight_count':4,'weight_mass_lb':.166,'weight_id_m':.033,'weight_od_m':.0555,
        'known_ring_inertia_kg_m2':4*J_RING,'as_built_confirmed_by_user':True},
        'plant':asdict(plant),'load':load,'load_profiles':profiles,'plant_fit':plant_fit,'load_fit':load_fit,
        'validation':{'plant':pv,'load_common':lv,
                      'closed_loop':[{k:v for k,v in r.items() if k not in ['t','rpm','prediction']} for r in cv]},
        'sources':sources,
        'assumptions':['Positive-drive/coast effective motor law; not a transistor-level PWM model.',
        'Effective inertia and sensor delay are jointly identified, not independent physical measurements.',
        'Four Gaussian load lobes approximate aggregate loading; their centers are not measured launch events.',
        'Torque load scales linearly with wheel speed near the calibration range.',
        'Bus voltage is prescribed; changed controller current does not recalculate battery sag.',
        'No ball launch velocity, trajectory, or scoring model is calibrated.']}
    args.model.write_text(json.dumps(model,indent=2))
    (args.output/'fit-validation.json').write_text(json.dumps(model['validation'],indent=2))
    (args.output/'closed-loop-validation.json').write_text(json.dumps(cv,indent=2))
    print('Unloaded validation:',pv,flush=True)
    print('Loaded validation:',lv,flush=True)
    # Keep fit arrays in an analysis-only file to support exact report rendering.
    arrays=[]
    for label,group,load_arg in [('plant',segments,None),('load',loaded,load)]:
        for s in group:
            arrays.append({k:s[k] for k in ['run','label','match','training']} |
                {'group':label,'t':s['t'].tolist(),'relative_feed_t':None if s['relative_feed_t'] is None else s['relative_feed_t'].tolist(),
                 'rpm':s['rpm'].tolist(),'prediction':replay(plant,s,load_arg).tolist(),
                 'u':s['u'].tolist(),'v':s['v'].tolist()})
    (args.output/'validation-arrays.json').write_text(json.dumps(arrays))

if __name__=='__main__':
    main()
