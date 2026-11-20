/* Offline companion to flywheel_dynamics_2026.py; no hardware/network APIs. */
(function (host) {
  'use strict';
  const RAD = 2 * Math.PI / 60;
  const defaults = {
    target_rpm: 3196, kp: .0009, ks: .09, kv: .000134,
    up_slew_per_s: 1.5, down_slew_per_s: 1.5, ff_gain_min: 1,
    ff_gain_max: 1.05, nominal_v: 12.5, voltage_filter_tau_s: .5,
    boost_start_floor: .75, boost_end_floor: .75, boost_delay_s: .18,
    boost_duration_s: .5, boost_voltage_scaled: false, boost_bypasses_slew: true,
    overspeed_taper_rpm: 0, period_s: .033, feed_s: 2,
    boost_command_lag_periods: 1, startup_profile:false, startup_up_per_s:10,
    startup_down_per_s:6, startup_threshold_rpm:200,startup_max_s:2
  };
  const clip = (x, lo, hi) => Math.max(lo, Math.min(hi, x));
  function simulate(plant, settings, load, voltage=12.5, duration=3.5, h=.001, initial={}) {
    const c = {...defaults, ...settings};
    const initialRPM=initial.initial_rpm||0,change=initial.target_change_s||0,ramp=initial.target_ramp_s||0;
    const targetAt=t=>initial.initial_target_rpm===undefined ? c.target_rpm :
      initial.initial_target_rpm+clip(ramp?(t-change)/ramp:Number(t>=change),0,1)*(c.target_rpm-initial.initial_target_rpm);
    let w=initialRPM*RAD, sensor=w, measured=w, power=initial.initial_power||0, request=power, floor=0;
    let filteredV=voltage, nextControl=0, previousControl=0, nextOutput=0;
    const history=[w], rows=[];
    let startup=c.startup_profile&&initialRPM<c.startup_threshold_rpm,stableSince=null,lastMeasured=initialRPM,handoff=null;
    const amps=load.amplitudes_nm, centers=load.centers_s, widths=load.widths_s;
    const gain=load.strength===undefined ? 1 : load.strength;
    const shift=load.shift_s || 0;
    const lo=Math.min(...centers.map((x,i)=>x-3*widths[i]))+shift;
    const hi=Math.max(...centers.map((x,i)=>x+3*widths[i]))+shift;
    const sumLoad=amps.map(()=>0), sumRPM=amps.map(()=>0), energy=amps.map(()=>0);
    let squared=0, totalLoad=0, minRPM=Infinity, maxRPM=0, fullPower=0,timeError2=0,contactSeconds=0;
    for(let n=0;n<=Math.ceil(duration/h);n++) {
      const t=Math.min(n*h,duration);
      const position=Math.max(0,n-(plant.sensor_delay_s||0)/h);
      const left=Math.min(Math.floor(position),history.length-1),right=Math.min(left+1,history.length-1);
      const fraction=position-left;
      measured=history[left]*(1-fraction)+history[right]*fraction;
      const target=targetAt(t);
      if(t+1e-10>=nextControl) {
        const dt=t===0 ? c.period_s : t-previousControl;
        filteredV+=dt/(c.voltage_filter_tau_s+dt)*(voltage-filteredV);
        const mrpm=measured/RAD;
        if(startup) {
          const stable=Math.abs(mrpm-target)<=50 && Math.abs(mrpm-lastMeasured)/dt<=100;
          stableSince=stable ? (stableSince===null?t:stableSince) : null;
          if(t>=c.startup_max_s || t>=c.feed_s || (stableSince!==null && t-stableSince>=.15)){
            startup=false;handoff=t;
          }
        }
        const vg=clip(c.nominal_v/Math.max(1,filteredV),c.ff_gain_min,c.ff_gain_max);
        const ff=(c.ks+c.kv*target)*vg;
        request=clip(c.kp*(target-mrpm)+ff,0,1);
        const elapsed=t-c.feed_s-c.boost_command_lag_periods*c.period_s;
        const active=c.boost_duration_s>0 && c.boost_start_floor+c.boost_end_floor>0 &&
          c.boost_delay_s<=elapsed && elapsed<c.boost_delay_s+c.boost_duration_s;
        floor=0;
        if(active) {
          const ratio=(elapsed-c.boost_delay_s)/c.boost_duration_s;
          floor=c.boost_start_floor+ratio*(c.boost_end_floor-c.boost_start_floor);
          if(c.boost_voltage_scaled) floor*=c.nominal_v/Math.max(1,filteredV);
          floor=clip(floor,0,1);
          if(c.overspeed_taper_rpm>0) {
            const a=clip((target+c.overspeed_taper_rpm-mrpm)/c.overspeed_taper_rpm,0,1);
            floor=request+Math.max(0,floor-request)*a;
          }
        }
        const wanted=Math.max(request,floor);
        power=active && c.boost_bypasses_slew ? wanted :
          clip(wanted,power-(startup?c.startup_down_per_s:c.down_slew_per_s)*dt,
            power+(startup?c.startup_up_per_s:c.up_slew_per_s)*dt);
        power=clip(power,0,1);
        lastMeasured=mrpm;
        previousControl=t;nextControl+=c.period_s;
      }
      const rpm=w/RAD;
      const torque=amps.map((a,i)=>gain*a*Math.exp(-.5*((t-c.feed_s-centers[i]-shift)/widths[i])**2)*
                            Math.max(0,w)/(load.reference_rpm*RAD));
      const tau=torque.reduce((a,b)=>a+b,0);
      if(lo<=t-c.feed_s && t-c.feed_s<=hi) {
        minRPM=Math.min(minRPM,rpm);maxRPM=Math.max(maxRPM,rpm);
        timeError2+=h*(rpm-target)**2;contactSeconds+=h;
        if(power>=.99) fullPower+=h;
        squared+=tau*h*(rpm-target)**2;totalLoad+=tau*h;
        torque.forEach((v,i)=>{sumLoad[i]+=v*h;sumRPM[i]+=v*h*rpm;energy[i]+=v*h*w;});
      }
      if(t+1e-10>=nextOutput) {
        rows.push({t_s:t,physical_rpm:rpm,measured_rpm:measured/RAD,power,
          controller_request:request,boost_floor:floor,voltage,filtered_v:filteredV,load_nm:tau,target_rpm:target,
          startup_active:startup,estimated_drive_current_a:Math.max(0,power*voltage-plant.ke_v_s_rad*w)/plant.resistance_ohm});
        nextOutput+=.01;
      }
      if(t>=duration)break;
      const step=Math.min(h,duration-t);
      const drive=plant.kt_nm_a/plant.resistance_ohm*Math.max(0,power*voltage-plant.ke_v_s_rad*w);
      w=Math.max(0,w+step*(drive-plant.drag_nm-tau)/plant.inertia_kg_m2);
      sensor+=-Math.expm1(-step/Math.max(.0001,plant.sensor_tau_s))*(w-sensor);
      history.push(sensor);
    }
    const proxy=sumLoad.map((den,i)=>den>0 ? sumRPM[i]/den : null);
    const sum=a=>a.reduce((x,y)=>x+y,0);
    const before=rows.filter(r=>r.t_s<c.feed_s);
    const summary={
      contact_window_rms_error_rpm:contactSeconds>0 ? Math.sqrt(timeError2/contactSeconds) : null,
      load_weighted_rms_error_rpm:totalLoad>0 ? Math.sqrt(squared/totalLoad) : null,
      load_weighted_mean_rpm:sum(sumLoad)>0 ? sum(sumRPM)/sum(sumLoad) : null,
      contact_proxy_rpms:proxy,
      contact_proxy_range_rpm:proxy.every(v=>v!==null) ? Math.max(...proxy)-Math.min(...proxy) : null,
      min_contact_rpm:Number.isFinite(minRPM)?minRPM:null,max_contact_rpm:maxRPM,
      load_work_j:sum(energy),full_power_during_contact_s:fullPower,
      contact_start_s:c.feed_s+lo,contact_end_s:c.feed_s+hi,startup_handoff_s:handoff,
      peak_estimated_drive_current_a:Math.max(...rows.map(r=>r.estimated_drive_current_a))
    };
    if(before.length) {
      summary.rpm_at_feed=rows.reduce((a,b)=>Math.abs(a.t_s-c.feed_s)<Math.abs(b.t_s-c.feed_s)?a:b).physical_rpm;
      summary.pre_feed_peak_rpm=Math.max(...before.map(r=>r.physical_rpm));
      const eligible=before.filter(r=>r.t_s>=change+ramp);
      summary.first_within_50_s=(eligible.find(r=>Math.abs(r.physical_rpm-c.target_rpm)<=50)||{}).t_s ?? null;
      summary.settled_within_50_s=(eligible.find((r,i)=>c.feed_s-r.t_s>=.25 && eligible.slice(i).every(s=>Math.abs(s.physical_rpm-c.target_rpm)<=50))||{}).t_s ?? null;
    }
    return {settings:c,rows,summary};
  }

  const controllerPresets={
    current:{down:1.5,up:1.5,boost:.75,boostEnd:.75,delay:.18,duration:.5,
      scaleBoost:false,ffMin:1,ffMax:1.05,taper:0,kp:.0009,startup:false,startUp:10,startDown:6},
  };
  controllerPresets['startup-only']={...controllerPresets.current,startup:true};
  controllerPresets['startup-scaled']={...controllerPresets['startup-only'],scaleBoost:true,ffMin:.9};
  controllerPresets.down={...controllerPresets.current,down:3};
  controllerPresets.scaled={...controllerPresets.current,scaleBoost:true,ffMin:.9};
  controllerPresets.combined={...controllerPresets.scaled,down:3};
  controllerPresets['earlier-ramp']={...controllerPresets.combined,boost:.8,boostEnd:.9,delay:.04,taper:100};
  const stateDefaults={...controllerPresets['startup-scaled'],
    weights:4,strength:1,shift:0,width:1,feed:2,view:'sequence',preset:'startup-scaled',showPower:true,
    aScenario:'auto-start',aVoltage:13.8,aInitial:0,aTarget:3196,aRamp:0,
    bScenario:'tele-late',bVoltage:11.8,bInitial:3150,bTarget:3350,bRamp:1};
  function compare(model,state={}) {
    const s={...stateDefaults,...state};
    const plant={...model.plant,inertia_kg_m2:model.plant.inertia_kg_m2+
      (s.weights-model.plant.calibrated_weight_count)*model.plant.weight_inertia_kg_m2};
    const load={...model.load,strength:s.strength,shift_s:s.shift,
      widths_s:model.load.widths_s.map(x=>x*s.width),amplitudes_nm:model.load.amplitudes_nm.map(x=>x/s.width)};
    const panels=['a','b'].map(prefix=>{
      const voltage=s[prefix+'Voltage'],initialRPM=s[prefix+'Initial'],target=s[prefix+'Target'];
      const initial={initial_rpm:initialRPM,
        initial_power:initialRPM>0?clip((plant.ke_v_s_rad*initialRPM*RAD+
          plant.drag_nm*plant.resistance_ohm/plant.kt_nm_a)/voltage,0,1):0};
      if(initialRPM>0)Object.assign(initial,{initial_target_rpm:initialRPM,target_change_s:.5,target_ramp_s:s[prefix+'Ramp']});
      const current=simulate(plant,{target_rpm:target,feed_s:s.feed},load,voltage,Math.max(3.5,s.feed+1),.001,initial);
      const candidate=simulate(plant,{target_rpm:target,down_slew_per_s:s.down,up_slew_per_s:s.up,
        kp:s.kp,ff_gain_min:s.ffMin,ff_gain_max:s.ffMax,boost_start_floor:s.boost,
        boost_end_floor:s.boostEnd,boost_delay_s:s.delay,boost_duration_s:s.duration,
        boost_voltage_scaled:s.scaleBoost,overspeed_taper_rpm:s.taper,feed_s:s.feed,
        startup_profile:s.startup,startup_up_per_s:s.startUp,startup_down_per_s:s.startDown},
        load,voltage,Math.max(3.5,s.feed+1),.001,initial);
      return {prefix,initialRPM,target,voltage,current,candidate};
    });
    return {state:s,panels};
  }
  function chartConfig(result,quantity='rpm') {
    const s=result.state, full=s.view==='sequence';
    const sets=result.panels.map(panel=>panel.current.rows.map((r,i)=>{
      const b=panel.candidate.rows[i];
      return quantity==='rpm'?[r.t_s,r.physical_rpm,b.physical_rpm,r.target_rpm]:
        [r.t_s,100*r.power,100*b.power,100*Math.max(b.controller_request,b.boost_floor)];
    }).filter(r=>full || s.feed-.15<=r[0] && r[0]<=s.feed+.85));
    const values=sets.flatMap(rows=>rows.flatMap(r=>r.slice(1)));
    const ylo=quantity==='power'||full?0:Math.floor((Math.min(...values)-100)/100)*100;
    const yhi=quantity==='power'?100:Math.ceil((Math.max(...values)+100)/100)*100;
    const series=quantity==='rpm'?['Current settings','Candidate','RPM reference']:['Current output','Candidate output','Candidate request'];
    return {charts:result.panels.map((panel,i)=>({id:'flywheel-'+quantity+'-'+panel.prefix,
      title:panel.prefix.toUpperCase()+' · '+panel.voltage.toFixed(1)+' V · '+panel.initialRPM+' → '+panel.target+' RPM',
      description:quantity==='rpm'?'Estimated physical RPM. Running target changes start at 0.5 s; feed at '+s.feed+' s.':
        'Motor power commands, percent. Candidate request includes boost before slew. Same time scale as RPM.',
      height:quantity==='rpm'?300:200,columns:['Seconds',...series],rows:sets[i],
      spec:{type:'line',x:'Seconds',y:series,xLabel:'Seconds after enable / scenario start',
        yLabel:quantity==='rpm'?'Estimated wheel RPM':'Power command (%)',startAtZero:false,
        yDomain:[ylo,yhi],xDomain:full?[0,Math.max(3.5,s.feed+1)]:[s.feed-.15,s.feed+.85],
        showLegend:true,valueDecimals:0,colors:{[series[0]]:'var(--chart-1)',[series[1]]:'var(--chart-2)',[series[2]]:'var(--secondary)'}}}))};
  }
  const api={defaults,stateDefaults,controllerPresets,simulate,compare,chartConfig};
  if(typeof module!=='undefined' && module.exports)module.exports=api;
  host.FlywheelModel2026=api;
})(typeof globalThis!=='undefined'?globalThis:this);
