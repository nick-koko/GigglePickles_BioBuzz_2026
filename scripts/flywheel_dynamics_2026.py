#!/usr/bin/env python3
"""Offline, calibrated flywheel simulator. Never accesses robot hardware.

Use fit_flywheel_dynamics_2026.py to identify parameters from logs. The motor
law is an effective positive-drive/coast model, not a detailed Hub PWM model.
Empirical load lobes are not verified individual ball contact/release times.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass, replace
import json
import math
from pathlib import Path

RAD_PER_RPM = 2 * math.pi / 60

def clip(x, lo, hi):
    return max(lo, min(hi, x))

@dataclass(frozen=True)
class Plant:
    inertia_kg_m2: float
    ke_v_s_rad: float
    kt_nm_a: float
    resistance_ohm: float
    drag_nm: float
    sensor_tau_s: float
    sensor_delay_s: float = 0.0
    calibrated_weight_count: int = 4
    weight_inertia_kg_m2: float = .0000392412
    viscous_drag_nm_s_rad: float = 0.0

    def drag(self, omega):
        return self.drag_nm + self.viscous_drag_nm_s_rad * max(0.0, omega)

    def with_weights(self, count):
        j = self.inertia_kg_m2 + (count - self.calibrated_weight_count) * self.weight_inertia_kg_m2
        if not math.isfinite(j) or j <= 0:
            raise ValueError('Weight count gives nonpositive total inertia')
        return replace(self, inertia_kg_m2=j, calibrated_weight_count=count)

    def accel(self, omega, power, voltage, load_nm=0):
        drive_nm = self.kt_nm_a / self.resistance_ohm * max(
            0.0, power * voltage - self.ke_v_s_rad * omega)
        return (drive_nm - self.drag(omega) - load_nm) / self.inertia_kg_m2

@dataclass(frozen=True)
class Controller:
    target_rpm: float = 3196
    kp: float = .0009
    ks: float = .09
    kv: float = .000134
    up_slew_per_s: float = 1.5
    down_slew_per_s: float = 1.5
    ff_gain_min: float = 1.0
    ff_gain_max: float = 1.05
    nominal_v: float = 12.5
    voltage_filter_tau_s: float = .5
    boost_start_floor: float = .75
    boost_end_floor: float = .75
    boost_delay_s: float = .18
    boost_duration_s: float = .5
    boost_voltage_scaled: bool = False
    boost_bypasses_slew: bool = True
    overspeed_taper_rpm: float = 0.0
    period_s: float = .033
    feed_s: float = 2.0
    boost_command_lag_periods: int = 1
    startup_profile: bool = False
    startup_up_per_s: float = 10.0
    startup_down_per_s: float = 6.0
    startup_threshold_rpm: float = 200.0
    startup_max_s: float = 2.0
    startup_exit_max_slope_rpm_s: float = 200.0

    def __post_init__(self):
        for k, v in asdict(self).items():
            if isinstance(v, (float, int)) and not math.isfinite(v):
                raise ValueError(f'{k} must be finite')
        if not (0 < self.target_rpm <= 6000 and self.kp >= 0 and self.ks >= 0 and self.kv >= 0):
            raise ValueError('Invalid target or gains')
        if not (0 <= self.ff_gain_min <= self.ff_gain_max and self.nominal_v > 0):
            raise ValueError('Invalid voltage compensation')
        if min(self.up_slew_per_s, self.down_slew_per_s, self.feed_s, self.boost_duration_s,
               self.boost_delay_s, self.overspeed_taper_rpm) < 0 or self.period_s <= 0:
            raise ValueError('Times, rates and taper must be nonnegative')
        if not (0 <= self.boost_start_floor <= 1 and 0 <= self.boost_end_floor <= 1):
            raise ValueError('Boost floors must be in [0,1]')
        if min(self.startup_up_per_s, self.startup_down_per_s, self.startup_threshold_rpm,
               self.startup_max_s, self.startup_exit_max_slope_rpm_s) < 0:
            raise ValueError('Startup rates, threshold and duration must be nonnegative')

def load_lobes(t, load, omega):
    """Torque lobes scaled linearly with wheel speed (constant fitted energy law).

    Extrapolation in speed/contact physics is uncertain. The profile amplitudes
    and timing are fitted near the logged ~3000–3300 RPM range.
    """
    scale = max(0, omega) / (load.get('reference_rpm', 3196) * RAD_PER_RPM)
    gain = load.get('strength', 1.0)
    shift = load.get('shift_s', 0.0)
    return [gain * amp * math.exp(-.5 * ((t - center - shift) / width)**2) * scale
            for center, width, amp in zip(load['centers_s'], load['widths_s'], load['amplitudes_nm'])]

def motor_step(plant, omega, measured, power, voltage, load_nm, h):
    # Small-step Euler dynamics; first-order effective measurement lag.
    omega = max(0, omega + h * plant.accel(omega, power, voltage, load_nm))
    measured += -math.expm1(-h / max(.0001, plant.sensor_tau_s)) * (omega - measured)
    return omega, measured

def control_step(settings, measured_rpm, filtered_v, previous_power, dt, elapsed):
    gain = clip(settings.nominal_v / max(1, filtered_v), settings.ff_gain_min, settings.ff_gain_max)
    ff = (settings.ks + settings.kv * settings.target_rpm) * gain
    request = clip(settings.kp * (settings.target_rpm - measured_rpm) + ff, 0, 1)
    delayed_elapsed = elapsed - settings.boost_command_lag_periods * settings.period_s
    active = (settings.boost_duration_s > 0 and settings.boost_start_floor + settings.boost_end_floor > 0
              and settings.boost_delay_s <= delayed_elapsed < settings.boost_delay_s + settings.boost_duration_s)
    floor = 0.0
    if active:
        fraction = (delayed_elapsed - settings.boost_delay_s) / settings.boost_duration_s
        floor = settings.boost_start_floor + fraction * (settings.boost_end_floor - settings.boost_start_floor)
        if settings.boost_voltage_scaled:
            # Separate from the existing capped FF compensation. This preserves
            # nominal boost voltage until available power reaches 1.
            floor *= settings.nominal_v / max(1, filtered_v)
        floor = clip(floor, 0, 1)
        if settings.overspeed_taper_rpm > 0:
            alpha = clip((settings.target_rpm + settings.overspeed_taper_rpm - measured_rpm)
                         / settings.overspeed_taper_rpm, 0, 1)
            floor = request + max(0, floor - request) * alpha
    wanted = max(request, floor)
    if active and settings.boost_bypasses_slew:
        power = wanted
    else:
        power = clip(wanted, previous_power - settings.down_slew_per_s * dt,
                     previous_power + settings.up_slew_per_s * dt)
    return clip(power, 0, 1), request, floor, active

def simulate(plant, settings, load, voltage=12.5, duration_s=3.5, h=.001,
             initial_rpm=0, initial_power=0, feed_s=None, initial_target_rpm=None,
             target_change_s=0.0, target_ramp_s=0.0):
    """Closed-loop simulation, with prescribed loaded bus voltage (float/callable).

    Changes in current do not feed back into a battery impedance model. The
    physical wheel state and the lagged RPM readout are both returned.
    """
    if h <= 0 or duration_s <= 0 or initial_rpm < 0 or not 0 <= initial_power <= 1:
        raise ValueError('Invalid simulation interval/initial condition')
    if feed_s is not None:
        settings = replace(settings, feed_s=feed_s)
    if target_change_s < 0 or target_ramp_s < 0:
        raise ValueError('Target timing must be nonnegative')
    def target_at(t):
        if initial_target_rpm is None:
            return settings.target_rpm
        fraction = clip((t-target_change_s)/target_ramp_s, 0, 1) if target_ramp_s else float(t>=target_change_s)
        return initial_target_rpm + fraction*(settings.target_rpm-initial_target_rpm)
    vfun = voltage if callable(voltage) else lambda t: float(voltage)
    if vfun(0) <= 0:
        raise ValueError('Voltage must be positive')
    omega = sensor_filtered = measured = initial_rpm * RAD_PER_RPM
    sensor_history = [sensor_filtered]
    power, request, floor, active = initial_power, initial_power, 0.0, False
    vf = vfun(0)
    next_control = 0.0
    previous_control = 0.0
    rows = []
    lobes_n = len(load['centers_s'])
    sum_load = [0.0] * lobes_n
    sum_rpm = [0.0] * lobes_n
    sum_energy = [0.0] * lobes_n
    sum_error2, total_load = 0.0, 0.0
    time_error2, contact_seconds = 0.0, 0.0
    min_active, max_active, active_seconds = float('inf'), 0.0, 0.0
    next_output = 0.0
    startup = settings.startup_profile and initial_rpm < settings.startup_threshold_rpm
    stable_since = None
    last_measured_rpm = initial_rpm
    startup_end = None
    in_band_since = None
    ready_at = None
    for n in range(int(math.ceil(duration_s / h)) + 1):
        t = min(n * h, duration_s)
        v = float(vfun(t))
        if not math.isfinite(v) or v <= 0:
            raise ValueError('Voltage input must remain finite and positive')
        position = max(0, n - plant.sensor_delay_s / h)
        left = min(int(position), len(sensor_history)-1)
        right = min(left+1, len(sensor_history)-1)
        fraction = position-left
        measured = sensor_history[left] * (1-fraction) + sensor_history[right] * fraction
        target = target_at(t)
        if t + 1e-10 >= next_control:
            elapsed_dt = settings.period_s if t == 0 else t - previous_control
            vf += elapsed_dt / (settings.voltage_filter_tau_s + elapsed_dt) * (v - vf)
            mrpm = measured / RAD_PER_RPM
            if t < settings.feed_s and t >= target_change_s + target_ramp_s:
                if abs(mrpm - settings.target_rpm) > 75:
                    in_band_since = None
                elif in_band_since is None:
                    in_band_since = t
                elif ready_at is None and t - in_band_since >= .15:
                    ready_at = t
            if startup:
                stable = (abs(mrpm-target) <= 50 and
                          abs(mrpm-last_measured_rpm)/elapsed_dt <= settings.startup_exit_max_slope_rpm_s)
                stable_since = t if stable and stable_since is None else stable_since if stable else None
                # A one-way mode handoff, never a permission gate for feeding.
                # Timeout/feeding also hands off: shot sag cannot re-arm startup.
                if (t >= settings.startup_max_s or t >= settings.feed_s or
                    (stable_since is not None and t-stable_since >= .15)):
                    startup, startup_end = False, t
            effective = replace(settings, target_rpm=target,
                up_slew_per_s=settings.startup_up_per_s if startup else settings.up_slew_per_s,
                down_slew_per_s=settings.startup_down_per_s if startup else settings.down_slew_per_s)
            power, request, floor, active = control_step(effective, mrpm, vf,
                power, elapsed_dt, t - settings.feed_s)
            last_measured_rpm = mrpm
            previous_control, next_control = t, next_control + settings.period_s
        lobes = load_lobes(t - settings.feed_s, load, omega)
        tau = sum(lobes)
        rpm = omega / RAD_PER_RPM
        # Active window is defined by fitted contact support, not feed timer.
        contact_lo = min(c - 3*w for c, w in zip(load['centers_s'], load['widths_s'])) + load.get('shift_s', 0)
        contact_hi = max(c + 3*w for c, w in zip(load['centers_s'], load['widths_s'])) + load.get('shift_s', 0)
        in_window = contact_lo <= t - settings.feed_s <= contact_hi
        if in_window:
            min_active, max_active = min(min_active, rpm), max(max_active, rpm)
            time_error2 += h * (rpm - target)**2
            contact_seconds += h
            active_seconds += h * (power >= .99)
            sum_error2 += tau * h * (rpm - target)**2
            total_load += tau * h
            for j, torque in enumerate(lobes):
                sum_load[j] += torque * h
                sum_rpm[j] += torque * h * rpm
                sum_energy[j] += torque * h * omega
        if t + 1e-10 >= next_output:
            rows.append({'t_s': t, 'physical_rpm': rpm, 'measured_rpm': measured / RAD_PER_RPM,
                         'power': power, 'controller_request': request, 'boost_floor': floor,
                         'voltage': v, 'filtered_v': vf, 'load_nm': tau, 'target_rpm': target,
                         'startup_active': startup,
                         'estimated_drive_current_a': max(0, power*v-plant.ke_v_s_rad*omega)/plant.resistance_ohm})
            next_output += .01
        if t >= duration_s:
            break
        omega, sensor_filtered = motor_step(plant, omega, sensor_filtered, power, v, tau, min(h, duration_s-t))
        sensor_history.append(sensor_filtered)
    contact_rpms = [num/den if den > 0 else None for num, den in zip(sum_rpm, sum_load)]
    summary = {
        'contact_window_rms_error_rpm': math.sqrt(time_error2/contact_seconds) if contact_seconds>0 else None,
        'load_weighted_rms_error_rpm': math.sqrt(sum_error2 / total_load) if total_load > 0 else None,
        'load_weighted_mean_rpm': sum(sum_rpm) / sum(sum_load) if sum(sum_load) > 0 else None,
        'contact_proxy_rpms': contact_rpms,
        'contact_proxy_range_rpm': max(contact_rpms)-min(contact_rpms) if all(x is not None for x in contact_rpms) else None,
        'min_contact_rpm': None if min_active == float('inf') else min_active,
        'max_contact_rpm': max_active, 'load_work_j': sum(sum_energy),
        'full_power_during_contact_s': active_seconds,
        'contact_start_s': settings.feed_s + contact_lo,
        'contact_end_s': settings.feed_s + contact_hi,
        'startup_handoff_s': startup_end,
        'measured_ready_75rpm_150ms_s': ready_at,
        'peak_estimated_drive_current_a': max(r['estimated_drive_current_a'] for r in rows),
    }
    before = [r for r in rows if r['t_s'] < settings.feed_s]
    if before:
        summary['rpm_at_feed'] = min(rows, key=lambda r: abs(r['t_s']-settings.feed_s))['physical_rpm']
        summary['pre_feed_peak_rpm'] = max(r['physical_rpm'] for r in before)
        eligible = [r for r in before if r['t_s'] >= target_change_s+target_ramp_s]
        summary['first_within_50_s'] = next((r['t_s'] for r in eligible
                                            if abs(r['physical_rpm'] - settings.target_rpm) <= 50), None)
        summary['settled_within_50_s'] = next((r['t_s'] for i, r in enumerate(eligible)
            if settings.feed_s-r['t_s'] >= .25 and
            all(abs(s['physical_rpm'] - settings.target_rpm) <= 50 for s in eligible[i:])), None)
    return {'settings': asdict(settings), 'rows': rows, 'summary': summary}

def load_model(path, calibration='league'):
    data = json.loads(Path(path).read_text())
    key = {'league': 'plant', 'startup': 'startup_plant'}[calibration]
    return Plant(**data[key]), data['load'], data

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=Path(__file__).with_suffix('.json'))
    parser.add_argument('--target', type=float, default=3196)
    parser.add_argument('--calibration', choices=['league', 'startup'], default='league')
    parser.add_argument('--tested-startup', action='store_true', help='Use Quick Screen pre-feed settings')
    parser.add_argument('--fourth-shift-ms', type=float, default=0, help='Shift only the fourth modeled load pulse')
    parser.add_argument('--voltage', type=float, default=12.5)
    parser.add_argument('--down', type=float, default=1.5)
    parser.add_argument('--ff-min', type=float, default=1.0)
    parser.add_argument('--boost', type=float, default=.75)
    parser.add_argument('--boost-end', type=float)
    parser.add_argument('--boost-voltage-scaled', action='store_true')
    parser.add_argument('--taper', type=float, default=0)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    plant, load, data = load_model(args.model, args.calibration)
    if not -60 <= args.fourth_shift_ms <= 60:
        parser.error('--fourth-shift-ms must be between -60 and 60')
    load = {**load, 'centers_s': [*load['centers_s'][:3], load['centers_s'][3]+args.fourth_shift_ms/1000]}
    ctrl = Controller(target_rpm=args.target, down_slew_per_s=args.down, ff_gain_min=args.ff_min,
        boost_start_floor=args.boost, boost_end_floor=args.boost if args.boost_end is None else args.boost_end,
        boost_voltage_scaled=args.boost_voltage_scaled, overspeed_taper_rpm=args.taper)
    if args.tested_startup:
        ctrl = replace(ctrl, startup_profile=True, startup_up_per_s=6, startup_down_per_s=6,
                       ff_gain_min=.90, ff_gain_max=1.10)
    result = simulate(plant, ctrl, load, voltage=args.voltage)
    print(json.dumps(result['summary'], indent=2))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(result['rows'][0]))
            writer.writeheader(); writer.writerows(result['rows'])

if __name__ == '__main__':
    main()
