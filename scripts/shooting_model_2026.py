#!/usr/bin/env python3
"""Offline shooting diagnostics; standard library only, never sends robot commands.

Controller defaults are a source snapshot, not automatically linked to Panels.
Flight is an uncalibrated no-drag sensitivity model, NOT an RPM recommendation.
See Shooting_Analysis_2026.md for identification, assumptions and experiments.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics

G = 9.80665


def finite(*values):
    if not all(math.isfinite(v) for v in values):
        raise ValueError("All model inputs must be finite")


def clip(value, low, high):
    return max(low, min(high, value))


def controller_step(target_rpm, measured_rpm, battery_v, previous_power, dt_s,
                    kp=.0009, ki=0, kd=0, ks=.09, kv=.000134, ka=0,
                    integral=0, derivative_rpm_s=0, acceleration_rpm_s=0,
                    filtered_v=None, voltage_comp=True, nominal_v=12.5,
                    gain_min=1, gain_max=1.05, slew_per_s=1.5,
                    boost_floor=None, boost_bypass=True):
    """Normal enabled update; integral/derivative/filter states supplied, not simulated."""
    if filtered_v is None:
        filtered_v = battery_v
    finite(target_rpm, measured_rpm, battery_v, previous_power, dt_s, kp, ki, kd,
           ks, kv, ka, integral, derivative_rpm_s, acceleration_rpm_s, filtered_v,
           nominal_v, gain_min, gain_max, slew_per_s)
    if dt_s <= 0 or battery_v <= 0 or filtered_v <= 0 or gain_min > gain_max:
        raise ValueError("Positive time/voltage and ordered compensation bounds required")
    if not 0 <= previous_power <= 1 or target_rpm < 0 or measured_rpm < 0:
        raise ValueError("Power must be in [0,1] and RPM nonnegative")
    if boost_floor is not None:
        finite(boost_floor)
        if not 0 <= boost_floor <= 1:
            raise ValueError("Boost floor must be in [0,1]")
    gain = clip(nominal_v / filtered_v, gain_min, gain_max) if voltage_comp and filtered_v >= 7 else 1
    feedback = kp * (target_rpm - measured_rpm) + ki * integral + kd * derivative_rpm_s
    ff = (ks if target_rpm > 0 else 0) + kv * target_rpm + ka * acceleration_rpm_s
    before_boost = clip(feedback + gain * ff, 0, 1)
    step = max(0, slew_per_s) * dt_s
    without = clip(before_boost, previous_power - step, previous_power + step)
    requested = max(before_boost, boost_floor) if boost_floor is not None else before_boost
    applied = requested if boost_floor is not None and boost_bypass else clip(
        requested, previous_power - step, previous_power + step)
    if target_rpm == 0:
        before_boost = without = requested = applied = feedback = ff = 0
    return dict(compensation_gain=gain, feedback_power=feedback, raw_feedforward_power=ff,
                controller_power_before_boost=before_boost,
                no_boost_output_same_loop=without, output_before_slew=requested,
                applied_power=applied, boost_applied_power_difference=applied-without,
                approximate_motor_voltage=applied*battery_v,
                note="Duty times bus voltage is an estimate; no motor/ball dynamics simulated")


def flight(rpm, diameter_m, coupling, angle_deg, height_m, distance_m, target_height_m):
    """Stationary, aligned shot crossing a vertical plane distance_m from the exit."""
    finite(rpm, diameter_m, coupling, angle_deg, height_m, distance_m, target_height_m)
    if min(rpm, diameter_m, coupling, distance_m) <= 0 or not -89 < angle_deg < 89:
        raise ValueError("Positive RPM/diameter/coupling/distance and angle between -89 and 89 required")
    if min(height_m, target_height_m) < 0:
        raise ValueError("Heights must be nonnegative")
    theta = math.radians(angle_deg)
    rim_speed = math.pi * diameter_m * rpm / 60
    speed = coupling * rim_speed
    vx, vz = speed * math.cos(theta), speed * math.sin(theta)
    t = distance_m / vx
    z = height_m + vz*t - .5*G*t*t
    ground_time = (vz + math.sqrt(vz*vz + 2*G*height_m)) / G
    denominator = 2 * math.cos(theta)**2 * (distance_m*math.tan(theta) - (target_height_m-height_m))
    required_speed = math.sqrt(G*distance_m**2/denominator) if denominator > 0 else None
    return dict(model="No drag, no spin lift, no collision; calibrated coupling required",
                wheel_rim_speed_m_s=rim_speed, exit_speed_m_s=speed,
                time_to_goal_plane_s=t, height_at_goal_plane_m=z,
                vertical_error_at_goal_plane_m=z-target_height_m,
                vertical_speed_at_goal_plane_m_s=vz-G*t,
                descending_at_goal_plane=(vz-G*t < 0), reaches_plane_before_ground=(t <= ground_time),
                ideal_required_exit_speed_m_s=required_speed,
                ideal_required_rpm=(required_speed*60/(coupling*math.pi*diameter_m)
                                    if required_speed is not None else None))


def inertia_change(before_rpm, after_rpm, inertia_multiplier):
    """Same energy draw/initial speed; ignores motor work during contact and altered slip."""
    finite(before_rpm, after_rpm, inertia_multiplier)
    if before_rpm <= 0 or not 0 <= after_rpm <= before_rpm or inertia_multiplier <= 0:
        raise ValueError("Require before>0, 0<=after<=before, inertia multiplier>0")
    retained_energy = 1 - (1 - (after_rpm/before_rpm)**2) / inertia_multiplier
    if retained_energy < 0:
        raise ValueError("Assumed extraction exceeds available energy at this inertia")
    return dict(ideal_after_rpm=before_rpm*math.sqrt(retained_energy),
                note="Illustrative equal-energy comparison, not a measured recovery prediction")


def annulus_inertia(mass_kg, inside_diameter_m, outside_diameter_m):
    finite(mass_kg, inside_diameter_m, outside_diameter_m)
    if mass_kg <= 0 or not 0 <= inside_diameter_m < outside_diameter_m:
        raise ValueError("Positive mass and 0 <= inside diameter < outside diameter required")
    return .5 * mass_kg * ((inside_diameter_m/2)**2 + (outside_diameter_m/2)**2)


def guide_geometry(wheel_diameter_m, guide_radius_m, ball_diameter_m,
                   shaft_height_m, edge_vertical_offset_m):
    """Concentric guide, upward travel at left edge. Nominal geometry only.

    Offset is guide-edge height minus shaft height. Undeformed ball radius is
    used to offset its center from the guide; actual elastic release can differ.
    """
    finite(wheel_diameter_m, guide_radius_m, ball_diameter_m, shaft_height_m, edge_vertical_offset_m)
    if min(wheel_diameter_m, guide_radius_m, ball_diameter_m, shaft_height_m) <= 0:
        raise ValueError("Positive geometry dimensions required")
    if guide_radius_m <= max(wheel_diameter_m/2, ball_diameter_m/2):
        raise ValueError("Guide radius must exceed wheel and ball radii")
    if abs(edge_vertical_offset_m) >= guide_radius_m:
        raise ValueError("Edge vertical offset must lie inside guide radius")
    clearance = guide_radius_m-wheel_diameter_m/2
    return dict(radial_clearance_m=clearance,
                geometric_interference_m=ball_diameter_m-clearance,
                approximate_guide_tangent_elevation_deg=math.degrees(
                    math.acos(-edge_vertical_offset_m/guide_radius_m)),
                approximate_ball_center_height_at_guide_edge_m=shaft_height_m
                    + edge_vertical_offset_m*(guide_radius_m-ball_diameter_m/2)/guide_radius_m)


def hardware_comparison(profile):
    p = json.loads(Path(profile).read_text(encoding="utf-8"))
    mass = p['weight_mass_lb_each'] * .45359237
    each = annulus_inertia(mass, p['weight_inside_diameter_m'], p['weight_outside_diameter_m'])
    old, new = p['current_weight_count'], p['proposed_weight_count']
    if not isinstance(old,int) or not isinstance(new,int) or min(old,new) < 1:
        raise ValueError("Positive integer weight counts required")
    finite(p['wheel_diameter_m'], *p['comparison_rpm'])
    if p['wheel_diameter_m'] <= 0 or any(rpm <= 0 for rpm in p['comparison_rpm']):
        raise ValueError("Positive wheel diameter and comparison RPM required")
    return dict(profile=p, weight_mass_kg_each=mass, weight_inertia_kg_m2_each=each,
                guide_geometry_estimates=[dict(ball_diameter_m=diameter, **guide_geometry(
                    p['wheel_diameter_m'], p['guide_radius_m'], diameter, p['reported_mount_height_m'],
                    p['cad_approx_guide_edge_height_above_base_m']-p['cad_approx_shaft_height_above_base_m']))
                    for diameter in (p['ball_nominal_diameter_m'],p['ball_typical_reported_diameter_m'])],
                current_weights_inertia_kg_m2=old*each, proposed_weights_inertia_kg_m2=new*each,
                weights_only_inertia_increase_percent=100*(new/old-1),
                comparisons=[dict(rpm=rpm, rim_speed_m_s=math.pi*p['wheel_diameter_m']*rpm/60,
                    current_weights_energy_J=.5*old*each*(rpm*math.pi/30)**2,
                    proposed_weights_energy_J=.5*new*each*(rpm*math.pi/30)**2)
                    for rpm in p['comparison_rpm']],
                note="Weights only: total inertia/energy also includes wheel, hubs, shaft and reflected motor inertia. "
                     "Rim speed is not ball speed. Guide geometry uses explicitly provisional CAD edge coordinates; "
                     "actual free-flight angle/height remain uncalibrated.")


def number(row, key):
    try:
        value = float(row.get(key, ""))
        return value if math.isfinite(value) else None
    except (ValueError, TypeError):
        return None


def stats(rows, key):
    values = sorted(v for row in rows if (v := number(row, key)) is not None)
    if not values:
        return None
    return dict(n=len(values), min=values[0], median=statistics.median(values),
                p95=values[max(0, math.ceil(.95*len(values))-1)], max=values[-1])


def summarize(patterns):
    from convert_logs_2026 import input_files, file_info, rows as csv_rows
    summaries, sessions = [], set()
    for path in input_files(patterns):
        session, detail = file_info(path)
        if not detail:
            continue  # Do not count basic events as additional measured samples.
        if session in sessions:
            raise ValueError(f"Duplicate detailed log for session {session}")
        sessions.add(session)
        all_rows = [row for _, row in csv_rows(path, "run")]
        samples = [r for r in all_rows if r.get("event") == "DETAIL"]
        configs = [dict(match_t_ms=number(r, "match_t_ms"), note=r.get("note", ""))
                   for r in all_rows if r.get("event") == "CONFIG"]
        # Older detail logs store provenance only in the companion basic file.
        companion = path.with_name("match.csv")
        if not configs and companion.exists():
            other_session, _ = file_info(companion)
            if other_session == session:
                configs = [dict(match_t_ms=t, note=r.get("note", ""))
                           for t, r in csv_rows(companion, "run") if r.get("event") == "CONFIG"]
        groups = {}
        for row in samples:
            if row.get("sequence_active") == "true":
                groups.setdefault(row.get("sequence_id"), []).append(row)
        volleys = []
        for sequence_id, active in groups.items():
            first = active[0]
            start = number(first, "sequence_start_t_ms")
            if start is None:
                start = number(first, "match_t_ms")
            pre = [r for r in samples if start-200 <= number(r, "match_t_ms") < start]
            # These are feed-window extrema; never inferred individual ball launches.
            volleys.append(dict(sequence_id=sequence_id, start_ms=start,
                first_observed_active_ms=number(first, "match_t_ms"),
                start_observation_lag_ms=number(first, "match_t_ms")-start,
                samples=len(active), profile=first.get("profile"),
                source=first.get("rpm_source"), tracked_start_balls=number(first,"sequence_start_ball_count"),
                pre_200ms_rpm=stats(pre,"measured_rpm"),
                feed_target_rpm=stats(active,"target_rpm"),
                feed_controller_target_rpm=stats(active,"controller_target_rpm"),
                feed_measured_rpm=stats(active,"measured_rpm"),
                feed_battery_v=stats(active,"battery_raw_v"),
                feed_current_a=stats(active,"shooter_current_a"),
                feed_extra_boost_power=stats(active,"boost_applied_power_difference"),
                feed_shooter_sample_age_ms=stats(active,"shooter_sample_age_ms")))
        summaries.append(dict(session=session, file=str(path), configurations=configs,
            detail_samples=len(samples), loop_period_ms=stats(samples,"loop_period_ms"),
            capture_ms=stats(samples,"previous_capture_ms"),
            dropped_detail_rows=stats(samples,"dropped_detail_rows"),
            volleys=volleys))
    if not summaries:
        raise ValueError("No detailed logs found; enable ENABLE_SHOT_DETAIL_LOG for this analysis")
    return dict(note="Observed feed-window summaries only. No inferred launch times, scores or causality. "
                     "Percentiles are sample-based. Drop counters are observed snapshots, not final totals.",
                runs=summaries)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    controller = commands.add_parser("controller", help="One controller step, no plant simulation")
    for name, default in (("target-rpm",2950),("measured-rpm",2950),("battery-v",13.5),
                          ("previous-power",.5),("dt-s",.02),("kp",.0009),("ks",.09),("kv",.000134)):
        controller.add_argument("--"+name, type=float, default=default)
    controller.add_argument("--boost-floor", type=float)
    projectile = commands.add_parser("flight", help="Ideal sensitivity calculation; fill measured geometry")
    for name in ("rpm","diameter-m","coupling","angle-deg","height-m","distance-m","target-height-m"):
        projectile.add_argument("--"+name, type=float, required=True)
    inertia = commands.add_parser("inertia", help="Equal-energy comparison for different inertia")
    for name in ("before-rpm","after-rpm","inertia-multiplier"):
        inertia.add_argument("--"+name, type=float, required=True)
    summary = commands.add_parser("summarize", help="Summarize actual downloaded shot_detail.csv logs")
    summary.add_argument("--input", nargs="+", required=True)
    hardware = commands.add_parser("hardware", help="72 mm wheel and three-versus-four weight calculation")
    hardware.add_argument("--profile", type=Path, default=Path(__file__).with_name("shooting_hardware_2026.json"))
    for command in (controller, projectile, inertia, summary, hardware):
        command.add_argument("--output", type=Path, help="New JSON file; existing files are never overwritten")
    args = vars(parser.parse_args())
    command, output = args.pop("command"), args.pop("output")
    try:
        if command == "controller": result = controller_step(**args)
        elif command == "flight": result = flight(**args)
        elif command == "inertia": result = inertia_change(**args)
        elif command == "hardware": result = hardware_comparison(args['profile'])
        else: result = summarize(args["input"])
        text = json.dumps(result, indent=2, allow_nan=False) + "\n"
        if output:
            with output.open("x", encoding="utf-8") as stream:
                stream.write(text)
        else:
            print(text, end="")
    except (ValueError, OSError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
