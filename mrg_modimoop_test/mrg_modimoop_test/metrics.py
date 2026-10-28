"""Controller assertions on recorded ROS and Gazebo observations."""
import math


def wrap(value):
    return math.remainder(value, 2 * math.pi)


def evaluate(cfg, rows, start):
    mode = cfg['mode']
    heading = rows['truth'] if mode == 'simulation' else rows['heading']
    samples = [(r[0] - start, r[3] if mode == 'simulation' else r[1])
               for r in heading if start <= r[0] <= start + cfg['duration_s']]
    targets = cfg['targets']
    errors = []
    for t, h in samples:
        target = targets[0]['heading_deg']
        for entry in targets:
            if t >= entry['at_s']:
                target = entry['heading_deg']
        errors.append((t, math.degrees(wrap(math.radians(target) - h))))
    metrics = {'sample_count': len(samples), 'heading_error_deg': errors}
    failures = []
    if len(samples) < 3:
        return metrics, ['insufficient measured heading samples']
    if not all(math.isfinite(h) for _, h in samples):
        failures.append('nonfinite measured heading')
    tail_start = max(targets[-1]['at_s'], cfg['duration_s'] - cfg['evaluation'].get('steady_window_s', 2.0))
    tail = [abs(e) for t, e in errors if t >= tail_start]
    metrics['steady_state_error_deg'] = sum(tail) / len(tail) if tail else None
    if not tail:
        failures.append('insufficient steady-state samples')
    band = cfg['evaluation'].get('settling_band_deg', 10.0)
    dwell = cfg['evaluation'].get('settling_dwell_s', 1.0)
    final_start = targets[-1]['at_s']
    settling = None
    for i, (t, e) in enumerate(errors):
        if t < final_start or abs(e) > band:
            continue
        window = [(tj, ej) for tj, ej in errors[i:] if tj <= t + dwell]
        if window and window[-1][0] >= t + dwell and all(abs(ej) <= band for _, ej in window):
            settling = t - final_start
            break
    metrics['settling_time_s'] = settling
    initial_error = errors[0][1]
    after = [e for t, e in errors if t >= final_start]
    metrics['overshoot_deg'] = (max(0.0, -min(after)) if initial_error > 0 else
                                max(0.0, max(after))) if after and len(targets) == 1 else None
    for kind in ('rudder', 'backwing'):
        values = [(t - start, value) for t, value in rows[kind]
                  if start <= t <= start + cfg['duration_s']]
        metrics[f'{kind}_count'] = len(values)
        if not values:
            failures.append(f'missing {kind} output')
            continue
        if not all(math.isfinite(v) for _, v in values):
            failures.append(f'nonfinite {kind} output')
        limit = cfg['thresholds'].get(f'{kind}_limit_rad', 0.785398 if kind == 'rudder' else 0.349066)
        violations = [v for _, v in values if math.isfinite(v) and abs(v) > limit + 1e-5]
        metrics[f'{kind}_limit_violations'] = len(violations)
        if violations:
            failures.append(f'{kind} exceeded {limit} rad')
        saturation = 0.0
        for (t1, v1), (t2, _) in zip(values, values[1:]):
            if abs(v1) >= limit - 0.01:
                saturation += max(0.0, t2 - t1)
        metrics[f'{kind}_saturation_s'] = saturation
    thresholds = cfg['thresholds']
    if 'max_steady_error_deg' in thresholds and metrics['steady_state_error_deg'] is not None:
        if metrics['steady_state_error_deg'] > thresholds['max_steady_error_deg']:
            failures.append(f"steady heading error {metrics['steady_state_error_deg']:.2f} deg exceeds {thresholds['max_steady_error_deg']} deg")
    if 'max_settling_s' in thresholds:
        if settling is None or settling > thresholds['max_settling_s']:
            failures.append(f'heading did not settle within {thresholds["max_settling_s"]} s')
    if 'max_overshoot_deg' in thresholds and metrics['overshoot_deg'] is not None:
        if metrics['overshoot_deg'] > thresholds['max_overshoot_deg']:
            failures.append(f'overshoot exceeds {thresholds["max_overshoot_deg"]} deg')
    if 'max_initial_wrapped_error_deg' in thresholds and errors:
        if abs(errors[0][1]) > thresholds['max_initial_wrapped_error_deg']:
            failures.append(f'initial wrapped error {errors[0][1]:.2f} deg too large')
    if 'min_error_reduction_deg' in thresholds and tail:
        initial_window = [abs(e) for t, e in errors if t <= 2.0]
        initial_mean = sum(initial_window) / len(initial_window) if initial_window else abs(errors[0][1])
        reduction = initial_mean - metrics['steady_state_error_deg']
        metrics['error_reduction_deg'] = reduction
        if reduction < thresholds['min_error_reduction_deg']:
            failures.append(f'heading error reduced only {reduction:.2f} deg; requires {thresholds["min_error_reduction_deg"]} deg')
    if mode == 'simulation' and 'min_speed_mps' in thresholds:
        truth = [r for r in rows['truth'] if start <= r[0] <= start + cfg['duration_s']]
        speeds = [math.hypot(b[1] - a[1], b[2] - a[2]) / (b[0] - a[0])
                  for a, b in zip(truth, truth[1:]) if b[0] > a[0]]
        speed = sum(speeds) / len(speeds) if speeds else None
        metrics['mean_ground_speed_mps'] = speed
        if speed is None or speed < thresholds['min_speed_mps']:
            failures.append(f'vessel ground speed {speed} m/s below {thresholds["min_speed_mps"]} m/s')
    if mode == 'simulation' and 'min_forward_speed_mps' in thresholds:
        values = [r[1] for r in rows['odom'] if start <= r[0] <= start + cfg['duration_s']]
        speed = sum(values) / len(values) if values else None
        metrics['mean_forward_speed_mps'] = speed
        if speed is None or speed < thresholds['min_forward_speed_mps']:
            failures.append(f'body forward speed {speed} m/s below {thresholds["min_forward_speed_mps"]} m/s')
    if 'min_rudder_change_rad' in thresholds:
        change_t = targets[-1]['at_s']
        before = [v for t, v in rows['rudder'] if start <= t < start + change_t]
        after_values = [v for t, v in rows['rudder'] if start + change_t + 0.1 <= t <= start + change_t + 1.0]
        delta = abs(sum(after_values) / len(after_values) - sum(before[-10:]) / len(before[-10:])) if before and after_values else None
        metrics['rudder_change_rad'] = delta
        if delta is None or delta < thresholds['min_rudder_change_rad']:
            failures.append('rudder did not respond enough after target change')
    if 'safe_after_s' in cfg['evaluation']:
        t0 = start + cfg['evaluation']['safe_after_s']
        for kind in ('rudder', 'backwing'):
            recent = [v for t, v in rows[kind] if t0 <= t <= start + cfg['duration_s']]
            if len(recent) < 3 or any(abs(v) > 0.02 for v in recent):
                failures.append(f'{kind} did not hold safe zero after stale input')
    return metrics, failures
