"""Small, validated controller scenario format."""
import math
import re
from pathlib import Path

import yaml


class ScenarioError(ValueError):
    pass


def load(path, overrides=()):
    path = Path(path)
    try:
        data = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as exc:
        raise ScenarioError(f'{path}: {exc}') from exc
    if not isinstance(data, dict):
        raise ScenarioError(f'{path}: expected a YAML mapping')
    for item in overrides:
        if '=' not in item:
            raise ScenarioError(f'override {item!r}: expected key=value')
        key, value = item.split('=', 1)
        cursor = data
        parts = key.split('.')
        for part in parts[:-1]:
            if part not in cursor or not isinstance(cursor[part], dict):
                raise ScenarioError(f'override {key!r}: unknown key')
            cursor = cursor[part]
        if parts[-1] not in cursor:
            raise ScenarioError(f'override {key!r}: unknown key')
        cursor[parts[-1]] = yaml.safe_load(value)
    allowed = {'name', 'description', 'mode', 'duration_s', 'warmup_s',
               'startup_timeout_s', 'wall_timeout_s', 'seed', 'initial',
               'targets', 'input', 'parameters', 'thresholds', 'evaluation',
               'simulation'}
    unknown = set(data) - allowed
    if unknown:
        raise ScenarioError(f'{path}: unknown fields: {sorted(unknown)}')
    for key in ('name', 'description', 'mode', 'duration_s', 'targets',
                'initial', 'thresholds'):
        if key not in data:
            raise ScenarioError(f'{path}: missing {key}')
    if data['mode'] not in ('isolated', 'simulation'):
        raise ScenarioError(f'{path}: mode must be isolated or simulation')
    if not isinstance(data['name'], str) or not data['name']:
        raise ScenarioError('name must be nonempty text')
    if not re.fullmatch(r'[A-Za-z0-9_-]+', data['name']):
        raise ScenarioError('name may contain only letters, digits, underscore, and hyphen')
    if not isinstance(data['description'], str):
        raise ScenarioError('description must be text')
    if not isinstance(data['initial'], dict):
        raise ScenarioError('initial must be a mapping')
    for key in ('heading_deg', 'speed_mps'):
        if key not in data['initial'] or not _finite(data['initial'][key]):
            raise ScenarioError(f'initial.{key} must be finite')
    if data['initial']['speed_mps'] < 0:
        raise ScenarioError('initial.speed_mps must be nonnegative')
    if not isinstance(data['targets'], list) or not data['targets']:
        raise ScenarioError('targets must be a nonempty list')
    previous = -1.0
    for i, target in enumerate(data['targets']):
        if not isinstance(target, dict) or set(target) != {'at_s', 'heading_deg'}:
            raise ScenarioError(f'targets[{i}] needs at_s and heading_deg')
        if not all(_finite(v) for v in target.values()):
            raise ScenarioError(f'targets[{i}] values must be finite')
        if target['at_s'] <= previous or target['at_s'] < 0:
            raise ScenarioError('target times must be nonnegative and increasing')
        previous = target['at_s']
    if data['targets'][0]['at_s'] != 0:
        raise ScenarioError('first target must be at 0 seconds')
    for key, default in [('warmup_s', 0.5), ('startup_timeout_s', 15.0),
                         ('wall_timeout_s', 60.0)]:
        data.setdefault(key, default)
    for key in ('duration_s', 'warmup_s', 'startup_timeout_s', 'wall_timeout_s'):
        if not _finite(data[key]) or data[key] < 0 or (key != 'warmup_s' and data[key] == 0):
            raise ScenarioError(f'{key} must be a finite positive duration')
    if previous >= data['duration_s']:
        raise ScenarioError('last target must precede duration_s')
    for key in ('input', 'parameters', 'thresholds', 'evaluation', 'simulation'):
        data.setdefault(key, {})
        if not isinstance(data[key], dict):
            raise ScenarioError(f'{key} must be a mapping')
    nested_allowed = {
        'initial': {'heading_deg', 'speed_mps'},
        'input': {'heading_deg', 'heading_sequence', 'speed_mps',
                  'yaw_rate_rad_s', 'wind_x_mps', 'wind_y_mps', 'stop_at_s'},
        'evaluation': {'steady_window_s', 'settling_band_deg',
                       'settling_dwell_s', 'safe_after_s'},
        'simulation': {'wind_world_mps', 'gui'},
        'thresholds': {'rudder_limit_rad', 'backwing_limit_rad',
                       'max_steady_error_deg', 'max_settling_s',
                       'max_overshoot_deg', 'max_initial_wrapped_error_deg',
                       'min_rudder_change_rad', 'min_error_reduction_deg',
                       'min_speed_mps', 'min_forward_speed_mps'}}
    for section, keys in nested_allowed.items():
        extra = set(data[section]) - keys
        if extra:
            raise ScenarioError(f'{section}: unknown fields {sorted(extra)}')
    for section in ('input', 'evaluation', 'simulation'):
        for key, value in data[section].items():
            if key not in ('heading_sequence', 'wind_world_mps', 'gui') and not _finite(value):
                raise ScenarioError(f'{section}.{key} must be finite')
    if 'gui' in data['simulation'] and not isinstance(data['simulation']['gui'], bool):
        raise ScenarioError('simulation.gui must be true or false')
    if 'wind_world_mps' in data['simulation']:
        wind = data['simulation']['wind_world_mps']
        if not isinstance(wind, list) or len(wind) != 3 or not all(_finite(v) for v in wind):
            raise ScenarioError('simulation.wind_world_mps must be three finite values')
    for key, value in data['evaluation'].items():
        if value < 0 or (key != 'safe_after_s' and value == 0):
            raise ScenarioError(f'evaluation.{key} must be positive')
    sequence = data['input'].get('heading_sequence', [])
    if not isinstance(sequence, list):
        raise ScenarioError('input.heading_sequence must be a list')
    previous = -1.0
    for i, entry in enumerate(sequence):
        if not isinstance(entry, dict) or set(entry) != {'at_s', 'heading_deg'} or not all(_finite(v) for v in entry.values()):
            raise ScenarioError(f'input.heading_sequence[{i}] needs finite at_s and heading_deg')
        if entry['at_s'] <= previous or entry['at_s'] < 0:
            raise ScenarioError('input.heading_sequence times must increase from zero')
        previous = entry['at_s']
    if 'seed' in data and (not isinstance(data['seed'], int) or isinstance(data['seed'], bool)):
        raise ScenarioError('seed must be an integer')
    if not all(isinstance(key, str) and key for key in data['parameters']):
        raise ScenarioError('parameters keys must be nonempty strings')
    for key, value in data['thresholds'].items():
        if not _finite(value) or value < 0:
            raise ScenarioError(f'thresholds.{key} must be finite and nonnegative')
    data['source_file'] = str(path.resolve())
    return data


def _finite(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)
