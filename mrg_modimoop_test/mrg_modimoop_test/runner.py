"""Owned-process scenario runner for Modimoop controller tests."""
import argparse
import csv
import json
import os
import signal
import shlex
import subprocess
import sys
import time
import uuid
import threading
from collections import deque
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory

from .metrics import evaluate
from .ros_io import BenchIO
from .scenario import ScenarioError, load


class InfrastructureError(RuntimeError):
    pass


def _versions():
    info = {'ros_distro': os.environ.get('ROS_DISTRO'),
            'gz_version': os.environ.get('GZ_VERSION'),
            'python': sys.version.split()[0]}
    try:
        info['gazebo_sim'] = subprocess.run(['gz', 'sim', '--versions'], text=True,
                                           capture_output=True, timeout=3).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        info['gazebo_sim'] = 'unavailable'
    for package in ('mrg_modimoop_control', 'mrg_modimoop_description',
                    'lyoko_gz_worlds', 'ros_gz_bridge'):
        try:
            share = Path(get_package_share_directory(package))
            info[package] = {'prefix': str(Path(get_package_prefix(package))),
                             'version': ET.parse(share / 'package.xml').getroot().findtext('version')}
        except Exception:
            info[package] = 'unavailable'
    return info


def _robot(path):
    import xacro
    source = Path(get_package_share_directory('mrg_modimoop_description')) / 'urdf/mrg_modimoop.urdf.xacro'
    xml = xacro.process_file(str(source)).toxml()
    if '</robot>' not in xml:
        raise InfrastructureError(f'robot description lacks closing tag: {source}')
    path.write_text(xml)


def _world(path, wind):
    source = Path(get_package_share_directory('lyoko_gz_worlds')) / 'worlds/ocean.world'
    tree = ET.parse(source)
    velocity = tree.getroot().find('./world/wind/linear_velocity')
    if velocity is None:
        raise InfrastructureError(f'world wind velocity missing in {source}')
    velocity.text = ' '.join(str(value) for value in wind)
    tree.write(path, encoding='unicode', xml_declaration=True)


class OwnedProcesses:
    def __init__(self, directory, env):
        self.directory, self.env, self.items = directory, env, []
        self.gazebo_children = {}
        self.gazebo_start_tick = None
        self.last_child_scan = 0.0

    @staticmethod
    def _process_identity(pid):
        """Read parent PID and Linux start tick, so a reused PID is never signalled."""
        try:
            fields = Path(f'/proc/{pid}/stat').read_text().rsplit(') ', 1)[1].split()
            return int(fields[1]), int(fields[19])
        except (OSError, IndexError, ValueError):
            return None

    def _capture_gazebo_children(self):
        gazebo = next((proc for name, proc, *_ in self.items if name == 'gazebo'), None)
        if gazebo is None:
            return
        processes = {}
        for entry in Path('/proc').iterdir():
            if entry.name.isdecimal():
                identity = self._process_identity(int(entry.name))
                if identity is not None:
                    processes[int(entry.name)] = identity
        known = {pid for pid, tick in self.gazebo_children.items()
                 if pid in processes and processes[pid][1] == tick}
        if gazebo.pid in processes and processes[gazebo.pid][1] == self.gazebo_start_tick:
            known.add(gazebo.pid)
        for _ in range(6):
            found = {pid for pid, (parent, _) in processes.items() if parent in known}
            if found <= known:
                break
            known.update(found)
        for pid in known:
            if pid != gazebo.pid and pid in processes:
                self.gazebo_children.setdefault(pid, processes[pid][1])

    def _signal_gazebo_children(self, sig):
        for pid, start_tick in self.gazebo_children.items():
            identity = self._process_identity(pid)
            if identity is not None and identity[1] == start_tick:
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    pass

    def start(self, name, command, allow_success_exit=False, on_line=None):
        log = open(self.directory / f'{name}.log', 'wb')
        try:
            proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    env=self.env, start_new_session=True)
        except Exception:
            log.close()
            raise
        tail = deque(maxlen=16)
        def drain():
            written = 0
            pending = b''
            while True:
                chunk = proc.stdout.read(65536)
                if not chunk:
                    break
                if on_line:
                    pending += chunk
                    while b'\n' in pending:
                        line, pending = pending.split(b'\n', 1)
                        try:
                            on_line(line.decode('utf-8', errors='replace'))
                        except Exception as exc:
                            log.write(f'\n[bench parser error: {exc}]\n'.encode())
                tail.append(chunk)
                if written < 5_000_000:
                    part = chunk[:5_000_000 - written]
                    log.write(part)
                    written += len(part)
            if tail and written >= 5_000_000:
                log.write(b'\n[bench: log capped at 5 MB; final output follows]\n')
                for chunk in tail:
                    log.write(chunk)
            log.flush()
        thread = threading.Thread(target=drain, daemon=True)
        thread.start()
        self.items.append((name, proc, log, allow_success_exit, thread))
        if name == 'gazebo':
            identity = self._process_identity(proc.pid)
            self.gazebo_start_tick = identity[1] if identity else None
        return proc

    def check(self):
        if time.monotonic() - self.last_child_scan > 0.25:
            self._capture_gazebo_children()
            self.last_child_scan = time.monotonic()
        for name, proc, _, allow_success_exit, thread in self.items:
            if proc.poll() is not None and (proc.returncode != 0 or not allow_success_exit):
                thread.join(timeout=1)
                reason = f'{name} process ended unexpectedly (launcher code {proc.returncode})'
                if name == 'gazebo':
                    log_text = (self.directory / 'gazebo.log').read_text(errors='replace')[-2_000_000:]
                    if 'CGAL ERROR' in log_text:
                        reason += '; Gazebo child aborted in wave hydrodynamics with a CGAL assertion'
                    elif 'Ogre::InternalErrorException' in log_text:
                        reason += '; Gazebo GUI renderer aborted with an Ogre error'
                raise InfrastructureError(reason + f'; see {name}.log')

    def close(self):
        self._capture_gazebo_children()
        for _, proc, _, _, _ in reversed(self.items):
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + 5
        for _, proc, _, _, _ in reversed(self.items):
            try:
                proc.wait(timeout=max(0.1, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                proc.wait(timeout=2)
        self._signal_gazebo_children(signal.SIGINT)
        child_deadline = time.monotonic() + 2
        while time.monotonic() < child_deadline:
            if not any((identity := self._process_identity(pid)) is not None and
                       identity[1] == tick for pid, tick in self.gazebo_children.items()):
                break
            time.sleep(0.05)
        self._signal_gazebo_children(signal.SIGKILL)
        for _, _, log, _, thread in self.items:
            thread.join(timeout=2)
            log.close()


def _record(directory, rows):
    for name, values in rows.items():
        with (directory / f'{name}.csv').open('w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['time_s', *({'truth': ['x_m', 'y_m', 'yaw_rad'],
                                            'odom': ['u_mps', 'v_mps', 'yaw_rate_rad_s'],
                                            'wind': ['x_mps', 'y_mps'],
                                            'anemometer': ['x_mps', 'y_mps', 'z_mps']}.get(name, ['value_rad'])
                                         if name != 'debug' else [f'field_{i}' for i in range(20)])])
            writer.writerows(values)


def _charts(directory, cfg, rows, start):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception as exc:
        (directory / 'charts_error.txt').write_text(str(exc))
        return
    heading = rows['truth'] if cfg['mode'] == 'simulation' else rows['heading']
    t = [r[0] - start for r in heading if start <= r[0] <= start + cfg['duration_s']]
    measured = [r[3] if cfg['mode'] == 'simulation' else r[1]
                for r in heading if start <= r[0] <= start + cfg['duration_s']]
    target = []
    for instant in t:
        value = cfg['targets'][0]['heading_deg']
        for entry in cfg['targets']:
            if instant >= entry['at_s']:
                value = entry['heading_deg']
        target.append(value)
    import math
    error = [math.degrees(math.remainder(math.radians(cmd) - actual, 2 * math.pi))
             for cmd, actual in zip(target, measured)]
    fig, axes = plt.subplots(3, 1, figsize=(9, 10))
    axes[0].plot(t, [math.degrees(v) for v in measured], label='measured')
    axes[0].plot(t, target, label='target')
    axes[0].legend()
    axes[0].set_ylabel('heading (deg ENU yaw)')
    axes[1].plot(t, error)
    axes[1].set_ylabel('wrapped error (deg)')
    for name in ('rudder', 'backwing'):
        values = [(x - start, y) for x, y in rows[name] if start <= x <= start + cfg['duration_s']]
        axes[2].plot([v[0] for v in values], [v[1] for v in values], label=name)
    axes[2].legend()
    axes[2].set_ylabel('command (rad)')
    axes[2].set_xlabel('test time (s)')
    fig.tight_layout()
    fig.savefig(directory / 'controller.png')
    plt.close(fig)
    if cfg['mode'] == 'simulation' and rows['truth']:
        fig, ax = plt.subplots()
        ax.plot([r[1] for r in rows['truth']], [r[2] for r in rows['truth']])
        ax.set_aspect('equal', adjustable='datalim')
        ax.set(xlabel='east x (m)', ylabel='north y (m)', title='Gazebo model trajectory')
        fig.savefig(directory / 'trajectory.png')
        plt.close(fig)
    if cfg['mode'] == 'simulation' and rows['wingsail']:
        fig, ax = plt.subplots()
        ax.plot([r[0] - start for r in rows['wingsail']],
                [r[1] for r in rows['wingsail']])
        ax.set(xlabel='test time (s)', ylabel='wingsail angle relative to hull (rad)',
               title='Passive wingsail joint position')
        fig.savefig(directory / 'wingsail.png')
        plt.close(fig)


def run(cfg, output, ros_domain_id=None, gz_partition=None):
    directory = Path(output).resolve() / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '_' + cfg['name'] + '_' + uuid.uuid4().hex[:6])
    directory.mkdir(parents=True)
    (directory / 'resolved.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))
    (directory / 'versions.json').write_text(json.dumps(_versions(), indent=2))
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(ros_domain_id if ros_domain_id is not None else 100 + os.getpid() % 100)
    gui = cfg['mode'] == 'simulation' and cfg['simulation'].get('gui', False)
    partition = gz_partition if gz_partition is not None else (
        None if gui else 'modimoop_bench_' + uuid.uuid4().hex[:12])
    os.environ['ROS_DOMAIN_ID'] = env['ROS_DOMAIN_ID']
    if partition is None:
        env.pop('GZ_PARTITION', None)
        os.environ.pop('GZ_PARTITION', None)
        gz_inspect = 'env -u GZ_PARTITION gz topic -l'
    else:
        env['GZ_PARTITION'] = partition
        os.environ['GZ_PARTITION'] = partition
        gz_inspect = f'GZ_PARTITION={shlex.quote(partition)} gz topic -l'
    (directory / 'runtime.json').write_text(json.dumps({
        'ros_domain_id': env['ROS_DOMAIN_ID'], 'gz_partition': partition,
        'gz_partition_mode': 'default' if partition is None else 'named'}, indent=2))
    print(f"{cfg['name']}: ROS_DOMAIN_ID={env['ROS_DOMAIN_ID']} (inspect while running with: ROS_DOMAIN_ID={env['ROS_DOMAIN_ID']} ros2 topic list --no-daemon)", flush=True)
    print(f"{cfg['name']}: Gazebo topics while running: {gz_inspect}", flush=True)
    processes = OwnedProcesses(directory, env)
    io = None
    result = {'scenario': cfg['name'], 'status': 'infrastructure_error', 'reasons': [], 'metrics': {}}
    start = None
    try:
        prefix = '/bench/input'
        rclpy.init()
        io = BenchIO(cfg['mode'], prefix)
        if cfg['mode'] == 'simulation':
            world = directory / 'ocean_bench.world'
            _world(world, cfg['simulation'].get('wind_world_mps', [6.0, 0.0, 0.0]))
            robot = directory / 'robot.urdf'
            _robot(robot)
            gz_args = ('-r ' if cfg['simulation'].get('gui', False) else
                       '-r -s --headless-rendering ') + str(world)
            processes.start('gazebo', ['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py',
                                      'gz_args:=' + gz_args])
            processes.start('clock_bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                                             '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])
            bridge_yaml = Path(get_package_share_directory('mrg_modimoop_description')) / 'config/vehicle_gz_bridge.yaml'
            processes.start('vehicle_bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                                               '--ros-args', '-p', f'config_file:={bridge_yaml}'])
            processes.start('boat_spawn', ['ros2', 'run', 'ros_gz_sim', 'create',
                                           '-file', str(robot), '-name', 'none_mrg_modimoop',
                                           '-x', '0', '-y', '0', '-z', '0',
                                           '-Y', str(cfg['initial']['heading_deg'] * 3.141592653589793 / 180)],
                            allow_success_exit=True)
        params = {'odom_topic': prefix + '/odom', 'heading_topic': prefix + '/heading',
                  'target_heading_topic': prefix + '/target', 'wind_body_topic': prefix + '/wind',
                  'use_gain_schedule': False, 'use_sim_time': cfg['mode'] == 'simulation',
                  'rudder_sign': -1.0, 'backwing_sign': 1.0,
                  'delta_r_max_rad': 0.785398, 'delta_w_max_rad': 0.349066}
        params.update(cfg['parameters'])
        binary = str(Path(get_package_prefix('mrg_modimoop_control')) / 'lib/mrg_modimoop_control/lqr_heading_controller')
        args = [binary, '--ros-args']
        for key, value in params.items():
            args.extend(['-p', f'{key}:={str(value).lower() if isinstance(value, bool) else value}'])
        processes.start('controller', args)
        startup = time.monotonic()
        last_publish = 0.0
        while not io.ready():
            processes.check()
            if time.monotonic() - startup > cfg['startup_timeout_s']:
                try:
                    topics = subprocess.run(['gz', 'topic', '-l'], env=env, text=True,
                                            capture_output=True, timeout=3).stdout.splitlines()
                except (OSError, subprocess.TimeoutExpired):
                    topics = ['Gazebo topic listing unavailable']
                raise InfrastructureError(
                    'startup timeout: '
                    f'heading subscribers={io.heading_pub.get_subscription_count()}, '
                    f'odom subscribers={io.odom_pub.get_subscription_count()}, '
                    f'rudder messages={len(io.rows["rudder"])}, '
                    f'clock={io.clock}, truth TF messages={io.truth_message_count}, selected poses={len(io.rows["truth"])}, '
                    f'wind command subscribers={io.gz_wind_pub.has_connections() if io.gz_wind_pub else "n/a"}, '
                    f'anemometer messages={len(io.rows["anemometer"])}, '
                    f'pose names={sorted(io.truth_frames_seen)}, '
                    f'pose topics={[t for t in topics if "pose" in t]}')
            io.spin()
            if time.monotonic() - last_publish > 0.05:
                io.publish(0, cfg, warmup=True)
                last_publish = time.monotonic()
        warmup_start = io.now()
        wall_start = time.monotonic()
        while io.now() - warmup_start < cfg['warmup_s']:
            processes.check()
            if time.monotonic() - wall_start > cfg['wall_timeout_s']:
                raise InfrastructureError('wall timeout during warmup; simulator may be stalled')
            io.spin()
            if time.monotonic() - last_publish > 0.05:
                io.publish(0, cfg, warmup=True)
                last_publish = time.monotonic()
        start = io.now()
        wall_start = time.monotonic()
        last_clock = io.now()
        last_progress = time.monotonic()
        while io.now() - start < cfg['duration_s']:
            processes.check()
            if time.monotonic() - wall_start > cfg['wall_timeout_s']:
                raise InfrastructureError('execution wall timeout')
            io.spin()
            if cfg['mode'] == 'simulation':
                if io.now() > last_clock:
                    last_clock, last_progress = io.now(), time.monotonic()
                if time.monotonic() - last_progress > 3:
                    raise InfrastructureError('simulation clock stalled for 3 wall seconds')
            if time.monotonic() - last_publish > 0.05:
                io.publish(io.now() - start, cfg)
                last_publish = time.monotonic()
        result['metrics'], result['reasons'] = evaluate(cfg, io.rows, start)
        if result['metrics'].get('sample_count', 0) < 3 or 'insufficient steady-state samples' in result['reasons']:
            result['status'] = 'infrastructure_error'
        else:
            result['status'] = 'assertion_failed' if result['reasons'] else 'passed'
    except Exception as exc:
        result['reasons'].append(str(exc))
    except KeyboardInterrupt:
        result['status'] = 'interrupted'
        result['reasons'].append('interrupted by user')
    finally:
        try:
            if io:
                _record(directory, io.rows)
                if start is not None:
                    _charts(directory, cfg, io.rows, start)
                io.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as exc:
            result['status'] = 'infrastructure_error'
            result['reasons'].append(f'artifact or ROS shutdown error: {exc}')
        finally:
            processes.close()
        (directory / 'result.json').write_text(json.dumps(result, indent=2))
        summary = '\n'.join(f'- {key}: {value}' for key, value in result['metrics'].items()
                            if key != 'heading_error_deg')
        (directory / 'report.md').write_text('# ' + cfg['name'] + '\n\n' +
            f"Status: **{result['status']}**\n\n" +
            ('\n'.join('- ' + reason for reason in result['reasons']) or 'All evaluated assertions passed.') +
            '\n\n## Metrics\n\n' + (summary or 'Evaluation incomplete.') +
            '\n\nFull results: `result.json`. Recordings: `*.csv`.\n')
    print(f"{cfg['name']}: {result['status']} — {directory}")
    for reason in result['reasons']:
        print('  ' + reason)
    return 0 if result['status'] == 'passed' else (1 if result['status'] == 'assertion_failed' else 2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('scenario', nargs='?')
    group.add_argument('--suite', choices=['isolated', 'all'])
    parser.add_argument('--set', action='append', default=[], metavar='KEY=VALUE')
    parser.add_argument('--output', default='bench_results')
    parser.add_argument('--gui', action='store_true', help='open Gazebo GUI for simulation scenarios')
    parser.add_argument('--ros-domain-id', type=int, metavar='ID',
                        help='use this ROS domain; GUI runs otherwise use the caller domain')
    parser.add_argument('--gz-partition', metavar='NAME',
                        help='use a named Gazebo partition; GUI runs otherwise use the Gazebo default')
    args = parser.parse_args()
    try:
        if args.suite:
            directory = Path(get_package_share_directory('mrg_modimoop_test')) / 'scenarios'
            paths = sorted(directory.glob('*.yaml'))
            paths = [p for p in paths if args.suite == 'all' or load(p)['mode'] == 'isolated']
            if not paths:
                raise ScenarioError('suite contains no scenarios')
        else:
            paths = [Path(args.scenario)]
        codes = []
        if args.ros_domain_id is not None and not 0 <= args.ros_domain_id <= 232:
            raise ScenarioError('--ros-domain-id must be between 0 and 232')
        if args.gz_partition is not None and not args.gz_partition.strip():
            raise ScenarioError('--gz-partition must be nonempty')
        visible_domain = args.ros_domain_id
        if args.gui and visible_domain is None:
            try:
                visible_domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
            except ValueError as exc:
                raise ScenarioError('ROS_DOMAIN_ID must be an integer') from exc
        for path in paths:
            cfg = load(path, args.set)
            if args.gui:
                if cfg['mode'] != 'simulation':
                    raise ScenarioError('--gui applies only to simulation scenarios')
                cfg['simulation']['gui'] = True
            codes.append(run(cfg, args.output, visible_domain, args.gz_partition))
        return max(codes)
    except ScenarioError as exc:
        print(f'scenario error: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
