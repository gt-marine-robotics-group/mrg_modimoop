"""ROS boundary for the controller bench. No controller calculations live here."""
import math
import time

import rclpy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from rclpy.qos import qos_profile_sensor_data


def wrap(angle):
    return math.remainder(angle, 2 * math.pi)


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class BenchIO:
    def __init__(self, mode, prefix):
        self.mode = mode
        self.node = rclpy.create_node('modimoop_bench')
        self.t0_wall = time.monotonic()
        self.clock = None
        self.last_clock_wall = None
        self.rows = {'rudder': [], 'backwing': [], 'debug': [], 'truth': [],
                     'heading': [], 'odom': [], 'target': [], 'wind': [],
                     'anemometer': [], 'wingsail': []}
        self.node.create_subscription(Clock, '/clock', self._clock, 10)
        self.node.create_subscription(Float64, '/modimoop/cmd/rudder_pos',
                                      lambda m: self._scalar('rudder', m), 10)
        self.node.create_subscription(Float64, '/modimoop/cmd/backwing_pos',
                                      lambda m: self._scalar('backwing', m), 10)
        self.node.create_subscription(Float64MultiArray,
                                      '/modimoop/control/lqr_heading/debug',
                                      self._debug, 10)
        self.heading_pub = self.node.create_publisher(Float64, prefix + '/heading', 10)
        self.target_pub = self.node.create_publisher(Float64, prefix + '/target', 10)
        self.odom_pub = self.node.create_publisher(Odometry, prefix + '/odom', qos_profile_sensor_data)
        self.wind_pub = self.node.create_publisher(Vector3, prefix + '/wind', qos_profile_sensor_data)
        self.last_truth = None
        self.truth_frames_seen = set()
        self.truth_message_count = 0
        self.gz_node = None
        self.gz_wind_pub = None
        self.last_gz_wind_send = 0.0
        self.last_anemometer = None
        if mode == 'simulation':
            from gz.transport13 import Node as GazeboNode
            from gz.msgs10.pose_v_pb2 import Pose_V
            from gz.msgs10.wind_pb2 import Wind
            self.gz_node = GazeboNode()
            self.gz_node.subscribe(Pose_V, '/world/ocean/pose/info', self.ingest_gz_pose)
            self.gz_wind_pub = self.gz_node.advertise('/world/ocean/wind', Wind)
            self.node.create_subscription(Vector3, '/modimoop/anemometer',
                                          self._anemometer, qos_profile_sensor_data)
            self.node.create_subscription(JointState, '/modimoop/joint_state',
                                          self._joint_state, qos_profile_sensor_data)

    def now(self):
        return (self.clock if self.clock is not None else 0.0) if self.mode == 'simulation' else time.monotonic() - self.t0_wall

    def _clock(self, msg):
        self.clock = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.last_clock_wall = time.monotonic()

    def _scalar(self, name, msg):
        self.rows[name].append((self.now(), msg.data))

    def _debug(self, msg):
        self.rows['debug'].append((self.now(), *msg.data))

    def _anemometer(self, msg):
        self.last_anemometer = (self.now(), msg.x, msg.y, msg.z)
        self.rows['anemometer'].append(self.last_anemometer)

    def _joint_state(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name == 'wingsail_joint':
                self.rows['wingsail'].append((self.now(), position))
                break

    def command_sim_wind(self, cfg):
        """Set WindEffects' seed; it owns the wind velocity used by ASV sails."""
        if self.gz_wind_pub is None or not self.gz_wind_pub.has_connections():
            return
        now = time.monotonic()
        if now - self.last_gz_wind_send < 0.5:
            return
        from gz.msgs10.wind_pb2 import Wind
        wind = Wind()
        wind.linear_velocity.x, wind.linear_velocity.y, wind.linear_velocity.z = (
            cfg['simulation'].get('wind_world_mps', [6.0, 0.0, 0.0]))
        self.gz_wind_pub.publish(wind)
        self.last_gz_wind_send = now

    def ingest_gz_pose(self, message):
        """Select the named world-frame model pose from Gazebo transport."""
        self.truth_message_count += 1
        stamp = message.header.stamp
        t = float(stamp.sec) + float(stamp.nsec) * 1e-9
        if self.last_truth is not None and t - self.last_truth[0] < 0.04:
            return
        for pose in message.pose:
            name = pose.name
            self.truth_frames_seen.add(name)
            if name != 'none_mrg_modimoop':
                continue
            self.last_truth = (t, pose.position.x, pose.position.y, yaw(pose.orientation))
            self.rows['truth'].append(self.last_truth)

    def spin(self, seconds=0.02):
        rclpy.spin_once(self.node, timeout_sec=seconds)

    def publish(self, elapsed, cfg, warmup=False):
        if self.mode == 'simulation':
            self.command_sim_wind(cfg)
        if warmup:
            target = self.last_truth[3] if self.mode == 'simulation' and self.last_truth else \
                math.radians(cfg['initial']['heading_deg'])
        else:
            target_deg = cfg['targets'][0]['heading_deg']
            for entry in cfg['targets']:
                if elapsed >= entry['at_s']:
                    target_deg = entry['heading_deg']
            target = math.radians(target_deg)
        target_msg = Float64(data=target)
        self.target_pub.publish(target_msg)
        self.rows['target'].append((self.now(), target))
        input_cfg = cfg['input']
        if elapsed >= input_cfg.get('stop_at_s', float('inf')):
            return
        if self.mode == 'simulation':
            if self.last_truth is None:
                return
            stamp, x, y, heading = self.last_truth
            old = getattr(self, '_previous_truth', None)
            if old is not None and stamp > old[0]:
                dt = stamp - old[0]
                dx, dy = (x - old[1]) / dt, (y - old[2]) / dt
                u = dx * math.cos(heading) + dy * math.sin(heading)
                v = -dx * math.sin(heading) + dy * math.cos(heading)
                r = wrap(heading - old[3]) / dt
                self._velocity = (u, v, r)
                self._previous_truth = self.last_truth
            elif old is None:
                self._previous_truth = self.last_truth
            u, v, r = getattr(self, '_velocity', (0.0, 0.0, 0.0))
        else:
            heading = math.radians(input_cfg.get('heading_deg', cfg['initial']['heading_deg']))
            for entry in input_cfg.get('heading_sequence', []):
                if elapsed >= entry['at_s']:
                    heading = math.radians(entry['heading_deg'])
            u, v, r = input_cfg.get('speed_mps', cfg['initial']['speed_mps']), 0.0, input_cfg.get('yaw_rate_rad_s', 0.0)
        h_msg = Float64(data=heading)
        odom = Odometry()
        odom.twist.twist.linear.x = u
        odom.twist.twist.linear.y = v
        odom.twist.twist.angular.z = r
        self.heading_pub.publish(h_msg)
        self.odom_pub.publish(odom)
        self.rows['heading'].append((self.now(), heading))
        self.rows['odom'].append((self.now(), u, v, r))
        if self.mode == 'simulation':
            if self.last_anemometer is None or self.now() - self.last_anemometer[0] > 1.0:
                wind = None
            else:
                # The anemometer link is yawed pi from the hull. It measures
                # apparent wind (world wind minus sensor velocity); add measured
                # hull velocity for the controller's body-frame true-wind input.
                wind = Vector3(x=-self.last_anemometer[1] + u,
                               y=-self.last_anemometer[2] + v)
        else:
            wind = Vector3(x=float(input_cfg.get('wind_x_mps', 6.0)),
                           y=float(input_cfg.get('wind_y_mps', 0.0)))
        if wind is not None:
            self.wind_pub.publish(wind)
            self.rows['wind'].append((self.now(), wind.x, wind.y))

    def ready(self):
        return (self.heading_pub.get_subscription_count() > 0 and
                self.odom_pub.get_subscription_count() > 0 and
                bool(self.rows['rudder']) and bool(self.rows['backwing']) and
                (self.mode != 'simulation' or
                 (self.clock is not None and self.last_truth is not None and
                  self.gz_wind_pub.has_connections() and
                  bool(self.rows['anemometer']))))
