#!/usr/bin/env python3

from concurrent.futures import ThreadPoolExecutor
from math import atan2, cos, pi, sin
import os
import random
import time
from threading import Lock

import subprocess

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory


class PedestrianManager(Node):
    def __init__(self):
        super().__init__('pedestrian_manager')
        self.declare_parameter('world_name', 'cafe_world')
        self.declare_parameter('model_variant', 'person_standing')
        self.declare_parameter('target_model', 'Target')
        self.declare_parameter('update_period', 0.2)
        self.declare_parameter('random_seed', 631)
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.world_name = self.get_parameter('world_name').get_parameter_value().string_value
        self.model_variant = self.get_parameter('model_variant').get_parameter_value().string_value
        self.target_model = self.get_parameter('target_model').get_parameter_value().string_value
        self.random_seed = self.get_parameter('random_seed').get_parameter_value().integer_value
        self._rng = random.Random(self.random_seed)

        self.package_share = get_package_share_directory('cpe631_ros2')
        self.models_path = os.path.join(self.package_share, 'models')

        self.gz_set_pose_service = f'/world/{self.world_name}/set_pose'
        self.gz_set_pose_vector_service = f'/world/{self.world_name}/set_pose_vector'
        self._last_set_pose_error = 0.0
        self._pose_executor = ThreadPoolExecutor(max_workers=1)
        self._pose_lock = Lock()
        self._pose_update_inflight = False
        self._pending_pose_batch = None
        self._last_update_time = None

        time.sleep(2.0)
        self._spawn_target(3.0, 9.5)

        self.pedestrians = self._make_initial_pedestrians()
        self._initialize_pedestrian_state()

        for name, data in self.pedestrians.items():
            px, py = data['current']
            self._spawn_pedestrian(name, px, py, data['yaw'])

        self.create_service(Trigger, 'reset_pedestrians', self._reset_pedestrians_cb)

        # Publisher for data_collector / social_nav_node consumption
        self.declare_parameter('output_topic', '/pedestrian_poses')
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self._pose_pub = self.create_publisher(PoseArray, out_topic, 10)
        self.get_logger().info(f'Publishing pedestrian poses on {out_topic}')

        update_period = max(
            0.05,
            self.get_parameter('update_period').get_parameter_value().double_value,
        )
        self.timer = self.create_timer(update_period, self._update_positions)

    def _make_initial_pedestrians(self):
        return {
            # 4 fixed-route pedestrians (linear back-and-forth)
            'ped_1': {'start': (3.0, 5.0),  'end': (3.0, -8.0),  'speed': 1.2,
                      'current': (3.0, 5.0),  'direction': 1, 'mode': 'linear'},
            'ped_2': {'start': (-3.0, 5.0), 'end': (-3.0, 0.0),  'speed': 0.8,
                      'current': (-3.0, 5.0), 'direction': 1, 'mode': 'linear'},
            'ped_3': {'start': (2.0, -4.5), 'end': (-3.0, -4.0), 'speed': 0.8,
                      'current': (2.0, -4.5), 'direction': 1, 'mode': 'linear'},
            'ped_4': {'start': (0.0, 6.0),  'end': (3.5, 6.5),   'speed': 0.5,
                      'current': (0.0, 6.0),  'direction': 1, 'mode': 'linear'},
            # 5th pedestrian: random walking for realistic scene
            'ped_5': {
                'start': (-1.0, 2.0), 'end': (-1.0, 2.0), 'speed': 0.6,
                'current': (-1.0, 2.0), 'direction': 1, 'mode': 'random',
                'bounds': (-4.0, 4.0, -3.0, 7.0),   # x_min, x_max, y_min, y_max
                'waypoint': (0.0, 0.0),
            },
        }

    def _initialize_pedestrian_state(self):
        self._rng = random.Random(self.random_seed)
        for name, data in self.pedestrians.items():
            data['current'] = data['start']
            data['direction'] = 1
            if data['mode'] == 'random':
                data['waypoint'] = self._random_waypoint(data['bounds'])
                yaw = self._get_yaw(data['current'], data['waypoint'])
            else:
                yaw = self._get_yaw(data['start'], data['end'])
            data['yaw'] = yaw

    def _reset_pedestrians_cb(self, _request, response):
        self.pedestrians = self._make_initial_pedestrians()
        self._initialize_pedestrian_state()
        self._last_update_time = None
        pose_batch = []
        for name, data in self.pedestrians.items():
            px, py = data['current']
            pose_batch.append((name, px, py, 0.0, data['yaw']))
        self._set_poses(pose_batch)
        self._publish_poses()
        response.success = True
        response.message = 'pedestrians reset to initial positions'
        self.get_logger().info(response.message)
        return response

    def _spawn_target(self, px, py):
        target_path = os.path.join(self.models_path, self.target_model, 'model.sdf')
        self._spawn_entity('target', target_path, px, py, 0.19, 0.0)

    def _spawn_pedestrian(self, model_name, px, py, yaw):
        model_path = os.path.join(self.models_path, self.model_variant, 'model.sdf')
        self._spawn_entity(model_name, model_path, px, py, 0.0, yaw)

    def _spawn_entity(self, name, sdf_path, px, py, pz, yaw):
        if not os.path.exists(sdf_path):
            self.get_logger().error(f'Model file not found: {sdf_path}')
            return
        env = os.environ.copy()
        env['GZ_SIM_RESOURCE_PATH'] = f"{self.models_path}:{env.get('GZ_SIM_RESOURCE_PATH', '')}"
        command = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', self.world_name,
            '-name', name,
            '-file', sdf_path,
            '-x', str(px),
            '-y', str(py),
            '-z', str(pz),
            '-Y', str(yaw),
        ]
        self.get_logger().info(f'cmd: {" ".join(command)}')
        result = subprocess.run(command, capture_output=True, text=True, env=env)
        if result.returncode != 0:
            self.get_logger().error(
                f'Failed to spawn entity {name}: {result.stderr.strip()}'
            )

    def _random_waypoint(self, bounds):
        x_min, x_max, y_min, y_max = bounds
        return (self._rng.uniform(x_min, x_max), self._rng.uniform(y_min, y_max))

    def _update_positions(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_update_time is None:
            dt = 0.1
        else:
            dt = max(1e-3, min(now - self._last_update_time, 0.25))
        self._last_update_time = now

        pose_batch = []
        for name, data in self.pedestrians.items():
            current_x, current_y = data['current']
            step_size = data['speed'] * dt

            if data['mode'] == 'random':
                target_x, target_y = data['waypoint']
                dx = target_x - current_x
                dy = target_y - current_y
                distance = (dx ** 2 + dy ** 2) ** 0.5
                if distance <= step_size:
                    data['waypoint'] = self._random_waypoint(data['bounds'])
                    new_x, new_y = target_x, target_y
                else:
                    ratio = step_size / distance
                    new_x = current_x + ratio * dx
                    new_y = current_y + ratio * dy
            else:
                target_x, target_y = data['end'] if data['direction'] == 1 else data['start']
                dx = target_x - current_x
                dy = target_y - current_y
                distance = (dx ** 2 + dy ** 2) ** 0.5
                if distance <= step_size:
                    data['direction'] *= -1
                    new_x, new_y = target_x, target_y
                else:
                    ratio = step_size / distance
                    new_x = current_x + ratio * dx
                    new_y = current_y + ratio * dy

            if (new_x, new_y) != (current_x, current_y):
                yaw = self._get_yaw((current_x, current_y), (new_x, new_y))
            else:
                yaw = data.get('yaw', 0.0)
            pose_batch.append((name, new_x, new_y, 0.0, yaw))
            data['current'] = (new_x, new_y)
            data['yaw'] = yaw

        self._set_poses(pose_batch)
        self._publish_poses()

    def _publish_poses(self):
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'map'
        for data in self.pedestrians.values():
            p = Pose()
            p.position.x = data['current'][0]
            p.position.y = data['current'][1]
            p.position.z = 0.0
            yaw = data.get('yaw', 0.0)
            p.orientation.z = sin(yaw / 2.0)
            p.orientation.w = cos(yaw / 2.0)
            pose_array.poses.append(p)
        self._pose_pub.publish(pose_array)

    def _set_pose(self, name, px, py, pz, yaw):
        self._set_poses([(name, px, py, pz, yaw)])

    def _set_poses(self, pose_batch):
        if not pose_batch:
            return
        pose_batch = tuple(pose_batch)
        env = os.environ.copy()
        env.setdefault('GZ_IP', '127.0.0.1')
        with self._pose_lock:
            if self._pose_update_inflight:
                self._pending_pose_batch = pose_batch
                return
            self._pose_update_inflight = True
        self._pose_executor.submit(self._run_set_pose_batch, pose_batch, env)

    def _run_set_pose_batch(self, pose_batch, env):
        batch = pose_batch
        while batch is not None:
            command = [
                'gz', 'service',
                '-s', self.gz_set_pose_vector_service,
                '--reqtype', 'gz.msgs.Pose_V',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '500',
                '--req', self._gz_pose_vector_request(batch),
            ]
            try:
                result = subprocess.run(
                    command, capture_output=True, text=True, env=env, timeout=1.0
                )
            except subprocess.TimeoutExpired:
                now = time.time()
                if now - self._last_set_pose_error > 5.0:
                    self.get_logger().warning('Timed out moving pedestrians')
                    self._last_set_pose_error = now
            else:
                if result.returncode != 0:
                    now = time.time()
                    if now - self._last_set_pose_error > 5.0:
                        detail = result.stderr.strip() or result.stdout.strip()
                        self.get_logger().warning(f'Failed to move pedestrians: {detail}')
                        self._last_set_pose_error = now
            with self._pose_lock:
                batch = self._pending_pose_batch
                self._pending_pose_batch = None
                if batch is None:
                    self._pose_update_inflight = False
                    return

    def _set_pose_legacy(self, name, px, py, pz, yaw):
        request = self._gz_pose_request(name, px, py, pz, yaw)
        command = [
            'gz', 'service',
            '-s', self.gz_set_pose_service,
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '500',
            '--req', request,
        ]
        env = os.environ.copy()
        env.setdefault('GZ_IP', '127.0.0.1')
        self._pose_executor.submit(self._run_set_pose, name, command, env)

    def _run_set_pose(self, name, command, env):
        try:
            result = subprocess.run(
                command, capture_output=True, text=True, env=env, timeout=1.0
            )
        except subprocess.TimeoutExpired:
            now = time.time()
            if now - self._last_set_pose_error > 5.0:
                self.get_logger().warning(f'Timed out moving {name}')
                self._last_set_pose_error = now
            return
        if result.returncode != 0:
            now = time.time()
            if now - self._last_set_pose_error > 5.0:
                detail = result.stderr.strip() or result.stdout.strip()
                self.get_logger().warning(f'Failed to move {name}: {detail}')
                self._last_set_pose_error = now

    def _gz_pose_request(self, name, px, py, pz, yaw):
        qz = sin(yaw / 2.0)
        qw = cos(yaw / 2.0)
        return (
            f'name: "{name}" '
            f'position {{ x: {px:.6f} y: {py:.6f} z: {pz:.6f} }} '
            f'orientation {{ x: 0 y: 0 z: {qz:.6f} w: {qw:.6f} }}'
        )

    def _gz_pose_vector_request(self, pose_batch):
        return ' '.join(
            f'pose {{ {self._gz_pose_request(name, px, py, pz, yaw)} }}'
            for name, px, py, pz, yaw in pose_batch
        )

    def _get_yaw(self, start, end):
        return atan2(end[1] - start[1], end[0] - start[0]) + pi / 2


def main():
    rclpy.init()
    node = PedestrianManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node._pose_executor.shutdown(wait=False)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
