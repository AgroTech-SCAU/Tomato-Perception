"""Fixed-world ripe targets for the robot's kinematic visual-servo simulation.

The timer samples TF independently of service requests. Query responses retain
that sample's timestamp; moving the tool changes camera coordinates, not fruit
coordinates. No image detector, camera driver or hardware is started here.
"""

from dataclasses import dataclass
from pathlib import Path
import math
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from vision_interfaces.msg import TargetPoint
from vision_interfaces.srv import GetTargets
from visualization_msgs.msg import Marker
import yaml


def transform_matrix(transform):
    """Return parent_T_child for a geometry_msgs/Transform."""
    q = transform.rotation
    values = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = float(np.linalg.norm(values))
    if not np.isfinite(values).all() or abs(norm - 1.0) > .01:
        raise ValueError('TF quaternion must be finite and normalized')
    x, y, z, w = values / norm
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ]
    p = transform.translation
    matrix[:3, 3] = [p.x, p.y, p.z]
    if not np.isfinite(matrix).all():
        raise ValueError('TF translation must be finite')
    return matrix


@dataclass(frozen=True)
class TargetSample:
    header: Header
    positions: tuple
    received_ns: int


class SimTargetServer(Node):
    def __init__(self):
        super().__init__('sim_target_server')
        defaults = {
            'service_name': '/vision/get_targets',
            'calibration_file': '',
            'world_frame': 'base_link',
            'tool_frame': 'tool0',
            'tcp_frame': 'tip',
            'camera_frame': 'camera_color_optical_frame',
            'initial_target_distance_m': .30,
            'sample_rate_hz': 30.0,
            'confidence': .95,
            'default_confidence': .40,
            'default_max_frame_age_sec': .50,
            'dropout_start_request': 2,
            'dropout_requests': 0,
            'ambiguous_start_request': 2,
            'ambiguous_requests': 0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter('world_target', [], ParameterDescriptor(dynamic_typing=True))
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Kinematic simulation requires use_sim_time=false')
        for name in ('sample_rate_hz', 'initial_target_distance_m', 'default_max_frame_age_sec'):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ('confidence', 'default_confidence'):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or not 0 < value <= 1:
                raise ValueError(f'{name} must be in (0, 1]')
        for name in ('dropout_start_request', 'ambiguous_start_request'):
            if self.get_parameter(name).value < 1:
                raise ValueError(f'{name} must be at least 1')
        for name in ('dropout_requests', 'ambiguous_requests'):
            if self.get_parameter(name).value < 0:
                raise ValueError(f'{name} must be nonnegative')

        self._world_frame = self.get_parameter('world_frame').value
        self._tool_frame = self.get_parameter('tool_frame').value
        self._tcp_frame = self.get_parameter('tcp_frame').value
        self._camera_frame = self.get_parameter('camera_frame').value
        calibration_path = Path(self.get_parameter('calibration_file').value).expanduser()
        if not calibration_path.is_file():
            raise FileNotFoundError(f'calibration_file does not exist: {calibration_path}')
        calibration = yaml.safe_load(calibration_path.read_text())
        if (calibration.get('parent_frame') != self._tool_frame or
                calibration.get('child_frame') != self._camera_frame or
                calibration.get('stored_transform_convention') != 'camera_to_gripper (^gripper T_camera)'):
            raise ValueError('Calibration frames/direction must match tool_frame and camera_frame')
        self._tool_camera = np.asarray(calibration['transform_matrix'], dtype=float)
        if (self._tool_camera.shape != (4, 4) or not np.isfinite(self._tool_camera).all() or
                not np.allclose(self._tool_camera[3], [0, 0, 0, 1], atol=1e-8) or
                not np.allclose(self._tool_camera[:3, :3].T @ self._tool_camera[:3, :3], np.eye(3), atol=1e-6) or
                not np.isclose(np.linalg.det(self._tool_camera[:3, :3]), 1.0, atol=1e-6)):
            raise ValueError('Calibration transform_matrix must be a rigid transform')
        configured_target = list(self.get_parameter('world_target').value)
        self._world_target = None
        if configured_target:
            if len(configured_target) != 3 or not np.isfinite(configured_target).all():
                raise ValueError('world_target must be empty or three finite coordinates in metres')
            self._world_target = np.array([*configured_target, 1.], dtype=float)
        self._world_ambiguity_offset = None
        self._sample = None
        self._requests = 0
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self._markers = self.create_publisher(
            Marker, '/simulation/targets', QoSProfile(depth=2, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._timer = self.create_timer(1. / self.get_parameter('sample_rate_hz').value, self._sample_frame)
        self._service = self.create_service(
            GetTargets, self.get_parameter('service_name').value, self._get_targets)
        self.get_logger().info('Simulation target service ready; waiting for stamped tool/TCP TF')

    def _sample_frame(self):
        try:
            world_tool = self._tf.lookup_transform(self._world_frame, self._tool_frame, Time())
            stamp = world_tool.header.stamp
            if stamp.sec == 0 and stamp.nanosec == 0:
                # A static-only chain has no acquisition time for a moving camera.
                return
            matrix = transform_matrix(world_tool.transform)
            if self._world_target is None or self._world_ambiguity_offset is None:
                world_tcp = self._tf.lookup_transform(
                    self._world_frame, self._tcp_frame, Time.from_msg(stamp))
                initial_tcp = transform_matrix(world_tcp.transform)
                if self._world_target is None:
                    distance = float(self.get_parameter('initial_target_distance_m').value)
                    self._world_target = initial_tcp @ [0., 0., distance, 1.]
                    self.get_logger().info(f'Fixed world fruit at {self._world_target[:3].tolist()}')
                self._world_ambiguity_offset = initial_tcp[:3, 0] * .01
            camera_world = np.linalg.inv(matrix @ self._tool_camera)
            second = self._world_target.copy()
            second[:3] += self._world_ambiguity_offset
            positions = tuple((camera_world @ point)[:3] for point in (self._world_target, second))
            self._sample = TargetSample(
                Header(stamp=stamp, frame_id=self._camera_frame), positions, time.monotonic_ns())
            self._publish_markers(stamp)
        except TransformException:
            # Keep the last actual sample so a TF outage becomes STALE_FRAME.
            pass

    def _publish_markers(self, stamp):
        points = [self._world_target[:3], self._world_target[:3] + self._world_ambiguity_offset]
        for index, point in enumerate(points):
            marker = Marker()
            marker.header = Header(stamp=stamp, frame_id=self._world_frame)
            marker.ns = 'ripe_targets'
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = (Marker.ADD if index == 0 or self.get_parameter('ambiguous_requests').value > 0
                             else Marker.DELETE)
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, point)
            marker.pose.orientation.w = 1.
            marker.scale.x = marker.scale.y = marker.scale.z = .05
            marker.color.r = .9
            marker.color.g = .1
            marker.color.b = .05
            marker.color.a = 1.
            self._markers.publish(marker)

    @staticmethod
    def _error(response, code, message):
        response.success = False
        response.status_code = code
        response.message = message
        response.targets = []
        return response

    def _in_request_window(self, name):
        start = self.get_parameter(name + '_start_request').value
        count = self.get_parameter(name + '_requests').value
        return start <= self._requests < start + count

    def _get_targets(self, request, response):
        sample = self._sample
        if sample is None:
            return self._error(response, GetTargets.Response.NO_FRAME, 'No stamped tool/TCP TF sample is available')
        response.header = sample.header
        max_age = float(request.max_frame_age_sec)
        confidence = float(request.min_confidence)
        if not math.isfinite(max_age) or not math.isfinite(confidence) or confidence > 1:
            return self._error(response, GetTargets.Response.BAD_FRAME, 'Frame age/confidence request is invalid')
        if max_age <= 0:
            max_age = float(self.get_parameter('default_max_frame_age_sec').value)
        if confidence <= 0:
            confidence = float(self.get_parameter('default_confidence').value)
        stamp_ns = sample.header.stamp.sec * 10**9 + sample.header.stamp.nanosec
        source_age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        age = max(source_age, (time.monotonic_ns() - sample.received_ns) / 1e9)
        if source_age < -.05 or age > max_age:
            return self._error(response, GetTargets.Response.STALE_FRAME, f'Latest TF sample is stale: {age:.3f}s')
        self._requests += 1
        wanted = {value.strip().lower() for value in request.class_filter if value.strip()}
        targets = []
        if (not self._in_request_window('dropout') and (not wanted or 'ripe' in wanted) and
                self.get_parameter('confidence').value >= confidence):
            positions = sample.positions if self._in_request_window('ambiguous') else sample.positions[:1]
            for point in positions:
                if point[2] <= 0:
                    continue
                target = TargetPoint()
                target.semantic_label = 'ripe'
                target.class_id = 0
                target.confidence = self.get_parameter('confidence').value
                target.position.x, target.position.y, target.position.z = map(float, point)
                # No rendered image in this stage; pixel/bbox/depth-MAD fields stay zero.
                targets.append(target)
        response.success = True
        response.status_code = GetTargets.Response.OK
        response.message = f'OK: {len(targets)} simulated target(s); request={self._requests}'
        response.targets = targets
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SimTargetServer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
