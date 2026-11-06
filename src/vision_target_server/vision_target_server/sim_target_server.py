"""Fixed-world ripe targets for the robot's kinematic visual-servo simulation.

The timer samples TF independently of service requests. Query responses retain
that sample's timestamp; moving the tool changes camera coordinates, not fruit
coordinates. No image detector, camera driver or hardware is started here.
"""

from dataclasses import dataclass
from copy import deepcopy
from pathlib import Path
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from vision_interfaces.msg import TargetPoint
from vision_interfaces.srv import GetTargets, ManageHarvest
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
            'harvest_mode': False,
            'harvest_service_name': '/simulation/harvest',
            'fruit_offsets_m': [-.055, 0., .10, 0., 0., .12, .055, 0., .10],
            'basket_offset_m': [0., -.04, 0.],
            'grasp_tolerance_m': .006,
            'release_tolerance_m': .01,
            'retreat_distance_m': .10,
            'standoff_m': .08,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter('world_target', [], ParameterDescriptor(dynamic_typing=True))
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Kinematic simulation requires use_sim_time=false')
        for name in ('sample_rate_hz', 'initial_target_distance_m', 'default_max_frame_age_sec',
                     'grasp_tolerance_m', 'release_tolerance_m', 'retreat_distance_m', 'standoff_m'):
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
        self._harvest_mode = self.get_parameter('harvest_mode').value
        offsets = np.asarray(self.get_parameter('fruit_offsets_m').value, dtype=float)
        if offsets.ndim != 1 or offsets.size == 0 or offsets.size % 3 or not np.isfinite(offsets).all():
            raise ValueError('fruit_offsets_m must contain one or more finite local XYZ triples')
        self._fruit_offsets = offsets.reshape((-1, 3))
        self._basket_offset = np.asarray(self.get_parameter('basket_offset_m').value, dtype=float)
        if self._basket_offset.shape != (3,) or not np.isfinite(self._basket_offset).all():
            raise ValueError('basket_offset_m must contain three finite local coordinates')
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
        self._initial_tcp = None
        self._initial_orientation = None
        self._latest_tcp = None
        self._fruits = []
        self._fruit_states = []
        self._selected_index = None
        self._held_index = None
        self._attached_offset = None
        self._harvest_phase = 'Waiting for initial TCP'
        self._sample = None
        self._requests = 0
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self._markers = self.create_publisher(
            Marker, '/simulation/targets', QoSProfile(depth=64, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._timer = self.create_timer(1. / self.get_parameter('sample_rate_hz').value, self._sample_frame)
        self._service = self.create_service(
            GetTargets, self.get_parameter('service_name').value, self._get_targets)
        self._harvest_service = self.create_service(
            ManageHarvest, self.get_parameter('harvest_service_name').value, self._manage_harvest)
        self.get_logger().info('Simulation target service ready; waiting for stamped tool/TCP TF')

    def _sample_frame(self):
        try:
            world_tool = self._tf.lookup_transform(self._world_frame, self._tool_frame, Time())
            stamp = world_tool.header.stamp
            if stamp.sec == 0 and stamp.nanosec == 0:
                # A static-only chain has no acquisition time for a moving camera.
                return
            matrix = transform_matrix(world_tool.transform)
            world_tcp = self._tf.lookup_transform(
                self._world_frame, self._tcp_frame, Time.from_msg(stamp))
            self._latest_tcp = transform_matrix(world_tcp.transform)
            if self._initial_tcp is None:
                self._initial_tcp = self._latest_tcp.copy()
                self._initial_orientation = deepcopy(world_tcp.transform.rotation)
                if self._harvest_mode:
                    self._reset_harvest()
            if self._world_target is None or self._world_ambiguity_offset is None:
                if self._world_target is None and not self._harvest_mode:
                    distance = float(self.get_parameter('initial_target_distance_m').value)
                    self._world_target = self._initial_tcp @ [0., 0., distance, 1.]
                    self.get_logger().info(f'Fixed world fruit at {self._world_target[:3].tolist()}')
                self._world_ambiguity_offset = self._initial_tcp[:3, 0] * .01
            camera_world = np.linalg.inv(matrix @ self._tool_camera)
            target = self._world_target
            if self._harvest_mode:
                target = (self._fruits[self._selected_index]
                          if self._selected_index is not None and self._held_index is None else None)
            points = []
            if target is not None:
                second = target.copy()
                second[:3] += self._world_ambiguity_offset
                points = [target, second]
            positions = tuple((camera_world @ point)[:3] for point in points)
            self._sample = TargetSample(
                Header(stamp=stamp, frame_id=self._camera_frame), positions, time.monotonic_ns())
            self._publish_markers(stamp)
        except TransformException:
            # Keep the last actual sample so a TF outage becomes STALE_FRAME.
            pass

    def _publish_markers(self, stamp):
        if self._harvest_mode:
            self._publish_harvest_markers(stamp)
            return
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

    def _reset_harvest(self):
        self._fruits = [self._initial_tcp @ [*offset, 1.] for offset in self._fruit_offsets]
        self._fruit_states = ['available'] * len(self._fruits)
        self._selected_index = None
        self._held_index = None
        self._attached_offset = None
        self._sample = None
        self._requests = 0
        self._harvest_phase = 'Ready to select fruit'

    def _pose(self, point, stamp):
        pose = PoseStamped()
        pose.header = Header(stamp=stamp, frame_id=self._world_frame)
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, point[:3])
        pose.pose.orientation = deepcopy(self._initial_orientation)
        return pose

    def _place_point(self, index):
        # Separate slots make each successful release visible in the basket
        offset = self._basket_offset.copy()
        offset[0] += (index - (len(self._fruits) - 1) / 2.) * .055
        return self._initial_tcp @ [*offset, 1.]

    def _fill_harvest_response(self, response, stamp):
        response.fruit_index = 0 if self._selected_index is None else self._selected_index + 1
        response.total_fruits = len(self._fruits)
        response.harvested_count = self._fruit_states.count('harvested')
        response.finished = bool(self._fruits) and response.harvested_count == len(self._fruits)
        response.home_pose = self._pose(self._initial_tcp[:, 3], stamp)
        index = self._selected_index
        retreat = self._initial_tcp[:, 3].copy() if index is None else self._fruits[index].copy()
        if index is not None:
            retreat[:3] -= self._initial_tcp[:3, 2] * self.get_parameter('retreat_distance_m').value
        response.retreat_pose = self._pose(retreat, stamp)
        response.place_pose = self._pose(self._place_point(index or 0), stamp)
        return response

    def _manage_harvest(self, request, response):
        response.success = False
        if not self._harvest_mode:
            response.message = 'Harvest commands require harvest_mode=true'
            return response
        if self._initial_tcp is None:
            response.message = 'No stamped initial TCP is available yet'
            return response
        stamp = self.get_clock().now().to_msg()
        if request.command == ManageHarvest.Request.RESET:
            self._reset_harvest()
            response.success = True
            response.message = 'Harvest scene reset from the initial TCP reference'
        elif request.command == ManageHarvest.Request.SELECT_NEXT:
            if self._held_index is not None:
                response.message = 'Release the attached fruit before selecting another fruit'
            else:
                if self._selected_index is None:
                    self._selected_index = next(
                        (index for index, state in enumerate(self._fruit_states) if state == 'available'), None)
                response.success = True
                self._harvest_phase = ('All fruit collected' if self._selected_index is None else
                                       f'Approaching fruit {self._selected_index + 1}')
                response.message = self._harvest_phase
                self._sample = None
        elif request.command in (ManageHarvest.Request.GRASP, ManageHarvest.Request.RELEASE):
            try:
                world_tcp = self._tf.lookup_transform(self._world_frame, self._tcp_frame, Time())
                stamp = world_tcp.header.stamp
                source_ns = stamp.sec * 10**9 + stamp.nanosec
                age = (self.get_clock().now().nanoseconds - source_ns) / 1e9
                if source_ns == 0 or age < -.05 or age > self.get_parameter('default_max_frame_age_sec').value:
                    raise ValueError('A fresh stamped TCP is required for grasp/release')
                current_tcp = transform_matrix(world_tcp.transform)
                self._latest_tcp = current_tcp
                self._apply_harvest_command(request.command, current_tcp, response)
            except (TransformException, ValueError) as error:
                response.message = str(error)
        else:
            response.message = f'Unknown harvest command: {request.command}'
        self._publish_harvest_markers(stamp)
        return self._fill_harvest_response(response, stamp)

    def _apply_harvest_command(self, command, current_tcp, response):
        index = self._selected_index
        if command == ManageHarvest.Request.GRASP:
            if index is None or self._held_index is not None or self._fruit_states[index] != 'available':
                response.message = 'Select an available fruit before grasping'
                return
            distance = float(np.linalg.norm(current_tcp[:3, 3] - self._fruits[index][:3]))
            if distance > self.get_parameter('grasp_tolerance_m').value:
                response.message = f'TCP is {distance * 1000:.1f} mm from fruit; grasp rejected'
                return
            self._held_index = index
            self._attached_offset = np.linalg.inv(current_tcp) @ self._fruits[index]
            self._fruit_states[index] = 'attached'
            self._harvest_phase = f'Carrying fruit {index + 1} to basket'
            self._sample = None
            response.message = f'Fruit {index + 1} attached to the measured TCP'
        else:
            if self._held_index is None:
                response.message = 'No fruit is attached; release rejected'
                return
            index = self._held_index
            distance = float(np.linalg.norm(current_tcp[:3, 3] - self._place_point(index)[:3]))
            if distance > self.get_parameter('release_tolerance_m').value:
                response.message = f'TCP is {distance * 1000:.1f} mm from basket slot; release rejected'
                return
            self._fruits[index] = current_tcp @ self._attached_offset
            self._fruit_states[index] = 'harvested'
            self._held_index = None
            self._attached_offset = None
            self._selected_index = None
            self._harvest_phase = f'Fruit {index + 1} released in basket'
            self._sample = None
            response.message = self._harvest_phase
        response.success = True

    def _marker(self, namespace, marker_id, kind, point, scale, color, stamp):
        marker = Marker()
        marker.header = Header(stamp=stamp, frame_id=self._world_frame)
        marker.ns = namespace
        marker.id = marker_id
        marker.type = kind
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, point[:3])
        marker.pose.orientation.w = 1.
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def _publish_harvest_markers(self, stamp):
        if self._initial_tcp is None:
            return
        for index, fruit in enumerate(self._fruits):
            state = self._fruit_states[index]
            point = (self._latest_tcp @ self._attached_offset
                     if index == self._held_index and self._latest_tcp is not None else fruit)
            color = ((.2, .85, .25, 1.) if state == 'harvested' else
                     (1., .65, .05, 1.) if index == self._selected_index else (.9, .1, .05, 1.))
            self._markers.publish(self._marker(
                'harvest_fruits', index, Marker.SPHERE, point, (.05, .05, .05), color, stamp))
            label_position = point.copy()
            label_position[2] += .045
            label = self._marker('harvest_fruit_labels', index, Marker.TEXT_VIEW_FACING,
                                 label_position, (0., 0., .022), (1., 1., 1., 1.), stamp)
            label.text = f'Fruit {index + 1}: {state}'
            self._markers.publish(label)
        basket = self._initial_tcp @ [*self._basket_offset, 1.]
        basket[:3] -= self._initial_tcp[:3, 2] * .04
        basket_marker = self._marker('harvest_basket', 0, Marker.CUBE, basket,
                                     (.16, .11, .025), (.2, .55, .85, .55), stamp)
        basket_marker.pose.orientation = deepcopy(self._initial_orientation)
        self._markers.publish(basket_marker)
        if self._latest_tcp is not None:
            self._markers.publish(self._marker('harvest_tcp', 0, Marker.SPHERE,
                                               self._latest_tcp[:, 3], (.012, .012, .012),
                                               (.1, .8, 1., 1.), stamp))
        for marker_id in (0, 1):
            goal = self._marker('harvest_goals', marker_id,
                                Marker.SPHERE if marker_id == 0 else Marker.LINE_LIST,
                                self._initial_tcp[:, 3], (.014, .014, .014),
                                (.2, .8, 1., 1.) if marker_id == 0 else (1., 1., .2, 1.), stamp)
            if marker_id == 1:
                goal.scale.x = .003
                # Extend beyond the fruit sphere so its centre is visible in RViz
                for axis in range(3):
                    for sign in (-1., 1.):
                        values = [0., 0., 0.]
                        values[axis] = sign * .035
                        goal.points.append(Point(x=values[0], y=values[1], z=values[2]))
            if self._selected_index is None or self._held_index is not None:
                goal.action = Marker.DELETE
            else:
                point = self._fruits[self._selected_index].copy()
                if marker_id == 0:
                    point[:3] -= self._initial_tcp[:3, 2] * self.get_parameter('standoff_m').value
                goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = map(float, point[:3])
            self._markers.publish(goal)
        phase_position = self._initial_tcp[:, 3].copy()
        phase_position[2] += .22
        phase = self._marker('harvest_status', 0, Marker.TEXT_VIEW_FACING, phase_position,
                             (0., 0., .025), (1., 1., 1., 1.), stamp)
        phase.text = (f'{self._harvest_phase}\nCollected: '
                      f'{self._fruit_states.count("harvested")}/{len(self._fruits)}')
        if self._selected_index is not None and self._held_index is None and self._latest_tcp is not None:
            distance = np.linalg.norm(self._latest_tcp[:3, 3] - self._fruits[self._selected_index][:3])
            phase.text += f'\nTCP to fruit: {distance * 1000:.1f} mm'
        self._markers.publish(phase)

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
