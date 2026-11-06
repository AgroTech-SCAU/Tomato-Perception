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
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from vision_interfaces.msg import TargetPoint
from vision_interfaces.srv import GetTargets, ManageHarvest
from visualization_msgs.msg import Marker, MarkerArray
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


def parse_harvest_basket(config):
    """Read the basket contract shared with the simulation collision scene."""
    if not isinstance(config, dict) or not isinstance(config.get('frame_id'), str) or not config['frame_id'].strip():
        raise ValueError('Harvest scene requires a nonempty frame_id')
    basket = config.get('basket')
    if not isinstance(basket, dict) or not isinstance(basket.get('id'), str) or not basket['id'].strip():
        raise ValueError('Harvest scene requires a basket with a nonempty id')
    result = {'frame_id': config['frame_id'], 'id': basket['id']}
    for key in ('position', 'dimensions'):
        values = basket.get(key)
        if (not isinstance(values, list) or len(values) != 3 or
                any(type(value) not in (int, float) or not math.isfinite(value) for value in values)):
            raise ValueError(f'Basket {key} must contain three finite numbers')
        if key == 'dimensions' and any(value <= 0 for value in values):
            raise ValueError('Basket dimensions must be positive')
        result[key] = [float(value) for value in values]
    if basket.get('slot_axis') not in ('x_axis', 'y_axis'):
        raise ValueError('Basket slot_axis must be x_axis or y_axis')
    result['slot_axis'] = basket['slot_axis']
    orientation = basket.get('release_orientation_xyzw')
    if (not isinstance(orientation, list) or len(orientation) != 4 or
            any(type(value) not in (int, float) or not math.isfinite(value) for value in orientation)):
        raise ValueError('Basket release_orientation_xyzw must contain four finite numbers')
    norm = math.sqrt(sum(value * value for value in orientation))
    if abs(norm - 1.) > .01:
        raise ValueError('Basket release_orientation_xyzw must be normalized')
    result['release_orientation_xyzw'] = [float(value) / norm for value in orientation]
    for key in ('slot_spacing_m', 'release_clearance_m'):
        value = basket.get(key)
        if (type(value) not in (int, float) or not math.isfinite(value) or value < 0 or
                key == 'slot_spacing_m' and value == 0):
            raise ValueError(f'Basket {key} must be finite and {"positive" if key == "slot_spacing_m" else "nonnegative"}')
        result[key] = float(value)
    return result


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
            'harvest_scene_file': '',
            'fruit_offsets_m': [-.055, 0., .10, 0., 0., .12, .055, 0., .10],
            'basket_offset_frame': 'world',
            'basket_offset_m': [.20, .05, -.06],
            'basket_slot_axis': 'x_axis',
            'grasp_tolerance_m': .006,
            'release_tolerance_m': .01,
            'gripper_open_positions_m': [0., 0.],
            'gripper_closed_positions_m': [-.007145, -.007145],
            'gripper_grasp_min_close_fraction': .97,
            'gripper_open_tolerance_fraction': .03,
            'gripper_feedback_max_age_sec': .50,
            'retreat_distance_m': .10,
            'standoff_m': .08,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter('world_target', [], ParameterDescriptor(dynamic_typing=True))
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Kinematic simulation requires use_sim_time=false')
        for name in ('sample_rate_hz', 'initial_target_distance_m', 'default_max_frame_age_sec',
                     'grasp_tolerance_m', 'release_tolerance_m', 'retreat_distance_m', 'standoff_m',
                     'gripper_feedback_max_age_sec'):
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
        self._gripper_open = np.asarray(self.get_parameter('gripper_open_positions_m').value, dtype=float)
        self._gripper_closed = np.asarray(self.get_parameter('gripper_closed_positions_m').value, dtype=float)
        for name, values in (('gripper_open_positions_m', self._gripper_open),
                             ('gripper_closed_positions_m', self._gripper_closed)):
            if values.shape != (2,) or not np.isfinite(values).all():
                raise ValueError(f'{name} must contain two finite positions for joint7-1 and joint7-2')
        if np.any(np.isclose(self._gripper_open, self._gripper_closed, atol=1e-9)):
            raise ValueError('Gripper open and closed positions must differ for each finger')
        for name in ('gripper_grasp_min_close_fraction', 'gripper_open_tolerance_fraction'):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or not 0 < value < 1:
                raise ValueError(f'{name} must be in (0, 1)')
        if (self.get_parameter('gripper_open_tolerance_fraction').value >=
                self.get_parameter('gripper_grasp_min_close_fraction').value):
            raise ValueError('Gripper open tolerance must be below the grasp close fraction')

        self._world_frame = self.get_parameter('world_frame').value
        self._tool_frame = self.get_parameter('tool_frame').value
        self._tcp_frame = self.get_parameter('tcp_frame').value
        self._camera_frame = self.get_parameter('camera_frame').value
        self._harvest_mode = self.get_parameter('harvest_mode').value
        offsets = np.asarray(self.get_parameter('fruit_offsets_m').value, dtype=float)
        if offsets.ndim != 1 or offsets.size == 0 or offsets.size % 3 or not np.isfinite(offsets).all():
            raise ValueError('fruit_offsets_m must contain one or more finite local XYZ triples')
        self._fruit_offsets = offsets.reshape((-1, 3))
        self._basket_offset_frame = self.get_parameter('basket_offset_frame').value
        if self._basket_offset_frame not in ('world', 'tcp'):
            raise ValueError('basket_offset_frame must be world or tcp')
        self._basket_offset = np.asarray(self.get_parameter('basket_offset_m').value, dtype=float)
        if self._basket_offset.shape != (3,) or not np.isfinite(self._basket_offset).all():
            raise ValueError('basket_offset_m must contain three finite coordinates')
        self._basket_slot_axis = self.get_parameter('basket_slot_axis').value
        if self._basket_slot_axis not in ('x_axis', 'y_axis'):
            raise ValueError('basket_slot_axis must be x_axis or y_axis')
        self._harvest_basket = None
        scene_file = self.get_parameter('harvest_scene_file').value
        if scene_file:
            self._harvest_basket = parse_harvest_basket(yaml.safe_load(Path(scene_file).expanduser().read_text()))
            if self._harvest_basket['frame_id'] != self._world_frame:
                raise ValueError('Harvest scene frame_id must match world_frame')
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
        self._harvest_marker_reset_pending = True
        self._sample = None
        self._gripper_states = {}
        self._requests = 0
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        if self._harvest_mode:
            self._gripper_subscription = self.create_subscription(
                JointState, '/joint_states', self._gripper_feedback, qos_profile_sensor_data)
        self._markers = self.create_publisher(
            Marker, '/simulation/targets', QoSProfile(depth=64, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._marker_frames = self.create_publisher(
            MarkerArray, '/simulation/targets_array', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
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
            if self._initial_tcp is None and not self._harvest_mode:
                self._initial_tcp = self._latest_tcp.copy()
                self._initial_orientation = deepcopy(world_tcp.transform.rotation)
            if self._initial_tcp is None:
                # The harvest workflow anchors its scene after unfolding to the observation pose.
                return
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
        self._harvest_marker_reset_pending = True

    def _pose(self, point, stamp):
        pose = PoseStamped()
        pose.header = Header(stamp=stamp, frame_id=self._world_frame)
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, point[:3])
        pose.pose.orientation = deepcopy(self._initial_orientation)
        return pose

    def _place_point(self, index):
        # Separate slots make each successful release visible in the basket
        if self._harvest_basket is not None:
            basket = self._harvest_basket
            point = np.array([*basket['position'], 1.])
            point[2] += basket['dimensions'][2] / 2. + .025 + basket['release_clearance_m']
            point[0 if basket['slot_axis'] == 'x_axis' else 1] += (
                index - (len(self._fruits) - 1) / 2.) * basket['slot_spacing_m']
            return point
        offset = self._basket_offset.copy()
        offset[0 if self._basket_slot_axis == 'x_axis' else 1] += (index - (len(self._fruits) - 1) / 2.) * .055
        if self._basket_offset_frame == 'world':
            point = self._initial_tcp[:, 3].copy()
            point[:3] += offset
            return point
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
        if self._harvest_basket is not None:
            orientation = response.place_pose.pose.orientation
            orientation.x, orientation.y, orientation.z, orientation.w = self._harvest_basket['release_orientation_xyzw']
        return response

    def _manage_harvest(self, request, response):
        response.success = False
        if not self._harvest_mode:
            response.message = 'Harvest commands require harvest_mode=true'
            return response
        if self._latest_tcp is None:
            response.message = 'No stamped TCP is available yet'
            return response
        stamp = self.get_clock().now().to_msg()
        if request.command == ManageHarvest.Request.RESET:
            try:
                world_tcp, current_tcp = self._fresh_tcp()
                stamp = world_tcp.header.stamp
                self._initial_tcp = current_tcp.copy()
                self._latest_tcp = current_tcp
                self._initial_orientation = deepcopy(world_tcp.transform.rotation)
                self._world_ambiguity_offset = self._initial_tcp[:3, 0] * .01
                self._reset_harvest()
                response.success = True
                response.message = 'Harvest scene anchored at the measured observation TCP'
            except (TransformException, ValueError) as error:
                response.message = str(error)
                return response
        elif self._initial_tcp is None:
            response.message = 'Reset the harvest scene after reaching the observation pose'
            return response
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
                world_tcp, current_tcp = self._fresh_tcp()
                stamp = world_tcp.header.stamp
                self._latest_tcp = current_tcp
                self._apply_harvest_command(request.command, current_tcp, response)
            except (TransformException, ValueError) as error:
                response.message = str(error)
        else:
            response.message = f'Unknown harvest command: {request.command}'
        self._publish_harvest_markers(stamp)
        return self._fill_harvest_response(response, stamp)

    def _fresh_tcp(self):
        world_tcp = self._tf.lookup_transform(self._world_frame, self._tcp_frame, Time())
        stamp = world_tcp.header.stamp
        source_ns = stamp.sec * 10**9 + stamp.nanosec
        age = (self.get_clock().now().nanoseconds - source_ns) / 1e9
        if source_ns == 0 or age < -.05 or age > self.get_parameter('default_max_frame_age_sec').value:
            raise ValueError('A fresh stamped TCP is required for harvest commands')
        return world_tcp, transform_matrix(world_tcp.transform)

    def _gripper_feedback(self, message):
        source_ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        received_ns = time.monotonic_ns()
        for index, name in enumerate(message.name):
            if name not in ('joint7-1', 'joint7-2'):
                continue
            if index >= len(message.position) or not math.isfinite(message.position[index]):
                self._gripper_states.pop(name, None)
                continue
            self._gripper_states[name] = (message.position[index], source_ns, received_ns)

    def _check_gripper(self, grasping):
        """Require measured finger travel before changing fruit ownership."""
        positions = []
        now_ns = self.get_clock().now().nanoseconds
        received_ns = time.monotonic_ns()
        max_age = self.get_parameter('gripper_feedback_max_age_sec').value
        for name in ('joint7-1', 'joint7-2'):
            sample = self._gripper_states.get(name)
            if sample is None:
                raise ValueError(f'Missing gripper feedback for {name}; harvest command rejected')
            position, source_ns, arrival_ns = sample
            source_age = (now_ns - source_ns) / 1e9
            arrival_age = (received_ns - arrival_ns) / 1e9
            if source_ns == 0 or source_age < -.05 or source_age > max_age or arrival_age > max_age:
                raise ValueError(f'Stale gripper feedback for {name}; harvest command rejected')
            positions.append(position)
        fractions = ((np.asarray(positions) - self._gripper_open) /
                     (self._gripper_closed - self._gripper_open))
        tolerance = self.get_parameter('gripper_open_tolerance_fraction').value
        if np.any(fractions < -tolerance) or np.any(fractions > 1. + tolerance):
            raise ValueError('Gripper feedback is outside the calibrated finger travel; harvest command rejected')
        if grasping:
            minimum = self.get_parameter('gripper_grasp_min_close_fraction').value
            if np.any(fractions < minimum):
                raise ValueError('Both fingers must reach the grasp position before attaching fruit')
        elif np.any(np.abs(fractions) > tolerance):
            raise ValueError('Both fingers must be open before releasing fruit')

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
            self._check_gripper(grasping=True)
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
            self._check_gripper(grasping=False)
            released = current_tcp @ self._attached_offset
            if self._harvest_basket is not None:
                basket = self._harvest_basket
                released[2] = basket['position'][2] + basket['dimensions'][2] / 2. + .025
            self._fruits[index] = released
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
        markers = []
        if self._harvest_marker_reset_pending:
            for namespace in ('ripe_targets', 'harvest_fruits', 'harvest_fruit_labels',
                              'harvest_basket', 'harvest_tcp', 'harvest_goals', 'harvest_status'):
                clear = self._marker(namespace, -1, Marker.CUBE, self._initial_tcp[:, 3],
                                     (0., 0., 0.), (0., 0., 0., 0.), stamp)
                clear.action = Marker.DELETEALL
                markers.append(clear)
        for index, fruit in enumerate(self._fruits):
            state = self._fruit_states[index]
            point = (self._latest_tcp @ self._attached_offset
                     if index == self._held_index and self._latest_tcp is not None else fruit)
            color = ((.2, .85, .25, 1.) if state == 'harvested' else
                     (1., .65, .05, 1.) if index == self._selected_index else (.9, .1, .05, 1.))
            markers.append(self._marker(
                'harvest_fruits', index, Marker.SPHERE, point, (.05, .05, .05), color, stamp))
            label_position = point.copy()
            label_position[2] += .045
            if state == 'harvested':
                label_position[2] += index * .06
            # Humble keeps the default one-metre text's space width unless scale.x is explicit.
            label = self._marker('harvest_fruit_labels', index, Marker.TEXT_VIEW_FACING,
                                 label_position, (.011, 0., .022), (1., 1., 1., 1.), stamp)
            label.text = f'Fruit {index + 1}: {state}'
            markers.append(label)
        if self._harvest_basket is not None:
            basket = self._harvest_basket['position']
            basket_scale = self._harvest_basket['dimensions']
        else:
            basket = self._place_point((len(self._fruits) - 1) / 2.)
            if self._basket_offset_frame == 'world':
                # A 50 mm fruit rests on the top of the 25 mm basket base.
                basket[2] -= .0375
            else:
                basket[:3] -= self._initial_tcp[:3, 2] * .04
            basket_scale = (.16, .11, .025) if self._basket_slot_axis == 'x_axis' else (.11, .16, .025)
        basket_marker = self._marker('harvest_basket', 0, Marker.CUBE, basket,
                                     basket_scale, (.2, .55, .85, .55), stamp)
        if self._harvest_basket is None and self._basket_offset_frame == 'tcp':
            basket_marker.pose.orientation = deepcopy(self._initial_orientation)
        markers.append(basket_marker)
        if self._latest_tcp is not None:
            markers.append(self._marker('harvest_tcp', 0, Marker.SPHERE,
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
            markers.append(goal)
        phase_position = self._initial_tcp[:, 3].copy()
        phase_position[2] += .22
        phase = self._marker('harvest_status', 0, Marker.TEXT_VIEW_FACING, phase_position,
                             (.0125, 0., .025), (1., 1., 1., 1.), stamp)
        phase.text = (f'{self._harvest_phase}\nCollected: '
                      f'{self._fruit_states.count("harvested")}/{len(self._fruits)}')
        if self._selected_index is not None and self._held_index is None and self._latest_tcp is not None:
            distance = np.linalg.norm(self._latest_tcp[:3, 3] - self._fruits[self._selected_index][:3])
            phase.text += f'\nTCP to fruit: {distance * 1000:.1f} mm'
        markers.append(phase)
        # One DDS sample keeps fruit, labels and progress in the same scene snapshot.
        self._marker_frames.publish(MarkerArray(markers=markers))
        self._harvest_marker_reset_pending = False

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
