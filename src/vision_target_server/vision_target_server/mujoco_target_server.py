"""Ground-truth observations and workflow adapter for the MuJoCo harvest scene.

Fruit poses, contacts and collection counts come exclusively from the physics
plugin. This node transforms those measurements into the existing vision API;
it does not move bodies, simulate attachment or run the image detector.
"""

from collections import deque
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, Transform, Vector3
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header, UInt64
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from tomato_picker_interfaces.msg import HarvestPhysicsState
from vision_interfaces.msg import TargetPoint
from vision_interfaces.srv import GetTargets, ManageHarvest
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from .sim_target_server import TargetSample, parse_harvest_basket, transform_matrix


def _stamp_ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def _pose_matrix(pose):
    return transform_matrix(Transform(
        translation=Vector3(x=pose.position.x, y=pose.position.y, z=pose.position.z),
        rotation=pose.orientation))


@dataclass(frozen=True)
class PhysicsSample:
    state: HarvestPhysicsState
    received_ns: int


class MujocoTargetServer(Node):
    def __init__(self):
        super().__init__('mujoco_target_server', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        defaults = {
            'service_name': '/vision/get_targets',
            'harvest_service_name': '/simulation/harvest',
            'physics_state_topic': '/simulation/physics/state',
            'physics_service_prefix': '/simulation/physics',
            'planning_scene_revision_topic': '',
            'physics_service_timeout_s': 15.0,
            'calibration_file': '',
            'harvest_scene_file': '',
            'world_frame': 'base_link',
            'tool_frame': 'tool0',
            'tcp_frame': 'tip',
            'camera_frame': 'camera_color_optical_frame',
            'camera_info_topic': '/camera/color/camera_info',
            'sample_rate_hz': 30.0,
            'confidence': .95,
            'default_confidence': .40,
            'default_max_frame_age_sec': .50,
            'retreat_distance_m': .06,
            'standoff_m': .02,
            'basket_wall_thickness_m': .004,
            'basket_wall_height_m': .025,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name in ('physics_service_timeout_s', 'sample_rate_hz', 'default_max_frame_age_sec',
                     'retreat_distance_m', 'standoff_m', 'basket_wall_thickness_m', 'basket_wall_height_m'):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ('confidence', 'default_confidence'):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or not 0 < value <= 1:
                raise ValueError(f'{name} must be in (0, 1]')
        self._world_frame = self.get_parameter('world_frame').value
        self._tool_frame = self.get_parameter('tool_frame').value
        self._tcp_frame = self.get_parameter('tcp_frame').value
        self._camera_frame = self.get_parameter('camera_frame').value
        scene_path = Path(self.get_parameter('harvest_scene_file').value).expanduser()
        if not scene_path.is_file():
            raise FileNotFoundError(f'harvest_scene_file does not exist: {scene_path}')
        self._basket = parse_harvest_basket(yaml.safe_load(scene_path.read_text()))
        if self._basket['frame_id'] != self._world_frame:
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
        self._condition = threading.Condition()
        self._command_lock = threading.Lock()
        self._history = deque(maxlen=120)
        self._latest = None
        self._planning_scene_revision = -1
        self._scene_revision_topic = self.get_parameter("planning_scene_revision_topic").value
        self._sample = None
        self._latest_tcp = None
        self._camera_info = None
        self._marker_reset_pending = True
        self._marker_lock = threading.Lock()
        self._last_marker_ns = -1
        self._callback_group = ReentrantCallbackGroup()
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self._subscription = self.create_subscription(
            HarvestPhysicsState, self.get_parameter('physics_state_topic').value,
            self._physics_state, qos_profile_sensor_data, callback_group=self._callback_group)
        self._scene_revision_subscription = None
        if self._scene_revision_topic:
            self._scene_revision_subscription = self.create_subscription(
                UInt64, self._scene_revision_topic, self._receive_scene_revision,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                callback_group=self._callback_group)
        self._camera_subscription = self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._receive_camera_info, qos_profile_sensor_data, callback_group=self._callback_group)
        prefix = self.get_parameter('physics_service_prefix').value.rstrip('/')
        self._physics_clients = {
            command: self.create_client(Trigger, prefix + '/' + name, callback_group=self._callback_group)
            for command, name in (
                (ManageHarvest.Request.RESET, 'reset'),
                (ManageHarvest.Request.SELECT_NEXT, 'select_next'),
                (ManageHarvest.Request.GRASP, 'grasp'),
                (ManageHarvest.Request.RELEASE, 'release'))
        }
        self._markers = self.create_publisher(
            MarkerArray, '/simulation/targets_array',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._timer = self.create_timer(
            1. / self.get_parameter('sample_rate_hz').value, self._sample_frame,
            callback_group=self._callback_group)
        self._service = self.create_service(
            GetTargets, self.get_parameter('service_name').value, self._get_targets,
            callback_group=self._callback_group)
        self._harvest_service = self.create_service(
            ManageHarvest, self.get_parameter('harvest_service_name').value, self._manage_harvest,
            callback_group=self._callback_group)
        self.get_logger().info(
            'MuJoCo ground-truth target adapter ready; observations use physics poses, not image inference')

    def _receive_scene_revision(self, message):
        with self._condition:
            self._planning_scene_revision = message.data
            self._condition.notify_all()

    def _receive_camera_info(self, message):
        if message.header.frame_id == self._camera_frame and message.k[0] > 0 and message.k[4] > 0:
            with self._condition:
                self._camera_info = message

    def _physics_state(self, state):
        if state.header.frame_id != self._world_frame or len(state.fruit_states) != len(state.fruit_poses):
            self.get_logger().error('Physics state frame/fruit arrays do not match the configured harvest scene')
            return
        sample = PhysicsSample(state, time.monotonic_ns())
        with self._condition:
            previous = self._latest
            self._latest = sample
            self._history.append(sample)
            if (previous is None or state.revision != previous.state.revision or
                    state.selected_index != previous.state.selected_index or not state.ready):
                self._sample = None
            if previous is not None and (state.revision < previous.state.revision or
                                         _stamp_ns(state.header.stamp) < _stamp_ns(previous.state.header.stamp)):
                self._planning_scene_revision = -1
                self._history.clear()
                self._history.append(sample)
                self._marker_reset_pending = True
            self._condition.notify_all()
        self._publish_markers(state)

    def _sample_frame(self):
        with self._condition:
            latest = self._latest
            history = tuple(self._history)
        if latest is None or not latest.state.ready:
            return
        try:
            latest_tool = self._tf.lookup_transform(self._world_frame, self._tool_frame, Time())
            tool_ns = _stamp_ns(latest_tool.header.stamp)
            # TF may arrive one control tick behind physics; use a retained physics
            # frame at exactly the requested TF time, never restamp an older pose.
            measured = next((sample for sample in reversed(history)
                             if 0 < _stamp_ns(sample.state.header.stamp) <= tool_ns and
                             sample.state.revision == latest.state.revision and
                             sample.state.selected_index == latest.state.selected_index), None)
            if measured is None:
                return
            state = measured.state
            stamp = state.header.stamp
            world_tool = self._tf.lookup_transform(
                self._world_frame, self._tool_frame, Time.from_msg(stamp))
            world_tcp = self._tf.lookup_transform(
                self._world_frame, self._tcp_frame, Time.from_msg(stamp))
            camera_world = np.linalg.inv(transform_matrix(world_tool.transform) @ self._tool_camera)
            positions = ()
            index = state.selected_index - 1
            if 0 <= index < len(state.fruit_poses) and state.fruit_states[index] == 0 and not state.held_index:
                point = state.fruit_poses[index].position
                positions = ((camera_world @ [point.x, point.y, point.z, 1.])[:3],)
            target_sample = TargetSample(
                Header(stamp=stamp, frame_id=self._camera_frame), positions, measured.received_ns)
            with self._condition:
                if self._latest is not None and self._latest.state.revision == state.revision:
                    self._sample = target_sample
                    self._latest_tcp = transform_matrix(world_tcp.transform)
        except (TransformException, ValueError):
            # Preserve the acquisition stamp so an interrupted TF stream ages out.
            return

    def _place_point(self, state, index):
        point = np.asarray(self._basket['position'], dtype=float).copy()
        point[2] += self._basket['dimensions'][2] / 2. + .025 + self._basket['release_clearance_m']
        point[0 if self._basket['slot_axis'] == 'x_axis' else 1] += (
            index - (len(state.fruit_poses) - 1) / 2.) * self._basket['slot_spacing_m']
        return point

    def _pose(self, state, point):
        pose = PoseStamped()
        pose.header = deepcopy(state.header)
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, point)
        pose.pose.orientation = deepcopy(state.initial_tcp.pose.orientation)
        return pose

    def _fill_harvest_response(self, response, state):
        response.fruit_index = state.selected_index
        response.total_fruits = len(state.fruit_poses)
        response.harvested_count = state.harvested_count
        response.finished = bool(state.fruit_poses) and state.harvested_count == len(state.fruit_poses)
        if not state.ready:
            return response
        initial = _pose_matrix(state.initial_tcp.pose)
        response.home_pose = self._pose(state, initial[:3, 3])
        index = state.selected_index - 1
        retreat = initial[:3, 3].copy()
        if 0 <= index < len(state.fruit_poses):
            point = state.fruit_poses[index].position
            retreat = np.asarray([point.x, point.y, point.z]) - initial[:3, 2] * self.get_parameter('retreat_distance_m').value
        response.retreat_pose = self._pose(state, retreat)
        response.place_pose = self._pose(state, self._place_point(state, max(index, 0)))
        orientation = response.place_pose.pose.orientation
        orientation.x, orientation.y, orientation.z, orientation.w = self._basket['release_orientation_xyzw']
        return response

    @staticmethod
    def _command_confirmed(command, state, previous_count):
        if not state.ready:
            return False
        if command == ManageHarvest.Request.GRASP:
            return state.held_index > 0
        if command == ManageHarvest.Request.RELEASE:
            return state.held_index == 0 and state.harvested_count > previous_count
        return True

    def _manage_harvest(self, request, response):
        response.success = False
        client = self._physics_clients.get(request.command)
        if client is None:
            response.message = f'Unknown harvest command: {request.command}'
            return response
        if not self._command_lock.acquire(blocking=False):
            response.message = 'A physics harvest command is already running'
            return response
        try:
            deadline = time.monotonic() + self.get_parameter('physics_service_timeout_s').value
            with self._condition:
                before = self._latest
                revision = -1 if before is None else before.state.revision
                previous_count = 0 if before is None else before.state.harvested_count
            if not client.wait_for_service(timeout_sec=max(0., deadline - time.monotonic())):
                response.message = f'Physics service is unavailable: {client.srv_name}'
                return response
            future = client.call_async(Trigger.Request())
            completed = threading.Event()
            future.add_done_callback(lambda _: completed.set())
            if not completed.wait(max(0., deadline - time.monotonic())):
                future.cancel()
                response.message = 'Physics harvest command timed out'
                return response
            result = future.result()
            if result is None or not result.success:
                response.message = 'Physics service returned no result' if result is None else result.message
                with self._condition:
                    current = self._latest
                return response if current is None else self._fill_harvest_response(response, current.state)
            with self._condition:
                while rclpy.ok():
                    current = self._latest
                    if (current is not None and current.state.revision > revision and
                            self._command_confirmed(request.command, current.state, previous_count) and
                            (not self._scene_revision_topic or
                             self._planning_scene_revision >= current.state.revision)):
                        break
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        response.message = ('Physics command succeeded but its matching planning scene has not arrived'
                                            if self._scene_revision_topic and current is not None and
                                            current.state.revision > revision else
                                            'Physics command succeeded but its resulting state has not arrived')
                        return response if current is None else self._fill_harvest_response(response, current.state)
                    self._condition.wait(min(remaining, .1))
                if current is None:
                    response.message = 'Physics state is unavailable'
                    return response
                self._sample = None
                if request.command == ManageHarvest.Request.RESET:
                    self._marker_reset_pending = True
            response.success = True
            response.message = result.message
            self._publish_markers(current.state)
            return self._fill_harvest_response(response, current.state)
        except Exception as error:
            response.message = f'Physics harvest command failed: {error}'
            self.get_logger().error(response.message)
            return response
        finally:
            self._command_lock.release()

    @staticmethod
    def _error(response, code, message):
        response.success = False
        response.status_code = code
        response.message = message
        response.targets = []
        return response

    def _get_targets(self, request, response):
        with self._condition:
            sample = self._sample
            info = self._camera_info
        if sample is None:
            return self._error(response, GetTargets.Response.NO_FRAME,
                               'No synchronized physics/TF ground-truth sample is available')
        response.header = sample.header
        max_age = float(request.max_frame_age_sec)
        confidence = float(request.min_confidence)
        if not math.isfinite(max_age) or not math.isfinite(confidence) or confidence > 1:
            return self._error(response, GetTargets.Response.BAD_FRAME, 'Frame age/confidence request is invalid')
        if max_age <= 0:
            max_age = float(self.get_parameter('default_max_frame_age_sec').value)
        if confidence <= 0:
            confidence = float(self.get_parameter('default_confidence').value)
        source_age = (self.get_clock().now().nanoseconds - _stamp_ns(sample.header.stamp)) / 1e9
        age = max(source_age, (time.monotonic_ns() - sample.received_ns) / 1e9)
        if source_age < -.05 or age > max_age:
            return self._error(response, GetTargets.Response.STALE_FRAME,
                               f'Latest physics ground-truth sample is stale: {age:.3f}s')
        wanted = {value.strip().lower() for value in request.class_filter if value.strip()}
        targets = []
        if (not wanted or 'ripe' in wanted) and self.get_parameter('confidence').value >= confidence:
            for point in sample.positions:
                if not np.isfinite(point).all() or point[2] <= 0:
                    continue
                target = TargetPoint()
                target.semantic_label = 'ripe'
                target.class_id = 0
                target.confidence = self.get_parameter('confidence').value
                target.position.x, target.position.y, target.position.z = map(float, point)
                if info is not None:
                    target.pixel_u = int(round(info.k[0] * point[0] / point[2] + info.k[2]))
                    target.pixel_v = int(round(info.k[4] * point[1] / point[2] + info.k[5]))
                targets.append(target)
        response.success = True
        response.status_code = GetTargets.Response.OK
        response.message = f'OK: {len(targets)} MuJoCo ground-truth target(s); no image inference'
        response.targets = targets
        return response

    def _marker(self, namespace, index, kind, point, scale, color, state):
        marker = Marker()
        marker.header = deepcopy(state.header)
        marker.ns = namespace
        marker.id = index
        marker.type = kind
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, point)
        marker.pose.orientation.w = 1.
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def _publish_markers(self, state):
        with self._marker_lock:
            stamp_ns = _stamp_ns(state.header.stamp)
            if stamp_ns < self._last_marker_ns and not self._marker_reset_pending:
                return
            self._last_marker_ns = stamp_ns
            self._publish_marker_snapshot(state)

    def _publish_marker_snapshot(self, state):
        with self._condition:
            reset = self._marker_reset_pending
            self._marker_reset_pending = False
            tcp = None if self._latest_tcp is None else self._latest_tcp.copy()
        markers = []
        if reset:
            clear = Marker()
            clear.header = deepcopy(state.header)
            clear.action = Marker.DELETEALL
            markers.append(clear)
        names = ('available', 'held', 'falling', 'collected', 'dropped')
        colors = ((.9, .1, .05, 1.), (1., .65, .05, 1.), (1., .8, .15, 1.),
                  (.2, .85, .25, 1.), (.55, .55, .55, 1.))
        for index, pose in enumerate(state.fruit_poses):
            point = np.array([pose.position.x, pose.position.y, pose.position.z])
            code = state.fruit_states[index]
            color = colors[code] if code < len(colors) else (.55, .55, .55, 1.)
            if state.selected_index == index + 1 and code == 0:
                color = (1., .65, .05, 1.)
            fruit = self._marker('harvest_fruits', index, Marker.SPHERE, point, (.05, .05, .05), color, state)
            fruit.pose.orientation = deepcopy(pose.orientation)
            markers.append(fruit)
            point[2] += .045 + index * .025
            label = self._marker('harvest_fruit_labels', index, Marker.TEXT_VIEW_FACING, point,
                                 (.009, 0., .018), (1., 1., 1., 1.), state)
            label.text = f'{index + 1} {names[code] if code < len(names) else "unknown"}'
            markers.append(label)
        basket = np.asarray(self._basket['position'])
        size = self._basket['dimensions']
        markers.append(self._marker('harvest_basket', 0, Marker.CUBE, basket, size, (.2, .55, .85, .6), state))
        thickness = self.get_parameter('basket_wall_thickness_m').value
        height = self.get_parameter('basket_wall_height_m').value
        for axis in (0, 1):
            for side in (-1, 1):
                point = basket.copy()
                point[axis] += side * (size[axis] + thickness) / 2.
                point[2] += (size[2] + height) / 2.
                dimensions = [size[0] + 2 * thickness, size[1] + 2 * thickness, height]
                dimensions[axis] = thickness
                markers.append(self._marker('harvest_basket', 1 + axis * 2 + (side + 1) // 2,
                                            Marker.CUBE, point, dimensions, (.2, .55, .85, .35), state))
        if tcp is not None:
            markers.append(self._marker('harvest_tcp', 0, Marker.SPHERE, tcp[:3, 3],
                                        (.012, .012, .012), (.1, .8, 1., 1.), state))
        index = state.selected_index - 1
        selected = (state.ready and 0 <= index < len(state.fruit_poses) and
                    state.fruit_states[index] == 0 and not state.held_index)
        for marker_id in (0, 1):
            point = basket.copy()
            if selected:
                fruit = state.fruit_poses[index].position
                point = np.array([fruit.x, fruit.y, fruit.z])
                if marker_id == 0:
                    point -= (_pose_matrix(state.initial_tcp.pose)[:3, 2] *
                              self.get_parameter('standoff_m').value)
            goal = self._marker('harvest_goals', marker_id,
                                Marker.SPHERE if marker_id == 0 else Marker.LINE_LIST,
                                point, (.014, .014, .014),
                                (.2, .8, 1., 1.) if marker_id == 0 else (1., 1., .2, 1.), state)
            if marker_id == 1:
                goal.scale.x = .003
                for axis in range(3):
                    for sign in (-1., 1.):
                        values = [0., 0., 0.]
                        values[axis] = sign * .035
                        goal.points.append(Point(x=values[0], y=values[1], z=values[2]))
            if not selected:
                goal.action = Marker.DELETE
            markers.append(goal)
        position = basket.copy()
        position[2] += .30
        status = self._marker('harvest_status', 0, Marker.TEXT_VIEW_FACING, position,
                              (.01, 0., .020), (1., 1., 1., 1.), state)
        status.text = f'MuJoCo: {state.phase}\nCollected: {state.harvested_count}/{len(state.fruit_poses)}'
        markers.append(status)
        self._markers.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = MujocoTargetServer()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
