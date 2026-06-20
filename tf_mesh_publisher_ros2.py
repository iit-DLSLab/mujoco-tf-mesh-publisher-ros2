#!/usr/bin/env python3
"""ROS2 XML robot state publisher for MuJoCo-style models in this repository.

This node reads a MuJoCo XML model (including <include file="..."/>), subscribes to
sensor_msgs/JointState, and publishes TF transforms similarly to robot_state_publisher.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]  # (w, x, y, z)


@dataclass
class TransformSpec:
    parent_frame: str
    child_frame: str
    body_pos: Vec3
    body_quat: Quat
    joint_name: Optional[str]
    joint_type: str  # "hinge", "slide", or "fixed"
    joint_axis: Vec3
    joint_pos: Vec3


@dataclass
class MeshAssetSpec:
    path: Path
    scale: Vec3


@dataclass
class MeshMarkerSpec:
    frame_id: str
    mesh_path: Path
    pos: Vec3
    quat: Quat
    scale: Vec3
    rgba: Tuple[float, float, float, float]


def _parse_vec3(text: Optional[str], default: Vec3 = (0.0, 0.0, 0.0)) -> Vec3:
    if not text:
        return default
    values = [float(v) for v in text.split()]
    if len(values) != 3:
        raise ValueError(f"Expected 3 values, got {values}")
    return values[0], values[1], values[2]


def _quat_normalize(q: Quat) -> Quat:
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n < 1e-12:
        return 1.0, 0.0, 0.0, 0.0
    return q[0] / n, q[1] / n, q[2] / n, q[3] / n


def _quat_mul(q1: Quat, q2: Quat) -> Quat:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def _quat_from_axis_angle(axis: Vec3, angle: float) -> Quat:
    ax, ay, az = axis
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if norm < 1e-12:
        return 1.0, 0.0, 0.0, 0.0
    ax /= norm
    ay /= norm
    az /= norm
    half = 0.5 * angle
    s = math.sin(half)
    return _quat_normalize((math.cos(half), ax * s, ay * s, az * s))


def _quat_rotate_vec(q: Quat, v: Vec3) -> Vec3:
    # v' = q * (0, v) * q_conjugate
    w, x, y, z = q
    vx, vy, vz = v
    # first multiplication q * (0, v)
    rw = -x * vx - y * vy - z * vz
    rx = w * vx + y * vz - z * vy
    ry = w * vy + z * vx - x * vz
    rz = w * vz + x * vy - y * vx
    # multiply by conjugate
    return (
        -rw * x + rx * w - ry * z + rz * y,
        -rw * y + rx * z + ry * w - rz * x,
        -rw * z - rx * y + ry * x + rz * w,
    )


def _quat_from_euler_xyz(roll: float, pitch: float, yaw: float) -> Quat:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    # x-y-z intrinsic sequence
    return _quat_normalize(
        (
            cr * cp * cy - sr * sp * sy,
            sr * cp * cy + cr * sp * sy,
            cr * sp * cy - sr * cp * sy,
            cr * cp * sy + sr * sp * cy,
        )
    )


def _parse_body_quat(body_elem: ET.Element, angle_in_radians: bool) -> Quat:
    quat_attr = body_elem.get("quat")
    if quat_attr:
        qw, qx, qy, qz = [float(v) for v in quat_attr.split()]
        return _quat_normalize((qw, qx, qy, qz))

    axisangle_attr = body_elem.get("axisangle")
    if axisangle_attr:
        vals = [float(v) for v in axisangle_attr.split()]
        if len(vals) != 4:
            raise ValueError(f"axisangle expects 4 values, got {vals}")
        ax, ay, az, angle = vals
        if not angle_in_radians:
            angle = math.radians(angle)
        return _quat_from_axis_angle((ax, ay, az), angle)

    euler_attr = body_elem.get("euler")
    if euler_attr:
        roll, pitch, yaw = [float(v) for v in euler_attr.split()]
        if not angle_in_radians:
            roll = math.radians(roll)
            pitch = math.radians(pitch)
            yaw = math.radians(yaw)
        return _quat_from_euler_xyz(roll, pitch, yaw)

    return 1.0, 0.0, 0.0, 0.0


def _resolve_includes(xml_path: Path, visited: set[Path]) -> List[Tuple[ET.Element, Path]]:
    xml_path = xml_path.resolve()
    if xml_path in visited:
        return []
    visited.add(xml_path)

    root = ET.parse(xml_path).getroot()
    roots: List[Tuple[ET.Element, Path]] = []

    for include_elem in root.findall("include"):
        include_file = include_elem.get("file")
        if not include_file:
            continue
        include_path = (xml_path.parent / include_file).resolve()
        roots.extend(_resolve_includes(include_path, visited))

    roots.append((root, xml_path))
    return roots


def _collect_mesh_assets(roots_with_paths: List[Tuple[ET.Element, Path]]) -> Dict[str, MeshAssetSpec]:
    assets: Dict[str, MeshAssetSpec] = {}
    for root, xml_path in roots_with_paths:
        asset_elem = root.find("asset")
        if asset_elem is None:
            continue
        for mesh_elem in asset_elem.findall("mesh"):
            mesh_file = mesh_elem.get("file")
            if not mesh_file:
                continue
            mesh_name = mesh_elem.get("name")
            if not mesh_name:
                mesh_name = Path(mesh_file).stem
            scale = _parse_vec3(mesh_elem.get("scale"), (1.0, 1.0, 1.0))
            assets[mesh_name] = MeshAssetSpec(path=(xml_path.parent / mesh_file).resolve(), scale=scale)
    return assets


def _collect_transform_specs(
    root_xml: Path,
    world_frame: str,
    frame_prefix: str,
) -> Tuple[List[TransformSpec], List[MeshMarkerSpec]]:
    roots = _resolve_includes(root_xml, visited=set())
    mesh_assets = _collect_mesh_assets(roots)

    # If angle is not specified in compiler, MuJoCo default is degrees.
    angle_in_radians = False
    for root, _ in roots:
        compiler = root.find("compiler")
        if compiler is not None and compiler.get("angle") is not None:
            angle_in_radians = compiler.get("angle") == "radian"

    specs: List[TransformSpec] = []
    mesh_specs: List[MeshMarkerSpec] = []
    used_names: Dict[str, int] = {}

    def unique_frame(name: str) -> str:
        base = f"{frame_prefix}{name}"
        if base not in used_names:
            used_names[base] = 0
            return base
        used_names[base] += 1
        return f"{base}_{used_names[base]}"

    def walk_body(parent_frame: str, body_elem: ET.Element) -> None:
        body_name = body_elem.get("name", "unnamed_body")
        child_frame = unique_frame(body_name)

        body_pos = _parse_vec3(body_elem.get("pos"), (0.0, 0.0, 0.0))
        body_quat = _parse_body_quat(body_elem, angle_in_radians)

        joint_elems = body_elem.findall("joint")
        joint_name: Optional[str] = None
        joint_type = "fixed"
        joint_axis: Vec3 = (0.0, 0.0, 1.0)
        joint_pos: Vec3 = (0.0, 0.0, 0.0)

        if joint_elems:
            joint = joint_elems[0]
            joint_name = joint.get("name")
            joint_type = joint.get("type", "hinge")
            joint_axis = _parse_vec3(joint.get("axis"), (0.0, 0.0, 1.0))
            joint_pos = _parse_vec3(joint.get("pos"), (0.0, 0.0, 0.0))

            if joint_type not in ("hinge", "slide"):
                joint_type = "fixed"
                joint_name = None

        specs.append(
            TransformSpec(
                parent_frame=parent_frame,
                child_frame=child_frame,
                body_pos=body_pos,
                body_quat=body_quat,
                joint_name=joint_name,
                joint_type=joint_type,
                joint_axis=joint_axis,
                joint_pos=joint_pos,
            )
        )

        for geom in body_elem.findall("geom"):
            mesh_name = geom.get("mesh")
            geom_type = geom.get("type")
            if not mesh_name and geom_type != "mesh":
                continue

            geom_class = geom.get("class", "")
            if "collision" in geom_class.lower():
                continue

            if not mesh_name:
                continue
            asset = mesh_assets.get(mesh_name)
            if asset is None:
                continue
            if not asset.path.exists():
                continue

            geom_pos = _parse_vec3(geom.get("pos"), (0.0, 0.0, 0.0))
            geom_quat = _parse_body_quat(geom, angle_in_radians)
            geom_rgba = geom.get("rgba")
            if geom_rgba:
                vals = [float(v) for v in geom_rgba.split()]
                if len(vals) == 4:
                    rgba = (vals[0], vals[1], vals[2], vals[3])
                else:
                    rgba = (0.8, 0.8, 0.8, 1.0)
            else:
                rgba = (0.8, 0.8, 0.8, 1.0)

            mesh_specs.append(
                MeshMarkerSpec(
                    frame_id=child_frame,
                    mesh_path=asset.path,
                    pos=geom_pos,
                    quat=geom_quat,
                    scale=asset.scale,
                    rgba=rgba,
                )
            )

        for child_body in body_elem.findall("body"):
            walk_body(child_frame, child_body)

    for root, _ in roots:
        worldbody = root.find("worldbody")
        if worldbody is None:
            continue
        for top_body in worldbody.findall("body"):
            walk_body(world_frame, top_body)

    return specs, mesh_specs


class XmlRobotStatePublisher(Node):
    def __init__(
        self,
        xml_path: Path,
        world_frame: str,
        frame_prefix: str,
        joint_states_topic: str,
        publish_rate: float,
        use_sim_time: bool,
        mesh_topic: str,
        mesh_publish_rate: float,
    ) -> None:
        super().__init__("xml_robot_state_publisher")

        # Keep RViz and TF timestamps consistent with simulation time when requested.
        self.set_parameters([Parameter("use_sim_time", value=use_sim_time)])

        self._xml_path = xml_path
        self._joint_positions: Dict[str, float] = {}
        self._specs, self._mesh_specs = _collect_transform_specs(
            root_xml=xml_path,
            world_frame=world_frame,
            frame_prefix=frame_prefix,
        )

        self._dynamic_specs = [s for s in self._specs if s.joint_name is not None and s.joint_type in ("hinge", "slide")]
        self._static_specs = [s for s in self._specs if s not in self._dynamic_specs]

        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_static_broadcaster = StaticTransformBroadcaster(self)

        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._mesh_pub = self.create_publisher(MarkerArray, mesh_topic, marker_qos)
        self._joint_states_sub = self.create_subscription(
            JointState,
            joint_states_topic,
            self._joint_state_callback,
            10,
        )

        period = 1.0 / max(publish_rate, 1e-3)
        self._timer = self.create_timer(period, self._publish_tf)
        self._mesh_timer = self.create_timer(1.0 / max(mesh_publish_rate, 1e-3), self._publish_meshes)

        self.get_logger().info(f"Loaded XML model: {xml_path}")
        self.get_logger().info(f"Parsed transforms: total={len(self._specs)} dynamic={len(self._dynamic_specs)} static={len(self._static_specs)}")
        self.get_logger().info(f"Parsed visual meshes: {len(self._mesh_specs)}")
        self.get_logger().info(f"Subscribing to JointState topic: {joint_states_topic}")
        self.get_logger().info(f"use_sim_time={use_sim_time}")
        self.get_logger().info(f"Mesh topic: {mesh_topic}")

        if not self._specs:
            self.get_logger().warning("No body transforms found in XML. Check that the selected file has a worldbody/body tree.")

        self._publish_static_tf_once()
        self._publish_meshes()

    def _joint_state_callback(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self._joint_positions[name] = msg.position[i]

    def _build_transform(self, spec: TransformSpec, stamp) -> TransformStamped:
        q_total = spec.body_quat
        t_local = list(spec.body_pos)

        if spec.joint_name and spec.joint_type in ("hinge", "slide"):
            q = self._joint_positions.get(spec.joint_name, 0.0)
            if spec.joint_type == "hinge":
                q_joint = _quat_from_axis_angle(spec.joint_axis, q)
                q_total = _quat_mul(spec.body_quat, q_joint)

                # Account for MuJoCo joint pos pivot in body frame.
                p = spec.joint_pos
                rp = _quat_rotate_vec(q_joint, p)
                delta = (p[0] - rp[0], p[1] - rp[1], p[2] - rp[2])
                delta_parent = _quat_rotate_vec(spec.body_quat, delta)
                t_local[0] += delta_parent[0]
                t_local[1] += delta_parent[1]
                t_local[2] += delta_parent[2]
            else:  # slide
                axis = spec.joint_axis
                axis_norm = math.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2])
                if axis_norm > 1e-12:
                    axis = (axis[0] / axis_norm, axis[1] / axis_norm, axis[2] / axis_norm)
                delta_body = (axis[0] * q, axis[1] * q, axis[2] * q)
                delta_parent = _quat_rotate_vec(spec.body_quat, delta_body)
                t_local[0] += delta_parent[0]
                t_local[1] += delta_parent[1]
                t_local[2] += delta_parent[2]

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = spec.parent_frame
        t.child_frame_id = spec.child_frame
        t.transform.translation.x = float(t_local[0])
        t.transform.translation.y = float(t_local[1])
        t.transform.translation.z = float(t_local[2])

        qn = _quat_normalize(q_total)
        t.transform.rotation.x = float(qn[1])
        t.transform.rotation.y = float(qn[2])
        t.transform.rotation.z = float(qn[3])
        t.transform.rotation.w = float(qn[0])
        return t

    def _publish_tf(self) -> None:
        now = self.get_clock().now().to_msg()
        transforms = [self._build_transform(spec, now) for spec in self._dynamic_specs]
        if transforms:
            self._tf_broadcaster.sendTransform(transforms)

    def _publish_static_tf_once(self) -> None:
        now = self.get_clock().now().to_msg()
        transforms = [self._build_transform(spec, now) for spec in self._static_specs]
        if transforms:
            self._tf_static_broadcaster.sendTransform(transforms)

    def _publish_meshes(self) -> None:
        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()
        for idx, spec in enumerate(self._mesh_specs):
            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = spec.frame_id
            marker.ns = "xml_meshes"
            marker.id = idx
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            marker.frame_locked = True
            marker.mesh_resource = f"file://{spec.mesh_path}"
            marker.mesh_use_embedded_materials = True

            marker.pose.position.x = float(spec.pos[0])
            marker.pose.position.y = float(spec.pos[1])
            marker.pose.position.z = float(spec.pos[2])
            marker.pose.orientation.w = float(spec.quat[0])
            marker.pose.orientation.x = float(spec.quat[1])
            marker.pose.orientation.y = float(spec.quat[2])
            marker.pose.orientation.z = float(spec.quat[3])

            marker.scale.x = float(spec.scale[0])
            marker.scale.y = float(spec.scale[1])
            marker.scale.z = float(spec.scale[2])

            marker.color.r = float(spec.rgba[0])
            marker.color.g = float(spec.rgba[1])
            marker.color.b = float(spec.rgba[2])
            marker.color.a = float(spec.rgba[3])
            marker.lifetime.sec = 0
            marker_array.markers.append(marker)

        self._mesh_pub.publish(marker_array)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Publish TF from MuJoCo XML robot model using JointState input"
    )
    parser.add_argument(
        "--xml",
        required=True,
        type=Path,
        help="Path to MuJoCo XML model (e.g. robot_model/go2/scene_flat.xml)",
    )
    parser.add_argument(
        "--world-frame",
        default="world",
        help="Name of root TF frame",
    )
    parser.add_argument(
        "--frame-prefix",
        default="",
        help="Optional prefix prepended to all generated body frame names",
    )
    parser.add_argument(
        "--joint-states-topic",
        default="/joint_states",
        help="JointState topic used for joint positions",
    )
    parser.add_argument(
        "--publish-rate",
        default=60.0,
        type=float,
        help="TF publish rate in Hz",
    )
    parser.add_argument(
        "--use-sim-time",
        action="store_true",
        help="Use /clock time for TF timestamps",
    )
    parser.add_argument(
        "--mesh-topic",
        default="/xml_robot_meshes",
        help="MarkerArray topic used to publish mesh visualization",
    )
    parser.add_argument(
        "--mesh-publish-rate",
        default=1.0,
        type=float,
        help="Mesh MarkerArray publish rate in Hz",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = parse_args(argv)
    xml_path = args.xml.resolve()
    if not xml_path.exists():
        raise FileNotFoundError(f"XML file not found: {xml_path}")

    rclpy.init(args=None)
    node = XmlRobotStatePublisher(
        xml_path=xml_path,
        world_frame=args.world_frame,
        frame_prefix=args.frame_prefix,
        joint_states_topic=args.joint_states_topic,
        publish_rate=args.publish_rate,
        use_sim_time=args.use_sim_time,
        mesh_topic=args.mesh_topic,
        mesh_publish_rate=args.mesh_publish_rate,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
