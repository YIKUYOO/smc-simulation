"""Marker factory — construct RViz visualization_msgs/Marker objects."""

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Time
import math

FRAME_ID = "map"


def _apply_color(marker: Marker, rgba: tuple[float, float, float, float]) -> None:
    marker.color.r = rgba[0]
    marker.color.g = rgba[1]
    marker.color.b = rgba[2]
    marker.color.a = rgba[3]


def _stamp(sec: int, nanosec: int) -> Time:
    t = Time()
    t.sec = sec
    t.nanosec = nanosec
    return t


def create_robot_marker(
    x: float,
    y: float,
    stamp: Time,
    ns: str = "smc_robot",
    marker_id: int = 0,
    marker_type: int = Marker.SPHERE,
    scale: float = 0.35,
    color: tuple[float, float, float, float] = (0.0, 1.0, 1.0, 1.0),
) -> Marker:
    m = Marker()
    m.header.frame_id = FRAME_ID
    m.header.stamp = stamp
    m.ns = ns
    m.id = marker_id
    m.type = marker_type
    m.action = Marker.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = 0.0
    m.pose.orientation.w = 1.0
    m.scale.x = scale
    m.scale.y = scale
    m.scale.z = scale
    _apply_color(m, color)
    return m


def create_target_marker(x: float, y: float, stamp: Time) -> Marker:
    m = Marker()
    m.header.frame_id = FRAME_ID
    m.header.stamp = stamp
    m.ns = "smc_target"
    m.id = 1
    m.type = Marker.CYLINDER
    m.action = Marker.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = 0.0
    m.pose.orientation.w = 1.0
    m.scale.x = 0.5
    m.scale.y = 0.5
    m.scale.z = 0.05
    m.color.r = 1.0
    m.color.g = 0.0
    m.color.b = 0.0
    m.color.a = 0.85
    return m


def create_trajectory_marker(
    points: list,
    stamp: Time,
    ns: str = "smc_trajectory",
    marker_id: int = 2,
    color: tuple[float, float, float, float] = (0.0, 1.0, 0.0, 0.8),
    width: float = 0.03,
) -> Marker:
    m = Marker()
    m.header.frame_id = FRAME_ID
    m.header.stamp = stamp
    m.ns = ns
    m.id = marker_id
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = width
    _apply_color(m, color)
    m.points = points
    return m


def create_sliding_surface_marker(
    sx: float, sy: float, rx: float, ry: float, stamp: Time
) -> Marker:
    m = Marker()
    m.header.frame_id = FRAME_ID
    m.header.stamp = stamp
    m.ns = "smc_sliding"
    m.id = 3
    m.type = Marker.ARROW
    m.action = Marker.ADD

    mag = math.sqrt(sx * sx + sy * sy)
    vis_scale = min(mag, 3.0) * 0.5  # cap visual length

    start = Point()
    start.x = rx
    start.y = ry
    start.z = 0.05

    end = Point()
    if mag > 1e-6:
        end.x = rx + sx / mag * vis_scale
        end.y = ry + sy / mag * vis_scale
    else:
        end.x = rx
        end.y = ry
    end.z = 0.05

    m.points = [start, end]
    m.scale.x = 0.06  # shaft diameter
    m.scale.y = 0.12  # head diameter
    m.scale.z = 0.0
    m.color.r = 1.0
    m.color.g = 1.0
    m.color.b = 0.0
    m.color.a = 0.9
    return m


def create_control_arrow_marker(
    ux: float, uy: float, rx: float, ry: float, stamp: Time
) -> Marker:
    m = Marker()
    m.header.frame_id = FRAME_ID
    m.header.stamp = stamp
    m.ns = "smc_control"
    m.id = 4
    m.type = Marker.ARROW
    m.action = Marker.ADD

    mag = math.sqrt(ux * ux + uy * uy)
    vis_scale = min(mag, 5.0) * 0.3

    start = Point()
    start.x = rx
    start.y = ry
    start.z = 0.1

    end = Point()
    if mag > 1e-6:
        end.x = rx + ux / mag * vis_scale
        end.y = ry + uy / mag * vis_scale
    else:
        end.x = rx
        end.y = ry
    end.z = 0.1

    m.points = [start, end]
    m.scale.x = 0.05
    m.scale.y = 0.10
    m.scale.z = 0.0
    m.color.r = 1.0
    m.color.g = 0.0
    m.color.b = 1.0
    m.color.a = 0.9
    return m


def create_error_line_marker(
    rx: float,
    ry: float,
    tx: float,
    ty: float,
    stamp: Time,
    ns: str = "smc_error",
    marker_id: int = 5,
    color: tuple[float, float, float, float] = (1.0, 1.0, 1.0, 0.4),
    width: float = 0.02,
) -> Marker:
    m = Marker()
    m.header.frame_id = FRAME_ID
    m.header.stamp = stamp
    m.ns = ns
    m.id = marker_id
    m.type = Marker.LINE_LIST
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = width

    p1 = Point()
    p1.x = rx
    p1.y = ry
    p1.z = 0.0
    p2 = Point()
    p2.x = tx
    p2.y = ty
    p2.z = 0.0
    m.points = [p1, p2]

    _apply_color(m, color)
    return m


def create_info_text_marker(
    text: str,
    rx: float,
    ry: float,
    stamp: Time,
    ns: str = "smc_info",
    marker_id: int = 6,
    color: tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0),
    offset_y: float = 0.8,
    scale_z: float = 0.3,
) -> Marker:
    m = Marker()
    m.header.frame_id = FRAME_ID
    m.header.stamp = stamp
    m.ns = ns
    m.id = marker_id
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.pose.position.x = rx
    m.pose.position.y = ry + offset_y
    m.pose.position.z = 0.5
    m.pose.orientation.w = 1.0
    m.scale.z = scale_z
    _apply_color(m, color)
    m.text = text
    return m


def create_disturbance_arrow_marker(
    fx: float,
    fy: float,
    rx: float,
    ry: float,
    stamp: Time,
    ns: str,
    marker_id: int,
    color: tuple[float, float, float, float] = (1.0, 0.5, 0.0, 0.95),
) -> Marker:
    m = Marker()
    m.header.frame_id = FRAME_ID
    m.header.stamp = stamp
    m.ns = ns
    m.id = marker_id
    m.type = Marker.ARROW
    m.action = Marker.ADD

    mag = math.sqrt(fx * fx + fy * fy)
    vis_scale = min(max(mag * 0.12, 0.4), 1.8)

    start = Point()
    start.x = rx
    start.y = ry
    start.z = 0.18

    end = Point()
    if mag > 1e-6:
        end.x = rx + fx / mag * vis_scale
        end.y = ry + fy / mag * vis_scale
    else:
        end.x = rx
        end.y = ry
    end.z = 0.18

    m.points = [start, end]
    m.scale.x = 0.07
    m.scale.y = 0.14
    m.scale.z = 0.0
    _apply_color(m, color)
    return m
