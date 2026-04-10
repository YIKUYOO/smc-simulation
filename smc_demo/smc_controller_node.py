"""
SMC Controller Node — main ROS 2 node for sliding mode control demo.

Publishes:
  /smc/markers  (MarkerArray)   — all visualization markers
  /smc/path     (Path)          — trajectory trail
  /smc/state    (Float64MultiArray) — raw state for external plotting

Broadcasts:
  map -> base_link  (dynamic TF, updated every step)

Parameters are dynamically reconfigurable at runtime.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

from smc_demo.smc_math import (
    SmcState,
    SmcTarget,
    SmcParams,
    SmcOutput,
    compute_control,
    step_dynamics,
)
from smc_demo import marker_factory as mf


class SmcControllerNode(Node):

    def __init__(self):
        super().__init__("smc_controller")

        # --- Declare parameters ---
        self.declare_parameter("initial_x", 8.0)
        self.declare_parameter("initial_y", 6.0)
        self.declare_parameter("target_x", 0.0)
        self.declare_parameter("target_y", 0.0)
        self.declare_parameter("lambda_gain", 4.0)
        self.declare_parameter("k_gain", 8.0)
        self.declare_parameter("phi", 0.2)
        self.declare_parameter("use_saturation", True)
        self.declare_parameter("mass", 1.0)
        self.declare_parameter("damping", 0.1)
        self.declare_parameter("dt", 0.02)
        self.declare_parameter("sim_rate", 50.0)
        self.declare_parameter("enable_comparison", False)
        self.declare_parameter("disturbance_enabled", False)
        self.declare_parameter("disturbance_start", 1.5)
        self.declare_parameter("disturbance_duration", 0.35)
        self.declare_parameter("disturbance_fx", 3.0)
        self.declare_parameter("disturbance_fy", -4.0)

        # --- Read initial values ---
        ix = self.get_parameter("initial_x").value
        iy = self.get_parameter("initial_y").value

        self._state = SmcState(x=ix, y=iy, vx=0.0, vy=0.0)
        self._baseline_state = SmcState(x=ix, y=iy, vx=0.0, vy=0.0)
        self._target = SmcTarget(
            x=self.get_parameter("target_x").value,
            y=self.get_parameter("target_y").value,
        )
        self._params = self._build_smc_params()
        self._dt = self.get_parameter("dt").value
        self._enable_comparison = self.get_parameter("enable_comparison").value
        self._disturbance_enabled = self.get_parameter("disturbance_enabled").value
        self._disturbance_start = self.get_parameter("disturbance_start").value
        self._disturbance_duration = self.get_parameter("disturbance_duration").value
        self._disturbance_fx = self.get_parameter("disturbance_fx").value
        self._disturbance_fy = self.get_parameter("disturbance_fy").value

        # --- Publishers ---
        self._pub_markers = self.create_publisher(MarkerArray, "/smc/markers", 10)
        self._pub_path = self.create_publisher(Path, "/smc/path", 10)
        self._pub_state = self.create_publisher(Float64MultiArray, "/smc/state", 10)

        # --- TF broadcaster (dynamic) ---
        self._tf_broadcaster = TransformBroadcaster(self)

        # --- Trajectory storage ---
        self._trajectory_points: list[Point] = []
        self._baseline_trajectory_points: list[Point] = []
        self._path_msg = Path()
        self._path_msg.header.frame_id = "map"

        # --- Bookkeeping ---
        self._step_count = 0
        self._converged = False
        self._sim_time = 0.0

        # --- Dynamic reconfigure ---
        self.add_on_set_parameters_callback(self._param_callback)

        # --- Timer ---
        rate = self.get_parameter("sim_rate").value
        period = 1.0 / rate
        self._timer = self.create_timer(period, self._step_callback)

        self.get_logger().info(
            f"SMC Controller started: ({ix}, {iy}) -> "
            f"({self._target.x}, {self._target.y})  "
            f"mode={'sat' if self._params.use_saturation else 'sign'}  "
            f"comparison={'on' if self._enable_comparison else 'off'}"
        )

    # ------------------------------------------------------------------
    # Parameter helpers
    # ------------------------------------------------------------------

    def _build_smc_params(self) -> SmcParams:
        return SmcParams(
            lambda_gain=self.get_parameter("lambda_gain").value,
            k_gain=self.get_parameter("k_gain").value,
            phi=self.get_parameter("phi").value,
            use_saturation=self.get_parameter("use_saturation").value,
            mass=self.get_parameter("mass").value,
            damping=self.get_parameter("damping").value,
        )

    def _param_callback(self, params: list[Parameter]) -> SetParametersResult:
        for p in params:
            self.get_logger().info(f"Param changed: {p.name} = {p.value}")
        self._target = SmcTarget(
            x=self.get_parameter("target_x").value,
            y=self.get_parameter("target_y").value,
        )
        self._params = self._build_smc_params()
        self._dt = self.get_parameter("dt").value
        self._enable_comparison = self.get_parameter("enable_comparison").value
        self._disturbance_enabled = self.get_parameter("disturbance_enabled").value
        self._disturbance_start = self.get_parameter("disturbance_start").value
        self._disturbance_duration = self.get_parameter("disturbance_duration").value
        self._disturbance_fx = self.get_parameter("disturbance_fx").value
        self._disturbance_fy = self.get_parameter("disturbance_fy").value
        return SetParametersResult(successful=True)

    def _current_disturbance_force(self) -> tuple[float, float]:
        if not self._disturbance_enabled:
            return (0.0, 0.0)
        disturbance_end = self._disturbance_start + self._disturbance_duration
        if self._disturbance_start <= self._sim_time < disturbance_end:
            return (self._disturbance_fx, self._disturbance_fy)
        return (0.0, 0.0)

    def _zero_control_output(self, state: SmcState) -> SmcOutput:
        ex = state.x - self._target.x
        ey = state.y - self._target.y
        return SmcOutput(
            ux=0.0,
            uy=0.0,
            sx=state.vx + self._params.lambda_gain * ex,
            sy=state.vy + self._params.lambda_gain * ey,
            ex=ex,
            ey=ey,
        )

    # ------------------------------------------------------------------
    # Main simulation step
    # ------------------------------------------------------------------

    def _step_callback(self):
        now = self.get_clock().now().to_msg()
        disturbance_force = self._current_disturbance_force()

        # 1. Compute control
        ctrl = compute_control(self._state, self._target, self._params)
        baseline_ctrl = (
            self._zero_control_output(self._baseline_state)
            if self._enable_comparison
            else None
        )

        # 2. Step dynamics (immutable — returns new state)
        self._state = step_dynamics(
            self._state,
            ctrl,
            self._dt,
            self._params,
            external_force=disturbance_force,
        )
        if self._enable_comparison and baseline_ctrl is not None:
            self._baseline_state = step_dynamics(
                self._baseline_state,
                baseline_ctrl,
                self._dt,
                self._params,
                external_force=disturbance_force,
            )

        # 3. Accumulate trajectory (cap at 5000 points)
        pt = Point()
        pt.x = self._state.x
        pt.y = self._state.y
        pt.z = 0.0
        self._trajectory_points.append(pt)
        if len(self._trajectory_points) > 5000:
            self._trajectory_points = self._trajectory_points[-5000:]
        if self._enable_comparison and baseline_ctrl is not None:
            baseline_pt = Point()
            baseline_pt.x = self._baseline_state.x
            baseline_pt.y = self._baseline_state.y
            baseline_pt.z = 0.0
            self._baseline_trajectory_points.append(baseline_pt)
            if len(self._baseline_trajectory_points) > 5000:
                self._baseline_trajectory_points = self._baseline_trajectory_points[-5000:]

        # 4. Build markers
        err_mag = math.sqrt(ctrl.ex**2 + ctrl.ey**2)
        s_mag = math.sqrt(ctrl.sx**2 + ctrl.sy**2)
        mode_str = "sat" if self._params.use_saturation else "sign"
        disturbance_mag = math.sqrt(
            disturbance_force[0] ** 2 + disturbance_force[1] ** 2
        )
        disturbance_state = (
            f"ON ({disturbance_force[0]:.1f}, {disturbance_force[1]:.1f})"
            if disturbance_mag > 0.0
            else "OFF"
        )
        baseline_err_mag = (
            math.sqrt(baseline_ctrl.ex**2 + baseline_ctrl.ey**2)
            if baseline_ctrl is not None
            else 0.0
        )
        info = (
            f"SMC |e|={err_mag:.3f}  |s|={s_mag:.3f}\n"
            f"u=({ctrl.ux:.2f},{ctrl.uy:.2f})  mode={mode_str}"
        )
        status_text = (
            "SMC only\n"
            f"Disturbance: {disturbance_state}\n"
            f"SMC error: {err_mag:.3f}"
        )
        if self._enable_comparison and baseline_ctrl is not None:
            status_text = (
                "SMC vs no control\n"
                f"Disturbance: {disturbance_state}\n"
                f"SMC error: {err_mag:.3f}\n"
                f"No-control error: {baseline_err_mag:.3f}"
            )

        markers = MarkerArray()
        markers.markers = [
            mf.create_robot_marker(self._state.x, self._state.y, now),
            mf.create_target_marker(self._target.x, self._target.y, now),
            mf.create_trajectory_marker(self._trajectory_points, now),
            mf.create_sliding_surface_marker(
                ctrl.sx, ctrl.sy, self._state.x, self._state.y, now
            ),
            mf.create_control_arrow_marker(
                ctrl.ux, ctrl.uy, self._state.x, self._state.y, now
            ),
            mf.create_error_line_marker(
                self._state.x, self._state.y,
                self._target.x, self._target.y, now
            ),
            mf.create_info_text_marker(
                info, self._state.x, self._state.y, now
            ),
            mf.create_info_text_marker(
                status_text,
                -2.0,
                9.0,
                now,
                ns="smc_status_panel",
                marker_id=20,
                color=(1.0, 1.0, 1.0, 1.0),
                offset_y=0.0,
                scale_z=0.35,
            ),
            mf.create_info_text_marker(
                "SMC",
                self._state.x,
                self._state.y,
                now,
                ns="smc_robot_label",
                marker_id=21,
                color=(0.0, 1.0, 1.0, 1.0),
                offset_y=0.45,
                scale_z=0.22,
            ),
        ]
        if disturbance_mag > 0.0:
            markers.markers.append(
                mf.create_disturbance_arrow_marker(
                    disturbance_force[0],
                    disturbance_force[1],
                    self._state.x,
                    self._state.y,
                    now,
                    ns="smc_disturbance",
                    marker_id=40,
                )
            )
        if self._enable_comparison and baseline_ctrl is not None:
            markers.markers.extend(
                [
                    mf.create_robot_marker(
                        self._baseline_state.x,
                        self._baseline_state.y,
                        now,
                        ns="baseline_robot",
                        marker_id=30,
                        marker_type=mf.Marker.CUBE,
                        scale=0.3,
                        color=(0.85, 0.85, 0.85, 1.0),
                    ),
                    mf.create_trajectory_marker(
                        self._baseline_trajectory_points,
                        now,
                        ns="baseline_trajectory",
                        marker_id=31,
                        color=(1.0, 0.55, 0.1, 0.95),
                        width=0.025,
                    ),
                    mf.create_error_line_marker(
                        self._baseline_state.x,
                        self._baseline_state.y,
                        self._target.x,
                        self._target.y,
                        now,
                        ns="baseline_error",
                        marker_id=32,
                        color=(1.0, 0.6, 0.2, 0.35),
                        width=0.015,
                    ),
                    mf.create_info_text_marker(
                        "No control",
                        self._baseline_state.x,
                        self._baseline_state.y,
                        now,
                        ns="baseline_robot_label",
                        marker_id=33,
                        color=(1.0, 0.72, 0.35, 1.0),
                        offset_y=0.45,
                        scale_z=0.22,
                    ),
                ]
            )
            if disturbance_mag > 0.0:
                markers.markers.extend(
                    [
                        mf.create_disturbance_arrow_marker(
                            disturbance_force[0],
                            disturbance_force[1],
                            self._baseline_state.x,
                            self._baseline_state.y,
                            now,
                            ns="baseline_disturbance",
                            marker_id=41,
                        ),
                    ]
                )
        self._pub_markers.publish(markers)

        # 5. Publish path
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = now
        pose.pose.position.x = self._state.x
        pose.pose.position.y = self._state.y
        self._path_msg.header.stamp = now
        self._path_msg.poses.append(pose)
        if len(self._path_msg.poses) > 5000:
            self._path_msg.poses = self._path_msg.poses[-5000:]
        self._pub_path.publish(self._path_msg)

        # 6. Publish raw state
        state_msg = Float64MultiArray()
        state_msg.data = [
            self._state.x, self._state.y,
            self._state.vx, self._state.vy,
            ctrl.sx, ctrl.sy,
            ctrl.ux, ctrl.uy,
            err_mag, s_mag,
        ]
        self._pub_state.publish(state_msg)

        # 7. Broadcast dynamic TF: map -> base_link
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "map"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self._state.x
        tf.transform.translation.y = self._state.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(tf)

        # 8. Periodic logging
        self._step_count += 1
        if self._step_count % 100 == 0:
            self.get_logger().info(
                f"step={self._step_count}  pos=({self._state.x:.3f},{self._state.y:.3f})  "
                f"|e|={err_mag:.3f}  |s|={s_mag:.3f}  "
                f"disturbance={'on' if disturbance_mag > 0.0 else 'off'}"
            )

        # 9. Convergence detection
        if not self._converged and err_mag < 0.01:
            self._converged = True
            self.get_logger().info("CONVERGED — error < 0.01")
        self._sim_time += self._dt


def main(args=None):
    rclpy.init(args=args)
    node = SmcControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
