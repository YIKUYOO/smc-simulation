"""
SMC Math Module — Pure mathematical core for Sliding Mode Control.

System model: damped double integrator in 2D
    m * x_ddot = u_x - b * x_dot
    m * y_ddot = u_y - b * y_dot

Sliding surface: s = e_dot + lambda * e, where e = pos - target
Control law:
    u_eq  = m * (-lambda * v + (b/m) * v)   # equivalent control with damping compensation
          = -(m * lambda - b) * v
    u_sw  = -K * switching(s)                # sign(s) or sat(s/phi)
    u     = u_eq + u_sw
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class SmcState:
    # 当前 x 位置
    x: float
    # 当前 y 位置
    y: float
    # 当前 x 方向速度
    vx: float
    # 当前 y 方向速度
    vy: float


@dataclass(frozen=True)
class SmcTarget:
    # 目标点 x 坐标
    x: float
    # 目标点 y 坐标
    y: float


@dataclass(frozen=True)
class SmcParams:
    # lambda 决定误差回到“理想轨道”的快慢
    lambda_gain: float = 2.0
    # k 决定切换控制的力度
    k_gain: float = 5.0
    # phi 是饱和函数边界层宽度，越小越接近 sign
    phi: float = 0.3
    # 是否启用更平滑的 saturation，而不是硬切换 sign
    use_saturation: bool = False
    # 系统质量 m
    mass: float = 1.0
    # 线性阻尼 b
    damping: float = 0.1


@dataclass(frozen=True)
class SmcOutput:
    # x 方向总控制输入
    ux: float
    # y 方向总控制输入
    uy: float
    # x 方向滑模面
    sx: float
    # y 方向滑模面
    sy: float
    # x 方向位置误差
    ex: float
    # y 方向位置误差
    ey: float


def sign(value: float) -> float:
    # 大于 0 返回 +1，说明当前偏差在正侧
    if value > 0.0:
        return 1.0
    # 小于 0 返回 -1，说明当前偏差在负侧
    elif value < 0.0:
        return -1.0
    # 等于 0 说明正好在切换面上
    return 0.0


def saturation(value: float, phi: float) -> float:
    # 先把输入按边界层宽度 phi 做归一化
    ratio = value / phi
    # 在边界层内部时直接按比例输出，这样更平滑
    if abs(ratio) <= 1.0:
        return ratio
    # 超出边界层后退化成 sign，只保留方向信息
    return sign(ratio)


def compute_sliding_surface(
    state: SmcState, target: SmcTarget, params: SmcParams
) -> tuple:
    # 位置误差 = 当前状态 - 目标状态
    ex = state.x - target.x
    ey = state.y - target.y
    # 滑模面定义：s = e_dot + lambda * e
    # 这里 e_dot 就是速度，因为目标点是静止的
    sx = state.vx + params.lambda_gain * ex
    sy = state.vy + params.lambda_gain * ey
    # 返回两个方向上的滑模面值
    return sx, sy


def compute_control(
    state: SmcState, target: SmcTarget, params: SmcParams
) -> SmcOutput:
    # 先计算当前位置相对目标点的误差
    ex = state.x - target.x
    ey = state.y - target.y
    # 再计算滑模面，用来判断系统是否偏离了理想收敛轨道
    sx, sy = compute_sliding_surface(state, target, params)

    # Equivalent control with explicit damping compensation:
    #   From s_dot = 0 on the model m*a = u - b*v:
    #     s_dot = a + lambda*v = (u - b*v)/m + lambda*v = 0
    #     => u_eq = b*v - m*lambda*v = -(m*lambda - b)*v
    # 把等效控制的共同系数单独提出来，便于理解
    coeff = params.mass * params.lambda_gain - params.damping
    # 等效控制负责让系统沿着理想滑模动态前进
    ux_eq = -coeff * state.vx
    uy_eq = -coeff * state.vy

    # Switching control
    # use_saturation=True 时使用平滑边界层，课堂演示更稳定
    if params.use_saturation:
        # ux_sw/uy_sw 负责把系统“拉回”滑模面
        ux_sw = -params.k_gain * saturation(sx, params.phi)
        uy_sw = -params.k_gain * saturation(sy, params.phi)
    else:
        # sign 是经典滑模形式，纠偏更硬，但更容易抖振
        ux_sw = -params.k_gain * sign(sx)
        uy_sw = -params.k_gain * sign(sy)

    # 总控制 = 等效控制 + 切换控制
    return SmcOutput(
        ux=ux_eq + ux_sw,
        uy=uy_eq + uy_sw,
        sx=sx,
        sy=sy,
        ex=ex,
        ey=ey,
    )


def step_dynamics(
    state: SmcState,
    control: SmcOutput,
    dt: float,
    params: SmcParams,
    external_force: tuple[float, float] = (0.0, 0.0),
) -> SmcState:
    # 根据动力学方程计算 x 方向加速度：
    # m * a = 控制输入 + 外部扰动 - 阻尼
    ax = (control.ux + external_force[0] - params.damping * state.vx) / params.mass
    # y 方向同理
    ay = (control.uy + external_force[1] - params.damping * state.vy) / params.mass
    # 用欧拉积分先更新速度
    vx_new = state.vx + ax * dt
    vy_new = state.vy + ay * dt
    # 再用更新后的速度推进位置
    x_new = state.x + vx_new * dt
    y_new = state.y + vy_new * dt
    # 返回下一时刻的新状态，不直接修改原状态
    return SmcState(x=x_new, y=y_new, vx=vx_new, vy=vy_new)


if __name__ == "__main__":
    state = SmcState(x=8.0, y=6.0, vx=0.0, vy=0.0)
    target = SmcTarget(x=0.0, y=0.0)
    params = SmcParams()
    dt = 0.02

    for i in range(500):
        ctrl = compute_control(state, target, params)
        state = step_dynamics(state, ctrl, dt, params)
        if i % 50 == 0:
            err = math.sqrt(ctrl.ex**2 + ctrl.ey**2)
            s_mag = math.sqrt(ctrl.sx**2 + ctrl.sy**2)
            print(
                f"t={i*dt:.2f}  pos=({state.x:.3f},{state.y:.3f})  "
                f"|e|={err:.3f}  |s|={s_mag:.3f}  u=({ctrl.ux:.2f},{ctrl.uy:.2f})"
            )
