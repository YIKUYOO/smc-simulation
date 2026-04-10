# `smc_demo`

用于课堂演示二维滑模控制（SMC）在 ROS 2 + RViz2 中的收敛过程。

## 环境要求

- Ubuntu
- ROS 2 Humble 或更高版本
- `colcon`
- `rviz2`

## 构建

```bash
cd ~/smc
source /opt/ros/humble/setup.bash
colcon build --packages-select smc_demo
source install/setup.bash
```

如果目标机器使用的不是 Humble，请把上面的 `humble` 替换成对应发行版。

## 课堂演示启动

```bash
cd ~/smc
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch smc_demo smc_demo.launch.py
```

默认 launch 现在会同时展示 `SMC` 和 `No control` 两条运动：

- `use_saturation:=true`
- `lambda_gain:=4.0`
- `k_gain:=8.0`
- `phi:=0.2`
- `enable_comparison:=true`
- `disturbance_enabled:=true`
- `disturbance_start:=1.5`
- `disturbance_duration:=0.35`
- `disturbance_fx:=3.0`
- `disturbance_fy:=-4.0`

预期现象：

- 青色球体是 `SMC`，灰色方块是 `No control`
- 两者从 `(8, 6)` 同时出发并在 `1.5s` 时受到同一侧向强扰动
- `SMC` 会被打偏后重新拉回目标，`No control` 会继续漂离目标
- RViz 中持续显示双轨迹、误差线、控制箭头、滑模面箭头和扰动箭头
- 约 5 秒内 `SMC` 进入目标邻域并输出收敛日志

## 非 GUI 验证

```bash
cd ~/smc/smc_demo
./scripts/verify_smc_demo.sh
```

这个脚本会自动执行：

- 构建 `smc_demo`
- 以 `start_rviz:=false` 启动 launch
- 检查 `/smc/state` 和 `/tf` 是否能收到消息
- 检查退出时是否出现 traceback

## 常用参数

```bash
ros2 launch smc_demo smc_demo.launch.py initial_x:=10.0 initial_y:=-4.0
ros2 launch smc_demo smc_demo.launch.py target_x:=2.0 target_y:=1.0
ros2 launch smc_demo smc_demo.launch.py start_rviz:=false
ros2 launch smc_demo smc_demo.launch.py enable_comparison:=false disturbance_enabled:=false
```
