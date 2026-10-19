# slithers_ros2

ROS 2 wrapper for the [**slithers**](../slithers_lib) library — a Lie theory-based IK
planner for mobile manipulators.

This package contains **no algorithm code**. All the kinematics, IK and the planning loop
are in `libslithers` (`slithers::Planner`). This package handles ROS parameters, topics
and launching. If you change the algorithm in `slithers_lib`, the simulator code and this
node both pick up the change.

```
slithers_ros2/
├── src/slithers_node.cpp     the node
├── launch/slithers.launch.py launch file with all parameters
├── CMakeLists.txt            links slithers::slithers
└── package.xml               depends on slithers, rclcpp, geometry_msgs, std_msgs
```

Tested with ROS 2 **Jazzy**. It should also work on Humble: it only uses rclcpp and
standard messages.

---

## Building

Both packages go in the same colcon workspace:

```bash
mkdir -p ~/slithers_ws/src
cd ~/slithers_ws/src
ln -s ~/slithers_lib .      # or copy / git clone
ln -s ~/slithers_ros2 .

cd ~/slithers_ws
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

colcon builds `slithers` (the library) first, because `slithers_ros2` depends on it.

**Use `-DCMAKE_BUILD_TYPE=Release`.** Without it colcon builds unoptimised code and the IK
runs many times slower (more than 12 s per trial instead of well under a second for `holo`).

---

## Running

```bash
ros2 launch slithers_ros2 slithers.launch.py                       # holo, 3 paths
ros2 launch slithers_ros2 slithers.launch.py robot_type:=husky
ros2 launch slithers_ros2 slithers.launch.py num_paths:=4

# or directly, overriding any parameter:
ros2 run slithers_ros2 slithers_node --ros-args -p robot_type:=husky -p dt:=0.05
```

Watch the output:

```bash
ros2 topic echo /slithers/joint_velocities
ros2 topic echo /slithers/tool_pose
```

In RViz2, set the fixed frame to `world` and add two **Pose** displays, one for
`/slithers/tool_pose` and one for `/slithers/pose_goal`, to compare the planned tool path
with the reference.

---

## How the node works

1. **Startup.** The node declares and reads its parameters, creates the three publishers,
   and starts a one-shot 100 ms timer so the publishers are advertised before any data
   goes out.
2. **Setup.** When the timer fires, the node loads the robot model (`husky_ur5e()` or
   `husky_ur5e_holo()`), builds a `slithers::PlannerConfig` from the parameters, creates a
   `slithers::Planner`, and generates the four reference paths with the MATLAB `main.m`
   settings:

   | # | Path | a | b |
   |---|---|---|---|
   | 0 | helix | 0.5 | 0.05 |
   | 1 | sine | – | 0.3 |
   | 2 | horizontal helix | 0.3 | 0.15 |
   | 3 | spiral sine | 2.0 | 0.3 |

3. **Trials.** For trial `k = 0 … num_paths-1` it takes path `k % 4`, calls
   `planner.reset()`, and for each pose on the path calls `planner.step(goal)` and
   publishes the result on the three topics.
4. **Afterwards.** The node logs `All N trials complete` and keeps spinning; it doesn't exit.

The `Planner` step itself (base advance → IK solve → tool pose → state roll-forward) is
documented in `slithers_lib/README.md`.

> **Timing:** the node publishes each step as soon as it has been computed. It does **not**
> wait `dt` between steps. That's fine for visualising or recording a planned trajectory,
> but if you use the commands to drive a robot or simulator in a closed loop, step the
> planner from a timer at `dt` instead.

---

## Topics

All published, queue depth 10, `frame_id = "world"`.

| Topic | Type | Content |
|---|---|---|
| `/slithers/tool_pose` | `geometry_msgs/msg/PoseStamped` | end-effector pose the planner reached this step |
| `/slithers/pose_goal` | `geometry_msgs/msg/PoseStamped` | reference pose for this step |
| `/slithers/joint_velocities` | `std_msgs/msg/Float64MultiArray` | planner command `dx` (n values) |

`joint_velocities` layout follows the robot's screw order, with the base first:

| robot_type | n | entries |
|---|---|---|
| `husky` | 8 | `[yaw_rate, forward_speed, ur5e_j1 … ur5e_j6]` |
| `holo` | 9 | `[yaw_rate, vx, vy, ur5e_j1 … ur5e_j6]` |

Base entries are velocities (rad/s, m/s). Arm entries are joint angles relative to the UR5e
zero configuration, because that's how the library's `step_forward` uses them. The topic
keeps the name `joint_velocities` from the MATLAB port. See *Using SLITHERS with a
simulator* in the library README before using these values as actuator commands.

There are no subscribers yet: goals come from the built-in paths.

---

## Parameters

Declared on `slithers_node`; the defaults match MATLAB `main.m`.

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `robot_type` | string | `"holo"` | `"husky"` → `husky_ur5e()` (8 DOF); anything else → `husky_ur5e_holo()` (9 DOF) |
| `dt` | double | `0.1` | planner time step [s] |
| `time` | double | `20.0` | duration of each reference path [s] (path has `time/dt + 1` poses) |
| `num_paths` | int | `3` | number of trials; cycles through the 4 paths |
| `lambda_e` | double[6] | `[25]*6` | twist-error weights `[ωx ωy ωz ρx ρy ρz]`; wrong length → defaults + warning |
| `lambda_j` | double | `0.001` | jerk weight (reserved, unused) |
| `lambda_v_husky` | double[8] | `[1, 1, .25, .25, .1, .1, .1, .1]` | per-joint weights, used when `robot_type=husky` |
| `lambda_v_holo` | double[9] | `[1, 1, 1, .25, .25, .1, .1, .1, .1]` | per-joint weights, used otherwise |

If the selected `lambda_v_*` has the wrong length, it is padded with 0.1 or truncated,
and the node logs a warning.

Launch arguments (`slithers.launch.py`): `robot_type` (`holo` | `husky`) and `num_paths`.
The other parameters are set in the launch file with the defaults above. Edit the file, or
use `ros2 run … --ros-args -p name:=value`, to change them.

---

## Extending

To use SLITHERS in your own node:

```cmake
find_package(slithers REQUIRED)
target_link_libraries(my_node slithers::slithers)
```
```xml
<depend>slithers</depend>
```
```cpp
#include "slithers/slithers.hpp"
slithers::Planner planner(slithers::husky_ur5e_holo());
auto r = planner.step(goal);        // r.joint_velocities, r.tool_pose, r.base_pose
```

Natural next steps for this package:
- subscribe to a goal pose topic (e.g. `geometry_msgs/PoseStamped`) instead of the built-in paths,
- step the planner from a `dt` timer for closed-loop control,
- subscribe to odometry / `joint_states` and pass them to `planner.set_base_pose()` / `set_state()`,
- publish `sensor_msgs/JointState` and a `nav_msgs/Path` for RViz.

## License

MIT — Siddharth Kumar (sks4zk@virginia.edu)
