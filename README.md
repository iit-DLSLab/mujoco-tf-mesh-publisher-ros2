  <div style="display: flex; justify-content: space-around;">
    <img src="https://img.shields.io/badge/Mujoco%20-v3.7.0-blue" alt="Mujoco v3.7.0" style="margin-bottom: 1px;">
  </div>


## Overwiew

A script to show tf and mesh from mujoco models, similar to robot_state_publisher.


## Installation

This repo uses [pixi](https://pixi.sh) to provide a self-contained ROS2 environment (via [RoboStack](https://robostack.github.io/)), so no system-wide ROS2 install is required.

1. Install pixi (see the [official instructions](https://pixi.sh/latest/#installation)).

2. Clone this repo together with its submodule:



3. Install the environment. Two ROS2 distros are available as pixi environments, `humble` and `lyrical` — pick one:

```bash
pixi shell -e humble
# or
pixi shell -e lyrical
```


You can now run the commands below from inside the pixi shell.


## ROS2 XML State Publisher (MuJoCo XML)


```bash
python3 tf_mesh_publisher_ros2.py \
  --xml sim2real-robot-identification/robot_model/go2/scene_flat.xml \
  --world-frame world \
  --joint-states-topic /joint_states \
  --publish-rate 100
```

The script resolves `<include file="..."/>` and supports `hinge` and `slide`
joints from the XMLs in this repository. Missing joints are published at zero
position.

You must specify `--joint-states-topic`, `--blind-state-topic`, or both. Each
flag accepts one or more topics, which get merged into the same joint set —
useful when e.g. a quadruped HAL and an arm HAL publish separate topics for
the same XML:

```bash
python3 tf_mesh_publisher_ros2.py \
  --xml sim2real-robot-identification/robot_model/go2/scene_flat.xml \
  --joint-states-topic /arm/joint_states \
  --blind-state-topic /quadruped/blind_state
```

`--blind-state-topic` subscribes to `dls2_interface/msg/BlindState` (requires
the `dls2_interface` package built in `ros2_ws/`).

As robot_state_publisher does, the TF is stamped with the `JointState` header
stamp (unstamped messages and `BlindState`, whose `DlsHeader` is not a ROS
header, get the arrival time). Named `<site>` elements (e.g. `imu`, a lidar) are published as
fixed frames on their body.

When the robot is part of a larger TF tree:

- `--root-frame base_link` names the top-level body frame `base_link` instead
  of its body name.
- `--no-world-transform` does not publish `world -> <top-level body>`, e.g.
  when a state estimator already publishes it.
- ROS arguments go after `--ros-args`, e.g. `--ros-args -p use_sim_time:=true`
  to replay a bag with `ros2 bag play --clock`.
- If `ROS_LOCALHOST_ONLY` is unset, the script sets it to `1`: set it to the
  value the other nodes use (e.g. `ROS_LOCALHOST_ONLY=0`), or they will not see
  each other.

It also publishes mesh visualization (XML-only, no URDF needed) as
`visualization_msgs/MarkerArray` on `/xml_robot_meshes`.



### RViz notes

- Set `Fixed Frame` to `world` (or to your `--world-frame` value).
- Add a `TF` display to see frames.
- Check that TF exists with:

```bash
ros2 topic echo /tf --once
ros2 topic echo /tf_static --once
```

You can also open a ready-to-use RViz config from this repo:

```bash
rviz2 -d xml_tf_frames.rviz
```

The RViz config already includes:

- `TF` display for frames
- `MarkerArray` display for XML meshes (`/xml_robot_meshes`)

Meshes are published as `MarkerArray` and are visible in RViz via the `XML Meshes`
display. RViz `RobotModel` still requires URDF (`/robot_description`).


## How to contribute

PRs are very welcome (search for **TODO** in the issue, or add what you like)!


## Maintainer

This repository is maintained by [Giulio Turrisi](https://github.com/giulioturrisi).
