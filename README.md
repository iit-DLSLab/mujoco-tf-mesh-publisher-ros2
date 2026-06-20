  <div style="display: flex; justify-content: space-around;">
    <img src="https://img.shields.io/badge/Mujoco%20-v3.7.0-blue" alt="Mujoco v3.7.0" style="margin-bottom: 1px;">
  </div>


## Overwiew

A script to show tf and mesh from mujoco models, similar to robot_state_publisher.


## ROS2 XML State Publisher (MuJoCo XML)


```bash
python3 other/xml_robot_state_publisher_ros2.py \
  --xml robot_model/go2/scene_flat.xml \
  --world-frame world \
  --joint-states-topic /joint_states \
  --publish-rate 100
```

If your ROS graph uses simulation clock, add:

```bash
python3 other/xml_robot_state_publisher_ros2.py \
  --xml robot_model/go2/scene_flat.xml \
  --use-sim-time
```

The script resolves `<include file="..."/>` and supports `hinge` and `slide`
joints from the XMLs in this repository. By default, missing joints in
`/joint_states` are published at zero position.

It also publishes mesh visualization (XML-only, no URDF needed) as
`visualization_msgs/MarkerArray` on `/xml_robot_meshes`.

Example with manipulator:

```bash
python3 other/xml_robot_state_publisher_ros2.py --xml robot_model/z1/scene_flat.xml
```

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
rviz2 -d other/xml_tf_frames.rviz
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
