# ros2_shadow

Runs a candidate node beside a production node on the same live inputs, compares
their outputs, and reports where they disagree. Only production drives the robot.

## Requirements

* ROS 2 Jazzy
* Python 3.10 or later

## Build

```console
$ colcon build --packages-select ros2_shadow
$ source install/setup.bash
```

## Usage

Two processes. The bridge isolates the candidate, the comparison measures it.

```console
$ ros2 run ros2_shadow shadow config.yaml --bridge
$ ros2 run ros2_shadow shadow config.yaml
```

The candidate itself is launched in the candidate domain:

```console
$ ROS_DOMAIN_ID=42 ros2 run my_package my_candidate_node
```

## Isolation

A candidate sharing a DDS domain with production can always reach hardware.
Namespace remapping is cooperative: a node can build a topic name at runtime, or
use a service or an action, and remapping does not apply. Watching the graph and
reporting a breach happens after the message is already on the wire.

So the candidate runs in its own `ROS_DOMAIN_ID`, where the hardware topics do
not exist. The bridge is the only route between the two domains and carries
exactly the topics named in the config:

```yaml
isolation:
  production_domain: 0
  candidate_domain: 42
  inputs:
    - topic: /scan
      type: sensor_msgs/msg/LaserScan
    - topic: /odom
      type: nav_msgs/msg/Odometry
  outputs:
    - topic: /planner/cmd_vel
      type: geometry_msgs/msg/Twist
      republish_as: /shadow/planner/cmd_vel
```

The bridge prints what crosses before it starts, and config loading refuses any
`republish_as` that matches a forbidden topic.

This covers ROS-level access. A candidate that opens a serial port or writes to
a device node is outside its reach; use containers or user permissions there.

## Configuration

```yaml
production:
  topic: /planner/cmd_vel

shadow:
  topic: /shadow/planner/cmd_vel
  namespace: /shadow

comparison:
  type: geometry_msgs/msg/Twist
  synchronization:
    tolerance_ms: 20
  metrics:
    - name: linear_error
      warning: 0.05
      critical: 0.20
    - name: direction_reversal
      critical: 1.0

safety:
  forbidden_topics:
    - /cmd_vel
    - /joint_commands
    - /hardware/*
```

## Matching

The two nodes see the same inputs but finish at different times, so the newest
output from each is not a valid pair. Outputs are buffered and paired within
`tolerance_ms`.

Pairing uses `header.stamp` when the message carries one and it is populated,
and arrival time otherwise. Which clock was used is reported, because a pairing
resting on arrival time is a weaker claim than one resting on the stamp of the
input that produced both outputs. A zero stamp is treated as unpopulated.

Outputs with no partner are held for a grace period before being counted as
unmatched, so a late candidate is not recorded as a dropped one.

## Metrics

| Message type | Metrics |
| --- | --- |
| `geometry_msgs/msg/Twist` | `linear_error`, `angular_error`, `linear_x_error`, `direction_reversal` |
| `geometry_msgs/msg/PoseStamped` | `translation_error`, `rotation_error` |
| `trajectory_msgs/msg/JointTrajectory` | `joint_position_rmse`, `max_joint_delta`, `endpoint_delta`, `duration_delta` |

`direction_reversal` reports 1.0 when the two commands drive opposite ways along
x. As a magnitude, a candidate asking for -0.3 where production asks for +0.4 is
0.7 of error and indistinguishable from drift. As a behaviour it is the robot
going the wrong way.

Joint trajectories are aligned by joint name, since two planners need not agree
on ordering.

## Output

```
ros2_shadow  /shadow/planner/cmd_vel vs /planner/cmd_vel
  matched 349   unmatched prod 0   unmatched cand 0   pending 0
  paired on receive_time

  metric                        mean       p95       max
  angular_error               0.0249    0.0300    0.0300
  direction_reversal          0.1146    1.0000    1.0000
  linear_error                0.1184    0.6575    0.6727

  warnings 250   critical 40
```

Divergence is also published as `diagnostic_msgs/DiagnosticArray` on
`/shadow/divergence`. The process exits non-zero if any critical divergence was
seen or if the safety scanner suspended the run.

A sustained divergence produces one event per message. Repeats are counted and
folded into a line every two seconds rather than logged individually.

## Demo

Without a simulator, two publishers stand in for the pair:

```console
$ ros2 run ros2_shadow shadow_demo_pair
$ ros2 run ros2_shadow shadow config/demo_twist.yaml
```

The candidate tracks production, then drifts, then briefly commands the opposite
direction.

## Development

```console
$ python3 -m pytest src/ros2_shadow/test -q
```
