# px4-preflight-demo

Sample consumer of [`harelab-ucsc/px4-preflight-runner`][runner]. Demonstrates
how a project that talks to PX4 over uXRCE-DDS can use the runner to verify
its offboard control loop in CI without standing up its own simulator
infrastructure.

## What gets tested

The matrix is declared in [`.preflight/spec.yaml`](.preflight/spec.yaml) and
produces four jobs per push/PR:

| Case | Type | What it proves |
|---|---|---|
| `ros-unit-tests` | `ros_unit` | The ROS package builds and unit tests pass against the version-matched `px4_msgs`. |
| `px4-build` | `px4_build` | PX4-Autopilot @ `v1.16.0` compiles cleanly with the runner toolchain. |
| `sim-basic` | `sim` | PX4 SIH boots, the DDS bridge connects, and `vehicle_status` reaches ROS. |
| `offboard-waypoint-mission` | `sim` | The offboard commander arms the vehicle, switches to OFFBOARD, and flies the four-waypoint loop. |

## Repo layout

```
.preflight/
  spec.yaml                  # declarative test matrix
  tests/waypoint_check.py    # post-run assertion script

src/px4_preflight_demo/
  px4_preflight_demo/        # python package
  launch/                    # ros2 launch files
  test/                      # unit tests

.github/workflows/preflight.yml   # one-job-per-case fan-out
```

## Replaying flights

Every `sim` case uploads a `.ulg` and a `ros2 bag` (mcap) artifact. Download
the artifact for the case you care about and replay it in
[Hawkeye][hawkeye]:

```bash
hawkeye --replay flight.ulg
```

[runner]: https://github.com/harelab-ucsc/px4-preflight-runner
[hawkeye]: https://github.com/PX4/Hawkeye
