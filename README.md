# realsense_camera_rbnx

Robonix package wrapping the **Intel RealSense D435i** RGBD camera.
Exposes the camera's RGB + aligned-depth streams under generic
`primitive/camera/*` contracts so that mapping, scene, and any vision
skill can resolve topic names through atlas (no hardcoded
`/camera_435i/...` paths on the consumer side).

## Capability surface

The `mode` is the abstract communication pattern declared in the
contract TOML (rpc / topic_in / topic_out). The `transport` column
records how THIS package realises it on the wire — both columns matter
because the same mode can ride different middleware (e.g. an `rpc`
mode can be a gRPC method or an MCP tool call).

| Contract                                  | Mode      | Transport | Source / handler                                          |
| ----------------------------------------- | --------- | --------- | --------------------------------------------------------- |
| `robonix/primitive/camera/driver`         | rpc       | gRPC      | `Driver(CMD_INIT, config_json)` — lifecycle gate          |
| `robonix/primitive/camera/rgb`            | topic_out | ROS 2     | `/<cam>/color/image_raw` (sensor_msgs/Image)              |
| `robonix/primitive/camera/depth`          | topic_out | ROS 2     | `/<cam>/aligned_depth_to_color/image_raw`                 |
| `robonix/primitive/camera/extrinsics`     | topic_out | ROS 2     | latched TransformStamped (TODO: republish from /tf_static) |
| `robonix/primitive/camera/snapshot`       | rpc       | MCP       | one-shot RGB capture (TODO)                               |
| `robonix/primitive/camera/depth_snapshot` | rpc       | MCP       | one-shot depth capture (TODO)                             |

The D435i's internal IMU is **deliberately not** registered under
`primitive/imu/imu` — the Ranger Mini's MID-360 IMU is canonical for
that contract (better noise, co-located with the lidar for SLAM).
The camera IMU is still published to `/<cam>/imu` for anyone who wants
it directly.

## Driver-init lifecycle

`start.sh` only brings up the atlas bridge process. The bridge:

1. opens a gRPC server on port `REALSENSE_DRIVER_PORT` (default 50232),
2. registers the capability and declares **only** the
   `primitive/camera/driver` interface on atlas,
3. blocks on heartbeat awaiting `Driver(CMD_INIT, config_json)`.

When `rbnx boot` invokes Init it passes the manifest's `config:` block
as JSON. The handler:

1. parses config (camera name, RGB/depth profiles, align_depth, IMU on/off),
2. spawns `ros2 launch realsense2_camera rs_launch.py …`,
3. waits for the first frame on the configured RGB topic,
4. declares `primitive/camera/{rgb, depth}` on atlas,
5. returns `ok=true` so boot proceeds.

This means atlas only ever advertises endpoints we've confirmed are
publishing — no consumer ever connects to a silent topic.

## Layout

```
realsense_camera_rbnx/
├── package_manifest.yaml
├── realsense_camera/
│   └── atlas_bridge.py           driver gRPC + lazy Init
├── scripts/
│   ├── build.sh                  colcon build vendored src + rbnx codegen
│   └── start.sh                  source ROS, exec atlas_bridge
├── src/
│   └── realsense-ros/            VENDORED IntelRealSense/realsense-ros
└── .gitignore
```

## Config (passed via `Driver(CMD_INIT, config_json)`)

```json
{
  "camera_name":        "camera_435i",
  "rgb_profile":        "640x480x30",
  "depth_profile":      "848x480x30",
  "align_depth":        true,
  "enable_imu":         true,
  "enable_sync":        true,
  "sentinel_timeout_s": 30.0
}
```

`rgb_topic` / `depth_topic` can be overridden directly if your launch
uses a non-default namespace.

## Build / run standalone

```bash
bash scripts/build.sh
bash scripts/start.sh        # registers driver iface, waits for INIT
```

To drive Init manually (without rbnx boot):

```bash
# from any robonix gRPC client, call PrimitiveCameraDriver.Driver
# with command=0 and config_json='{}' against 127.0.0.1:50232
```

## License

This package: Apache-2.0 (matches realsense-ros upstream).
