# mc_engine3d

`mc_rtc` global plugin wrapping [Engine3D](https://github.com/PerceptionRobotique/Engine3D).

**Only works with the following PR : https://github.com/PerceptionRobotique/Engine3D/pull/7**

It renders a static scene (point cloud or mesh) from one or more virtual
cameras rigidly attached to frames of the controlled robot, and publishes each
camera stream on its own `image_transport` topic while an `mc_rtc` controller
runs. Because it is a plugin, it loads alongside whatever interface actually
drives the robot — `mc_rtc_ticker`, `mc_mujoco`, a real-robot interface, ...

## Dependencies

* `mc_rtc`
* `Engine3D` (needs the *secondary camera capture* support, i.e.
  `Camera::getFrame()` — see `ThomasDuvinage/Engine3D`)
* `OpenCV`, `cv_bridge`, `image_transport`, `rclcpp`

## Build

```bash
mkdir build && cd build
cmake ..
make
make install
```

`make install` places `Engine3d.so` in the `mc_rtc` plugin directory and the
default `Engine3d.yaml` next to it.

## Run

Add the plugin to any `mc_rtc` configuration and configure it through a matching
`Engine3d` section:

```bash
mc_rtc_ticker -f etc/mc_engine3d.yaml
```

The provided `etc/mc_engine3d.yaml` runs the `UR5e` with the `Posture` controller
as a self-contained smoke test.

Engine3D (`DIRECT`) needs a `QGuiApplication` for its offscreen GL context. If the
host interface has no Qt application, the plugin creates one on the `offscreen`
platform, so it also works headless.

## Configuration

Everything specific to the plugin lives in the `Engine3d` section, either in the
`mc_rtc` configuration or in the installed `Engine3d.yaml`. Both use the same
`Engine3d:` parent node; when the `mc_rtc` configuration has an `Engine3d`
section it replaces the plugin's own `Engine3d.yaml`.

```yaml
Plugins: [Engine3d]

Engine3d:
  mesh_model: "/path/to/scene.obj"   # .pts / .bin / .obj / .oct
  render_rate: 30                    # render/publish thread cap in Hz, 0 = uncapped

  MainCamera:                        # the engine main camera, configured in place
    frame: "tool0"
    size: [250, 250]
    projection_type: 0
    intrinsics: [256.2, 195.2, 244.6, 244.5]

  Cameras:                           # any number of extra cameras
    - name: "Camera1"
      frame: "tool0"
      size: [320, 240]
      projection_type: 0
      intrinsics: [256.0, 195.0, 160.0, 120.0]
```

### Camera options

| key                | required | description                                             |
|--------------------|----------|---------------------------------------------------------|
| `name`             | for `Cameras` | camera name, also used as the published topic name |
| `frame`            | yes      | robot frame the camera is rigidly attached to           |
| `size`             | yes      | `[width, height]` in pixels                             |
| `projection_type`  | yes      | `0` perspective, `1` orthographic, `2` equirectangular, `3` custom |
| `intrinsics`       | yes      | `[fx, fy, cx, cy]`                                       |
| `near_plane`       | no       | near clipping plane                                     |
| `far_plane`        | no       | far clipping plane                                      |
| `samples`          | no       | MSAA samples                                            |
| `background_color` | no       | `[r, g, b]` in `[0, 1]`, default `[0.5, 0.5, 0.5]`      |
| `save_path`        | no       | when set, every frame is also written to `<save_path>/<name>.png` |

## How it works

Two threads:

* **Control step** — the plugin's `after()` hook runs after each
  `MCGlobalController::run()` and hands the current pose of every camera frame to
  the render thread through a small mutex-guarded snapshot. It never touches the
  engine.
* **Render thread** (`renderLoop()`) owns all Engine3D calls. Each iteration it
  reads the latest pose snapshot, moves the cameras, calls
  `engine_->takePicture()` (renders the scene once for **all** cameras and caches
  each framebuffer), then reads each frame back with `Camera::getFrame()` and
  publishes it as a `bgr8` image stamped with the control time and the camera
  frame. It is paced by `render_rate`.

This keeps a heavy render (large point clouds) off the control step. The
effective render rate is logged every 5 s.

`MainCamera` is the camera already owned by the engine, configured in place, so
it stays part of the render loop; the entries of `Cameras` are created and
handed over to the engine via `Engine3D::addCamera`. The engine and all cameras
are given the render thread's affinity (`moveToThread`) before it starts, since
`Engine3D::DIRECT` only dispatches `takePicture()` on its owning thread.

## Remaining work

* Expose a scene-model pose (models are currently loaded at their own origin)
