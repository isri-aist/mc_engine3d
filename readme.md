# mc_engine3d

`mc_rtc` / [Engine3D](https://github.com/PerceptionRobotique/Engine3D) interface.

It renders a static scene (point cloud or mesh) from one or more virtual
cameras rigidly attached to frames of the controlled robot, and publishes each
camera stream on its own `image_transport` topic while an `mc_rtc` controller
runs.

## Dependencies

* `mc_rtc`
* `Engine3D` (needs the *secondary camera capture* support, i.e.
  `Camera::getFrame()` — see `ThomasDuvinage/Engine3D`)
* `OpenCV`, `cv_bridge`, `image_transport`, `rclcpp`
* `Boost.program_options`

## Build

```bash
mkdir build && cd build
cmake ..
make
```

## Run

```bash
./build/src/mc_engine3d -f etc/mc_engine3d.yaml
```

`-f/--conf` points to an `mc_rtc` configuration that also contains an `Engine3d`
section (see below). The provided `etc/mc_engine3d.yaml` runs the `UR5e` with the
`Posture` controller and Engine3D's `Suzanne.obj` sample as a self-contained
smoke test.

## Configuration

Everything specific to this interface lives in the `Engine3d` section of the
`mc_rtc` configuration:

```yaml
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

* **Control loop** (`run()`) steps the `mc_rtc` controller at its timestep and,
  each step, hands the current pose of every camera frame to the render thread
  through a small mutex-guarded snapshot. It never touches the engine.
* **Render thread** (`renderLoop()`) owns all Engine3D calls. Each iteration it
  reads the latest pose snapshot, moves the cameras, calls
  `engine_.takePicture()` (renders the scene once for **all** cameras and caches
  each framebuffer), then reads each frame back with `Camera::getFrame()` and
  publishes it as a `bgr8` image stamped with the control time and the camera
  frame. It is paced by `render_rate`.

This keeps a heavy render (large point clouds) off the control step. Effective
rates for both loops are logged every 5 s.

`MainCamera` is the camera already owned by the engine, configured in place, so
it stays part of the render loop; the entries of `Cameras` are created and
handed over to the engine via `Engine3D::addCamera`. The engine and all cameras
are given the render thread's affinity (`moveToThread`) before it starts, since
`Engine3D::DIRECT` only dispatches `takePicture()` on its owning thread.

## Remaining work

* Expose a scene-model pose (models are currently loaded at their own origin)
