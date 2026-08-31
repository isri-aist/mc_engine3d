# mc_engine3d

mc_rtc / Engine3d interface

## Configuration

The `Engine3d` section of the mc_rtc configuration describes the scene and the
cameras (see [etc/mc_engine3d.yaml](etc/mc_engine3d.yaml)):

* `mesh_model`: point cloud / mesh loaded in the scene
* `MainCamera`: the engine main camera, configured in place
* `Cameras`: additional cameras, each streamed on its own topic

Each camera is attached to a robot frame (`frame`) and published on an
`image_transport` topic named after the camera. Every camera is rendered on
each `engine.takePicture()` call and its framebuffer retrieved through
`Camera::getFrame()`.

## Remaining work

* Move capture / publishing to a dedicated thread
