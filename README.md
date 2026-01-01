# robomesh

robomesh is a small Rust library that loads URDF models, applies joint trajectories, and renders simple 2D top-view PNGs. The crate exposes PyO3 bindings so you can call it directly from Python for quick visualization or debugging workflows.

## Features
- URDF loading and forward kinematics via `urdf-rs` and the `k` crate
- 2D top-view link rendering on the XZ plane using `plotters`
- Visual geometry support, including mesh-based links (OBJ or STL) rendered as oriented projected bounding boxes
- Python-facing `RoboRenderer` class implemented with PyO3
- Joint targets accepted as Python mappings, JSON strings, or CSV trajectory files
- Single-frame PNG rendering or multi-frame trajectory export to a directory

## Building
If you want to build the Rust crate only, run:

```bash
cargo build --release
```

To use the library from Python, install a PyO3 build tool such as `maturin` or `setuptools-rust` and build a wheel in your environment. Network access to crates.io is required to download dependencies.

## Python usage
```python
import json
from robomesh import RoboRenderer

renderer = RoboRenderer("path/to/robot.urdf")
print("joint order:", renderer.joint_order())

# Render a single frame
actions = {"joint1": 0.2, "joint2": -0.1, "joint3": 0.0}
renderer.render_frame(actions, "frame.png")

# Render multiple frames (trajectory)
trajectory = [
    {"joint1": 0.0, "joint2": 0.0, "joint3": 0.0},
    {"joint1": 0.1, "joint2": -0.2, "joint3": 0.0},
]
renderer.render_trajectory(trajectory, "frames")

# JSON strings are also accepted
json_traj = json.dumps([
    {"joint1": 0.0, "joint2": 0.0},
    {"joint1": 0.3, "joint2": 0.1},
])
renderer.render_trajectory(json_traj, "frames_from_json")

# Load a CSV trajectory (headers must match joint names; optional "time" column is ignored for rendering)
renderer.render_trajectory_csv("trajectory.csv", "frames_from_csv")
```

Mesh files referenced by the URDF are resolved relative to the URDF file location when using `RoboRenderer(path)`. When loading
from a string (`from_urdf_string`), only absolute mesh paths will resolve correctly.

### CSV format
- The first row contains headers for every joint name (e.g., `joint1,joint2,joint3`).
- Values in each subsequent row are joint angles (radians) for that frame.
- An optional `time` column is allowed; it is read for completeness but ignored during rendering.
- All joints must be present in every row; missing or non-numeric values will raise an error.

### Output
- Each frame is saved as an 800x800 PNG showing link segments projected onto the XZ plane.
- Mesh visuals (OBJ and STL) are loaded and projected as oriented bounding rectangles; if a mesh file cannot be found or parsed, rendering will report an error with the missing path.
- All joints must be provided. Use `joint_order()` to verify the expected joint list before rendering.

## License
See [LICENSE](LICENSE).
