# robomesh

robomesh is a small Rust library that loads URDF models, applies joint trajectories, and renders simple 2D top-view PNGs. The crate can expose PyO3 bindings (behind an optional Cargo feature) so you can call it directly from Python for quick visualization or debugging workflows.

## Features
- URDF loading and forward kinematics via `urdf-rs` and the `k` crate
- 2D top-view link rendering on the XZ plane using `plotters`
- Visual geometry support, including mesh-based links (OBJ or STL) rendered as oriented projected bounding boxes
- Python-facing `RoboRenderer` class implemented with PyO3
- Joint targets accepted as Python mappings, JSON strings, or CSV trajectory files
- Single-frame PNG rendering or multi-frame trajectory export to a directory

## Building (Rust)
If you want to build the Rust crate only, run:

```bash
cargo build --release
```

Network access to crates.io is required to download dependencies.

To include the Python bindings in the build (e.g., when targeting a wheel), enable the `python` feature so that PyO3 is pulled in:

```bash
cargo build --release --features python
```

## Python bindings
Python packaging, virtual environments, and tooling are kept in a dedicated `python/` directory and are managed with [uv](https://github.com/astral-sh/uv) to keep the Rust workspace clean. See [`python/README.md`](python/README.md) for uv-based build and usage steps. The PyO3 dependency (v0.22) targets the stable CPython ABI for versions 3.8 through 3.13 and is only pulled in when you build with the `python` Cargo feature (as done by `uv run maturin ... --features python`).

## Python usage
```python
import json
from pathlib import Path
from robomesh import RoboRenderer

repo_root = Path(__file__).resolve().parent
sample_urdf = repo_root / "examples" / "two_link.urdf"
sample_csv = repo_root / "examples" / "wave.csv"

renderer = RoboRenderer(sample_urdf.as_posix())
print("joint order:", renderer.joint_order())

# Render a single frame of the sample arm
renderer.render_frame({"shoulder": 0.35, "elbow": -0.55}, "frame.png")

# Render multiple frames (trajectory)
trajectory = [
    {"shoulder": 0.0, "elbow": 0.0},
    {"shoulder": 0.45, "elbow": 0.25},
    {"shoulder": -0.2, "elbow": 0.4},
]
renderer.render_trajectory(trajectory, "frames")

# JSON strings are also accepted
json_traj = json.dumps([
    {"shoulder": 0.0, "elbow": 0.0},
    {"shoulder": 0.3, "elbow": 0.1},
])
renderer.render_trajectory(json_traj, "frames_from_json")

# Load a CSV trajectory (headers must match joint names; optional "time" column is ignored for rendering)
renderer.render_trajectory_csv(sample_csv.as_posix(), "frames_from_csv")
```

The `examples/` directory contains a tiny two-link arm URDF (`two_link.urdf`) and a short CSV trajectory (`wave.csv`) so you can try the renderer without hunting for assets. The sample uses simple geometry (boxes and cylinders), so there are no extra mesh files to resolve.

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
