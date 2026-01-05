# robomesh

robomesh is a small Rust library that loads URDF models, applies joint trajectories, and renders simple 2D top-view PNGs. The crate can expose PyO3 bindings (behind an optional Cargo feature) so you can call it directly from Python for quick visualization or debugging workflows.

## Features
- URDF loading and forward kinematics via `urdf-rs` and the `k` crate
- 2D top-view link rendering on the XZ plane using a lightweight image buffer backend (no font dependencies)
- Visual geometry support, including mesh-based links (OBJ or STL) and primitive URDF shapes rasterized as filled triangle meshes in the XZ plane
- Primitive mesh generation helpers to convert URDF `box`/`cylinder`/`sphere` elements into triangle meshes for export, plus ellipsoid builders when you need stretched spheres and capsule meshing for rounded links
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
sample_urdf = repo_root / "examples" / "three_link_capsule.urdf"
sample_csv = repo_root / "examples" / "wave.csv"

renderer = RoboRenderer(sample_urdf.as_posix())
print("joint order:", renderer.joint_order())

# Render a single frame of the sample arm
renderer.render_frame({"shoulder": 0.35, "elbow": -0.55, "wrist": 0.25}, "frame.png")

# Render multiple frames (trajectory)
trajectory = [
    {"shoulder": 0.0, "elbow": 0.0, "wrist": 0.0},
    {"shoulder": 0.45, "elbow": 0.25, "wrist": -0.2},
    {"shoulder": -0.2, "elbow": 0.4, "wrist": 0.35},
]
renderer.render_trajectory(trajectory, "frames")

# JSON strings are also accepted
json_traj = json.dumps([
    {"shoulder": 0.0, "elbow": 0.0, "wrist": 0.0},
    {"shoulder": 0.3, "elbow": 0.1, "wrist": 0.4},
])
renderer.render_trajectory(json_traj, "frames_from_json")

# Load a CSV trajectory (headers must match joint names; optional "time" column is ignored for rendering)
renderer.render_trajectory_csv(sample_csv.as_posix(), "frames_from_csv")
```

The `examples/` directory contains a small three-link arm URDF (`three_link_capsule.urdf`) built from capsule geometry (cylinders with spherical end-caps) and a short CSV trajectory (`wave.csv`) so you can try the renderer without hunting for assets. The sample uses only built-in primitive geometry, so there are no extra mesh files to resolve.

Mesh files referenced by the URDF are resolved relative to the URDF file location when using `RoboRenderer(path)`. When loading
from a string (`from_urdf_string`), only absolute mesh paths will resolve correctly.

### CSV format
- The first row contains headers for every joint name (e.g., `joint1,joint2,joint3`).
- Values in each subsequent row are joint angles (radians) for that frame.
- An optional `time` column is allowed; it is read for completeness but ignored during rendering.
- All joints must be present in every row; missing or non-numeric values will raise an error.

### Output
- Each frame is saved as an 800x800 PNG showing link segments projected onto the XZ plane.
- Mesh visuals (OBJ and STL) are loaded, scaled, and rasterized as their top-view triangles (primitives are tessellated automatically); if a mesh file cannot be found or parsed, rendering will report an error with the missing path.
- All joints must be provided. Use `joint_order()` to verify the expected joint list before rendering.

### Primitive mesh generation
If you want full triangle meshes for URDF primitives (instead of the renderer's projected rectangles), you can build them directly in Rust:

```rust
use robomesh::{mesh_from_geometry, MeshData, MeshTessellation};

let geom = urdf_rs::Geometry::Cylinder { radius: 0.05, length: 0.3 };
let tess = MeshTessellation { cylinder_radial_segments: 48, ..Default::default() };
let mesh: MeshData = mesh_from_geometry(&geom, &tess)?;
mesh.write_obj("cylinder.obj")?; // Writes a vertex+face OBJ file

// Capsules are supported too (total length includes the hemispherical caps)
let capsule = mesh_from_geometry(&urdf_rs::Geometry::Capsule { radius: 0.05, length: 0.3 }, &tess)?;
capsule.write_obj("capsule.obj")?;

// Build an ellipsoid with per-axis radii using the same tessellation controls
let ellipsoid = generate_ellipsoid_mesh([0.1, 0.05, 0.2], &tess)?;
ellipsoid.write_obj("ellipsoid.obj")?;
```

`mesh_from_visual` performs the same tessellation, loads external mesh files, and applies the visual's origin pose so that the returned vertices are in world coordinates.

## License
See [LICENSE](LICENSE).
