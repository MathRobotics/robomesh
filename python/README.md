# Python usage (uv-managed)

The Python bindings for `robomesh` live alongside the Rust crate but are managed separately with [uv](https://github.com/astral-sh/uv) so that Python tooling and dependencies do not leak into the Rust workflow.

## Prerequisites
- Rust toolchain (for building the extension module)
- Python 3.8–3.13 (PyO3 v0.22 in the Rust crate builds cleanly with 3.13)
- [uv](https://github.com/astral-sh/uv) installed

## Getting started
From this `python/` directory:

```bash
# Create and sync a virtual environment with uv
uv sync

# Build and install the robomesh extension into the uv environment
uv run maturin develop --manifest-path ../Cargo.toml --features python

# Try rendering from Python
uv run python - <<'PY'
from pathlib import Path
from robomesh import RoboRenderer

repo_root = Path(__file__).resolve().parent.parent
sample_urdf = repo_root / "examples" / "three_link_capsule.urdf"
mesh_urdf = repo_root / "examples" / "mesh_sample.urdf"
sample_csv = repo_root / "examples" / "wave.csv"

renderer = RoboRenderer(sample_urdf.as_posix())
print("joint order:", renderer.joint_order())
renderer.render_frame({"shoulder": 0.3, "elbow": -0.4, "wrist": 0.2}, "sample.png")
renderer.render_trajectory_csv(sample_csv.as_posix(), "frames")

# Render a single-frame mesh visual that references examples/mesh_link.obj
mesh_renderer = RoboRenderer(mesh_urdf.as_posix())
mesh_renderer.render_frame({}, "mesh.png")
PY

# Or run the ready-made example script alongside the bundled URDF/CSV
uv run python ../examples/python_example.py
```

`uv sync` will read the `pyproject.toml` here, create a `.venv`, and install `maturin` as the only Python dependency needed to build the extension. You can add more runtime dependencies to `project.dependencies` if you extend the Python surface area.

The bundled URDF (`examples/three_link_capsule.urdf`) uses capsule-style links (cylinders with spherical caps) so you can see how multiple primitive visuals combine into a smooth arm without any mesh files.

The mesh sample (`examples/mesh_sample.urdf`) references a small OBJ (`examples/mesh_link.obj`) so you can confirm that URDF `<mesh>` geometry tessellates and renders from Python.

## Notes
- The Rust crate remains the single source of functionality; this directory only orchestrates Python packaging and environment management.
- `uv run maturin build --features python` produces a Python wheel that you can distribute or install with `pip`.
- If you regenerate lock files with uv, commit them alongside this directory to keep Python dependencies reproducible.
