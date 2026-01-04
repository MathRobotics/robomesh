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
sample_urdf = repo_root / "examples" / "two_link.urdf"
sample_csv = repo_root / "examples" / "wave.csv"

renderer = RoboRenderer(sample_urdf.as_posix())
print("joint order:", renderer.joint_order())
renderer.render_frame({"shoulder": 0.3, "elbow": -0.4}, "sample.png")
renderer.render_trajectory_csv(sample_csv.as_posix(), "frames")
PY
```

`uv sync` will read the `pyproject.toml` here, create a `.venv`, and install `maturin` as the only Python dependency needed to build the extension. You can add more runtime dependencies to `project.dependencies` if you extend the Python surface area.

## Notes
- The Rust crate remains the single source of functionality; this directory only orchestrates Python packaging and environment management.
- `uv run maturin build --features python` produces a Python wheel that you can distribute or install with `pip`.
- If you regenerate lock files with uv, commit them alongside this directory to keep Python dependencies reproducible.
