"""Minimal Python example for robomesh rendering.

Run this after building the robomesh Python extension (see python/README.md).
"""
from pathlib import Path

from robomesh import RoboRenderer


def main() -> None:
    repo_root = Path(__file__).resolve().parent.parent
    sample_urdf = repo_root / "examples" / "three_link_capsule.urdf"
    sample_csv = repo_root / "examples" / "wave.csv"

    renderer = RoboRenderer(sample_urdf.as_posix())
    print("Joint order:", renderer.joint_order())

    output_dir = repo_root / "examples" / "frames"
    output_dir.mkdir(exist_ok=True)

    renderer.render_frame(
        {"shoulder": 0.35, "elbow": -0.55, "wrist": 0.3},
        (output_dir / "frame.png").as_posix(),
    )
    renderer.render_trajectory_csv(sample_csv.as_posix(), output_dir.as_posix())
    print(f"Rendered outputs in {output_dir}")


if __name__ == "__main__":
    main()
