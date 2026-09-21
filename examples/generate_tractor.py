"""Generate tractor USDA, URDF, and a lightweight diagnostic plot."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("config", nargs="?", default="configs/tractor_default.yaml")
    parser.add_argument("--output-dir", default="debug_out/tractor")
    args = parser.parse_args()

    # PhysxSchema is compiled against Isaac's bundled USD, not the separate
    # PyPI pxr package. Configure those libraries without launching Kit/RTX.
    from tractor_generation.isaac_usd_env import reexec_with_isaac_usd

    reexec_with_isaac_usd()

    from tractor_generation.config import load_tractor_config
    from tractor_generation.diagnostic import save_tractor_diagnostic
    from tractor_generation.urdf import write_tractor_urdf
    from tractor_generation.usd import write_tractor_usda

    config = load_tractor_config(args.config)
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    usd = write_tractor_usda(config, output / "tractor.usda")
    urdf = write_tractor_urdf(config, output / "tractor.urdf")
    png = save_tractor_diagnostic(config, output / "tractor_diagnostic.png")
    print(f"generated {usd}, {urdf}, and {png}")


if __name__ == "__main__":
    main()
