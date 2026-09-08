"""Generate the standalone procedural vegetation prototype library."""

from pathlib import Path

from generation.procedural_assets import generate_procedural_asset_library


def main() -> None:
    output_dir = Path("debug_out") / "procedural_assets"
    library = generate_procedural_asset_library(output_dir)
    print(f"generated {len(library.tree_assets)} tree and {len(library.weed_assets)} weed prototypes under {output_dir}")


if __name__ == "__main__":
    main()
