"""Build the standalone procedural vegetation asset library.

The geometry and texture algorithms are copied verbatim from the sibling
FarmGeneration prototype. This wrapper only defines stable variant seeds and
the directory layout consumed by farm exporters.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from generation.asset_generation.tree_generator import generate_pecan_tree_usd
from generation.asset_generation.weed_generator import (
    generate_ashweed_usd,
    generate_dry_grass_usd,
    generate_fennel_usd,
)


@dataclass(frozen=True)
class ProceduralAssetLibrary:
    tree_assets: tuple[Path, ...]
    weed_assets: tuple[Path, ...]


def generate_procedural_asset_library(
    output_dir: Path,
    tree_seeds: tuple[int, ...] = (101, 202, 303, 404),
    weed_seeds: tuple[int, int, int] = (101, 202, 303),
) -> ProceduralAssetLibrary:
    """Generate tree variants and one prototype for each weed species."""
    output_dir = Path(output_dir)
    if not tree_seeds:
        raise ValueError("tree_seeds must contain at least one seed")
    tree_assets = tuple(
        generate_pecan_tree_usd(
            output_dir / "trees" / f"tree_{index}" / f"pecan_tree_{index}.usda",
            seed=seed,
        )
        for index, seed in enumerate(tree_seeds, start=1)
    )
    weed_assets = (
        generate_dry_grass_usd(
            output_dir / "weeds" / "dry_grass" / "dry_grass.usda",
            seed=weed_seeds[0],
        ),
        generate_fennel_usd(
            output_dir / "weeds" / "fennel" / "fennel.usda",
            seed=weed_seeds[1],
        ),
        generate_ashweed_usd(
            output_dir / "weeds" / "ashweed" / "ashweed.usda",
            seed=weed_seeds[2],
        ),
    )
    return ProceduralAssetLibrary(
        tree_assets=tuple(Path(path) for path in tree_assets),
        weed_assets=tuple(Path(path) for path in weed_assets),
    )
