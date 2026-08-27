"""
generation/crops.py

Stage 7 of the generation DAG: trees + weeds per `CropArea`. Retargets
the existing grid-orchard generator's design to consume an arbitrary
polygon (a tessellated parcel, inset by `headland_width`) instead of a
fixed rectangle with a global origin -- clipping against the actual
polygon is required since T-tessellation parcels are not clean
rectangles.

Weeds are a simple uniform low-density zone over the whole parcel this
phase (headland + row-gap correlation is a later phase -- see CLAUDE.md
"Generation order" #7).
"""

from __future__ import annotations

import numpy as np
from shapely.geometry import Point, Polygon

from farm_ir.schema import CropArea, FarmScene, PlantingSpec, TreeInstance, WeedSpec, WeedZone

from generation._geom_utils import largest_polygon


def _planting_spec(config, seed: int) -> PlantingSpec:
    return PlantingSpec(
        row_spacing=config.row_spacing,
        tree_spacing=config.tree_spacing,
        row_angle_deg=config.row_angle_deg,
        headland_width=config.headland_width,
        species_mix=dict(config.species_mix),
        seed=int(seed),
    )


def _generate_tree_grid(planting_poly: Polygon, spec: PlantingSpec, rng: np.random.Generator):
    if planting_poly.is_empty:
        return []

    minx, miny, maxx, maxy = planting_poly.bounds
    cx, cy = planting_poly.centroid.x, planting_poly.centroid.y
    theta = np.radians(spec.row_angle_deg)
    row_dir = np.array([np.cos(theta), np.sin(theta)])      # along a row
    cross_dir = np.array([-np.sin(theta), np.cos(theta)])   # row-to-row

    diag = float(np.hypot(maxx - minx, maxy - miny))
    n_steps = int(np.ceil(diag / min(spec.row_spacing, spec.tree_spacing))) + 2

    species_names = list(spec.species_mix.keys())
    species_probs = np.array(list(spec.species_mix.values()), dtype=float)
    species_probs = species_probs / species_probs.sum()

    trees = []
    for i in range(-n_steps, n_steps + 1):
        for j in range(-n_steps, n_steps + 1):
            p = (np.array([cx, cy])
                 + row_dir * (i * spec.tree_spacing)
                 + cross_dir * (j * spec.row_spacing))
            if not planting_poly.contains(Point(p[0], p[1])):
                continue
            species = rng.choice(species_names, p=species_probs)
            trees.append(dict(
                position=(float(p[0]), float(p[1])),
                species=str(species),
                age=float(rng.uniform(2.0, 15.0)),
            ))
    return trees


def run(scene: FarmScene, config, rng: np.random.Generator) -> FarmScene:
    tree_idx = 0
    weed_idx = 0

    for pid, parcel in scene.parcels.items():
        if not isinstance(parcel, CropArea):
            continue

        poly = Polygon(parcel.polygon)
        inset = largest_polygon(
            poly.buffer(-config.headland_width, join_style="mitre", mitre_limit=config.mitre_limit)
        )
        if inset is None or inset.area <= 0:
            # Too small for a headland at this width -- plant to the
            # parcel edge rather than skip the parcel entirely.
            inset = poly

        parcel_seed = int(rng.integers(0, 2**63 - 1))
        parcel_rng = np.random.default_rng(parcel_seed)
        spec = _planting_spec(config, parcel_seed)
        parcel.planting = spec

        for t in _generate_tree_grid(inset, spec, parcel_rng):
            tid = f"tree_{tree_idx:05d}"
            tree_idx += 1
            scene.trees[tid] = TreeInstance(
                id=tid,
                position=t["position"],
                species=t["species"],
                age_years=t["age"],
                canopy_radius=config.canopy_radius,
                trunk_dbh=config.trunk_dbh,
                height=config.tree_height,
                tags={"parcel_id": pid},
            )
            parcel.refs.append(tid)

        wid = f"weed_{weed_idx:04d}"
        weed_idx += 1
        scene.weed_zones[wid] = WeedZone(
            id=wid,
            polygon=parcel.polygon,
            density_params=dict(config.weed_density_params),
            tags={"parcel_id": pid},
        )
        parcel.weeds = WeedSpec(density_params=dict(config.weed_density_params), seed=parcel_seed)
        parcel.weed_zone_refs.append(wid)

    return scene
