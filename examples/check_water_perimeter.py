"""Audit water-edge/ground agreement before and after terrain displacement."""

import argparse
from collections import defaultdict
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from shapely.geometry import Point, box

from export.usd import ChannelUndulationConfig, build_ground_mesh, build_road_surface_patches
from generation.orchestrator import load_farm
from generation.surface_height import sample_height


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", default="debug_out/farm_scenes/farm_seed_1_120m2.yaml")
    parser.add_argument("--output", default="debug_out/mesh/water_perimeter_report.json")
    args = parser.parse_args()
    scene, config = load_farm(args.scene)
    mesh = build_ground_mesh(scene, config.bounds, undulation=ChannelUndulationConfig())
    if mesh.surface_height is None or mesh.height_weights is None:
        raise ValueError("scene has no generated height field")
    ground = np.asarray(mesh.points)
    weights = np.asarray(mesh.height_weights)
    offsets = sample_height(mesh.surface_height, ground[:, :2])
    original_z = ground[:, 2] - offsets * weights
    tree = cKDTree(ground[:, :2])
    domain_edge = box(*config.bounds).boundary
    road_area = next(
        (patch.polygon for patch in build_road_surface_patches(scene, config.bounds)
         if not patch.is_crossing), None
    )
    road_edge = road_area.boundary if road_area is not None else None

    samples = defaultdict(list)
    segments = defaultdict(int)
    for surface in mesh.water_surfaces:
        for a, b in surface.boundary_edges:
            pa, pb = surface.points[a], surface.points[b]
            midpoint = Point((pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2)
            if domain_edge.distance(midpoint) <= 2e-6:
                kind = "map_edge"
            elif road_edge is not None and road_edge.distance(midpoint) <= 2e-6:
                kind = "road_cut"
            else:
                kind = "shoreline"
            segments[kind] += 1
            for water_point in (pa, pb):
                candidates = tree.query_ball_point(water_point[:2], r=1e-4)
                if not candidates:
                    samples[kind].append({"xy": list(water_point[:2]), "missing_ground": True})
                    continue
                index = min(candidates, key=lambda i: abs(original_z[i] - water_point[2]))
                samples[kind].append({
                    "xy": list(water_point[:2]),
                    "original_gap_m": float(original_z[index] - water_point[2]),
                    "current_gap_m": float(ground[index, 2] - water_point[2]),
                    "height_weight": float(weights[index]),
                })

    report = {"scene": args.scene, "shoreline_taper_m": config.shoreline_taper_m,
              "categories": {}}
    for kind, entries in samples.items():
        valid = [entry for entry in entries if "missing_ground" not in entry]
        original = np.abs([entry["original_gap_m"] for entry in valid])
        current = np.abs([entry["current_gap_m"] for entry in valid])
        aligned = [entry for entry in valid if abs(entry["original_gap_m"]) <= 1e-3]
        aligned_current = np.abs([entry["current_gap_m"] for entry in aligned])
        report["categories"][kind] = {
            "segments": segments[kind], "endpoint_samples": len(entries),
            "missing_ground_samples": len(entries) - len(valid),
            "originally_aligned_samples": len(aligned),
            "original_abs_gap_median_m": float(np.median(original)) if len(original) else None,
            "original_abs_gap_max_m": float(np.max(original)) if len(original) else None,
            "current_abs_gap_median_m": float(np.median(current)) if len(current) else None,
            "current_abs_gap_max_m": float(np.max(current)) if len(current) else None,
            "aligned_current_abs_gap_max_m": float(np.max(aligned_current)) if len(aligned_current) else None,
            "largest_original_mismatches": sorted(
                valid, key=lambda entry: abs(entry["original_gap_m"]), reverse=True
            )[:8],
        }
    destination = Path(args.output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2) + "\n")
    def metres(value):
        return "n/a" if value is None else f"{value:.6f} m"

    for kind, values in report["categories"].items():
        print(f"{kind}: {values['segments']} edges, original max "
              f"{metres(values['original_abs_gap_max_m'])}, "
              f"current max {metres(values['current_abs_gap_max_m'])}, "
              f"aligned max {metres(values['aligned_current_abs_gap_max_m'])}")
    print(f"Wrote {destination}")


if __name__ == "__main__":
    main()
