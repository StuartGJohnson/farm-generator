"""
generation/hydrology.py

Stage 3 of the generation DAG: `HydrologyNetwork` DERIVED from the
tessellated (and cleaned) parcel boundaries -- every parcel boundary edge
becomes a `HydrologyEdge`, every shared corner a `HydrologyNode` (see
CLAUDE.md principle 2 and "Generation order" #3). This phase every edge
is `ConnectionType.PIPED` -- there is no trunk, so nothing is
hydraulically continuous with anything else (see CLAUDE.md "Trunk / main
channel: deferred").

Shared-corner node merging uses `scipy.spatial.cKDTree` (see CLAUDE.md
"Library usage"). Vertex cleanup runs per-parcel-face in isolation
(generation/tessellation.py), so a shared corner between two adjacent
parcels can drift apart by up to roughly one erosion standoff if one
side's cleanup merged a nearby degenerate vertex into it and the other
side's didn't -- an accepted consequence of full-perimeter-per-parcel
independence (see CLAUDE.md "Roads"). The merge tolerance here is
therefore a real, modest distance (a small fraction of standoff), not a
floating-point-noise-scale tolerance like generation/crossings.py's
collinearity check.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree

from farm_ir.schema import (
    BoundaryRef,
    ConnectionType,
    FarmScene,
    HydrologyEdge,
    HydrologyNode,
    HydrologyNodeType,
)


def _merge_labels(points: np.ndarray, tol: float) -> np.ndarray:
    """Union-find over all point pairs within `tol` of each other; returns
    a canonical-index label per point."""
    n = len(points)
    parent = list(range(n))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    if n > 1 and tol > 0:
        tree = cKDTree(points)
        for a, b in tree.query_pairs(r=tol):
            union(a, b)

    return np.array([find(i) for i in range(n)])


def run(scene: FarmScene, config, rng) -> FarmScene:
    raw_points: list[tuple[float, float]] = []
    edge_specs: list[tuple[str, int, int]] = []  # (parcel_id, idx_a, idx_b)

    for pid, parcel in scene.parcels.items():
        coords = parcel.polygon
        n = len(coords)
        base = len(raw_points)
        raw_points.extend(coords)
        for i in range(n):
            edge_specs.append((pid, base + i, base + (i + 1) % n))

    if not raw_points:
        return scene

    points = np.array(raw_points)
    labels = _merge_labels(points, config.hydrology_node_merge_tol)

    canonical_ids: dict[int, str] = {}
    node_members: dict[str, list[np.ndarray]] = {}
    for i, lbl in enumerate(labels):
        if lbl not in canonical_ids:
            canonical_ids[lbl] = f"hnode_{len(canonical_ids):04d}"
        nid = canonical_ids[lbl]
        node_members.setdefault(nid, []).append(points[i])

    for nid, pts in node_members.items():
        mean_pos = np.mean(pts, axis=0)
        scene.hydrology.nodes[nid] = HydrologyNode(
            id=nid,
            position=(float(mean_pos[0]), float(mean_pos[1])),
            node_type=HydrologyNodeType.CONFLUENCE.value,
        )

    for edge_idx, (pid, ia, ib) in enumerate(edge_specs):
        node_a = canonical_ids[labels[ia]]
        node_b = canonical_ids[labels[ib]]
        pa, pb = points[ia], points[ib]
        eid = f"hedge_{edge_idx:04d}"
        scene.hydrology.edges[eid] = HydrologyEdge(
            id=eid,
            node_a=node_a,
            node_b=node_b,
            polyline=[(float(pa[0]), float(pa[1])), (float(pb[0]), float(pb[1]))],
            connection_type=ConnectionType.PIPED,
            top_width=config.hydrology_top_width,
            bottom_width=config.hydrology_bottom_width,
            depth=config.hydrology_depth,
            tags={"parcel_id": pid},
        )
        scene.parcels[pid].boundary_refs.append(BoundaryRef(network="hydrology", edge_id=eid))

    return scene
