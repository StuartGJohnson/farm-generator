"""
generation/crossings.py

Stage 6 of the generation DAG: strict collinear, vertex-to-vertex-only
crossings between existing per-parcel road-network vertices. See
CLAUDE.md "Crossings" for the full account of why this specific rule
(never a constructed point, collinear with a real road edge at BOTH
endpoints) is what's implemented, and why two earlier, more lenient
designs were rejected.

Deliberately NOT reduced to a minimum spanning tree -- every geometrically
valid pair is kept (see CLAUDE.md "Key lessons"). Where no valid
collinear pair exists between two parcels, there is simply no crossing
there; this can leave a parcel disconnected (rare), handled by
generation/orchestrator.py's retry-on-validation-failure wrapper, not by
loosening the geometric rule here.

Candidate proximity search uses `scipy.spatial.cKDTree.query_pairs` (see
CLAUDE.md "Library usage") instead of an O(V^2) double loop.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree

from farm_ir.schema import CrossingFeature, CrossingType, FarmScene, RoadClass, RoadEdge


def _parcel_rings(scene: FarmScene) -> dict[str, list[str]]:
    """Group RoadNode ids by parcel_id tag, preserving the ring order
    generation/roads.py inserted them in (one closed polygon loop per
    parcel)."""
    rings: dict[str, list[str]] = {}
    for nid, node in scene.roads.nodes.items():
        pid = node.tags.get("parcel_id")
        rings.setdefault(pid, []).append(nid)
    return rings


def _incident_directions(rings, positions) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """dir_in/dir_out unit vectors for every road vertex, from its ring's
    own adjacency (arriving-edge direction, leaving-edge direction)."""
    incident = {}
    for node_ids in rings.values():
        n = len(node_ids)
        for i, nid in enumerate(node_ids):
            prev_id, next_id = node_ids[i - 1], node_ids[(i + 1) % n]
            cur = positions[nid]
            dir_in = cur - positions[prev_id]
            dir_out = positions[next_id] - cur
            dir_in = dir_in / (np.linalg.norm(dir_in) + 1e-12)
            dir_out = dir_out / (np.linalg.norm(dir_out) + 1e-12)
            incident[nid] = (dir_in, dir_out)
    return incident


def _is_collinear(dir_a: np.ndarray, dir_b: np.ndarray, angle_tol_deg: float) -> bool:
    """
    True if unit vectors dir_a and dir_b lie along the same line --
    parallel OR anti-parallel, direction-agnostic. Uses the 2D cross
    product (signed perpendicular component) rather than a signed dot
    product specifically so both directions along the line count as
    collinear without separate +/- cases. `angle_tol_deg` must stay near
    the floating-point noise floor (~1e-13 deg observed for a genuine
    T-junction relationship), not a "looks small enough" value -- see
    CLAUDE.md "Key lessons" on why an 8-degree tolerance was measured and
    rejected.
    """
    cross = dir_a[0] * dir_b[1] - dir_a[1] * dir_b[0]
    return abs(cross) < np.sin(np.radians(angle_tol_deg))


def _nearest_hydrology_edge(scene: FarmScene, pid_a: str, pid_b: str, midpoint):
    """
    Best-effort association of a crossing with the hydrology channel it
    spans: among hydrology edges tagged to either parcel (see
    generation/hydrology.py), the one whose own midpoint is closest to
    the crossing's midpoint. Not a guaranteed exact match (T-junction
    supercell edges mean the "true" spanned segment isn't always
    unambiguous), but a reasonable, bounded-scope heuristic for
    `CrossingFeature.hydrology_edge_id` / `span_width`.
    """
    candidates = [e for e in scene.hydrology.edges.values()
                  if e.tags.get("parcel_id") in (pid_a, pid_b)]
    if not candidates:
        return None
    mids = np.array([np.mean(np.array(e.polyline), axis=0) for e in candidates])
    d = np.linalg.norm(mids - np.array(midpoint), axis=1)
    return candidates[int(np.argmin(d))]


def run(scene: FarmScene, config, rng) -> FarmScene:
    node_ids = list(scene.roads.nodes.keys())
    if not node_ids:
        return scene

    positions = {nid: np.array(scene.roads.nodes[nid].position) for nid in node_ids}
    rings = _parcel_rings(scene)
    incident = _incident_directions(rings, positions)
    node_parcel = {nid: scene.roads.nodes[nid].tags.get("parcel_id") for nid in node_ids}

    pts = np.array([positions[nid] for nid in node_ids])
    tree = cKDTree(pts)
    pairs = tree.query_pairs(r=config.connect_radius)

    edge_idx = len(scene.roads.edges)
    crossing_idx = 0

    for a, b in pairs:
        nid_a, nid_b = node_ids[a], node_ids[b]
        pid_a, pid_b = node_parcel[nid_a], node_parcel[nid_b]
        if pid_a == pid_b:
            continue

        V, W = pts[a], pts[b]
        d = float(np.linalg.norm(W - V))
        if d < 1e-9:
            continue
        crossing_dir = (W - V) / d

        dir_in_v, dir_out_v = incident[nid_a]
        ok_v = (_is_collinear(crossing_dir, dir_in_v, config.crossing_angle_tol_deg) or
                _is_collinear(crossing_dir, dir_out_v, config.crossing_angle_tol_deg))
        if not ok_v:
            continue

        dir_in_w, dir_out_w = incident[nid_b]
        ok_w = (_is_collinear(crossing_dir, dir_in_w, config.crossing_angle_tol_deg) or
                _is_collinear(crossing_dir, dir_out_w, config.crossing_angle_tol_deg))
        if not ok_w:
            continue

        eid = f"redge_{edge_idx:04d}"
        edge_idx += 1
        scene.roads.edges[eid] = RoadEdge(
            id=eid,
            node_a=nid_a,
            node_b=nid_b,
            polyline=[(float(V[0]), float(V[1])), (float(W[0]), float(W[1]))],
            width=config.road_width,
            surface=config.road_surface,
            road_class=RoadClass.CROSSING_SPUR,
        )

        midpoint = (float((V[0] + W[0]) / 2.0), float((V[1] + W[1]) / 2.0))
        hedge = _nearest_hydrology_edge(scene, pid_a, pid_b, midpoint)

        cid = f"crossing_{crossing_idx:04d}"
        crossing_idx += 1
        scene.crossings[cid] = CrossingFeature(
            id=cid,
            road_edge_id=eid,
            hydrology_edge_id=hedge.id if hedge is not None else "",
            location=midpoint,
            crossing_type=CrossingType.CULVERT,
            span_width=hedge.top_width if hedge is not None else d,
            refs=[pid_a, pid_b],
        )

    return scene
