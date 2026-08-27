"""
generation/roads.py

Stage 5 of the generation DAG: full-perimeter-per-parcel roads. Every
parcel's road is its own complete eroded perimeter
(`polygon.buffer(-standoff, join_style="mitre", mitre_limit=...)`), not a
selectively-one-sided pick via adjacency detection -- see CLAUDE.md
"Roads" for why the one-sided design was abandoned (repeated T-junction
coverage-gap bugs). This doubles the road along every shared channel; an
accepted simplification.

Uses `shapely.buffer` (GEOS) for the actual offset, per CLAUDE.md
"Library usage" -- hand-rolled polygon offsetting was found unreliable
in four distinct ways during prototyping (see CLAUDE.md "Key lessons").

Road nodes/edges are built fresh per parcel, with NO cross-parcel node
deduplication (unlike generation/hydrology.py's shared-corner merge) --
each parcel's eroded ring is an independently offset perimeter, so two
adjacent parcels' road vertices near a shared channel are offset inward
from each other by construction and are not expected to coincide. That
non-coincidence is exactly what full-perimeter roads intentionally trade
for coverage-gap safety.
"""

from __future__ import annotations

from shapely.geometry import Polygon

from farm_ir.schema import FarmScene, RoadClass, RoadEdge, RoadNode, RoadNodeType

from generation._geom_utils import exterior_coords, largest_polygon


def run(scene: FarmScene, config, rng) -> FarmScene:
    node_idx = 0
    edge_idx = 0

    for pid, parcel in scene.parcels.items():
        poly = Polygon(parcel.polygon)
        eroded = poly.buffer(-config.standoff, join_style="mitre", mitre_limit=config.mitre_limit)
        eroded = largest_polygon(eroded)
        if eroded is None or eroded.area <= 0:
            # Over-eroded away entirely -- parcel too small relative to
            # standoff. Left roadless rather than forcing a degenerate ring.
            continue

        coords = exterior_coords(eroded)
        node_ids = []
        for x, y in coords:
            nid = f"rnode_{node_idx:04d}"
            node_idx += 1
            scene.roads.nodes[nid] = RoadNode(
                id=nid,
                position=(float(x), float(y)),
                node_type=RoadNodeType.INTERSECTION.value,
                tags={"parcel_id": pid},
            )
            node_ids.append(nid)

        n = len(node_ids)
        for i in range(n):
            a, b = node_ids[i], node_ids[(i + 1) % n]
            pa, pb = coords[i], coords[(i + 1) % n]
            eid = f"redge_{edge_idx:04d}"
            edge_idx += 1
            scene.roads.edges[eid] = RoadEdge(
                id=eid,
                node_a=a,
                node_b=b,
                polyline=[(float(pa[0]), float(pa[1])), (float(pb[0]), float(pb[1]))],
                width=config.road_width,
                surface=config.road_surface,
                road_class=RoadClass.FRONTAGE,
                tags={"parcel_id": pid},
            )

    return scene
