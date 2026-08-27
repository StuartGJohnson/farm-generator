"""
generation/_geom_utils.py

Small shapely helpers shared by multiple generation stages. Leading
underscore signals this is generation-internal, not part of the public
generation API (that's orchestrator.generate_farm / generate_validated).
"""

from __future__ import annotations

import numpy as np
from shapely.geometry import MultiPolygon, Polygon


def largest_polygon(geom) -> Polygon | None:
    """
    Reduce a buffer/split result to its single largest polygon piece.
    A buffer op can split into a MultiPolygon (over-erosion of a narrow
    face) or come back empty (fully eroded away) -- callers treat both as
    "this face has no usable result" and keep only the dominant piece
    rather than trying to carry multiple slivers forward.
    """
    if geom is None or geom.is_empty:
        return None
    if isinstance(geom, Polygon):
        return geom
    if isinstance(geom, MultiPolygon):
        polys = list(geom.geoms)
        return max(polys, key=lambda p: p.area) if polys else None
    return None


def exterior_coords(poly: Polygon) -> np.ndarray:
    """Exterior ring vertices as an (N, 2) array, without shapely's
    closing duplicate of the first point."""
    coords = np.array(poly.exterior.coords)
    if len(coords) > 1 and np.allclose(coords[0], coords[-1]):
        coords = coords[:-1]
    return coords
