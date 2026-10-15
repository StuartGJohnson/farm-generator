"""Reusable mapping from Isaac local ENU metres to projected and WGS84 coordinates."""

from __future__ import annotations

import math
from dataclasses import dataclass
from functools import cached_property


@dataclass(frozen=True)
class GeoReference:
    projected_crs: str
    origin_easting: float
    origin_northing: float
    origin_altitude: float
    world_to_grid_heading_deg: float = 0.0

    @classmethod
    def from_wgs84_origin(
        cls, latitude: float, longitude: float, altitude: float,
        projected_crs: str = "EPSG:32610", world_to_grid_heading_deg: float = 0.0,
    ) -> "GeoReference":
        from pyproj import Transformer

        east, north = Transformer.from_crs("EPSG:4326", projected_crs, always_xy=True).transform(
            longitude, latitude
        )
        return cls(projected_crs, east, north, altitude, world_to_grid_heading_deg)

    def project_local(self, x: float, y: float) -> tuple[float, float]:
        heading = math.radians(self.world_to_grid_heading_deg)
        c, s = math.cos(heading), math.sin(heading)
        return self.origin_easting + c*x - s*y, self.origin_northing + s*x + c*y

    def to_wgs84(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        east, north = self.project_local(x, y)
        longitude, latitude = self._to_wgs84.transform(east, north)
        # This project uses a configured WGS84/ellipsoidal origin altitude.
        # Isaac Z is a metre offset from it; no terrain-relative height is inferred.
        return latitude, longitude, self.origin_altitude + z

    @cached_property
    def _to_wgs84(self):
        from pyproj import Transformer

        return Transformer.from_crs(self.projected_crs, "EPSG:4326", always_xy=True)
