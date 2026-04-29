"""Coordinate resolution: lat/lon -> lane-aligned goal candidates using lanelet2."""

from __future__ import annotations

import math
from typing import Optional

import lanelet2
from lanelet2.io import Origin

from geometry_msgs.msg import Point, Pose

from autoware_lanelet2_extension_python.projection import MGRSProjector
from autoware_lanelet2_extension_python.utility.query import (
    laneletLayer,
    roadLanelets,
    getLaneletsWithinRange,
)
from autoware_lanelet2_extension_python.utility.utilities import (
    getClosestCenterPose,
    getLateralDistanceToCenterline,
    getLaneletLength2d,
)

from autoware_claw.types import GoalCandidate


def _yaw_from_quaternion(q) -> float:
    """Extract yaw from a quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class CoordinateResolver:
    """Resolves lat/lon coordinates to lane-aligned goal poses.

    Uses lanelet2 map and MGRSProjector to convert GPS coordinates
    to map frame, then finds nearby road lanelets and computes
    centerline-aligned poses as goal candidates.
    """

    def __init__(
        self,
        map_path: str,
        origin_lat: float,
        origin_lon: float,
        origin_alt: float = 0.0,
    ) -> None:
        self._origin = Origin(origin_lat, origin_lon, origin_alt)
        self._projector = MGRSProjector(self._origin)
        self._lanelet_map = lanelet2.io.load(map_path, self._projector)
        all_lanelets = laneletLayer(self._lanelet_map)
        self._road_lanelets = roadLanelets(all_lanelets)

    def resolve_goal(
        self,
        lat: float,
        lon: float,
        search_radius: float = 50.0,
        max_candidates: int = 5,
    ) -> list[GoalCandidate]:
        """Convert lat/lon to lane-aligned goal candidates.

        Args:
            lat: Latitude in degrees.
            lon: Longitude in degrees.
            search_radius: Search radius in meters for nearby lanelets.
            max_candidates: Maximum number of candidates to return.

        Returns:
            List of GoalCandidate sorted by lateral distance to centerline.
        """
        # 1. lat/lon -> map coordinates via MGRSProjector
        gps_point = lanelet2.core.GPSPoint(lat, lon, 0.0)
        map_point_3d = self._projector.forward(gps_point)
        search_point = lanelet2.core.BasicPoint2d(map_point_3d.x, map_point_3d.y)

        # 2. Find road lanelets within search radius
        nearby = getLaneletsWithinRange(self._road_lanelets, search_point, search_radius)
        if not nearby:
            return []

        # 3. For each lanelet, compute closest centerline pose
        point_msg = Point(x=map_point_3d.x, y=map_point_3d.y, z=0.0)
        candidates = []
        for ll in nearby:
            pose = getClosestCenterPose(ll, point_msg)
            lateral_dist = getLateralDistanceToCenterline(ll, pose)
            yaw = _yaw_from_quaternion(pose.orientation)
            candidates.append(
                GoalCandidate(
                    lanelet_id=ll.id,
                    x=pose.position.x,
                    y=pose.position.y,
                    z=pose.position.z,
                    yaw_rad=yaw,
                    lateral_dist_m=abs(lateral_dist),
                )
            )

        # 4. Sort by lateral distance and return top candidates
        candidates.sort(key=lambda c: c.lateral_dist_m)
        return candidates[:max_candidates]

    def get_lane_info(
        self,
        x: float,
        y: float,
        search_radius: float = 10.0,
    ) -> list[dict]:
        """Get lane info near a map-frame coordinate.

        Args:
            x: X coordinate in map frame.
            y: Y coordinate in map frame.
            search_radius: Search radius in meters.

        Returns:
            List of dicts with lanelet info (id, length, subtype).
        """
        search_point = lanelet2.core.BasicPoint2d(x, y)
        nearby = getLaneletsWithinRange(self._road_lanelets, search_point, search_radius)
        result = []
        for ll in nearby:
            length = getLaneletLength2d(ll)
            subtype = ll.attributes.get("subtype", "unknown")
            speed_limit = ll.attributes.get("speed_limit", "unknown")
            result.append(
                {
                    "lanelet_id": ll.id,
                    "length_m": round(length, 2),
                    "subtype": subtype,
                    "speed_limit": speed_limit,
                }
            )
        return result
