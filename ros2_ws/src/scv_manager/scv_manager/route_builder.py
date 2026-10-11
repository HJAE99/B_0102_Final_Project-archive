import math
from dataclasses import dataclass
from pathlib import Path

import yaml

from .manager_core import Waypoint


@dataclass(frozen=True)
class SurveyRoute:
    zone_id: str
    waypoints: tuple[Waypoint, ...]


@dataclass(frozen=True)
class FacilityRoute:
    facility_id: str
    waypoints: tuple[Waypoint, ...]


class RouteBuilder:
    def __init__(
        self,
        zones_file: str,
        facilities_file: str,
    ):
        self.zones_file = Path(zones_file)
        self.facilities_file = Path(facilities_file)

    def build_survey_routes(
        self,
    ) -> list[SurveyRoute]:
        data = self._load_yaml(self.zones_file)
        routes = []

        for zone in data.get("zones", []):
            polygon = [
                (float(point[0]), float(point[1]))
                for point in zone["polygon"]
            ]
            spacing = float(zone.get("spacing", 1.0))

            waypoints = self._build_scanline_route(
                polygon,
                spacing,
            )

            if not waypoints:
                raise ValueError(
                    f"{zone['id']}의 polygon 좌표가 "
                    "아직 입력되지 않았습니다."
                )

            routes.append(
                SurveyRoute(
                    zone_id=str(zone["id"]),
                    waypoints=tuple(waypoints),
                )
            )

        return routes

    def build_facility_routes(
        self,
    ) -> list[FacilityRoute]:
        data = self._load_yaml(self.facilities_file)
        routes = []

        for facility in data.get("facilities", []):
            view = facility["view"]

            waypoint = Waypoint(
                x=float(view["x"]),
                y=float(view["y"]),
                yaw=float(view["yaw"]),
            )

            routes.append(
                FacilityRoute(
                    facility_id=str(facility["id"]),
                    waypoints=(waypoint,),
                )
            )

        return routes

    @staticmethod
    def _load_yaml(path: Path) -> dict:
        with path.open(
            "r",
            encoding="utf-8",
        ) as file:
            return yaml.safe_load(file) or {}

    @staticmethod
    def _build_scanline_route(
        polygon: list[tuple[float, float]],
        spacing: float,
    ) -> list[Waypoint]:
        if len(polygon) < 3:
            return []

        ys = [point[1] for point in polygon]
        min_y = min(ys)
        max_y = max(ys)

        if max_y <= min_y or spacing <= 0.0:
            return []

        first_y = min_y + spacing / 2.0

        if first_y >= max_y:
            scanline_ys = [
                (min_y + max_y) / 2.0
            ]
        else:
            scanline_ys = []
            y = first_y

            while y < max_y:
                scanline_ys.append(y)
                y += spacing

        waypoints = []
        forward = True

        for y in scanline_ys:
            intersections = []

            for index in range(len(polygon)):
                x1, y1 = polygon[index]
                x2, y2 = polygon[
                    (index + 1) % len(polygon)
                ]

                crosses = (
                    (y1 <= y < y2)
                    or (y2 <= y < y1)
                )

                if not crosses:
                    continue

                ratio = (y - y1) / (y2 - y1)
                x = x1 + ratio * (x2 - x1)
                intersections.append(x)

            intersections.sort()

            for index in range(
                0,
                len(intersections) - 1,
                2,
            ):
                left_x = intersections[index]
                right_x = intersections[index + 1]

                if forward:
                    start_x = left_x
                    end_x = right_x
                    yaw = 0.0
                else:
                    start_x = right_x
                    end_x = left_x
                    yaw = math.pi

                waypoints.append(
                    Waypoint(
                        x=start_x,
                        y=y,
                        yaw=yaw,
                    )
                )
                waypoints.append(
                    Waypoint(
                        x=end_x,
                        y=y,
                        yaw=yaw,
                    )
                )

                forward = not forward

        return waypoints
