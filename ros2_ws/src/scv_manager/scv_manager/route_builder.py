import math
import re
from dataclasses import dataclass
from pathlib import Path

import yaml

from scv_survey.cellmap_store import in_polygon

from .manager_core import Waypoint


@dataclass(frozen=True)
class SurveyRoute:
    zone_id: str
    waypoints: tuple[Waypoint, ...]


@dataclass(frozen=True)
class FacilityRoute:
    facility_id: str
    waypoints: tuple[Waypoint, ...]
    zone_id: str = ""


def wall_points(info, occupied_thresh: float = 0.65) -> list[tuple[float, float]]:
    """지도(P5 pgm)의 점유 픽셀 중심 [(x, y)] (map 좌표). info 는 read_map_yaml 결과."""
    raw = Path(info.pgm).read_bytes()
    head = re.match(rb"P5\s+(\d+)\s+(\d+)\s+(\d+)\s", raw)
    if head is None:
        raise ValueError(f"{info.pgm}: P5 pgm 이 아닙니다")
    width, height, maxval = map(int, head.groups())
    step = 2 if maxval > 255 else 1     # testbed_v1.pgm 은 16비트
    data = raw[head.end():]
    return [
        (
            info.origin[0] + (i % width + 0.5) * info.resolution,
            info.origin[1] + (height - 1 - i // width + 0.5) * info.resolution,
        )
        for i in range(width * height)
        if 1.0 - int.from_bytes(data[i * step:(i + 1) * step], "big") / maxval
        > occupied_thresh
    ]


class RouteBuilder:
    def __init__(
        self,
        zones_file: str,
        facilities_file: str,
        wall_bounds: tuple[float, ...] = (),
        wall_clearance: float = 0.0,
        origin: tuple[float, float] = (0.0, 0.0),
        walls=(),
        obstacle_clearance: float = 0.0,
    ):
        self.zones_file = Path(zones_file)
        self.facilities_file = Path(facilities_file)
        # 벽 안쪽 사각형 (xmin, ymin, xmax, ymax). 주면 경로를 벽에서
        # wall_clearance 만큼 안쪽으로 자르고 y 줄을 칸 중심에 맞춘다.
        self.wall_bounds = tuple(wall_bounds)
        self.wall_clearance = wall_clearance
        self.origin = origin
        # 지도의 점유 픽셀. 경로 점과 재측정 칸은 여기서 obstacle_clearance
        # 이상 떨어뜨린다 (사각형 wall_bounds 는 안쪽 벽 조각을 모른다).
        self.walls = list(walls)
        self.obstacle_clearance = obstacle_clearance
        self.skipped_facilities: list[str] = []

    def zones(self) -> dict[str, dict]:
        data = self._load_yaml(self.zones_file)
        return {str(zone["id"]): zone for zone in data.get("zones", [])}

    def inner_bounds(self) -> tuple[float, float, float, float]:
        xmin, ymin, xmax, ymax = self.wall_bounds
        c = self.wall_clearance
        return (xmin + c, ymin + c, xmax - c, ymax - c)

    def clear_of_walls(self, x: float, y: float) -> bool:
        c = self.obstacle_clearance
        return all(math.hypot(x - wx, y - wy) >= c for wx, wy in self.walls)

    def reachable_cells(
        self,
        zone_id: str,
        grid_cells,
    ) -> set[tuple[int, int]]:
        """중심이 zone 안이고 벽에서 wall_clearance 안쪽, 점유 픽셀에서 떨어진 칸."""
        polygon = self.zones()[zone_id]["polygon"]
        xmin, ymin, xmax, ymax = self.inner_bounds()
        return {
            (ix, iy)
            for ix, iy, cx, cy in grid_cells
            if in_polygon(cx, cy, polygon)
            and xmin <= cx <= xmax
            and ymin <= cy <= ymax
            and self.clear_of_walls(cx, cy)
        }

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

            dock = zone.get("dock")
            waypoints = self._build_scanline_route(
                polygon,
                spacing,
                bounds=self.inner_bounds() if self.wall_bounds else None,
                origin_y=self.origin[1],
                start_x=float(dock["x"]) if dock else None,
                clear=self.clear_of_walls if self.walls else None,
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
        zones = self.zones()
        routes = []
        self.skipped_facilities = []

        for facility in data.get("facilities", []):
            view = facility["view"]

            # 좌표를 아직 넣지 않은 설비 (0, 0) 는 건너뛴다
            if float(view["x"]) == 0.0 and float(view["y"]) == 0.0:
                self.skipped_facilities.append(str(facility["id"]))
                continue

            waypoint = Waypoint(
                x=float(view["x"]),
                y=float(view["y"]),
                yaw=float(view["yaw"]),
            )

            routes.append(
                FacilityRoute(
                    facility_id=str(facility["id"]),
                    waypoints=(waypoint,),
                    zone_id=next(
                        (
                            zone_id
                            for zone_id, zone in zones.items()
                            if in_polygon(
                                waypoint.x,
                                waypoint.y,
                                zone["polygon"],
                            )
                        ),
                        "",
                    ),
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
        bounds: tuple[float, float, float, float] | None = None,
        origin_y: float = 0.0,
        start_x: float | None = None,
        clear=None,
    ) -> list[Waypoint]:
        if len(polygon) < 3:
            return []

        ys = [point[1] for point in polygon]
        min_y = min(ys)
        max_y = max(ys)

        if max_y <= min_y or spacing <= 0.0:
            return []

        first_y = min_y + spacing / 2.0

        if bounds is not None:
            # 칸 중심 줄 중 polygon 과 벽 여유 안쪽에 드는 것만
            low = max(min_y, bounds[1])
            high = min(max_y, bounds[3])
            index = math.ceil((low - origin_y) / spacing - 0.5)
            scanline_ys = []

            while origin_y + (index + 0.5) * spacing < high:
                scanline_ys.append(origin_y + (index + 0.5) * spacing)
                index += 1
        elif first_y >= max_y:
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
        xs = [point[0] for point in polygon]
        # Dock 이 있는 쪽 끝에서 시작한다
        forward = start_x is None or start_x < (min(xs) + max(xs)) / 2.0

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

                if bounds is not None:
                    left_x = max(left_x, bounds[0])
                    right_x = min(right_x, bounds[2])
                    if left_x >= right_x:
                        continue

                if clear is not None:
                    # 줄 끝이 벽에 가까우면 안쪽으로 당긴다 (지도 해상도 5 cm 씩).
                    # shortcut: 줄 끝만 본다, 줄 가운데를 벽이 가로지르는 지도가 생기면 줄을 나눈다
                    while left_x < right_x and not clear(left_x, y):
                        left_x += 0.05
                    while left_x < right_x and not clear(right_x, y):
                        right_x -= 0.05
                    if left_x >= right_x:
                        continue

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
