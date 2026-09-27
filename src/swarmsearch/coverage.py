from .models import (
    RouteSegment,
    Waypoint,
    Zone,
)


def partition_rectangle(
    width_m: float,
    height_m: float,
    vehicle_names: list[str],
) -> dict[str, Zone]:

    if width_m <= 0 or height_m <= 0:
        raise ValueError(
            "Search dimensions must be positive."
        )

    if not vehicle_names:
        raise ValueError(
            "At least one vehicle is required."
        )

    zone_width = width_m / len(vehicle_names)

    zones: dict[str, Zone] = {}

    for index, name in enumerate(vehicle_names):
        zones[name] = Zone(
            x_min=index * zone_width,
            y_min=0.0,
            x_max=(index + 1) * zone_width,
            y_max=height_m,
        )

    return zones


def generate_lawnmower_path(
    zone: Zone,
    lane_spacing_m: float,
) -> list[Waypoint]:

    if lane_spacing_m <= 0:
        raise ValueError(
            "Lane spacing must be positive."
        )

    waypoints: list[Waypoint] = []

    y = zone.y_min
    left_to_right = True

    while y <= zone.y_max:

        if left_to_right:
            waypoints.extend(
                [
                    Waypoint(
                        east=zone.x_min,
                        north=y,
                    ),
                    Waypoint(
                        east=zone.x_max,
                        north=y,
                    ),
                ]
            )

        else:
            waypoints.extend(
                [
                    Waypoint(
                        east=zone.x_max,
                        north=y,
                    ),
                    Waypoint(
                        east=zone.x_min,
                        north=y,
                    ),
                ]
            )

        left_to_right = not left_to_right
        y += lane_spacing_m

    # Ensure the northern boundary is covered even when
    # lane spacing does not divide the zone height exactly.
    if (
        waypoints
        and waypoints[-1].north < zone.y_max
    ):
        if left_to_right:
            waypoints.extend(
                [
                    Waypoint(
                        east=zone.x_min,
                        north=zone.y_max,
                    ),
                    Waypoint(
                        east=zone.x_max,
                        north=zone.y_max,
                    ),
                ]
            )
        else:
            waypoints.extend(
                [
                    Waypoint(
                        east=zone.x_max,
                        north=zone.y_max,
                    ),
                    Waypoint(
                        east=zone.x_min,
                        north=zone.y_max,
                    ),
                ]
            )

    return waypoints


def build_search_plan(
    width_m: float,
    height_m: float,
    lane_spacing_m: float,
    vehicle_names: list[str],
) -> dict[str, RouteSegment]:

    zones = partition_rectangle(
        width_m,
        height_m,
        vehicle_names,
    )

    plan: dict[str, RouteSegment] = {}

    for name in vehicle_names:
        route = generate_lawnmower_path(
            zones[name],
            lane_spacing_m,
        )

        plan[name] = RouteSegment(
            segment_id=f"{name}-primary",
            source_uav=name,
            waypoints=route,
            kind="primary",
        )

    return plan