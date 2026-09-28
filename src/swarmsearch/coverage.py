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

    zone_width = (
        width_m / len(vehicle_names)
    )

    zones: dict[str, Zone] = {}

    for index, name in enumerate(
        vehicle_names
    ):
        zones[name] = Zone(
            x_min=index * zone_width,
            y_min=0.0,
            x_max=(index + 1)
            * zone_width,
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

        left_to_right = (
            not left_to_right
        )

        y += lane_spacing_m

    # Ensure the northern edge is included
    # even when lane spacing does not divide
    # the zone height exactly.
    if (
        waypoints
        and waypoints[-1].north
        < zone.y_max
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


def _mirror_route_east_west(
    route: list[Waypoint],
    zone: Zone,
) -> list[Waypoint]:
    """
    Mirror a route horizontally within its
    assigned zone.

    SW <-> SE
    NW <-> NE
    """

    midpoint_sum = (
        zone.x_min + zone.x_max
    )

    return [
        Waypoint(
            east=(
                midpoint_sum
                - waypoint.east
            ),
            north=waypoint.north,
        )
        for waypoint in route
    ]


def _distance_squared(
    a: Waypoint,
    b: Waypoint,
) -> float:
    east_delta = (
        a.east - b.east
    )

    north_delta = (
        a.north - b.north
    )

    return (
        east_delta * east_delta
        + north_delta * north_delta
    )


def generate_position_aware_lawnmower_path(
    zone: Zone,
    lane_spacing_m: float,
    start_position: Waypoint,
) -> list[Waypoint]:
    """
    Generate the same complete lawnmower
    coverage pattern, but choose its entry
    orientation according to the UAV's real
    current position.

    Four valid traversals are considered:

        south-west entry
        south-east entry
        north-west entry
        north-east entry

    Coverage geometry remains unchanged.
    Only traversal direction changes.
    """

    base = generate_lawnmower_path(
        zone=zone,
        lane_spacing_m=(
            lane_spacing_m
        ),
    )

    mirrored = (
        _mirror_route_east_west(
            base,
            zone,
        )
    )

    candidates = [
        base,
        mirrored,
        list(reversed(base)),
        list(reversed(mirrored)),
    ]

    return min(
        candidates,
        key=lambda route: (
            _distance_squared(
                start_position,
                route[0],
            )
        ),
    )


def build_search_plan(
    width_m: float,
    height_m: float,
    lane_spacing_m: float,
    vehicle_names: list[str],
    start_positions: (
        dict[str, Waypoint] | None
    ) = None,
) -> dict[str, RouteSegment]:
    zones = partition_rectangle(
        width_m,
        height_m,
        vehicle_names,
    )

    plan: dict[
        str,
        RouteSegment,
    ] = {}

    for name in vehicle_names:
        start_position = (
            start_positions.get(name)
            if start_positions
            else None
        )

        if start_position is None:
            route = (
                generate_lawnmower_path(
                    zones[name],
                    lane_spacing_m,
                )
            )

        else:
            route = (
                generate_position_aware_lawnmower_path(
                    zone=zones[name],
                    lane_spacing_m=(
                        lane_spacing_m
                    ),
                    start_position=(
                        start_position
                    ),
                )
            )

        plan[name] = RouteSegment(
            segment_id=(
                f"{name}-primary"
            ),
            source_uav=name,
            waypoints=route,
            kind="primary",
        )

    return plan