from swarmsearch.coverage import (
    generate_position_aware_lawnmower_path,
)

from swarmsearch.models import (
    Waypoint,
    Zone,
)


ZONE = Zone(
    x_min=0.0,
    y_min=0.0,
    x_max=20.0,
    y_max=30.0,
)


def first_point(
    start: Waypoint,
) -> tuple[float, float]:
    route = (
        generate_position_aware_lawnmower_path(
            zone=ZONE,
            lane_spacing_m=15.0,
            start_position=start,
        )
    )

    return (
        route[0].east,
        route[0].north,
    )


def test_nearest_south_west_entry():
    assert first_point(
        Waypoint(
            east=-5.0,
            north=-2.0,
        )
    ) == (
        0.0,
        0.0,
    )


def test_nearest_south_east_entry():
    assert first_point(
        Waypoint(
            east=24.0,
            north=-2.0,
        )
    ) == (
        20.0,
        0.0,
    )


def test_nearest_north_west_entry():
    assert first_point(
        Waypoint(
            east=-5.0,
            north=35.0,
        )
    ) == (
        0.0,
        30.0,
    )


def test_nearest_north_east_entry():
    assert first_point(
        Waypoint(
            east=24.0,
            north=35.0,
        )
    ) == (
        20.0,
        30.0,
    )


def test_orientation_does_not_change_coverage():
    starts = [
        Waypoint(
            east=-5.0,
            north=-5.0,
        ),
        Waypoint(
            east=25.0,
            north=-5.0,
        ),
        Waypoint(
            east=-5.0,
            north=35.0,
        ),
        Waypoint(
            east=25.0,
            north=35.0,
        ),
    ]

    route_sets = []

    for start in starts:
        route = (
            generate_position_aware_lawnmower_path(
                zone=ZONE,
                lane_spacing_m=15.0,
                start_position=start,
            )
        )

        route_sets.append(
            {
                (
                    waypoint.east,
                    waypoint.north,
                )
                for waypoint
                in route
            }
        )

    assert all(
        route_set
        == route_sets[0]
        for route_set
        in route_sets
    )