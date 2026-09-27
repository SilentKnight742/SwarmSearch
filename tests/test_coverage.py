from swarmsearch.coverage import (
    build_search_plan,
)


def test_three_uav_search_plan():

    plan = build_search_plan(
        width_m=60,
        height_m=30,
        lane_spacing_m=15,
        vehicle_names=[
            "UAV-1",
            "UAV-2",
            "UAV-3",
        ],
    )

    assert len(plan) == 3

    assert len(
        plan["UAV-1"].waypoints
    ) == 6

    assert len(
        plan["UAV-2"].waypoints
    ) == 6

    assert len(
        plan["UAV-3"].waypoints
    ) == 6


def test_uav2_expected_route():

    plan = build_search_plan(
        width_m=60,
        height_m=30,
        lane_spacing_m=15,
        vehicle_names=[
            "UAV-1",
            "UAV-2",
            "UAV-3",
        ],
    )

    points = [
        (wp.east, wp.north)
        for wp in plan[
            "UAV-2"
        ].waypoints
    ]

    assert points == [
        (20.0, 0.0),
        (40.0, 0.0),

        (40.0, 15.0),
        (20.0, 15.0),

        (20.0, 30.0),
        (40.0, 30.0),
    ]