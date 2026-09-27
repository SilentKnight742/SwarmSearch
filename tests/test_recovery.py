from swarmsearch.coverage import (
    build_search_plan,
)

from swarmsearch.recovery import (
    build_recovery_segment,
    choose_recovery_vehicle,
)


def test_recovery_preserves_path():

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

    segment = plan["UAV-2"]

    # UAV-2 completed waypoint indexes 0 and 1.
    #
    # It is now attempting waypoint index 2.
    recovery = build_recovery_segment(
        failed_vehicle="UAV-2",
        segment=segment,
        current_index=2,
    )

    points = [
        (wp.east, wp.north)
        for wp in recovery.waypoints
    ]

    assert points == [
        (40.0, 0.0),
        (40.0, 15.0),
        (20.0, 15.0),
        (20.0, 30.0),
        (40.0, 30.0),
    ]


def test_failure_before_search():

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

    segment = plan["UAV-2"]

    recovery = build_recovery_segment(
        failed_vehicle="UAV-2",
        segment=segment,
        current_index=0,
    )

    assert (
        recovery.waypoints
        == segment.waypoints
    )


def test_completed_segment_has_no_recovery():

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

    segment = plan["UAV-2"]

    recovery = build_recovery_segment(
        failed_vehicle="UAV-2",
        segment=segment,
        current_index=len(
            segment.waypoints
        ),
    )

    assert recovery.waypoints == []


def test_choose_least_loaded_survivor():

    selected = choose_recovery_vehicle(
        candidates=[
            "UAV-1",
            "UAV-3",
        ],
        queue_lengths={
            "UAV-1": 2,
            "UAV-3": 1,
        },
    )

    assert selected == "UAV-3"