from .models import (
    RouteSegment,
    Waypoint,
)


def build_recovery_segment(
    failed_vehicle: str,
    segment: RouteSegment,
    current_index: int,
) -> RouteSegment:
    """
    Construct the ordered unfinished coverage path after a UAV
    becomes unavailable.

    current_index is the index of the waypoint the UAV was trying
    to reach when it failed.

    Example:

        original:
        A -> B -> C -> D -> E -> F

        UAV completes:
        A -> B

        fails while heading toward C

        recovery:
        B -> C -> D -> E -> F

    B is included so that the replacement UAV first repositions to
    the last known completed boundary and then traverses the exact
    unfinished coverage path.
    """

    if not segment.waypoints:
        return RouteSegment(
            segment_id=(
                f"{segment.segment_id}-recovery"
            ),
            source_uav=failed_vehicle,
            waypoints=[],
            kind="recovery",
        )

    if current_index < 0:
        raise ValueError(
            "current_index cannot be negative."
        )

    if current_index >= len(segment.waypoints):
        # Segment was already completed.
        return RouteSegment(
            segment_id=(
                f"{segment.segment_id}-recovery"
            ),
            source_uav=failed_vehicle,
            waypoints=[],
            kind="recovery",
        )

    if current_index == 0:
        # Failure before reaching the first point:
        # entire route remains.
        recovery_waypoints = list(
            segment.waypoints
        )

    else:
        last_completed = (
            segment.waypoints[
                current_index - 1
            ]
        )

        remaining = (
            segment.waypoints[
                current_index:
            ]
        )

        recovery_waypoints = [
            last_completed,
            *remaining,
        ]

    return RouteSegment(
        segment_id=(
            f"{segment.segment_id}-recovery"
        ),
        source_uav=failed_vehicle,
        waypoints=recovery_waypoints,
        kind="recovery",
    )


def choose_recovery_vehicle(
    candidates: list[str],
    queue_lengths: dict[str, int],
) -> str:
    """
    V1 recovery policy:

    Assign recovery work to the healthy UAV with the
    smallest queued workload.

    Later this can become distance/cost-aware without
    changing the coordinator interface.
    """

    if not candidates:
        raise RuntimeError(
            "No healthy UAVs available for recovery."
        )

    return min(
        candidates,
        key=lambda name: (
            queue_lengths.get(name, 0),
            name,
        ),
    )