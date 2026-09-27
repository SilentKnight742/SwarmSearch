from coverage import (
    partition_rectangle,
    generate_lawnmower_path,
)


WIDTH = 90
HEIGHT = 60
UAV_COUNT = 3


zones = partition_rectangle(
    WIDTH,
    HEIGHT,
    UAV_COUNT,
)

for index, zone in enumerate(zones, start=1):

    path = generate_lawnmower_path(
        zone,
        lane_spacing_m=15,
    )

    print(f"\nUAV-{index}")
    print(f"Zone: {zone}")

    for waypoint_number, waypoint in enumerate(path, start=1):
        print(
            f"  WP{waypoint_number}: "
            f"east={waypoint[0]:.1f}m, "
            f"north={waypoint[1]:.1f}m"
        )